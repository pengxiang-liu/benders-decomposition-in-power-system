#!/usr/bin/python
#
# Copyright 2019, Southeast University, Liu Pengxiang
#
# A distribution system planning model using logic-based Benders decomposition
# 
# This project develops a multistage planning model for distribution system where 
# investments in the distribution network and distributed generations are jointly 
# considered. The planning model is decomposed into three layers.
#
# Moreover, a logic-based Benders decomposition algorithm is developed to deal with
# the integer variables in the sub-problem


import sys
import math
import xlrd
import numpy as np
import matplotlib.pyplot as plt

from Algorithm import BranchBound
from System    import ReadData
from gurobipy  import *


# This function creates the upper-level planning problem (MILP), The Logic-based 
# Benders cut is added iteratively in a lazy constraint callback framework
def Planning_MP(Para,Info):

    # Create a gurobi model
    model = Model()

    # Create investment variables
    x_line  = model.addVars(Para.N_line,  Para.N_stage, vtype = GRB.BINARY)  # Binary variables of line
    x_sub   = model.addVars(Para.N_sub,   Para.N_stage, vtype = GRB.BINARY)  # Binary variables of Substation
    x_wind  = model.addVars(Para.N_wind,  Para.N_stage, vtype = GRB.BINARY)  # Binary variables of Wind farm
    x_solar = model.addVars(Para.N_solar, Para.N_stage, vtype = GRB.BINARY)  # Binary variables of PV station
    # Create fictitious variables
    f_line  = model.addVars(Para.N_line,  Para.N_stage, lb = -GRB.INFINITY)  # fictitious line flow
    f_load  = model.addVars(Para.N_bus,   Para.N_stage, lb = -GRB.INFINITY)  # fictitious load demand
    f_sub   = model.addVars(Para.N_sub,   Para.N_stage, lb = -GRB.INFINITY)  # fictitious power input

    # Set Objective
    obj = LinExpr()
    for t in range(Para.N_stage):
        Rec_rate = 0 # Reconvery rate in 5 years
        for y in range(Para.N_year_of_stage):
            Rec_rate = Rec_rate + (1 + Para.Int_rate) ** (-(t * Para.N_year_of_stage + y + 1))
        obj = obj + quicksum(Rec_rate * x_line [n,t] * Para.Line [n][8] * Para.Dep_line  for n in range(Para.N_line ))
        obj = obj + quicksum(Rec_rate * x_sub  [n,t] * Para.Sub  [n][4] * Para.Dep_sub   for n in range(Para.N_sub  ))
        obj = obj + quicksum(Rec_rate * x_wind [n,t] * Para.Wind [n][3] * Para.Dep_wind  for n in range(Para.N_wind ))
        obj = obj + quicksum(Rec_rate * x_solar[n,t] * Para.Solar[n][3] * Para.Dep_solar for n in range(Para.N_solar))
    model.setObjective(obj, GRB.MINIMIZE)

    # Constraint 1 (installation)
    for t in range(Para.N_stage-1):
        model.addConstrs(x_line [n,t] <= x_line [n,t+1] for n in range(Para.N_line ))
        model.addConstrs(x_sub  [n,t] <= x_sub  [n,t+1] for n in range(Para.N_sub  ))
        model.addConstrs(x_wind [n,t] <= x_wind [n,t+1] for n in range(Para.N_wind ))
        model.addConstrs(x_solar[n,t] <= x_solar[n,t+1] for n in range(Para.N_solar))

    # Constraint 2 (substation)
    for t in range(Para.N_stage):
        for n in range(Para.N_sub_new):
            line_head = Info.Line_head[int(Para.Sub_new[n][1])]
            line_tail = Info.Line_tail[int(Para.Sub_new[n][1])]
            model.addConstrs(x_line[i,t] <= x_sub[Para.N_sub_ext + n,t] for i in line_head)
            model.addConstrs(x_line[i,t] <= x_sub[Para.N_sub_ext + n,t] for i in line_tail)

    # Constraint 3 (fictitious power flow initialization)
    for t in range(Para.N_stage):
        for n in range(Para.N_line):
            if n < Para.N_line_ext:
                model.addConstr(f_line[n,t] >= -50)
                model.addConstr(f_line[n,t] <=  50)
            else:
                model.addConstr(f_line[n,t] >= -50 * x_line[n,t])
                model.addConstr(f_line[n,t] <=  50 * x_line[n,t])
        for n in range(Para.N_bus):
            if Para.Load[n][t] == 0:
                model.addConstr(f_load[n,t] == 0)
            else:
                model.addConstr(f_load[n,t] == 1)
        for n in range(Para.N_sub):
            model.addConstr(f_sub[n,t] >= 0)
            model.addConstr(f_sub[n,t] <= 50)

    # Constraint 4 (connectivity)
    for t in range(Para.N_stage):
        for n in range(Para.N_bus):
            line_head = Info.Line_head[n]
            line_tail = Info.Line_tail[n]
            expr = quicksum(f_line[i,t] for i in line_head) - quicksum(f_line[i,t] for i in line_tail)
            if Info.Sub[n] == []:
                model.addConstr(expr + f_load[n,t] == 0)
            else:
                model.addConstr(expr + f_load[n,t] == f_sub[int(Info.Sub[n][0]),t])
    
    model.optimize()


if __name__ == "__main__":
    # Input parameter
    filename = "../data/Data-IEEE-24.xlsx"
    [Para,Info] = ReadData(filename)
    Planning_MP(Para,Info)