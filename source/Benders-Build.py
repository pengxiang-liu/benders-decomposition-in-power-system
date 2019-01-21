#!/usr/bin/python
#
# Copyright 2019, Southeast University, Liu Pengxiang
#
# A distribution system planning model using logic-based Benders decomposition
# Demo version: 1 stage, 1 typical day, 2 scenarios
# 
# This project develops a multistage planning model for distribution system where 
# investments in the distribution network and distributed generations are jointly 
# considered. The planning model is decomposed into three layers.


import sys
import math
import xlrd
import time
import numpy as np
import matplotlib.pyplot as plt

from gurobipy import *


# This class builds the system parameter, including data of system, bus, line,
# substation, wind farm, solar station and typical day(load, wind and solar)
#
class Parameter(object):
    def __init__(self,Data_origin):
        # System
        self.N_stage = 1
        self.N_year_of_stage = 15
        self.N_day = 1  # number of typical day
        self.N_day_year = 365  # number of days in a year
        self.N_scenario = 2  # number of reconfiguration in a day
        self.N_hour = int(round(24/self.N_scenario))  # number of hour in a scenario
        self.Int_rate = 0.05  # interest rate
        self.Big_M = 245  # Big M
        self.Voltage = 35
        self.Voltage_low = 35 * 0.95
        self.Voltage_upp = 35 * 1.05
        # Bus data
        Bus = Data_origin[0]
        self.Bus = Bus
        self.N_bus = len(Bus)
        self.Coordinate = Bus[:,2:4]  # coordinate
        self.Load = Bus[:,4]  # load demand
        self.Load_angle = 0.95  # phase angle of load
        self.Cost_cutload = 200  # cost of load shedding
        # Line data
        Line = Data_origin[1]
        Year_line = 25  # life-time of line
        self.Line = Line
        self.Line_R = Line[:,4]  # resistence
        self.Line_X = Line[:,5]  # reactance
        self.Line_S = Line[:,6]  # capacity
        self.N_line = len(Line)
        self.Dep_line = Depreciation(Year_line,self.Int_rate)
        # Substation data
        Sub = Data_origin[2]
        Year_sub = 15  # life-time of substation
        self.Sub = Sub
        self.Sub_S = Sub[:,2]  # capacity
        self.N_sub = len(self.Sub)
        self.Cost_power = 70  # cost of power purchasing
        self.Dep_sub = Depreciation(Year_sub,self.Int_rate)
        # Wind data
        Wind = Data_origin[3]
        Year_wind = 15
        self.Wind = Wind
        self.N_wind = len(Wind)
        self.Cost_wind = Wind[0,4]  # cost of wind generation
        self.Cost_cutwind = 150  # cost of wind curtailment
        self.Wind_angle = 0.98  # phase angle of wind farm output
        self.Dep_wind = Depreciation(Year_wind,self.Int_rate)
        # Solar data
        Solar = Data_origin[4]
        Year_solar = 15
        self.Solar = Solar
        self.N_solar = len(Solar)
        self.Cost_solar = Solar[0,4]  # cost of solar generation
        self.Cost_cutsolar = 150  # cost of solar curtailment
        self.Solar_angle = 1.00  # phase angle of PV station output
        self.Dep_solar = Depreciation(Year_solar,self.Int_rate)
        # Typical data
        self.Typical_load  = Data_origin[5][:,1]  # load
        self.Typical_wind  = Data_origin[6][:,1]  # wind
        self.Typical_solar = Data_origin[7][:,1]  # solar


# This class builds the infomation for each bus i, including the set of lines with a
# head or tail end of bus i, substation, wind farm and solar station. If there is no
# related information, the corresponding matrix is set to empty
# 
class BusInfo(object):
    def __init__(self,Para):
        # line
        Line_head = [[] for i in range(Para.N_bus)]
        Line_tail = [[] for i in range(Para.N_bus)]
        for i in range(Para.N_line):
            head = Para.Line[i][1]
            tail = Para.Line[i][2]
            Line_head[int(round(head))].append(i)  # set of lines whose head-end is bus i
            Line_tail[int(round(tail))].append(i)  # set of lines whose tail-end is bus i
        self.Line_head = Line_head
        self.Line_tail = Line_tail
        # Substation
        Sub = [[] for i in range(Para.N_bus)]
        for i in range(Para.N_sub):
            Sub[int(round(Para.Sub[i][1]))].append(i)  # substation number of bus i
        self.Sub = Sub
        # Wind
        Wind = [[] for i in range(Para.N_bus)]
        for i in range(Para.N_wind):
            Wind[int(round(Para.Wind[i][1]))].append(i)  # wind farm number of bus i
        self.Wind = Wind
        # Solar
        Solar = [[] for i in range(Para.N_bus)]
        for i in range(Para.N_solar):
            Solar[int(round(Para.Solar[i][1]))].append(i)  # PV station number of bus i
        self.Solar = Solar


# This class restores the results of planning master problem
class ResultPlanning(object):
    def __init__(self,model,Para,x_line,x_sub,x_wind,x_solar,obj_con,obj_opr):
        self.x_line  = [int(round(x_line [i].x)) for i in range(Para.N_line )]
        self.x_sub   = [int(round(x_sub  [i].x)) for i in range(Para.N_sub  )]
        self.x_wind  = [int(round(x_wind [i].x)) for i in range(Para.N_wind )]
        self.x_solar = [int(round(x_solar[i].x)) for i in range(Para.N_solar)]
        self.x_star  = self.x_line + self.x_sub + self.x_wind + self.x_solar
        self.obj_con = obj_con.getValue()
        self.obj_opr = obj_opr.x
        self.obj = (model.getObjective()).getValue()  # objective


# This class restores the results of reconfiguration sub-problem
class ResultReconfig(object):
    def __init__(self,model,Para,y_line,y_sub,y_wind,y_solar,y_pos,y_neg,Var,N_con):
        # Reconfiguration variable
        self.y_line  = [int(round(y_line [i].x)) for i in range(Para.N_line )]
        self.y_sub   = [int(round(y_sub  [i].x)) for i in range(Para.N_sub  )]
        self.y_wind  = [int(round(y_wind [i].x)) for i in range(Para.N_wind )]
        self.y_solar = [int(round(y_solar[i].x)) for i in range(Para.N_solar)]
        self.y_pos   = [int(round(y_pos  [i].x)) for i in range(Para.N_line )]
        self.y_neg   = [int(round(y_neg  [i].x)) for i in range(Para.N_line )]
        # Power flow variable
        self.V_bus   = [Var[N_V_bus   + i].x for i in range(Para.N_bus  )]
        self.P_line  = [Var[N_P_line  + i].x for i in range(Para.N_line )]
        self.Q_line  = [Var[N_Q_line  + i].x for i in range(Para.N_line )]
        self.P_sub   = [Var[N_P_sub   + i].x for i in range(Para.N_sub  )]
        self.Q_sub   = [Var[N_Q_sub   + i].x for i in range(Para.N_sub  )]
        self.C_load  = [Var[N_C_load  + i].x for i in range(Para.N_bus  )]
        self.S_wind  = [Var[N_S_wind  + i].x for i in range(Para.N_wind )]
        self.C_wind  = [Var[N_C_wind  + i].x for i in range(Para.N_wind )]
        self.S_solar = [Var[N_S_solar + i].x for i in range(Para.N_solar)]
        self.C_solar = [Var[N_C_solar + i].x for i in range(Para.N_solar)]
        # Objective
        constr = model.getConstrs()
        self.N_con = N_con
        self.obj   = (model.getObjective()).getValue()
        self.rhs   = np.array([constr[n].rhs for n in range(N_con[-1])])


# This class restores the results of reconfiguration sub-problem
class ResultReconfigDual(object):
    def __init__(self,model,Para,N_con,Result_Planning):
        # Dual information for reconfiguration variables
        constr = model.getConstrs()
        self.N_con  = N_con
        self.obj    = (model.getObjective()).getValue()
        self.y_dual = np.array([constr[n].pi  for n in range(N_con[-2],N_con[-1])])
        self.y_star = np.array([constr[n].rhs for n in range(N_con[-2],N_con[-1])])
        self.x_star = Result_Planning.x_star
        self.rhs    = np.array([constr[n].rhs for n in range(N_con[-1])])


# This function input data from Excel files. The filtname can be changed to other
# power system for further study
#
def ReadData(filename):
    Data_origin = []
    readbook = xlrd.open_workbook(filename)
    # Data preprocessing
    for i in range(8):
        sheet = readbook.sheet_by_index(i+1)
        n_row = sheet.nrows
        n_col = sheet.ncols
        Coordinate = [1,n_row,0,n_col]  # coordinate of slice
        Data_temp = sheet._cell_values  # data in the Excel file
        Data_origin.append(np.array(Matrix_slice(Data_temp,Coordinate)))
    # Data formulation
    Para = Parameter(Data_origin)  # System parameter
    Info = BusInfo(Para)  # Bus Information
    return Para,Info


# This function slice the matrix for easy operation
#
def Matrix_slice(Matrix,Coordinate):
    row_start = Coordinate[0]
    row_end   = Coordinate[1]
    col_start = Coordinate[2]
    col_end   = Coordinate[3]
    Matrix_partitioned = []  # A partitioned matrix
    for i in range(row_end-row_start):
        Matrix_partitioned.append([])
        for j in range(col_end-col_start):
            Matrix_partitioned[i].append(Matrix[row_start+i][col_start+j])
    return Matrix_partitioned


# This function creates a depreciation calculator
#
def Depreciation(Life,Rate):
    return Rate*((1+Rate)**Life)/((1+Rate)**Life-1)


# This function creates the upper-level planning problem (MILP), The Logic-based
# Benders cut is added iteratively
#
def Planning(Para,Info,Logic,n_iter):
    #
    # minimize
    #       Investment costs of line, substation, wind farm and PV station
    # subject to
    #       1) If substation is not built, the line connected to it cannot be built
    #       2) If load is zero, the line connected to the load bus cannot be built
    #       3) fictitious power flow variables are initialized throuth constraints
    #       4) The planning solution should remain connected
    #
    model = Model()
    # Create investment variables
    x_line  = model.addVars(Para.N_line,  vtype = GRB.BINARY)  # line
    x_sub   = model.addVars(Para.N_sub,   vtype = GRB.BINARY)  # Substation
    x_wind  = model.addVars(Para.N_wind,  vtype = GRB.BINARY)  # Wind farm
    x_solar = model.addVars(Para.N_solar, vtype = GRB.BINARY)  # PV station
    model.update()
    x_var   = model.getVars()
    # Create fictitious variables
    f_line  = model.addVars(Para.N_line,  lb = -GRB.INFINITY)  # line flow
    f_load  = model.addVars(Para.N_bus,   lb = -GRB.INFINITY)  # load demand
    f_sub   = model.addVars(Para.N_sub,   lb = -GRB.INFINITY)  # power input
    # Create operation cost variables
    obj_opr = model.addVar()

    # Set objective
    obj_con = LinExpr()
    Rec_rate = 0  # Reconvery rate in 5 years
    for y in range(Para.N_year_of_stage):
        Rec_rate = Rec_rate + (1 + Para.Int_rate) ** (-(y + 1))
    obj_con = obj_con + quicksum(Rec_rate * x_line [n] * Para.Line [n,7] * Para.Dep_line  for n in range(Para.N_line ))
    obj_con = obj_con + quicksum(Rec_rate * x_sub  [n] * Para.Sub  [n,3] * Para.Dep_sub   for n in range(Para.N_sub  ))
    obj_con = obj_con + quicksum(Rec_rate * x_wind [n] * Para.Wind [n,3] * Para.Dep_wind  for n in range(Para.N_wind ))
    obj_con = obj_con + quicksum(Rec_rate * x_solar[n] * Para.Solar[n,3] * Para.Dep_solar for n in range(Para.N_solar))

    # Constraint 1 (substation)
    for n in range(Para.N_sub):
        line_head = Info.Line_head[int(round(Para.Sub[n,1]))]
        line_tail = Info.Line_tail[int(round(Para.Sub[n,1]))]
        model.addConstrs(x_line[i] <= x_sub[n] for i in line_head)
        model.addConstrs(x_line[i] <= x_sub[n] for i in line_tail)
    # Constraint 2 (load bus)
    for n in range(Para.N_bus):
        if Para.Load[n] == 0 and n < 20:
            line_head = Info.Line_head[n]
            line_tail = Info.Line_tail[n]
            model.addConstrs(x_line[i] == 0 for i in line_head)
            model.addConstrs(x_line[i] == 0 for i in line_tail)
    # Constraint 3 (fictitious power flow initialization)
    for n in range(Para.N_line):
        model.addConstr(f_line[n] >= -50 * x_line[n])
        model.addConstr(f_line[n] <=  50 * x_line[n])
    for n in range(Para.N_bus):
        model.addConstr(f_load[n] == 1)
    for n in range(Para.N_sub):
        model.addConstr(f_sub [n] >= 0)
        model.addConstr(f_sub [n] <= 50)
    # Constraint 4 (connectivity)
    for n in range(Para.N_bus):
        line_head = Info.Line_head[n]
        line_tail = Info.Line_tail[n]
        expr = quicksum(f_line[i] for i in line_head) - quicksum(f_line[i] for i in line_tail)
        if Info.Sub[n] == []:
            model.addConstr(expr + f_load[n] == 0)
        else:
            model.addConstr(expr + f_load[n] == f_sub[int(round(Info.Sub[n][0]))]) 
    # Constraint 5 (capacity)
    for s in range(Para.N_scenario):
        tp_load  = Para.Load * Para.Typical_load[s]  # load
        tp_wind  = Para.Wind [:,2] * Para.Typical_wind [s]  # wind
        tp_solar = Para.Solar[:,2] * Para.Typical_solar[s]  # solar
        expr = LinExpr()
        expr = expr + quicksum(x_sub[n] * Para.Sub_S[n] for n in range(Para.N_sub  ))
        expr = expr + quicksum(x_wind [n] * tp_wind [n] for n in range(Para.N_wind ))
        expr = expr + quicksum(x_solar[n] * tp_solar[n] for n in range(Para.N_solar))
        model.addConstr(expr >= sum(tp_load))

    # Logic-based Benders cut
    if n_iter == 0:
        model.addConstr(obj_opr == 0)
    else:
        for i in range(n_iter):
            dual_x = np.zeros(53)
            dual_f = 0
            for s in range(2*i, 2*i + Para.N_scenario):
                # Dual information
                dual   = Logic[s].y_dual  # dual variables
                beta   = Logic[s].obj     # objective
                x_star = Logic[s].x_star  # planning solution
                # 
                dual_x = dual_x + dual[0:53]
                dual_f = dual_f + beta
            model.addConstr(obj_opr >= dual_f + quicksum(dual_x[n] * (x_var[n] - x_star[n]) for n in range(53)))
    # Optimize    
    model.setObjective(obj_con + obj_opr, GRB.MINIMIZE)
    model.optimize()
    if model.status == GRB.Status.OPTIMAL:
        result = ResultPlanning(model,Para,x_line,x_sub,x_wind,x_solar,obj_con,obj_opr)
    return result


# This function creates the reconfiguration sub-problem (MILP). After the problem is solved,
# the binary reconfiguration variables are fixed and dual variables are returned for further
# analysis and operation. Note that the problem is formulated under a given scenario 's' of 
# typical day 'd' at stage 't'.
#
def Reconfig(Para,Info,Result_Planning,s):
    #
    # minimize
    #       Costs of power purchasing, load shedding, renewables generation and curtailment 
    # subject to
    #       1) Reconfiguration
    #       2) Radial topology
    #       3) Optimal Power flow
    #       4) Upper and lower bound
    #
    model = Model()
    global N_V_bus,  N_P_line, N_Q_line, N_P_sub, N_Q_sub
    global N_C_load, N_S_wind, N_C_wind, N_S_solar, N_C_solar
    # Typical data
    tp_load  = Para.Load * Para.Typical_load[s]  # load
    tp_wind  = Para.Wind [:,2] * Para.Typical_wind [s]  # wind
    tp_solar = Para.Solar[:,2] * Para.Typical_solar[s]  # solar
    # Capacity data
    Cap_line = [Result_Planning.x_line[n] * Para.Line_S[n] for n in range(Para.N_line)]
    Cap_sub  = [Result_Planning.x_sub [n] * Para.Sub_S [n] for n in range(Para.N_sub )]

    # Create reconfiguration variables
    y_line  = model.addVars(Para.N_line, vtype = GRB.BINARY)  # line
    y_sub   = model.addVars(Para.N_sub,  vtype = GRB.BINARY)  # Substation
    y_wind  = model.addVars(Para.N_wind, vtype = GRB.BINARY)  # Wind farm
    y_solar = model.addVars(Para.N_solar,vtype = GRB.BINARY)  # PV station
    y_pos   = model.addVars(Para.N_line, vtype = GRB.BINARY)  # positive line flow
    y_neg   = model.addVars(Para.N_line, vtype = GRB.BINARY)  # negative line flow
    # Create power flow variables
    N_V_bus   = 0  # Square of bus voltage
    N_P_line  = N_V_bus   + Para.N_bus    # Active power flow of line
    N_Q_line  = N_P_line  + Para.N_line   # Reactive power flow of line
    N_P_sub   = N_Q_line  + Para.N_line   # Active power injection at substation
    N_Q_sub   = N_P_sub   + Para.N_sub    # Reactive power injection at substation
    N_C_load  = N_Q_sub   + Para.N_sub    # Load shedding
    N_S_wind  = N_C_load  + Para.N_bus    # Wind output
    N_C_wind  = N_S_wind  + Para.N_wind   # Wind curtailment
    N_S_solar = N_C_wind  + Para.N_wind   # Solar output
    N_C_solar = N_S_solar + Para.N_solar  # Solar curtailment
    N_Var     = N_C_solar + Para.N_solar  # Number of all variables
    Var = model.addVars(N_Var,lb = -GRB.INFINITY)
    
    # Set objective
    obj = LinExpr()
    obj = obj + quicksum(Var[N_P_sub   + n] * Para.Cost_power    for n in range(Para.N_sub  ))
    obj = obj + quicksum(Var[N_C_load  + n] * Para.Cost_cutload  for n in range(Para.N_bus  ))
    obj = obj + quicksum(Var[N_S_wind  + n] * Para.Cost_wind     for n in range(Para.N_wind ))
    obj = obj + quicksum(Var[N_C_wind  + n] * Para.Cost_cutwind  for n in range(Para.N_wind ))
    obj = obj + quicksum(Var[N_S_solar + n] * Para.Cost_solar    for n in range(Para.N_solar))
    obj = obj + quicksum(Var[N_C_solar + n] * Para.Cost_cutsolar for n in range(Para.N_solar))
    obj = obj * Para.N_day_year * Para.N_hour * 5
    model.setObjective(obj, GRB.MINIMIZE)

    N_con = []  # indexing constraints
    # 0.Reconfiguration
    for n in range(Para.N_line):
        model.addConstr(y_line [n] <= Result_Planning.x_line [n])
    for n in range(Para.N_sub):
        model.addConstr(y_sub  [n] <= Result_Planning.x_sub  [n])
    for n in range(Para.N_wind):
        model.addConstr(y_wind [n] <= Result_Planning.x_wind [n])
    for n in range(Para.N_solar):
        model.addConstr(y_solar[n] <= Result_Planning.x_solar[n])
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 0

    # 1.Radial topology
    for n in range(Para.N_bus):
        line_head = Info.Line_head[n]
        line_tail = Info.Line_tail[n]
        expr = quicksum(y_pos[i] for i in line_tail) + quicksum(y_neg[i] for i in line_head)
        if Para.Load[n] > 0:  # load bus
            model.addConstr(expr == 1)
        else:  # none load bus
            model.addConstr(expr == 0)
    for n in range(Para.N_line):
        model.addConstr(y_pos[n] + y_neg[n] - y_line[n] == 0)
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 1

    # 2.Active power balance equation
    for n in range(Para.N_bus):
        line_head = Info.Line_head[n]
        line_tail = Info.Line_tail[n]
        expr = LinExpr()
        expr = expr - quicksum(Var[N_P_line + i] for i in line_head)  # power out
        expr = expr + quicksum(Var[N_P_line + i] for i in line_tail)  # power in
        expr = expr + Var[N_C_load + n] * Para.Load_angle  # load shedding
        if  Info.Sub[n]   != []:
            expr = expr + Var[N_P_sub + Info.Sub[n][0]]
        if  Info.Wind[n]  != []:
            expr = expr + Var[N_S_wind + Info.Wind[n][0]] * Para.Wind_angle
        if  Info.Solar[n] != []:
            expr = expr + Var[N_S_solar + Info.Solar[n][0]] * Para.Solar_angle
        model.addConstr(expr == tp_load[n] * Para.Load_angle)
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 2

    # 3.Reactive power balance equation
    for n in range(Para.N_bus):
        line_head = Info.Line_head[n]
        line_tail = Info.Line_tail[n]
        expr = LinExpr()
        expr = expr - quicksum(Var[N_Q_line + i] for i in line_head)  # power out
        expr = expr + quicksum(Var[N_Q_line + i] for i in line_tail)  # power in
        expr = expr + Var[N_C_load + n] * math.sqrt(1-Para.Load_angle**2)  # load shedding
        if  Info.Sub[n]   != []:
            expr = expr + Var[N_Q_sub + Info.Sub[n][0]]
        if  Info.Wind[n]  != []:
            expr = expr + Var[N_S_wind + Info.Wind[n][0]] * math.sqrt(1 - Para.Wind_angle ** 2)
        if  Info.Solar[n] != []:
            expr = expr + Var[N_S_solar + Info.Solar[n][0]] * math.sqrt(1 - Para.Solar_angle ** 2)
        model.addConstr(expr == tp_load[n] * math.sqrt(1 - Para.Load_angle ** 2))
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 3

    # 4.Voltage balance on line
    for n in range(Para.N_line):
        bus_head = Para.Line[n,1]
        bus_tail = Para.Line[n,2]
        expr = LinExpr()
        expr = expr + Var[N_V_bus + bus_head] - Var[N_V_bus + bus_tail]  # voltage difference
        expr = expr - Var[N_P_line + n] * 2 * Para.Line_R[n]
        expr = expr - Var[N_Q_line + n] * 2 * Para.Line_X[n]
        model.addConstr(expr >= -Para.Big_M * (1 - y_line[n]))
        model.addConstr(expr <=  Para.Big_M * (1 - y_line[n]))
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 4

    # 5.Renewables(wind farm)
    for n in range(Para.N_wind):
        expr = Var[N_S_wind  + n] + Var[N_C_wind  + n]
        model.addConstr(expr == y_wind[n] * tp_wind [n])
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 5

    # 6.Renewables(solar station)
    for n in range(Para.N_solar):
        expr = Var[N_S_solar + n] + Var[N_C_solar + n]
        model.addConstr(expr == y_solar[n] * tp_solar[n])
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 6

    # 7.Linearization of quadratic terms
    for n in range(Para.N_line):
        expr = Var[N_P_line + n] + Var[N_Q_line + n]
        model.addConstr(expr >= -math.sqrt(2) * y_line[n] * Cap_line[n])
        model.addConstr(expr <=  math.sqrt(2) * y_line[n] * Cap_line[n])
        expr = Var[N_P_line + n] - Var[N_Q_line + n]
        model.addConstr(expr >= -math.sqrt(2) * y_line[n] * Cap_line[n])
        model.addConstr(expr <=  math.sqrt(2) * y_line[n] * Cap_line[n])
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 7

    # 8.Linearization of quadratic terms
    for n in range(Para.N_sub):
        expr = Var[N_P_sub + n] + Var[N_Q_sub + n]
        model.addConstr(expr >= 0)
        model.addConstr(expr <= math.sqrt(2) * y_sub[n] * Cap_sub[n])
        expr = Var[N_P_sub + n] - Var[N_Q_sub + n]
        model.addConstr(expr >= 0)
        model.addConstr(expr <= math.sqrt(2) * y_sub[n] * Cap_sub[n])
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 8

    # 9.bounds of variables
    # 1) Voltage
    for n in range(Para.N_bus):
        model.addConstr(Var[N_V_bus + n] >= Para.Voltage_low ** 2)
        model.addConstr(Var[N_V_bus + n] <= Para.Voltage_upp ** 2)
    # 2) Power flow
    for n in range(Para.N_line):
        model.addConstr(Var[N_P_line + n] >= -y_line[n] * Cap_line[n])
        model.addConstr(Var[N_P_line + n] <=  y_line[n] * Cap_line[n])
    for n in range(Para.N_line):
        model.addConstr(Var[N_Q_line + n] >= -y_line[n] * Cap_line[n])
        model.addConstr(Var[N_Q_line + n] <=  y_line[n] * Cap_line[n])
    # 3) Substation
    for n in range(Para.N_sub):
        model.addConstr(Var[N_P_sub + n] >= 0)
        model.addConstr(Var[N_P_sub + n] <= y_sub[n] * Cap_sub[n])
    for n in range(Para.N_sub):
        model.addConstr(Var[N_Q_sub + n] >= 0)
        model.addConstr(Var[N_Q_sub + n] <= y_sub[n] * Cap_sub[n])
    # 4) Load shedding
    for n in range(Para.N_bus):
        model.addConstr(Var[N_C_load + n] >= 0)
        model.addConstr(Var[N_C_load + n] <= tp_load[n])
    # 5) Renewables
    for n in range(Para.N_wind):
        model.addConstr(Var[N_S_wind + n] >= 0)
        model.addConstr(Var[N_S_wind + n] <= y_wind[n] * tp_wind[n])
        model.addConstr(Var[N_C_wind + n] >= 0)
        model.addConstr(Var[N_C_wind + n] <= y_wind[n] * tp_wind[n])
    for n in range(Para.N_solar):
        model.addConstr(Var[N_S_solar + n] >= 0)
        model.addConstr(Var[N_S_solar + n] <= y_solar[n] * tp_solar[n])
        model.addConstr(Var[N_C_solar + n] >= 0)
        model.addConstr(Var[N_C_solar + n] <= y_solar[n] * tp_solar[n])
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 9

    # Optimize
    model.optimize()
    if model.status == GRB.Status.OPTIMAL:
        result = ResultReconfig(model,Para,y_line,y_sub,y_wind,y_solar,y_pos,y_neg,Var,N_con)
    return result


# This function creates the reconfiguration sub-problem where the binary reconfiguration 
# variables are fixed and dual variables are returned for further analysis and operation. 
#
def ReconfigDual(Para,Info,Result_Planning,Result_Reconfig,s):
    #
    # minimize
    #       Costs of power purchasing, load shedding, renewables generation and curtailment 
    # subject to
    #       1) Optimal Power flow
    #       2) Upper and lower bound
    #       3) fixed value
    #
    model = Model()
    global N_V_bus,  N_P_line, N_Q_line, N_P_sub, N_Q_sub
    global N_C_load, N_S_wind, N_C_wind, N_S_solar, N_C_solar
    # Typical data
    tp_load  = Para.Load * Para.Typical_load[s]  # load
    tp_wind  = Para.Wind [:,2] * Para.Typical_wind [s]  # wind
    tp_solar = Para.Solar[:,2] * Para.Typical_solar[s]  # solar

    # Create reconfiguration variables
    y_line  = model.addVars(Para.N_line )  # line
    y_sub   = model.addVars(Para.N_sub  )  # Substation
    y_wind  = model.addVars(Para.N_wind )  # Wind farm
    y_solar = model.addVars(Para.N_solar)  # PV station

    # Create power flow variables
    N_V_bus   = 0  # Square of bus voltage
    N_P_line  = N_V_bus   + Para.N_bus    # Active power flow of line
    N_Q_line  = N_P_line  + Para.N_line   # Reactive power flow of line
    N_P_sub   = N_Q_line  + Para.N_line   # Active power injection at substation
    N_Q_sub   = N_P_sub   + Para.N_sub    # Reactive power injection at substation
    N_C_load  = N_Q_sub   + Para.N_sub    # Load shedding
    N_S_wind  = N_C_load  + Para.N_bus    # Wind output
    N_C_wind  = N_S_wind  + Para.N_wind   # Wind curtailment
    N_S_solar = N_C_wind  + Para.N_wind   # Solar output
    N_C_solar = N_S_solar + Para.N_solar  # Solar curtailment
    N_Var     = N_C_solar + Para.N_solar  # Number of all variables
    Var = model.addVars(N_Var,lb = -GRB.INFINITY)
    
    # Set objective
    obj = LinExpr()
    obj = obj + quicksum(Var[N_P_sub   + n] * Para.Cost_power    for n in range(Para.N_sub  ))
    obj = obj + quicksum(Var[N_C_load  + n] * Para.Cost_cutload  for n in range(Para.N_bus  ))
    obj = obj + quicksum(Var[N_S_wind  + n] * Para.Cost_wind     for n in range(Para.N_wind ))
    obj = obj + quicksum(Var[N_C_wind  + n] * Para.Cost_cutwind  for n in range(Para.N_wind ))
    obj = obj + quicksum(Var[N_S_solar + n] * Para.Cost_solar    for n in range(Para.N_solar))
    obj = obj + quicksum(Var[N_C_solar + n] * Para.Cost_cutsolar for n in range(Para.N_solar))
    obj = obj * Para.N_day_year * Para.N_hour * 5
    
    Rec_rate = 0  # Reconvery rate in 5 years
    for y in range(Para.N_year_of_stage):
        Rec_rate = Rec_rate + (1 + Para.Int_rate) ** (-(y + 1))
    '''
    obj = obj + quicksum(Rec_rate * y_line [n] * Para.Line [n,7] * Para.Dep_line  for n in range(Para.N_line ))
    obj = obj + quicksum(Rec_rate * y_sub  [n] * Para.Sub  [n,3] * Para.Dep_sub   for n in range(Para.N_sub  ))
    obj = obj + quicksum(Rec_rate * y_wind [n] * Para.Wind [n,3] * Para.Dep_wind  for n in range(Para.N_wind ))
    obj = obj + quicksum(Rec_rate * y_solar[n] * Para.Solar[n,3] * Para.Dep_solar for n in range(Para.N_solar))
    '''
    cost_line = np.zeros(Para.N_line)
    for n in range(Para.N_line):
        cost_line[n] = Para.Line [n,7] * Para.Dep_line * Rec_rate
    
    model.setObjective(obj, GRB.MINIMIZE)

    N_con = []  # indexing constraints

    # 0.Active power balance equation
    for n in range(Para.N_bus):
        line_head = Info.Line_head[n]
        line_tail = Info.Line_tail[n]
        expr = LinExpr()
        expr = expr - quicksum(Var[N_P_line + i] for i in line_head)  # power out
        expr = expr + quicksum(Var[N_P_line + i] for i in line_tail)  # power in
        expr = expr + Var[N_C_load + n] * Para.Load_angle  # load shedding
        if  Info.Sub[n]   != []:
            expr = expr + Var[N_P_sub + Info.Sub[n][0]]
        if  Info.Wind[n]  != []:
            expr = expr + Var[N_S_wind + Info.Wind[n][0]] * Para.Wind_angle
        if  Info.Solar[n] != []:
            expr = expr + Var[N_S_solar + Info.Solar[n][0]] * Para.Solar_angle
        model.addConstr(expr == tp_load[n] * Para.Load_angle)
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 0

    # 1.Reactive power balance equation
    for n in range(Para.N_bus):
        line_head = Info.Line_head[n]
        line_tail = Info.Line_tail[n]
        expr = LinExpr()
        expr = expr - quicksum(Var[N_Q_line + i] for i in line_head)  # power out
        expr = expr + quicksum(Var[N_Q_line + i] for i in line_tail)  # power in
        expr = expr + Var[N_C_load + n] * math.sqrt(1-Para.Load_angle**2)  # load shedding
        if  Info.Sub[n]   != []:
            expr = expr + Var[N_Q_sub + Info.Sub[n][0]]
        if  Info.Wind[n]  != []:
            expr = expr + Var[N_S_wind + Info.Wind[n][0]] * math.sqrt(1 - Para.Wind_angle ** 2)
        if  Info.Solar[n] != []:
            expr = expr + Var[N_S_solar + Info.Solar[n][0]] * math.sqrt(1 - Para.Solar_angle ** 2)
        model.addConstr(expr == tp_load[n] * math.sqrt(1 - Para.Load_angle ** 2))
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 1

    # 2.Voltage balance on line
    for n in range(Para.N_line):
        bus_head = Para.Line[n,1]
        bus_tail = Para.Line[n,2]
        expr = LinExpr()
        expr = expr + Var[N_V_bus + bus_head] - Var[N_V_bus + bus_tail]  # voltage difference
        expr = expr - Var[N_P_line + n] * 2 * Para.Line_R[n]
        expr = expr - Var[N_Q_line + n] * 2 * Para.Line_X[n]
        model.addConstr(expr >= -Para.Big_M * (1 - y_line[n]))
        model.addConstr(expr <=  Para.Big_M * (1 - y_line[n]))
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 2

    # 3.Renewables(wind farm)
    for n in range(Para.N_wind):
        expr = Var[N_S_wind  + n] + Var[N_C_wind  + n]
        model.addConstr(expr == y_wind[n] * tp_wind [n])
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 3

    # 4.Renewables(solar station)
    for n in range(Para.N_solar):
        expr = Var[N_S_solar + n] + Var[N_C_solar + n]
        model.addConstr(expr == y_solar[n] * tp_solar[n])
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 4

    # 5.Linearization of quadratic terms
    for n in range(Para.N_line):
        expr = Var[N_P_line + n] + Var[N_Q_line + n]
        model.addConstr(expr >= -math.sqrt(2) * y_line[n] * Para.Line_S[n])
        model.addConstr(expr <=  math.sqrt(2) * y_line[n] * Para.Line_S[n])
        expr = Var[N_P_line + n] - Var[N_Q_line + n]
        model.addConstr(expr >= -math.sqrt(2) * y_line[n] * Para.Line_S[n])
        model.addConstr(expr <=  math.sqrt(2) * y_line[n] * Para.Line_S[n])
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 5

    # 6.Linearization of quadratic terms
    for n in range(Para.N_sub):
        expr = Var[N_P_sub + n] + Var[N_Q_sub + n]
        model.addConstr(expr >= 0)
        model.addConstr(expr <= math.sqrt(2) * y_sub[n] * Para.Sub_S[n])
        expr = Var[N_P_sub + n] - Var[N_Q_sub + n]
        model.addConstr(expr >= 0)
        model.addConstr(expr <= math.sqrt(2) * y_sub[n] * Para.Sub_S[n])
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 6

    # 7.bounds of variables
    # 1) Voltage
    for n in range(Para.N_bus):
        model.addConstr(Var[N_V_bus + n] >= Para.Voltage_low ** 2)
        model.addConstr(Var[N_V_bus + n] <= Para.Voltage_upp ** 2)
    # 2) Power flow
    for n in range(Para.N_line):
        model.addConstr(Var[N_P_line + n] >= -y_line[n] * Para.Line_S[n])
        model.addConstr(Var[N_P_line + n] <=  y_line[n] * Para.Line_S[n])
    for n in range(Para.N_line):
        model.addConstr(Var[N_Q_line + n] >= -y_line[n] * Para.Line_S[n])
        model.addConstr(Var[N_Q_line + n] <=  y_line[n] * Para.Line_S[n])
    # 3) Substation
    for n in range(Para.N_sub):
        model.addConstr(Var[N_P_sub + n] >= 0)
        model.addConstr(Var[N_P_sub + n] <= y_sub[n] * Para.Sub_S[n])
    for n in range(Para.N_sub):
        model.addConstr(Var[N_Q_sub + n] >= 0)
        model.addConstr(Var[N_Q_sub + n] <= y_sub[n] * Para.Sub_S[n])
    # 4) Load shedding
    for n in range(Para.N_bus):
        model.addConstr(Var[N_C_load + n] >= 0)
        model.addConstr(Var[N_C_load + n] <= tp_load[n])
    # 5) Renewables
    for n in range(Para.N_wind):
        model.addConstr(Var[N_S_wind + n] >= 0)
        model.addConstr(Var[N_S_wind + n] <= y_wind[n] * tp_wind[n])
        model.addConstr(Var[N_C_wind + n] >= 0)
        model.addConstr(Var[N_C_wind + n] <= y_wind[n] * tp_wind[n])
    for n in range(Para.N_solar):
        model.addConstr(Var[N_S_solar + n] >= 0)
        model.addConstr(Var[N_S_solar + n] <= y_solar[n] * tp_solar[n])
        model.addConstr(Var[N_C_solar + n] >= 0)
        model.addConstr(Var[N_C_solar + n] <= y_solar[n] * tp_solar[n])
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 7

    # 8.Fixed reconfiguration variables
    model.addConstrs(y_line [n] == Result_Reconfig.y_line [n] for n in range(Para.N_line ))
    model.addConstrs(y_sub  [n] == Result_Reconfig.y_sub  [n] for n in range(Para.N_sub  ))
    model.addConstrs(y_wind [n] == Result_Reconfig.y_wind [n] for n in range(Para.N_wind ))
    model.addConstrs(y_solar[n] == Result_Reconfig.y_solar[n] for n in range(Para.N_solar))
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))  # 8

    # Optimize
    model.optimize()
    if model.status == GRB.Status.OPTIMAL:
        result = ResultReconfigDual(model,Para,N_con,Result_Planning)
        
        # Modify the dual information
        for n in range(Para.N_line):
            if Result_Planning.x_line[n] == 1 and Result_Reconfig.y_line[n] == 0:
                result.y_dual[n] = cost_line[n]
        
    return result


# This function plots the planning solution in all stages
# The solution is given in three separated sub-plots
#
def PlotPlanning(Para,x_line):
    x = Para.Bus[:,2]
    y = Para.Bus[:,3]
    
    for n in range(Para.N_bus):  # Bus
        plt.text(x[n] + 3, y[n] + 3, '%s'%n)
        if n < Para.Sub[0,1]:
            plt.plot(x[n],y[n],'b.')
        else:
            plt.plot(x[n],y[n],'rs')
    for n in range(Para.N_line):  # Lines
        x1 = x[int(round(Para.Line[n,1]))]
        y1 = y[int(round(Para.Line[n,1]))]
        x2 = x[int(round(Para.Line[n,2]))]
        y2 = y[int(round(Para.Line[n,2]))]
        if x_line[n] == 1:
            plt.plot([x1,x2],[y1,y2],'r-')
        else:
            plt.plot([x1,x2],[y1,y2],'b--')
        plt.axis('equal')
    plt.show()


# This function plots the reconfiguration solution under a given scenario
# 
#
def PlotReconfiguration(Para,y_line):
    x = Para.Bus[:,2]
    y = Para.Bus[:,3]
    for n in range(Para.N_bus):  # Bus
        plt.text(x[n] + 3, y[n] + 3, '%s'%n)
        if n < Para.Sub[0,1]:
            plt.plot(x[n],y[n],'b.')
        else:
            plt.plot(x[n],y[n],'rs')
    for n in range(Para.N_line):  # Lines
        x1 = x[int(round(Para.Line[n,1]))]
        y1 = y[int(round(Para.Line[n,1]))]
        x2 = x[int(round(Para.Line[n,2]))]
        y2 = y[int(round(Para.Line[n,2]))]
        if y_line[n] == 1:
            plt.plot([x1,x2],[y1,y2],'r-')
        else:
            plt.plot([x1,x2],[y1,y2],'b--')
    plt.axis('equal')
    plt.show()


if __name__ == "__main__":

    time_start=time.time()
    # Input parameter
    filename = "../data/Data-IEEE-24-build.xlsx"
    [Para,Info] = ReadData(filename)
    
    # Logic-based Benders decomposition
    n_iter = 0  # index of iteration
    Logic = []  # Set of dual information
    lower_bound = []  # 
    upper_bound = []  #
    while True:
        Result_Planning = Planning(Para,Info,Logic,n_iter)
        obj_opr = 0
        for s in range(Para.N_scenario):
            Result_Reconfig = Reconfig(Para,Info,Result_Planning,s)
            Result_Dual = ReconfigDual(Para,Info,Result_Planning,Result_Reconfig,s)
            Logic.append(Result_Dual)
            obj_opr = obj_opr + Result_Reconfig.obj
        lower_bound.append(Result_Planning.obj)
        upper_bound.append(Result_Planning.obj_con + obj_opr)
        gap = (upper_bound[n_iter]-lower_bound[n_iter])/upper_bound[n_iter]
        if gap <= 1e-2 or n_iter > 250:
            break
        else:
            n_iter = n_iter + 1
    
    PlotPlanning(Para,Result_Planning.x_line)
    plt.plot(lower_bound)
    plt.plot(upper_bound)
    plt.show()
    
    time_end=time.time()
    print('totally cost',time_end-time_start)
