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
        self.N_stage = 3
        self.N_year_of_stage = 5
        self.N_day = 4  # number of typical day
        self.N_day_season = [90,91,92,92]  # number of days in each season
        self.N_scenario = 4  # number of reconfiguration in a day
        self.N_hour = int(24/self.N_scenario)  # number of hour in a scenario
        self.Int_rate = 0.05  # interest rate
        self.Big_M = 500  # Big M
        self.Voltage = 35
        self.Voltage_low = 35 * 0.9
        self.Voltage_upp = 35 * 1.1
        # Bus data
        Bus = Data_origin[0]
        self.Bus = Bus
        self.N_bus = len(Bus)
        self.Coordinate = Bus[:,2:4]  # coordinate
        self.Load = Bus[:,4:7]  # load demand
        self.Load_angle = 0.95  # phase angle of load
        self.Cost_cutload = 200  # cost of load shedding
        # Line data
        Line = Data_origin[1]
        Year_line = 25  # life-time of line
        self.Line = Line
        self.Line_ext = Line[np.where(Line[:,9] == 1)]  # existing line
        self.Line_new = Line[np.where(Line[:,9] == 0)]  # expandable line
        self.Line_R = Line[:,4]  # resistence
        self.Line_X = Line[:,5]  # reactance
        self.Line_S_ext = Line[:,6]  # existing capacity
        self.Line_S_new = Line[:,7]  # expandable capacity
        self.N_line = len(Line)
        self.N_line_ext = len(self.Line_ext)
        self.N_line_new = len(self.Line_new)
        self.Dep_line = Depreciation(Year_line,self.Int_rate)
        # Substation data
        Sub = Data_origin[2]
        Year_sub = 15  # life-time of substation
        self.Sub = Sub
        self.Sub_ext = Sub[np.where(Sub[:,5] == 1)]  # existing substation
        self.Sub_new = Sub[np.where(Sub[:,5] == 0)]  # expandable substation
        self.Sub_S_ext = Sub[:,2]  # existing capacity
        self.Sub_S_new = Sub[:,3]  # expandable capacity
        self.N_sub = len(self.Sub)
        self.N_sub_ext = len(self.Sub_ext)
        self.N_sub_new = len(self.Sub_new)
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
        self.Typical_load  = Data_origin[5][:,1:]  # load
        self.Typical_wind  = Data_origin[6][:,1:]  # wind
        self.Typical_solar = Data_origin[7][:,1:]  # solar


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
            Line_head[int(head)].append(i)  # set of lines whose head-end is bus i
            Line_tail[int(tail)].append(i)  # set of lines whose tail-end is bus i
        self.Line_head = Line_head
        self.Line_tail = Line_tail
        # Substation
        Sub = [[] for i in range(Para.N_bus)]
        for i in range(Para.N_sub):
            Sub[int(Para.Sub[i][1])].append(i)  # substation number of bus i
        self.Sub = Sub
        # Wind
        Wind = [[] for i in range(Para.N_bus)]
        for i in range(Para.N_wind):
            Wind[int(Para.Wind[i][1])].append(i)  # wind farm number of bus i
        self.Wind = Wind
        # Solar
        Solar = [[] for i in range(Para.N_bus)]
        for i in range(Para.N_solar):
            Solar[int(Para.Solar[i][1])].append(i)  # PV station number of bus i
        self.Solar = Solar


# This class restores the results of planning master problem
class ResultPlanning(object):
    def __init__(self,model,Para,x_line,x_sub,x_wind,x_solar):
        self.x_line  = [[int(x_line [i,j].x) for j in range(Para.N_stage)] for i in range(Para.N_line )]
        self.x_sub   = [[int(x_sub  [i,j].x) for j in range(Para.N_stage)] for i in range(Para.N_sub  )]
        self.x_wind  = [[int(x_wind [i,j].x) for j in range(Para.N_stage)] for i in range(Para.N_wind )]
        self.x_solar = [[int(x_solar[i,j].x) for j in range(Para.N_stage)] for i in range(Para.N_solar)]


# This class restores the results of reconfiguration sub-problem
class ResultReconfig(object):
    def __init__(self,model,Para,y_line,y_sub,y_wind,y_solar,Var):
        # Reconfiguration variable
        self.y_line  = [int(y_line [i].x) for i in range(Para.N_line )]
        self.y_sub   = [int(y_sub  [i].x) for i in range(Para.N_sub  )]
        self.y_wind  = [int(y_wind [i].x) for i in range(Para.N_wind )]
        self.y_solar = [int(y_solar[i].x) for i in range(Para.N_solar)]
        # Power flow variable
        self.V_bus   = [[Var[N_V_bus   + i,j].x for j in range(Para.N_hour)] for i in range(Para.N_bus  )]
        self.P_line  = [[Var[N_P_line  + i,j].x for j in range(Para.N_hour)] for i in range(Para.N_line )]
        self.Q_line  = [[Var[N_Q_line  + i,j].x for j in range(Para.N_hour)] for i in range(Para.N_line )]
        self.P_sub   = [[Var[N_P_sub   + i,j].x for j in range(Para.N_hour)] for i in range(Para.N_sub  )]
        self.Q_sub   = [[Var[N_Q_sub   + i,j].x for j in range(Para.N_hour)] for i in range(Para.N_sub  )]
        self.C_load  = [[Var[N_C_load  + i,j].x for j in range(Para.N_hour)] for i in range(Para.N_bus  )]
        self.S_wind  = [[Var[N_S_wind  + i,j].x for j in range(Para.N_hour)] for i in range(Para.N_wind )]
        self.C_wind  = [[Var[N_C_wind  + i,j].x for j in range(Para.N_hour)] for i in range(Para.N_wind )]
        self.S_solar = [[Var[N_S_solar + i,j].x for j in range(Para.N_hour)] for i in range(Para.N_solar)]
        self.C_solar = [[Var[N_C_solar + i,j].x for j in range(Para.N_hour)] for i in range(Para.N_solar)]


# This class restores the results of reconfiguration sub-problem
class ResultReconfigDual(object):
    def __init__(self,model,Para,N_cons):
        N_y_line  = N_cons
        N_y_sub   = N_y_line + Para.N_line
        N_y_wind  = N_y_sub  + Para.N_sub
        N_y_solar = N_y_wind + Para.N_wind
        constrs = model.getConstrs()
        self.dual_line  = [constrs[N_y_line  + i].pi for i in range(Para.N_line )]
        self.sual_sub   = [constrs[N_y_sub   + i].pi for i in range(Para.N_sub  )]
        self.dual_wind  = [constrs[N_y_wind  + i].pi for i in range(Para.N_wind )]
        self.dual_solar = [constrs[N_y_solar + i].pi for i in range(Para.N_solar)]


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
def Planning(Para,Info):
    #
    # minimize
    #       Investment costs of line, substation, wind farm and PV station
    # subject to
    #       1) Equipment cannot be removed once installed
    #       2) If substation is not built, the line connected to it cannot be built
    #       3) If load is zero, the line connected to the load bus cannot be built
    #       4) fictitious power flow variables are initialized throuth constraints
    #       5) The planning solution should remain connected
    #
    model = Model()
    # Create investment variables
    x_line  = model.addVars(Para.N_line,  Para.N_stage, vtype = GRB.BINARY)  # line
    x_sub   = model.addVars(Para.N_sub,   Para.N_stage, vtype = GRB.BINARY)  # Substation
    x_wind  = model.addVars(Para.N_wind,  Para.N_stage, vtype = GRB.BINARY)  # Wind farm
    x_solar = model.addVars(Para.N_solar, Para.N_stage, vtype = GRB.BINARY)  # PV station
    # Create fictitious variables
    f_line  = model.addVars(Para.N_line,  Para.N_stage, lb = -GRB.INFINITY)  # line flow
    f_load  = model.addVars(Para.N_bus,   Para.N_stage, lb = -GRB.INFINITY)  # load demand
    f_sub   = model.addVars(Para.N_sub,   Para.N_stage, lb = -GRB.INFINITY)  # power input

    # Set objective
    obj = LinExpr()
    for t in range(Para.N_stage):
        Rec_rate = 0  # Reconvery rate in 5 years
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
    # Constraint 3 (load bus)
    for t in range(Para.N_stage):
        for n in range(Para.N_bus):
            if Para.Load[n,t] == 0 and n < 20:
                line_head = Info.Line_head[n]
                line_tail = Info.Line_tail[n]
                model.addConstrs(x_line[i,t] == 0 for i in line_head)
                model.addConstrs(x_line[i,t] == 0 for i in line_tail)
    # Constraint 4 (fictitious power flow initialization)
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
    # Constraint 5 (connectivity)
    for t in range(Para.N_stage):
        for n in range(Para.N_bus):
            line_head = Info.Line_head[n]
            line_tail = Info.Line_tail[n]
            expr = quicksum(f_line[i,t] for i in line_head) - quicksum(f_line[i,t] for i in line_tail)
            if Info.Sub[n] == []:
                model.addConstr(expr + f_load[n,t] == 0)
            else:
                model.addConstr(expr + f_load[n,t] == f_sub[int(Info.Sub[n][0]),t])
    
    # Optimize
    model.optimize()
    if model.status == GRB.Status.OPTIMAL:
        result = ResultPlanning(model,Para,x_line,x_sub,x_wind,x_solar)
    return result


# This function creates the reconfiguration sub-problem (MILP). After the problem is solved,
# the binary reconfiguration variables are fixed and dual variables are returned for further
# analysis and operation. Note that the problem is formulated under a given scenario 's' of 
# typical day 'd' at stage 't'.
#
def Reconfiguration(Para,Info,Result_Planning,t,d,s):
    #
    # minimize
    #       Costs of power purchasing, load shedding, renewables generation and curtailment 
    # subject to
    #       1) Reconfiguration
    #       2) Optimal Power flow
    #       3) Upper and lower bound
    #
    model = Model()
    global N_V_bus,  N_P_line, N_Q_line, N_P_sub, N_Q_sub
    global N_C_load, N_S_wind, N_C_wind, N_S_solar, N_C_solar
    # Typical data
    hour_0   = int(Para.N_hour*s)  # start hour
    hour_1   = int(Para.N_hour*(s+1))  # end hour
    tp_load  = np.outer(Para.Load [:,t], Para.Typical_load [hour_0:hour_1,d])  # load
    tp_wind  = np.outer(Para.Wind [:,2], Para.Typical_wind [hour_0:hour_1,d])  # wind
    tp_solar = np.outer(Para.Solar[:,2], Para.Typical_solar[hour_0:hour_1,d])  # solar
    # Capacity data
    Cap_line = [Para.Line_S_ext[n] + Result_Planning.x_line[n][t] * Para.Line_S_new[n] for n in range(Para.N_line)]
    Cap_sub  = [Para.Sub_S_ext [n] + Result_Planning.x_sub [n][t] * Para.Sub_S_new [n] for n in range(Para.N_sub )]

    # Create reconfiguration variables
    y_line  = model.addVars(Para.N_line, vtype = GRB.BINARY)  # line
    y_sub   = model.addVars(Para.N_sub,  vtype = GRB.BINARY)  # Substation
    y_wind  = model.addVars(Para.N_wind, vtype = GRB.BINARY)  # Wind farm
    y_solar = model.addVars(Para.N_solar,vtype = GRB.BINARY)  # PV station
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
    Var = model.addVars(N_Var, Para.N_hour, lb = -GRB.INFINITY)
    
    # Set objective
    obj = LinExpr()
    for h in range(Para.N_hour):
        obj = obj + quicksum(Var[N_P_sub   + n, h] * Para.Cost_power    for n in range(Para.N_sub  ))
        obj = obj + quicksum(Var[N_C_load  + n, h] * Para.Cost_cutload  for n in range(Para.N_bus  ))
        obj = obj + quicksum(Var[N_S_wind  + n, h] * Para.Cost_wind     for n in range(Para.N_wind ))
        obj = obj + quicksum(Var[N_C_wind  + n, h] * Para.Cost_cutwind  for n in range(Para.N_wind ))
        obj = obj + quicksum(Var[N_S_solar + n, h] * Para.Cost_solar    for n in range(Para.N_solar))
        obj = obj + quicksum(Var[N_C_solar + n, h] * Para.Cost_cutsolar for n in range(Para.N_solar))
    obj = obj * Para.N_day_season[d]
    model.setObjective(obj, GRB.MINIMIZE)

    # Constraint 1 (Reconfiguration)
    N_load = np.size(np.nonzero(Para.Load[:,t]),1)
    model.addConstr(quicksum(y_line[n] for n in range(Para.N_line)) == N_load)  # radiality
    for n in range(Para.N_line):
        if n < Para.N_line_ext:
            model.addConstr(y_line[n] >= 0)
            model.addConstr(y_line[n] <= 1)
        else:
            model.addConstr(y_line[n] >= 0)
            model.addConstr(y_line[n] <= Result_Planning.x_line[n][t])
    for n in range(Para.N_sub):
        if n < Para.N_sub_ext:
            model.addConstr(y_sub[n] >= 0)
            model.addConstr(y_sub[n] <= 1)
        else:
            model.addConstr(y_sub[n] >= 0)
            model.addConstr(y_sub[n] <= Result_Planning.x_sub[n][t])
    for n in range(Para.N_wind):
        model.addConstr(y_wind [n] >= 0)
        model.addConstr(y_wind [n] <= Result_Planning.x_wind [n][t])
    for n in range(Para.N_wind):
        model.addConstr(y_solar[n] >= 0)
        model.addConstr(y_solar[n] <= Result_Planning.x_solar[n][t])

    # Constraint 2 (optimal power flow)
    for h in range(Para.N_hour):
        # 1.Active power balance equation
        for n in range(Para.N_bus):
            line_head = Info.Line_head[n]
            line_tail = Info.Line_tail[n]
            expr = LinExpr()
            expr = expr - quicksum(Var[N_P_line + i, h] for i in line_head)  # power out
            expr = expr + quicksum(Var[N_P_line + i, h] for i in line_tail)  # power in
            expr = expr + Var[N_C_load + n, h] * Para.Load_angle  # load shedding
            if  Info.Sub[n]   != []:
                expr = expr + Var[N_P_sub + Info.Sub[n][0], h]
            if  Info.Wind[n]  != []:
                expr = expr + Var[N_S_wind + Info.Wind[n][0], h] * Para.Wind_angle
            if  Info.Solar[n] != []:
                expr = expr + Var[N_S_solar + Info.Solar[n][0], h] * Para.Solar_angle
            model.addConstr(expr == tp_load[n,h] * Para.Load_angle)
        # 2.Reactive power balance equation
        for n in range(Para.N_bus):
            line_head = Info.Line_head[n]
            line_tail = Info.Line_tail[n]
            expr = LinExpr()
            expr = expr - quicksum(Var[N_Q_line + i, h] for i in line_head)  # power out
            expr = expr + quicksum(Var[N_Q_line + i, h] for i in line_tail)  # power in
            expr = expr + Var[N_C_load + n, h] * math.sqrt(1-Para.Load_angle**2)  # load shedding
            if  Info.Sub[n]   != []:
                expr = expr + Var[N_Q_sub + Info.Sub[n][0], h]
            if  Info.Wind[n]  != []:
                expr = expr + Var[N_S_wind + Info.Wind[n][0], h] * math.sqrt(1 - Para.Wind_angle ** 2)
            if  Info.Solar[n] != []:
                expr = expr + Var[N_S_solar + Info.Solar[n][0], h] * math.sqrt(1 - Para.Solar_angle ** 2)
            model.addConstr(expr == tp_load[n,h] * math.sqrt(1 - Para.Load_angle ** 2))
        # 3.Voltage balance on line
        for n in range(Para.N_line):
            bus_head = Para.Line[n,1]
            bus_tail = Para.Line[n,2]
            expr = LinExpr()
            expr = expr + Var[N_V_bus + bus_head, h] - Var[N_V_bus + bus_tail, h]  # voltage difference
            expr = expr - Var[N_P_line + n, h] * 2 * Para.Line_R[n]
            expr = expr - Var[N_Q_line + n, h] * 2 * Para.Line_X[n]
            model.addConstr(expr >= -Para.Big_M * (1 - y_line[n]))
            model.addConstr(expr <=  Para.Big_M * (1 - y_line[n]))
        # 4.Renewables
        for n in range(Para.N_wind):
            expr = Var[N_S_wind  + n, h] + Var[N_C_wind  + n, h]
            model.addConstr(expr == y_wind [n] * tp_wind [n,h])
        for n in range(Para.N_solar):
            expr = Var[N_S_solar + n, h] + Var[N_C_solar + n, h]
            model.addConstr(expr == y_solar[n] * tp_solar[n,h])
        # 5.Linearization of quadratic terms
        for n in range(Para.N_line):
            expr = Var[N_P_line + n, h] + Var[N_Q_line + n, h]
            model.addConstr(expr >= -math.sqrt(2) * y_line[n] * Cap_line[n])
            model.addConstr(expr <=  math.sqrt(2) * y_line[n] * Cap_line[n])
            expr = Var[N_P_line + n, h] - Var[N_Q_line + n, h]
            model.addConstr(expr >= -math.sqrt(2) * y_line[n] * Cap_line[n])
            model.addConstr(expr <=  math.sqrt(2) * y_line[n] * Cap_line[n])
        for n in range(Para.N_sub):
            expr = Var[N_P_sub + n, h] + Var[N_Q_sub + n, h]
            model.addConstr(expr >= -math.sqrt(2) * y_sub[n] * Cap_sub[n])
            model.addConstr(expr <=  math.sqrt(2) * y_sub[n] * Cap_sub[n])
            expr = Var[N_P_sub + n, h] - Var[N_Q_sub + n, h]
            model.addConstr(expr >= -math.sqrt(2) * y_sub[n] * Cap_sub[n])
            model.addConstr(expr <=  math.sqrt(2) * y_sub[n] * Cap_sub[n])
    
    # Constraint 3 (bounds of variables)
    for h in range(Para.N_hour):
        # 1.Voltage
        for n in range(Para.N_bus):
            model.addConstr(Var[N_V_bus + n, h] >= Para.Voltage_low ** 2)
            model.addConstr(Var[N_V_bus + n, h] <= Para.Voltage_upp ** 2)
        # 2.Power flow
        for n in range(Para.N_line):
            model.addConstr(Var[N_P_line + n, h] >= -y_line[n] * Cap_line[n])
            model.addConstr(Var[N_P_line + n, h] <=  y_line[n] * Cap_line[n])
            model.addConstr(Var[N_Q_line + n, h] >= -y_line[n] * Cap_line[n])
            model.addConstr(Var[N_Q_line + n, h] <=  y_line[n] * Cap_line[n])
        # 3.Substation
        for n in range(Para.N_sub):
            model.addConstr(Var[N_P_sub + n, h] >= 0)
            model.addConstr(Var[N_P_sub + n, h] <= y_sub[n] * Cap_sub[n])
            model.addConstr(Var[N_Q_sub + n, h] >= 0)
            model.addConstr(Var[N_Q_sub + n, h] <= y_sub[n] * Cap_sub[n])
        # 4.Load shedding
        for n in range(Para.N_bus):
            model.addConstr(Var[N_C_load + n, h] >= 0)
            model.addConstr(Var[N_C_load + n, h] <= tp_load[n,h])
        # 5.Renewables
        for n in range(Para.N_wind):
            model.addConstr(Var[N_S_wind + n, h] >= 0)
            model.addConstr(Var[N_S_wind + n, h] <= tp_wind[n,h])
            model.addConstr(Var[N_C_wind + n, h] >= 0)
            model.addConstr(Var[N_C_wind + n, h] <= tp_wind[n,h])
        for n in range(Para.N_solar):
            model.addConstr(Var[N_S_solar + n, h] >= 0)
            model.addConstr(Var[N_S_solar + n, h] <= tp_solar[n,h])
            model.addConstr(Var[N_C_solar + n, h] >= 0)
            model.addConstr(Var[N_C_solar + n, h] <= tp_solar[n,h])

    # Optimize
    model.optimize()
    if model.status == GRB.Status.OPTIMAL:
        result = ResultReconfig(model,Para,y_line,y_sub,y_wind,y_solar,Var)
        N_cons = model.getAttr(GRB.Attr.NumConstrs)
        # Fix binary reconfiguration variables
        model.addConstrs(y_line [n] == result.y_line [n] for n in range(Para.N_line ))
        model.addConstrs(y_sub  [n] == result.y_sub  [n] for n in range(Para.N_sub  ))
        model.addConstrs(y_wind [n] == result.y_wind [n] for n in range(Para.N_wind ))
        model.addConstrs(y_solar[n] == result.y_solar[n] for n in range(Para.N_solar))
        model.update()  # update all changes
        # Relax
        model = model.relax()  # relax binary variables to continuous variables
        model.optimize()
        if model.status == GRB.Status.OPTIMAL:
            result_dual = ResultReconfigDual(model,Para,N_cons)
    return result, result_dual


# This function plots the planning solution
#
def PlotSolution(Para,Result):
    x = Para.Bus[:,2]
    y = Para.Bus[:,3]
    for t in range(Para.N_stage):
        plt.subplot(1, Para.N_stage, t + 1)
        for n in range(Para.N_bus):  # Bus
            plt.text(x[n] + 3, y[n] + 3, '%s'%n)
            if n < Para.Sub[0,1]:
                plt.plot(x[n],y[n],'b.')
            else:
                plt.plot(x[n],y[n],'rs')
        for n in range(Para.N_line):  # Lines
            x1 = x[int(Para.Line[n,1])]
            y1 = y[int(Para.Line[n,1])]
            x2 = x[int(Para.Line[n,2])]
            y2 = y[int(Para.Line[n,2])]
            if Result[n][t] == 1 or n < Para.N_line_ext:
                plt.plot([x1,x2],[y1,y2],'r-')
            else:
                plt.plot([x1,x2],[y1,y2],'b--')
        plt.axis('equal')
    plt.show()


if __name__ == "__main__":

    time_start=time.time()
    # Input parameter
    filename = "../data/Data-IEEE-24.xlsx"
    [Para,Info] = ReadData(filename)
    
    # Logic-based Benders decomposition
    
    Result_Planning = Planning(Para,Info)

    # PlotSolution(Para,Result_Planning.x_line)

    for t in range(Para.N_stage):
        for d in range(Para.N_day):
            for s in range(Para.N_scenario):
                [Result_Reconfig,Result_Reconfig_Dual] = Reconfiguration(Para,Info,Result_Planning,2,2,2)
    time_end=time.time()
    print('totally cost',time_end-time_start)
