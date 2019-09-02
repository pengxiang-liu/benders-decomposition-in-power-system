#!/usr/bin/python
#
# Copyright 2019, Southeast University, Liu Pengxiang
#
# A distribution system planning model using Modern Benders decomposition
# 
# This project develops a multi-stage planning model for AC-DC distribution 
# system in Zhejiang province, China.


import sys
import csv
import math
import xlrd
import time
import numpy as np
import matplotlib.pyplot as plt

from gurobipy import *


# This class builds the system parameter
# Note: AC => 0, DC => 1, AC/DC => 2
#
class Parameter(object):
    def __init__(self,Data):
        # System
        self.N_stage  = 3   # number of stage
        self.N_year   = 5   # number of year in each stage
        self.N_scene  = 16  # number of reconfiguration scenario
        self.N_hour   = 6   # number of hour in each scenario
        self.N_time   = 90  # number of times
        self.Int_rate = 0.05  # interest rate
        self.Big_M = 245  # Big M
        self.Factor = [0.95,math.sqrt(1-0.95**2)]  # power factor
        self.Voltage = 35
        self.Voltage_low = 35 * 0.95
        self.Voltage_upp = 35 * 1.05
        # Cost
        self.Cost_load    = 70   # cost of load purchasing
        self.Cost_cutload = 200  # cost of load curtailment
        self.Cost_gen     = 10   # cost of renewable generation
        self.Cost_cutgen  = 200  # cost of renewable curtailment
        # Bus
        self.Bus  = Data[0]
        self.Bus_AC = self.Bus[np.where(self.Bus[:,7] == 0)]
        self.Bus_DC = self.Bus[np.where(self.Bus[:,7] == 1)]
        self.N_bus = len(self.Bus)
        self.N_bus_AC = len(self.Bus_AC)
        self.N_bus_DC = len(self.Bus_DC)
        self.Load = self.Bus[:,4:7]
        # Line
        self.Line = Data[1]
        self.Line_AC = self.Line[np.where(self.Line[:,9] == 0)]  # AC
        self.Line_DC = self.Line[np.where(self.Line[:,9] == 1)]  # DC
        self.N_line = len(self.Line)
        self.N_line_AC = len(self.Line_AC)
        self.N_line_DC = len(self.Line_DC)
        self.Line_R = self.Line[:,4]
        self.Line_X = self.Line[:,5]
        self.Line_S = self.Line[:,6:8]
        self.Line_S_max = (self.Line_S).sum(axis = 1)
        self.Dep_line = Depreciation(25,self.Int_rate)
        # Converter station
        self.Conv = Data[2]
        self.N_conv = len(self.Conv)
        self.Dep_conv = Depreciation(20,self.Int_rate)
        # Substation
        self.Sub = Data[3]
        self.N_sub = len(self.Sub)
        self.Sub_S = self.Sub[:,2:4]
        self.Dep_sub  = Depreciation(15,self.Int_rate)
        # Renewables generation
        self.Gen = Data[4]
        self.N_gen = len(self.Gen)
        self.Dep_gen  = Depreciation(15,self.Int_rate)
        # Typical day
        self.Typical_Day = Data[5]
        self.Ty_load = self.Typical_Day[:,1]
        self.Ty_gen  = self.Typical_Day[:,2:5]


# This class builds the infomation for each bus i, including the set of line
# and converter station with a head or tail end of bus i
# 
class BusInfo(object):
    def __init__(self,Para):
        # Set of lines whose head-end/tail-end is bus i
        Line_head = [[] for i in range(Para.N_bus)]
        Line_tail = [[] for i in range(Para.N_bus)]
        for i in range(Para.N_line):
            head = Para.Line[i][1]
            tail = Para.Line[i][2]
            Line_head[int(round(head))].append(i)
            Line_tail[int(round(tail))].append(i)
        self.Line_head = Line_head
        self.Line_tail = Line_tail
        # Set of converter station whose head-end/tail-end is bus i
        Conv_head = [[] for i in range(Para.N_bus)]
        Conv_tail = [[] for i in range(Para.N_bus)]
        for i in range(Para.N_conv):
            head = Para.Conv[i][1]
            tail = Para.Conv[i][2]
            Conv_head[int(round(head))].append(i)
            Conv_tail[int(round(tail))].append(i)
        self.Conv_head = Conv_head
        self.Conv_tail = Conv_tail


# This class restores the results of planning master problem
class ResultMasterMILP(object):
    def __init__(self,model,Para,var):
        variable = var.copy()
        x_line = np.zeros((Para.N_line, Para.N_stage))
        x_conv = np.zeros((Para.N_conv, Para.N_stage))
        x_sub  = np.zeros((Para.N_sub , Para.N_stage))
        x_gen  = np.zeros((Para.N_gen , Para.N_stage))
        y_line = np.zeros((Para.N_line, Para.N_scene, Para.N_stage))
        for n in range(Para.N_line):
            for t in range(Para.N_stage):
                x_line[n,t] = int(round(variable.pop(0)))
        for n in range(Para.N_conv):
            for t in range(Para.N_stage):
                x_conv[n,t] = int(round(variable.pop(0)))
        for n in range(Para.N_sub ):
            for t in range(Para.N_stage):
                x_sub [n,t] = int(round(variable.pop(0)))
        for n in range(Para.N_gen ):
            for t in range(Para.N_stage):
                x_gen [n,t] = int(round(variable.pop(0)))
        for n in range(Para.N_line):
            for s in range(Para.N_scene):
                for t in range(Para.N_stage):
                    y_line[n,s,t] = int(round(variable.pop(0)))
        self.x_line = x_line
        self.x_conv = x_conv
        self.x_sub  = x_sub
        self.x_gen  = x_gen
        self.y_line = y_line
        self.obj_con = variable[-2]
        self.obj_opr = variable[-1]


# This class restores the results of reconfiguration worker-problem
class ResultWorkerLP(object):
    def __init__(self,model,Para,N_con):
        # Get all variables
        var = model.getVars()
        var = np.array([var[i].x for i in range(len(var))])
        # 
        self.obj = (model.getObjective()).getValue()
        # Saving reconfiguration variables
        self.x_line = var[N_X_line : N_X_line + Para.N_line]
        self.x_conv = var[N_X_conv : N_X_conv + Para.N_conv]
        self.x_sub  = var[N_X_sub  : N_X_sub  + Para.N_sub ]
        self.x_gen  = var[N_X_gen  : N_X_gen  + Para.N_gen ]
        self.y_line = var[N_Y_line : N_Y_line + Para.N_line]
        # Saving operating variables
        opr = var[N_Index:].reshape((N_Var,Para.N_hour), order = 'A')
        self.V_bus  = opr[N_V_bus  : N_V_bus  + Para.N_bus , :]
        self.P_line = opr[N_P_line : N_P_line + Para.N_line, :]
        self.Q_line = opr[N_Q_line : N_Q_line + Para.N_line, :]
        self.P_conv = opr[N_P_conv : N_P_conv + Para.N_conv, :]
        self.Q_conv = opr[N_Q_conv : N_Q_conv + Para.N_conv, :]
        self.P_sub  = opr[N_P_sub  : N_P_sub  + Para.N_sub , :]
        self.Q_sub  = opr[N_Q_sub  : N_Q_sub  + Para.N_sub , :]
        self.C_load = opr[N_C_load : N_C_load + Para.N_bus , :]
        self.S_gen  = opr[N_S_gen  : N_S_gen  + Para.N_gen , :]
        self.C_gen  = opr[N_C_gen  : N_C_gen  + Para.N_gen , :]
        # Saving dual information
        constr = model.getConstrs()
        dual = [constr[n].pi  for n in range(N_con[-2],N_con[-1])]
        self.d_x_line = np.array([dual.pop(0) for n in range(Para.N_line)])
        self.d_x_conv = np.array([dual.pop(0) for n in range(Para.N_conv)])
        self.d_x_sub  = np.array([dual.pop(0) for n in range(Para.N_sub )])
        self.d_x_gen  = np.array([dual.pop(0) for n in range(Para.N_gen )])
        self.d_y_line = np.array([dual.pop(0) for n in range(Para.N_line)])
        

# This class formulates the traditional and logic Benders cut
class BendersInfo(object):
    def __init__(self,Para,Result_Planning):
        # Given planning and reconfiguration
        self.x_line = Result_Planning.x_line
        self.x_conv = Result_Planning.x_conv
        self.x_sub  = Result_Planning.x_sub
        self.x_gen  = Result_Planning.x_gen
        self.y_line = Result_Planning.y_line
        # Dual information
        self.d_x_line = np.zeros((Para.N_line, Para.N_stage))
        self.d_x_conv = np.zeros((Para.N_conv, Para.N_stage))
        self.d_x_sub  = np.zeros((Para.N_sub , Para.N_stage))
        self.d_x_gen  = np.zeros((Para.N_gen , Para.N_stage))
        self.d_y_line = np.zeros((Para.N_line, Para.N_scene, Para.N_stage))
        # Objective
        self.obj = 0


# This function input data from Excel files. The filtname can be changed 
# to other power system for further study
#
def ReadData(filename):
    Data_origin = []
    readbook = xlrd.open_workbook(filename)
    # Data preprocessing
    for i in range(6):  # sheet number
        sheet = readbook.sheet_by_index(i)
        n_row = sheet.nrows
        n_col = sheet.ncols
        Coordinate = [1,n_row,0,n_col]  # coordinate of slice
        Data_temp = sheet._cell_values  # data in the Excel file
        Data_origin.append(np.array(Matrix_slice(Data_temp,Coordinate)))
    return Data_origin


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
def Depreciation(life,rate):
    recovery = rate*((1+rate)**life)/((1+rate)**life-1)
    return recovery


# This class restores the 'plot' function
class PlotFunc(object):
    def __init__(self,Para):
        pass
    def Planning(self,Para,Res,t):
        x_line = Res.x_line[:,t]
        x_conv = Res.x_conv[:,t]
        for n in range(Para.N_line):
            if Para.Line[n,6] > 0:
                x_line[n] = 1
        self.Plot_Figure(Para,x_line,x_conv)
    def Reconfig(self,Para,Res,s,t):
        y_line = Res.y_line[:,s,t]
        x_conv = Res.x_conv[:,t]
        self.Plot_Figure(Para,y_line,x_conv)
    def Plot_Figure(self,Para,Line,Conv):
        x = (Para.Bus[:,2]).copy()
        y = (Para.Bus[:,3]).copy()
        for n in range(Para.N_bus):  # Bus
            if Para.Bus[n,7] == 1:
                y[n] = y[n] - 150
        for n in range(Para.N_bus):  # Bus
            plt.text(x[n] + 3, y[n] + 3, '%s'%n)
            if n in Para.Sub[:,1]:
                plt.plot(x[n],y[n],'rs')
            else:
                plt.plot(x[n],y[n],'b.')
        for n in range(Para.N_line):  # lines
            x1 = x[int(round(Para.Line[n,1]))]
            y1 = y[int(round(Para.Line[n,1]))]
            x2 = x[int(round(Para.Line[n,2]))]
            y2 = y[int(round(Para.Line[n,2]))]
            if Line[n] == 1:
                if Para.Line[n,9] == 0:
                    plt.plot([x1,x2],[y1,y2],'r-')
                if Para.Line[n,9] == 1:
                    plt.plot([x1,x2],[y1,y2],'b-')
            else:
                plt.plot([x1,x2],[y1,y2],'b--')
        for n in range(Para.N_conv):  # converters
            if Conv[n] == 1:
                head = int(round(Para.Conv[n,1]))
                tail = int(round(Para.Conv[n,2]))
                plt.plot(x[head],y[head],'bs')
                plt.plot(x[tail],y[tail],'bs')
        plt.axis('equal')
        plt.show()


# This function...
def Indexing(Para):
    # Master varibales
    global N_X_line, N_X_conv, N_X_sub, N_X_gen, N_Y_line, N_Index
    N_X_line = 0
    N_X_conv = N_X_line + Para.N_line
    N_X_sub  = N_X_conv + Para.N_conv
    N_X_gen  = N_X_sub  + Para.N_sub
    N_Y_line = N_X_gen  + Para.N_gen
    N_Index  = N_Y_line + Para.N_line
    # Operating variables
    global N_V_bus, N_P_line, N_Q_line, N_P_conv, N_Q_conv
    global N_P_sub, N_Q_sub,  N_C_load, N_S_gen,  N_C_gen,  N_Var
    N_V_bus  = 0  # Square of bus voltage
    N_P_line = N_V_bus  + Para.N_bus   # active   power flow
    N_Q_line = N_P_line + Para.N_line  # reactive power flow
    N_P_conv = N_Q_line + Para.N_line  # active   power flow
    N_Q_conv = N_P_conv + Para.N_conv  # reactive power compensation
    N_P_sub  = N_Q_conv + Para.N_conv  # power injection at substation
    N_Q_sub  = N_P_sub  + Para.N_sub   # power injection at substation
    N_C_load = N_Q_sub  + Para.N_sub   # Load shedding
    N_S_gen  = N_C_load + Para.N_bus   # renewables generation
    N_C_gen  = N_S_gen  + Para.N_gen   # renewables curtailment
    N_Var    = N_C_gen  + Para.N_gen   # Number of all variables


def createMasterMILP(Para,Info):
    #
    # minimize
    #       Investment costs of line, converter, substation and
    #       renewables generation
    # subject to
    #       1) ...
    #       2) ...
    #
    model = Model()

    # Investment variables
    x_line = model.addVars(Para.N_line, Para.N_stage, vtype = GRB.BINARY)
    x_conv = model.addVars(Para.N_conv, Para.N_stage, vtype = GRB.BINARY)
    x_sub  = model.addVars(Para.N_sub,  Para.N_stage, vtype = GRB.BINARY)
    x_gen  = model.addVars(Para.N_gen,  Para.N_stage, vtype = GRB.BINARY)
    # Reconfiguration variables
    y_line = model.addVars(Para.N_line, Para.N_scene, Para.N_stage, vtype = GRB.BINARY)
    y_pos  = model.addVars(Para.N_line, Para.N_scene, Para.N_stage, vtype = GRB.BINARY)
    y_neg  = model.addVars(Para.N_line, Para.N_scene, Para.N_stage, vtype = GRB.BINARY)
    # Fictitious power flow variables
    f_line = model.addVars(Para.N_line, Para.N_scene, Para.N_stage, lb = -1e2)
    f_conv = model.addVars(Para.N_conv, Para.N_scene, Para.N_stage, lb = -1e2)
    f_load = model.addVars(Para.N_bus,  Para.N_scene, Para.N_stage, lb = -1e2)
    f_gen  = model.addVars(Para.N_gen,  Para.N_scene, Para.N_stage, lb = -1e2)
    f_sub  = model.addVars(Para.N_sub,  Para.N_scene, Para.N_stage, lb = -1e2)
    # Projected operating costs
    obj_con = model.addVar()
    obj_opr = model.addVar()

    model.update()

    # Set objective
    cost_con = LinExpr()
    for t in range(Para.N_stage):
        RR = 0  # Reconvery rate in 5 years
        for y in range(Para.N_year):
            RR = RR + (1 + Para.Int_rate) ** (-(t * Para.N_year + y + 1))
        for n in range(Para.N_line):  # line
            cost_con = cost_con + RR * x_line[n,t] * Para.Line[n][8] * Para.Dep_line
        for n in range(Para.N_conv):  # converter
            cost_con = cost_con + RR * x_conv[n,t] * Para.Conv[n][4] * Para.Dep_conv
        for n in range(Para.N_sub):  # substation
            cost_con = cost_con + RR * x_sub [n,t] * Para.Sub [n][4] * Para.Dep_sub
        for n in range(Para.N_gen):  # renewables generation
            cost_con = cost_con + RR * x_gen [n,t] * Para.Gen [n][3] * Para.Dep_gen
    model.addConstr(obj_con == cost_con)

    # Constraint 1 (installation)
    for t in range(Para.N_stage-1):
        model.addConstrs(x_line[n,t] <= x_line[n,t+1] for n in range(Para.N_line))
        model.addConstrs(x_conv[n,t] <= x_conv[n,t+1] for n in range(Para.N_conv))
        model.addConstrs(x_sub [n,t] <= x_sub [n,t+1] for n in range(Para.N_sub ))
        model.addConstrs(x_gen [n,t] <= x_gen [n,t+1] for n in range(Para.N_gen ))
    
    # Constraint 2 (reconfiguration)
    for t in range(Para.N_stage):
        for s in range(Para.N_scene):
            for n in range(Para.N_line):
                if Para.Line[n,6] > 0:  # existing line
                    model.addConstr(y_line[n,s,t] <= 1)
                else:  # expandable line
                    if Para.Line[n,9] == 0:  # AC line
                        model.addConstr(y_line[n,s,t] <= x_line[n,t])
                    if Para.Line[n,9] == 1:  # DC line
                        model.addConstr(y_line[n,s,t] == x_line[n,t])
    
    # Constraint 3 (fictitious power flow initialization)
    for t in range(Para.N_stage):
        for s in range(Para.N_scene):
            # Select a scenario
            Data_gen  = np.zeros(Para.N_gen)
            Data_load = np.zeros(Para.N_bus)
            index = s * Para.N_hour + 1
            for n in range(Para.N_bus):
                Data_load[n] = Para.Load[n,t] * Para.Ty_load[index]
            for n in range(Para.N_gen):
                tp = int(Para.Gen[n,6])  # type of renewables
                Data_gen [n] = Para.Gen [n,2] * Para.Ty_gen [index,tp]
            # Initialize fictitious power flow
            for n in range(Para.N_line):
                expr_0 = Para.Line_S[n,0] + x_line[n,t] * Para.Line_S[n,1]
                expr_1 = y_line[n,s,t] * Para.Line_S_max[n]
                model.addConstr(f_line[n,s,t] >= -expr_0)
                model.addConstr(f_line[n,s,t] <=  expr_0)
                model.addConstr(f_line[n,s,t] >= -expr_1)
                model.addConstr(f_line[n,s,t] <=  expr_1)
            for n in range(Para.N_conv):
                expr_0 = x_conv[n,t] * Para.Conv[n,3]
                model.addConstr(f_conv[n,s,t] >= -expr_0)
                model.addConstr(f_conv[n,s,t] <=  expr_0)
            for n in range(Para.N_sub):
                expr_0 = Para.Sub_S[n,0] + x_sub[n,t] * Para.Sub_S[n,1]
                model.addConstr(f_sub [n,s,t] >=  0)
                model.addConstr(f_sub [n,s,t] <=  expr_0)
            for n in range(Para.N_gen):
                model.addConstr(f_gen [n,s,t] == x_gen[n,t] * Data_gen[n])
            for n in range(Para.N_bus):
                model.addConstr(f_load[n,s,t] == Data_load[n])
    
    # Constraint 4 (connectivity)
    for t in range(Para.N_stage):
        for s in range(Para.N_scene):
            for n in range(Para.N_bus):
                line_head = Info.Line_head[n]
                line_tail = Info.Line_tail[n]
                conv_head = Info.Conv_head[n]
                conv_tail = Info.Conv_tail[n]
                expr = LinExpr()
                expr = expr - f_load[n,s,t]
                expr = expr - quicksum(f_line[i,s,t] for i in line_head)
                expr = expr + quicksum(f_line[i,s,t] for i in line_tail)
                expr = expr - quicksum(f_conv[i,s,t] for i in conv_head)
                expr = expr + quicksum(f_conv[i,s,t] for i in conv_tail)
                if n in Para.Sub[:,1]:
                    bus_no = int(np.where(n == Para.Sub[:,1])[0])
                    expr = expr + f_sub[bus_no,s,t]
                if n in Para.Gen[:,1]:
                    bus_no = int(np.where(n == Para.Gen[:,1])[0])
                    expr = expr + f_gen[bus_no,s,t]
                model.addConstr(expr == 0)
    
    # Constraint 5 (radial topology)
    for t in range(Para.N_stage):
        for s in range(Para.N_scene):
            for n in range(Para.N_bus_AC):
                line_head = Info.Line_head[n]
                line_tail = Info.Line_tail[n]
                expr = LinExpr()
                expr = expr + quicksum(y_pos[i,s,t] for i in line_tail)
                expr = expr + quicksum(y_neg[i,s,t] for i in line_head)
                if Para.Load[n,t] > 0:  # load bus
                    model.addConstr(expr == 1)
                else:  # none load bus
                    model.addConstr(expr == 0)
            for n in range(Para.N_line):
                model.addConstr(y_pos[n,s,t] + y_neg[n,s,t] == y_line[n,s,t])
    
    # Constraint 6 (given condition)
    for t in range(Para.N_stage):
        '''
        for n in range(Para.N_line):
            model.addConstr(x_line[n,t] == 1)
        
        for n in range(Para.N_conv):
            model.addConstr(x_conv[n,t] == 1)
        
        for n in range(Para.N_sub):
            model.addConstr(x_sub [n,t] == 1)
        '''
        for n in range(Para.N_gen):
            if t >= Para.Gen[n,5]:
                model.addConstr(x_gen[n,t] == 1)
            else:
                model.addConstr(x_gen[n,t] == 0)
    
    # Set objective
    model.setObjective(obj_con + obj_opr, GRB.MINIMIZE)
    model.update()
    return model
   

# This function creates the reconfiguration worker problem. Dual variables 
# are returned for generating Benders cut. The problem is formulated under
# a given scenario 's' at stage 't'.
#
def createWorkerLP(Para,Info,s,t):
    #
    # minimize
    #       Costs of power purchasing, load shedding, renewables generation
    #       and curtailment 
    # subject to
    #       1) Reconfiguration
    #       2) Radial topology
    #       3) Optimal power flow
    #       4) Upper and lower bound
    #

    # Display
    number = t * Para.N_scene + s
    print('Formulating No.%d' % number)
    # Scenario Data
    Data_gen  = np.zeros((Para.N_gen,Para.N_hour))
    Data_load = np.zeros((Para.N_bus,Para.N_hour))
    for h in range(Para.N_hour):
        index_hour = s * Para.N_hour + h
        for n in range(Para.N_bus):
            Data_load[n,h] = Para.Load[n,t] * Para.Ty_load[index_hour]
        for n in range(Para.N_gen):
            gen_type = int(Para.Gen[n,6])  # type of renewables
            Data_gen [n,h] = Para.Gen [n,2] * Para.Ty_gen [index_hour,gen_type]

    # Model
    model = Model()

    # Create reconfiguration variables
    x_line = model.addVars(Para.N_line)  # line
    x_conv = model.addVars(Para.N_conv)  # converter
    x_sub  = model.addVars(Para.N_sub)   # substation
    x_gen  = model.addVars(Para.N_gen)   # renewables
    y_line = model.addVars(Para.N_line)  # reconfiguration
    # Create power flow variables
    Var = model.addVars(N_Var, Para.N_hour, lb = -GRB.INFINITY)

    # Set objective
    obj = LinExpr()
    for h in range(Para.N_hour):
        for n in range(Para.N_sub):
            obj = obj + Var[N_P_sub  + n, h] * Para.Cost_load
            #obj = obj + Var[N_Q_sub  + n, h] * Para.Cost_load
        for n in range(Para.N_gen):
            obj = obj + Var[N_S_gen  + n, h] * Para.Cost_gen
            obj = obj + Var[N_C_gen  + n, h] * Para.Cost_cutgen
        for n in range(Para.N_bus):
            obj = obj + Var[N_C_load + n, h] * Para.Cost_cutload
    obj = obj * Para.N_time  # number of times in a year
    model.setObjective(obj, GRB.MINIMIZE)

    # Set constraints
    for h in range(Para.N_hour):
        # 1.Active power balance equation
        for n in range(Para.N_bus):
            # Bus-Branch information
            line_head = Info.Line_head[n]
            line_tail = Info.Line_tail[n]
            conv_head = Info.Conv_head[n]
            conv_tail = Info.Conv_tail[n]
            # Formulate expression
            expr = LinExpr()
            expr = expr - quicksum(Var[N_P_line + i, h] for i in line_head)
            expr = expr + quicksum(Var[N_P_line + i, h] for i in line_tail)
            expr = expr - quicksum(Var[N_P_conv + i, h] for i in conv_head)
            expr = expr + quicksum(Var[N_P_conv + i, h] for i in conv_tail)
            if Para.Bus[n,7] == 0:  # AC bus
                expr = expr + Var[N_C_load + n, h] * Para.Factor[0]
            if Para.Bus[n,7] == 1:  # DC bus
                expr = expr + Var[N_C_load + n, h] * 1.0
            if n in Para.Sub[:,1]:
                bus_no = int(np.where(n == Para.Sub[:,1])[0])
                expr = expr + Var[N_P_sub + bus_no, h]
            if n in Para.Gen[:,1]:
                bus_no = int(np.where(n == Para.Gen[:,1])[0])
                if Para.Gen[bus_no,6] == 1:
                    expr = expr + Var[N_S_gen + bus_no, h] * 1.0
                else:
                    expr = expr + Var[N_S_gen + bus_no, h] * Para.Factor[0]
            # Add constraint
            if Para.Bus[n,7] == 0:  # AC bus
                model.addConstr(expr == Data_load[n,h] * Para.Factor[0])
            if Para.Bus[n,7] == 1:  # DC bus
                model.addConstr(expr == Data_load[n,h] * 1.0)
        
        # 2.Reactive power balance equation
        for n in range(Para.N_bus):
            # Bus-Branch information
            line_head = Info.Line_head[n]
            line_tail = Info.Line_tail[n]
            conv_head = Info.Conv_head[n]
            conv_tail = Info.Conv_tail[n]
            # Formulate expression
            expr = LinExpr()
            expr = expr - quicksum(Var[N_Q_line + i, h] for i in line_head)
            expr = expr + quicksum(Var[N_Q_line + i, h] for i in line_tail)
            if Para.Bus[n,7] == 0:  # AC bus
                expr = expr - quicksum(Var[N_Q_conv + i, h] for i in conv_head)
                expr = expr + quicksum(Var[N_Q_conv + i, h] for i in conv_tail)
                expr = expr + Var[N_C_load + n, h] * Para.Factor[1]
            if Para.Bus[n,7] == 1:  # DC bus
                expr = expr + Var[N_C_load + n, h] * 0.0
            if n in Para.Sub[:,1]:
                bus_no = int(np.where(n == Para.Sub[:,1])[0])
                expr = expr + Var[N_Q_sub + bus_no, h]
            if n in Para.Gen[:,1]:
                bus_no = int(np.where(n == Para.Gen[:,1])[0])
                if Para.Gen[bus_no,6] == 1:
                    expr = expr + Var[N_S_gen + bus_no, h] * 0.0
                else:
                    expr = expr + Var[N_S_gen + bus_no, h] * Para.Factor[1]
            # Add constraint
            if Para.Bus[n,7] == 0:  # AC bus
                model.addConstr(expr == Data_load[n,h] * Para.Factor[1])
            if Para.Bus[n,7] == 1:  # DC bus
                model.addConstr(expr == Data_load[n,h] * 0.0)
        
        # 3.Voltage balance on line
        for n in range(Para.N_line):
            bus_head = Para.Line[n,1]
            bus_tail = Para.Line[n,2]
            expr = LinExpr()
            expr = expr + Var[N_V_bus + bus_head, h]
            expr = expr - Var[N_V_bus + bus_tail, h]
            expr = expr - Var[N_P_line + n, h] * 2 * Para.Line_R[n]
            expr = expr - Var[N_Q_line + n, h] * 2 * Para.Line_X[n]
            model.addConstr(expr >= -Para.Big_M * (1 - y_line[n]))
            model.addConstr(expr <=  Para.Big_M * (1 - y_line[n]))
        
        # 4.Renewable generation
        for n in range(Para.N_gen):
            expr = LinExpr()
            expr = expr + Var[N_S_gen + n, h]
            expr = expr + Var[N_C_gen + n, h]
            model.addConstr(expr == x_gen[n] * Data_gen[n,h])
        
        # 5.Linearization of quadratic terms in line equations
        for n in range(Para.N_line):
            expr_0 = Var[N_P_line + n, h] + Var[N_Q_line + n, h]
            expr_1 = Para.Line_S[n,0] + x_line[n] * Para.Line_S[n,1]
            model.addConstr(expr_0 >= -1.414 * expr_1)
            model.addConstr(expr_0 <=  1.414 * expr_1)
            model.addConstr(expr_0 >= -1.414 * y_line[n] * Para.Line_S_max[n])
            model.addConstr(expr_0 <=  1.414 * y_line[n] * Para.Line_S_max[n])
        for n in range(Para.N_line):
            expr_0 = Var[N_P_line + n, h] - Var[N_Q_line + n, h]
            expr_1 = Para.Line_S[n,0] + x_line[n] * Para.Line_S[n,1]
            model.addConstr(expr_0 >= -1.414 * expr_1)
            model.addConstr(expr_0 <=  1.414 * expr_1)
            model.addConstr(expr_0 >= -1.414 * y_line[n] * Para.Line_S_max[n])
            model.addConstr(expr_0 <=  1.414 * y_line[n] * Para.Line_S_max[n])
        
        # 6.Linearization of quadratic terms in converter equations
        for n in range(Para.N_conv):
            expr_0 = Var[N_P_conv + n, h] + Var[N_Q_conv + n, h]
            expr_1 = x_conv[n] * Para.Conv[n,3]
            model.addConstr(expr_0 >= -1.414 * expr_1)
            model.addConstr(expr_0 <=  1.414 * expr_1)
        for n in range(Para.N_conv):
            expr_0 = Var[N_P_conv + n, h] - Var[N_Q_conv + n, h]
            expr_1 = x_conv[n] * Para.Conv[n,3]
            model.addConstr(expr_0 >= -1.414 * expr_1)
            model.addConstr(expr_0 <=  1.414 * expr_1)
        
        # 7.Linearization of quadratic terms in substation equations
        for n in range(Para.N_sub):
            expr_0 = Var[N_P_sub + n, h] + Var[N_Q_sub + n, h]
            expr_1 = Para.Sub_S[n,0] + x_sub[n] * Para.Sub_S[n,1]
            model.addConstr(expr_0 >= 0)
            model.addConstr(expr_0 <= 1.414 * expr_1)
        for n in range(Para.N_sub):
            expr_0 = Var[N_P_sub + n, h] - Var[N_Q_sub + n, h]
            expr_1 = Para.Sub_S[n,0] + x_sub[n] * Para.Sub_S[n,1]
            model.addConstr(expr_0 >= 0)
            model.addConstr(expr_0 <= 1.414 * expr_1)
        
        # 8.Bounds of variables
        # 1) Voltage
        for n in range(Para.N_bus):
            model.addConstr(Var[N_V_bus + n, h] >= Para.Voltage_low ** 2)
            model.addConstr(Var[N_V_bus + n, h] <= Para.Voltage_upp ** 2)
        # 2) power flow
        for n in range(Para.N_line):
            expr = Para.Line_S[n,0] + x_line[n] * Para.Line_S[n,1]
            model.addConstr(Var[N_P_line + n, h] >= -y_line[n] * Para.Line_S_max[n])
            model.addConstr(Var[N_P_line + n, h] <=  y_line[n] * Para.Line_S_max[n])
            model.addConstr(Var[N_P_line + n, h] >= -expr)
            model.addConstr(Var[N_P_line + n, h] <=  expr)
        for n in range(Para.N_line):
            if Para.Line[n,9] == 0:
                expr = Para.Line_S[n,0] + x_line[n] * Para.Line_S[n,1]
                model.addConstr(Var[N_Q_line + n, h] >= -y_line[n] * Para.Line_S_max[n])
                model.addConstr(Var[N_Q_line + n, h] <=  y_line[n] * Para.Line_S_max[n])
                model.addConstr(Var[N_Q_line + n, h] >= -expr)
                model.addConstr(Var[N_Q_line + n, h] <=  expr)
            if Para.Line[n,9] == 1:
                model.addConstr(Var[N_Q_line + n, h] ==  0)
        # 3) Converter
        for n in range(Para.N_conv):
            expr = x_conv[n] * Para.Conv[n,3]
            model.addConstr(Var[N_P_conv + n, h] >= -expr)
            model.addConstr(Var[N_P_conv + n, h] <=  expr)
        for n in range(Para.N_conv):
            expr = x_conv[n] * Para.Conv[n,3]
            model.addConstr(Var[N_Q_conv + n, h] >= -expr)
            model.addConstr(Var[N_Q_conv + n, h] <=  expr)
        # 4) Substation
        for n in range(Para.N_sub):
            expr = Para.Sub_S[n,0] + x_sub[n] * Para.Sub_S[n,1]
            model.addConstr(Var[N_P_sub + n, h] >= 0)
            model.addConstr(Var[N_P_sub + n, h] <= expr)
        for n in range(Para.N_sub):
            expr = Para.Sub_S[n,0] + x_sub[n] * Para.Sub_S[n,1]
            model.addConstr(Var[N_Q_sub + n, h] >= 0)
            model.addConstr(Var[N_Q_sub + n, h] <= expr)
        # 5) Load shedding
        for n in range(Para.N_bus):
            model.addConstr(Var[N_C_load + n, h] >= 0)
            model.addConstr(Var[N_C_load + n, h] <= Data_load[n,h])
        # 6) Renewables
        for n in range(Para.N_gen):
            model.addConstr(Var[N_S_gen + n, h] >= 0)
            model.addConstr(Var[N_S_gen + n, h] <= Data_gen[n,h])
        for n in range(Para.N_gen):
            model.addConstr(Var[N_C_gen + n, h] >= 0)
            model.addConstr(Var[N_C_gen + n, h] <= Data_gen[n,h])
    model.update()
    return model


# This function creates the worker linear programming model for each
# given scenario
#
def WorkerLP(Para,Info,Res_Master,WorkerPool,s,t):
    # Model formulation
    model = WorkerPool[t * Para.N_scene + s].copy()
    model.update()
    # Number of constraints
    N_con = []
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))
    # Add constraints
    var = model.getVars()
    for n in range(Para.N_line):
        model.addConstr(var[N_X_line + n] == Res_Master.x_line[n,t])
    for n in range(Para.N_conv):
        model.addConstr(var[N_X_conv + n] == Res_Master.x_conv[n,t])
    for n in range(Para.N_sub ):
        model.addConstr(var[N_X_sub  + n] == Res_Master.x_sub [n,t])
    for n in range(Para.N_gen ):
        model.addConstr(var[N_X_gen  + n] == Res_Master.x_gen [n,t])
    for n in range(Para.N_line):
        model.addConstr(var[N_Y_line + n] == Res_Master.y_line[n,s,t])
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))
    # Optimize
    model.Params.OutputFlag = 0  # turn off the display
    model.optimize()
    if model.status == GRB.Status.OPTIMAL:
        result = ResultWorkerLP(model,Para,N_con)
        return result
    else:
        return 0


# This function adds Benders cut to the master problem once an
# incumbent solution is found
#
def BendersCut(model,where):
    if where == GRB.Callback.MIPSOL:
        # Incumbent solutions
        Incumbent  = model.cbGetSolution(model._vars)
        Res_Master = ResultMasterMILP(model,Para,Incumbent)
        # Initialize Benders cut coefficient
        d_x_line = np.zeros((Para.N_line, Para.N_stage))
        d_x_conv = np.zeros((Para.N_conv, Para.N_stage))
        d_x_sub  = np.zeros((Para.N_sub , Para.N_stage))
        d_x_gen  = np.zeros((Para.N_gen , Para.N_stage))
        d_y_line = np.zeros((Para.N_line, Para.N_scene, Para.N_stage))
        d_object = 0
        # Operating worker linear programming
        for t in range(Para.N_stage):
            for s in range(Para.N_scene):
                result = WorkerLP(Para,Info,Res_Master,WorkerPool,s,t)
                # Formulate coefficient
                d_x_line[:,t] = d_x_line[:,t] + result.d_x_line
                d_x_conv[:,t] = d_x_conv[:,t] + result.d_x_conv
                d_x_sub [:,t] = d_x_sub [:,t] + result.d_x_sub
                d_x_gen [:,t] = d_x_gen [:,t] + result.d_x_gen
                d_y_line[:,s,t] = result.d_y_line
                d_object = d_object + result.obj
        # Formulate Benders cut
        i = 0  # index of variables
        c = d_object  # constant
        expr = LinExpr()
        for n in range(Para.N_line):
            for t in range(Para.N_stage):
                expr.addTerms(d_x_line[n,t], model._vars[i])
                c = c - d_x_line[n,t] * Incumbent[i]
                i = i + 1
        for n in range(Para.N_conv):
            for t in range(Para.N_stage):
                expr.addTerms(d_x_conv[n,t], model._vars[i])
                c = c - d_x_conv[n,t] * Incumbent[i]
                i = i + 1
        for n in range(Para.N_sub ):
            for t in range(Para.N_stage):
                expr.addTerms(d_x_sub [n,t], model._vars[i])
                c = c - d_x_sub [n,t] * Incumbent[i]
                i = i + 1
        for n in range(Para.N_gen ):
            for t in range(Para.N_stage):
                expr.addTerms(d_x_gen [n,t], model._vars[i])
                c = c - d_x_gen [n,t] * Incumbent[i]
                i = i + 1
        for n in range(Para.N_line):
            for s in range(Para.N_scene):
                for t in range(Para.N_stage):
                    expr.addTerms(d_y_line[n,s,t], model._vars[i])
                    c = c - d_y_line[n,s,t] * Incumbent[i]
                    i = i + 1
        model.cbLazy(model._vars[-1] >= expr + c)


# This function creates the DSEP model using benders decomposition
#
def BendersDSEP(MasterMILP,WorkerPool):
    # Copy
    model = MasterMILP.copy()
    model._vars = model.getVars()
    # Set parameters
    model.Params.lazyConstraints = 1
    model.Params.MIPGap = 0.025
    model.Params.TimeLimit = 36000  # 6 hours
    # Optimize
    model.optimize(BendersCut)
    # Result
    if model.status == GRB.Status.OPTIMAL:
        variable = [(model._vars[i]).x for i in range(len(model._vars))]
        result = ResultMasterMILP(model,Para,variable)
        return result
    else:
        return 0


# Main function
if __name__ == "__main__":

    # Input parameter
    filename = "data/Data-Ninghai.xlsx"  # file name
    Data = ReadData(filename)  # data
    Para = Parameter(Data)  # system parameter
    Info = BusInfo(Para)  # bus information
    plot = PlotFunc(Para)  # figure
    Indexing(Para)  # formulating global parameters

    # Create model
    MasterMILP = createMasterMILP(Para,Info)
    WorkerPool = []
    for t in range(Para.N_stage):
        for s in range(Para.N_scene):
            WorkerPool.append(createWorkerLP(Para,Info,s,t))
    
    # Benders decomposition
    Result_DSEP = BendersDSEP(MasterMILP,WorkerPool)
    
    # Save results
    with open('result/result.csv', 'w', newline = '') as f:
        writer = csv.writer(f)
        writer.writerows(Result_DSEP.x_line.tolist())
        writer.writerows(Result_DSEP.x_conv.tolist())
        writer.writerows(Result_DSEP.x_sub .tolist())
        writer.writerows(Result_DSEP.x_gen .tolist())
        writer.writerows(Result_DSEP.y_line.tolist())

    # Plot
    # plot.Planning(Para,Result_DSEP,2)
    # plot.Reconfig(Para,Result_DSEP,2,2)
    