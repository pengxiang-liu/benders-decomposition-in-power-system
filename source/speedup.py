#!/usr/bin/python
#
# Copyright 2019, Southeast University, Liu Pengxiang
#
# A distribution system planning model using Logic-Benders decomposition
# 
# This project develops a multi-stage planning model for AC-DC distribution 
# system in Zhejiang province, China. A logic-based Benders decomposition 
# algorithm is developed to deal with the integer variables in the 
# Master-problem


import sys
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
        self.Big_M = 250  # Big M
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
class ResultPlanning(object):
    def __init__(self,model,Para,x_line,x_conv,x_sub,x_gen,y_line,
                 obj_con,obj_opr):
        # Variables
        self.x_line = GurobiValue(x_line,'integer')
        self.x_conv = GurobiValue(x_conv,'integer')
        self.x_sub  = GurobiValue(x_sub, 'integer')
        self.x_gen  = GurobiValue(x_gen, 'integer')
        self.y_line = GurobiValue(y_line,'integer')
        # Objective
        self.obj_con = obj_con.getValue()
        self.obj_opr = obj_opr.x
        self.obj = (model.getObjective()).getValue()



# This class restores the results of reconfiguration worker-problem
class ResultReconfig(object):
    def __init__(self,model,Para,Var,N_con):
        # Saving objective
        self.obj = (model.getObjective()).getValue()
        # Saving variables
        '''
        var = GurobiValue(Var)
        self.V_bus  = var[N_V_bus  : N_V_bus  + Para.N_bus, :]
        self.P_line = var[N_P_line : N_P_line + Para.N_line,:]
        self.Q_line = var[N_Q_line : N_Q_line + Para.N_line,:]
        self.P_conv = var[N_P_conv : N_P_conv + Para.N_conv,:]
        self.Q_conv = var[N_Q_conv : N_Q_conv + Para.N_conv,:]
        self.P_sub  = var[N_P_sub  : N_P_sub  + Para.N_sub ,:]
        self.Q_sub  = var[N_Q_sub  : N_Q_sub  + Para.N_sub ,:]
        self.C_load = var[N_C_load : N_C_load + Para.N_bus ,:]
        self.S_gen  = var[N_S_gen  : N_S_gen  + Para.N_gen ,:]
        self.C_gen  = var[N_C_gen  : N_C_gen  + Para.N_gen ,:]
        '''
        # Saving dual information
        constr = model.getConstrs()
        dual = [constr[n].pi  for n in range(N_con[-2],N_con[-1])]
        self.dual_x_line = np.array([dual.pop(0) for n in range(Para.N_line)])
        self.dual_x_conv = np.array([dual.pop(0) for n in range(Para.N_conv)])
        self.dual_x_sub  = np.array([dual.pop(0) for n in range(Para.N_sub )])
        self.dual_x_gen  = np.array([dual.pop(0) for n in range(Para.N_gen )])
        self.dual_y_line = np.array([dual.pop(0) for n in range(Para.N_line)])
        

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
        self.dual_x_line = np.zeros((Para.N_line, Para.N_stage))
        self.dual_x_conv = np.zeros((Para.N_conv, Para.N_stage))
        self.dual_x_sub  = np.zeros((Para.N_sub , Para.N_stage))
        self.dual_x_gen  = np.zeros((Para.N_gen , Para.N_stage))
        self.dual_y_line = np.zeros((Para.N_line, Para.N_scene, Para.N_stage))
        # Objective
        self.obj = 0


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
        x = Para.Bus[:,2]
        y = Para.Bus[:,3]
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


# This function get the value of gurobi variables
def GurobiValue(var,string = 'continuous'):
    key = var.keys()
    cpy = var.copy()
    dim = key._tuplelist__tuplelen  # dimention
    for i in range(len(key)):
        if string == 'integer':
            cpy[key[i]] = int(round(var[key[i]].x))
        else:
            cpy[key[i]] = var[key[i]].x
    if dim == 1:
        dim_1 = len(key.select('*'))
        matrix_var = np.zeros(dim_1)
        for i in range(dim_1):
            matrix_var[i] = cpy[i]
    if dim == 2:
        dim_1 = len(key.select('*',0))
        dim_2 = len(key.select(0,'*'))
        matrix_var = np.zeros((dim_1,dim_2))
        for i in range(dim_1):
            for j in range(dim_2):
                matrix_var[i,j] = cpy[i,j]
    if dim == 3:
        dim_1 = len(key.select('*',0,0))
        dim_2 = len(key.select(0,'*',0))
        dim_3 = len(key.select(0,0,'*'))
        matrix_var = np.zeros((dim_1,dim_2,dim_3))
        for i in range(dim_1):
            for j in range(dim_2):
                for k in range(dim_3):
                    matrix_var[i,j,k] = cpy[i,j,k]
    return matrix_var


def Planning(Para,Info,Power):
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
    #
    obj_opr = model.addVar()

    # Set objective
    obj_con = LinExpr()
    for t in range(Para.N_stage):
        RR = 0  # Reconvery rate in 5 years
        for y in range(Para.N_year):
            RR = RR + (1 + Para.Int_rate) ** (-(t * Para.N_year + y + 1))
        for n in range(Para.N_line):  # line
            obj_con = obj_con + RR * x_line[n,t] * Para.Line[n][8] * Para.Dep_line
        for n in range(Para.N_conv):  # converter
            obj_con = obj_con + RR * x_conv[n,t] * Para.Conv[n][4] * Para.Dep_conv
        for n in range(Para.N_sub):  # substation
            obj_con = obj_con + RR * x_sub [n,t] * Para.Sub [n][4] * Para.Dep_sub
        for n in range(Para.N_gen):  # renewables generation
            obj_con = obj_con + RR * x_gen [n,t] * Para.Gen [n][3] * Para.Dep_gen

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
                    model.addConstr(y_line[n,s,t] <= x_line[n,t])
    
    # Constraint 3 (fictitious power flow initialization)
    for t in range(Para.N_stage):
        for s in range(Para.N_scene):
            for n in range(Para.N_bus):
                if Para.Load[n,t] > 0:  # load bus
                    model.addConstr(f_load[n,s,t] == 1)
                else:  # none load bus
                    model.addConstr(f_load[n,s,t] == 0)
            for n in range(Para.N_line):
                model.addConstr(f_line[n,s,t] >= -100 * y_line[n,s,t])
                model.addConstr(f_line[n,s,t] <=  100 * y_line[n,s,t])
            for n in range(Para.N_conv):
                model.addConstr(f_conv[n,s,t] >= -100 * x_conv[n,t])
                model.addConstr(f_conv[n,s,t] <=  100 * x_conv[n,t])
            for n in range(Para.N_sub):
                model.addConstr(f_sub [n,s,t] >=  0)
                model.addConstr(f_sub [n,s,t] <=  100)
            for n in range(Para.N_gen):
                model.addConstr(f_gen [n,s,t] ==  x_gen[n,t])
    
    # Constraint 4 (connectivity)
    for t in range(Para.N_stage):
        for s in range(Para.N_scene):
            for i in range(Para.N_bus):
                line_head = Info.Line_head[i]
                line_tail = Info.Line_tail[i]
                conv_head = Info.Conv_head[i]
                conv_tail = Info.Conv_tail[i]
                expr = LinExpr()
                expr = expr + f_load[i,s,t]
                expr = expr + quicksum(f_line[n,s,t] for n in line_head)
                expr = expr - quicksum(f_line[n,s,t] for n in line_tail)
                expr = expr + quicksum(f_conv[n,s,t] for n in conv_head)
                expr = expr - quicksum(f_conv[n,s,t] for n in conv_tail)
                if i in Para.Sub[:,1]:
                    bus_no = int(np.where(i == Para.Sub[:,1])[0])
                    expr = expr - f_sub[bus_no,s,t]
                if i in Para.Gen[:,1]:
                    bus_no = int(np.where(i == Para.Gen[:,1])[0])
                    expr = expr + f_gen[bus_no,s,t]
                model.addConstr(expr == 0)
    
    # Constraint 5 (cradial topology)
    for t in range(Para.N_stage):
        for s in range(Para.N_scene):
            for i in range(Para.N_bus_AC):
                line_head = Info.Line_head[i]
                line_tail = Info.Line_tail[i]
                expr = LinExpr()
                expr = expr + quicksum(y_pos[n,s,t] for n in line_tail)
                expr = expr + quicksum(y_neg[n,s,t] for n in line_head)
                if Para.Load[i,t] > 0:  # load bus
                    model.addConstr(expr == 1)
                else:  # none load bus
                    model.addConstr(expr == 0)
            for n in range(Para.N_line):
                model.addConstr(y_pos[n,s,t] + y_neg[n,s,t] == y_line[n,s,t])
    
    # Constraint 6 (renewable generation)
    for t in range(Para.N_stage):
        for n in range(Para.N_gen):
            if t >= Para.Gen[n,5]:
                model.addConstr(x_gen[n,t] == 1)
            else:
                model.addConstr(x_gen[n,t] == 0)

    # Benders cut
    if len(Power) == 0:
        model.addConstr(obj_opr >= 0)
    else:
        for i in range(len(Power)):
            expr = LinExpr()
            for t in range(Para.N_stage):
                for n in range(Para.N_line):
                    expr = expr + Power[i].dual_x_line[n,t] * (x_line[n,t] - Power[i].x_line[n,t])
                for n in range(Para.N_conv):
                    expr = expr + Power[i].dual_x_conv[n,t] * (x_conv[n,t] - Power[i].x_conv[n,t])
                for n in range(Para.N_sub ):
                    expr = expr + Power[i].dual_x_sub[n,t]  * (x_sub [n,t] - Power[i].x_sub [n,t])
                for n in range(Para.N_gen ):
                    expr = expr + Power[i].dual_x_gen[n,t]  * (x_gen [n,t] - Power[i].x_gen [n,t])
                for s in range(Para.N_scene):
                    for n in range(Para.N_line):
                        expr = expr + Power[i].dual_y_line[n,s,t] * (y_line[n,s,t] - Power[i].y_line[n,s,t])
            model.addConstr(obj_opr >= Power[i].obj + expr)
    
    # Optimize
    model.setObjective(obj_con + obj_opr, GRB.MINIMIZE)
    model.Params.MIPGap = 1e-2
    model.optimize()
    if model.status == GRB.Status.OPTIMAL:
        result = ResultPlanning(model,Para,x_line,x_conv,x_sub,x_gen,y_line,
                                obj_con,obj_opr)
    return result
   

# This function creates the reconfiguration worker problem. Dual variables
# are returned for generating Benders cut. The problem is formulated under
# a given scenario 's' at stage 't'.
#
def Reconfig(Para,Info,Result_Planning,s,t):
    #
    # minimize
    #       Costs of power purchasing, load shedding, renewables generation
    #       and curtailment 
    # subject to
    #       1) Reconfiguration
    #       2) Radial topology
    #       3) Optimal Power flow
    #       4) Upper and lower bound
    #
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
    global N_V_bus, N_P_line, N_Q_line, N_P_conv, N_Q_conv
    global N_P_sub, N_Q_sub,  N_C_load, N_S_gen,  N_C_gen

    # Create reconfiguration variables
    x_line = model.addVars(Para.N_line)  # line
    x_conv = model.addVars(Para.N_conv)  # converter
    x_sub  = model.addVars(Para.N_sub)   # substation
    x_gen  = model.addVars(Para.N_gen)   # renewables
    y_line = model.addVars(Para.N_line)  # reconfiguration
    # Create power flow variables
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
    Var = model.addVars(N_Var, Para.N_hour, lb = -GRB.INFINITY)

    # Set objective
    obj = LinExpr()
    for h in range(Para.N_hour):
        for n in range(Para.N_sub):
            obj = obj + Var[N_P_sub  + n, h] * Para.Cost_load
            obj = obj + Var[N_Q_sub  + n, h] * Para.Cost_load
        for n in range(Para.N_gen):
            obj = obj + Var[N_S_gen  + n, h] * Para.Cost_gen
            obj = obj + Var[N_C_gen  + n, h] * Para.Cost_cutgen
        for n in range(Para.N_bus):
            obj = obj + Var[N_C_load + n, h] * Para.Cost_cutload
    obj = obj * Para.N_time  # number of times in a year
    model.setObjective(obj, GRB.MINIMIZE)

    # Set constraints
    N_con = []  # indexing constraints

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
            model.addConstr(expr_0 >= -1.414 * y_line[n] * sum(Para.Line_S[n,:]))
            model.addConstr(expr_0 <=  1.414 * y_line[n] * sum(Para.Line_S[n,:]))
        for n in range(Para.N_line):
            expr_0 = Var[N_P_line + n, h] - Var[N_Q_line + n, h]
            expr_1 = Para.Line_S[n,0] + x_line[n] * Para.Line_S[n,1]
            model.addConstr(expr_0 >= -1.414 * expr_1)
            model.addConstr(expr_0 <=  1.414 * expr_1)
            model.addConstr(expr_0 >= -1.414 * y_line[n] * sum(Para.Line_S[n,:]))
            model.addConstr(expr_0 <=  1.414 * y_line[n] * sum(Para.Line_S[n,:]))
        
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
        # 2) Power flow
        for n in range(Para.N_line):
            expr = Para.Line_S[n,0] + x_line[n] * Para.Line_S[n,1]
            model.addConstr(Var[N_P_line + n, h] >= -y_line[n] * sum(Para.Line_S[n,:]))
            model.addConstr(Var[N_P_line + n, h] <=  y_line[n] * sum(Para.Line_S[n,:]))
            model.addConstr(Var[N_P_line + n, h] >= -expr)
            model.addConstr(Var[N_P_line + n, h] <=  expr)
        for n in range(Para.N_line):
            if Para.Line[n,9] == 0:
                expr = Para.Line_S[n,0] + x_line[n] * Para.Line_S[n,1]
                model.addConstr(Var[N_Q_line + n, h] >= -y_line[n] * sum(Para.Line_S[n,:]))
                model.addConstr(Var[N_Q_line + n, h] <=  y_line[n] * sum(Para.Line_S[n,:]))
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
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))

    model_save = model.copy()
    '''
    # Equations
    for n in range(Para.N_line):
        model.addConstr(x_line[n] == Result_Planning.x_line[n,t])
    for n in range(Para.N_conv):
        model.addConstr(x_conv[n] == Result_Planning.x_conv[n,t])
    for n in range(Para.N_sub):
        model.addConstr(x_sub [n] == Result_Planning.x_sub [n,t])
    for n in range(Para.N_gen):
        model.addConstr(x_gen [n] == Result_Planning.x_gen [n,t])
    for n in range(Para.N_line):
        model.addConstr(y_line[n] == Result_Planning.y_line[n,s,t])
    model.update()
    N_con.append(model.getAttr(GRB.Attr.NumConstrs))
    
    # Optimize
    model.optimize()
    if model.status == GRB.Status.OPTIMAL:
        result = ResultReconfig(model,Para,Var,N_con)
    '''
    return model_save


def Real_reconfig(Para,Info,Result_Planning,model,s,t):
    N_con = []
    m = model[t*Para.N_scene+s].copy()
    m.update()
    N_con.append(m.getAttr(GRB.Attr.NumConstrs))
    var = m.getVars()
    for n in range(Para.N_line):
        m.addConstr(var[n] == Result_Planning.x_line[n,t])
    for n in range(Para.N_conv):
        m.addConstr(var[n + Para.N_line] == Result_Planning.x_conv[n,t])
    for n in range(Para.N_sub):
        m.addConstr(var[n + Para.N_line + Para.N_conv] == Result_Planning.x_sub [n,t])
    for n in range(Para.N_gen):
        m.addConstr(var[n + Para.N_line + Para.N_conv + Para.N_sub] == Result_Planning.x_gen [n,t])
    for n in range(Para.N_line):
        m.addConstr(var[n + Para.N_line + Para.N_conv + Para.N_sub + Para.N_gen] == Result_Planning.y_line[n,s,t])
    m.update()
    N_con.append(m.getAttr(GRB.Attr.NumConstrs))
    # Optimize
    m.optimize()
    if m.status == GRB.Status.OPTIMAL:
        result = ResultReconfig(m,Para,var,N_con)
    return result


if __name__ == "__main__":

    time_start=time.time()

    # Input parameter
    filename = "data/Data-Ninghai.xlsx"  # file name
    Data = ReadData(filename)  # Data
    Para = Parameter(Data)  # System parameter
    Info = BusInfo(Para)  # Bus information

    # Benders decomposition
    lower_bound = []  #
    upper_bound = []  #
    gap = []
    # benders cut for power flow information
    Logic = []
    Power = []
    model = []
    # Iteration
    n_iter = 0  # index of iteration
    while True:
        # Master-problem
        Result_Planning = Planning(Para,Info,Power)
        # Worker-problem
        obj_opr = 0
        Benders_Info = BendersInfo(Para,Result_Planning)
        if n_iter == 0:
            for t in range(Para.N_stage):
                for s in range(Para.N_scene):
                    # Reconfiguration
                    print(t*Para.N_scene+s)
                    model.append(Reconfig(Para,Info,Result_Planning,s,t))
        for t in range(Para.N_stage):
            for s in range(Para.N_scene):
                Result_Reconfig = Real_reconfig(Para,Info,Result_Planning,model,s,t)
                obj_opr = obj_opr + Result_Reconfig.obj
                # Tranditional Benders cut information
                Benders_Info.dual_x_line[:,t]  += Result_Reconfig.dual_x_line
                Benders_Info.dual_x_conv[:,t]  += Result_Reconfig.dual_x_conv
                Benders_Info.dual_x_sub [:,t]  += Result_Reconfig.dual_x_sub
                Benders_Info.dual_x_gen [:,t]  += Result_Reconfig.dual_x_gen
                Benders_Info.dual_y_line[:,s,t] = Result_Reconfig.dual_y_line
        Benders_Info.obj = obj_opr
        Power.append(Benders_Info)
        # Updating lower and upper bound
        lower_bound.append(Result_Planning.obj)
        upper_bound.append(Result_Planning.obj_con + obj_opr)
        gap.append((upper_bound[-1]-lower_bound[-1])/upper_bound[-1])
        if gap[-1] <= 1e-2 or n_iter > 3000:
            break
        else:
            n_iter = n_iter + 1
    '''
    # Figure
    Plot = PlotFunc(Para)
    Plot.Planning(Para,Result_Planning,0)
    Plot.Reconfig(Para,Result_Planning,0,0)
    '''
    time_end=time.time()
    print('totally cost',time_end-time_start)
    
    n = 1
    