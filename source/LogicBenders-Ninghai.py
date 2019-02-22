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
        self.N_stage = 3   # number of stage
        self.N_year  = 5   # number of year in each stage
        self.N_scene = 16  # number of reconfiguration scenario
        self.N_hour  = 6   # number of hour in each scenario
        self.N_time  = 360/self.N_scene  # number of times
        self.Int_rate = 0.05  # interest rate
        self.Big_M = 500  # Big M
        self.Voltage = 35
        self.Voltage_low = 35 * 0.95
        self.Voltage_upp = 35 * 1.05
        # Bus
        self.Bus  = Data[0]
        self.Bus_AC = self.Bus[np.where(self.Bus[:,7] == 0)]
        self.Bus_DC = self.Bus[np.where(self.Bus[:,7] == 1)]
        self.N_bus = len(self.Bus)
        self.N_bus_AC = len(self.Bus_AC)
        self.N_bus_DC = len(self.Bus_DC)
        self.Load = self.Bus[:,4:7]
        self.Load_angle = 0.95  # phase angle of load
        self.Cost_cutload = 200  # cost of load shedding
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
        self.TypicalDay = Data[5]
        self.Typical_load  = self.TypicalDay[:,0]
        self.Typical_wind  = self.TypicalDay[:,1]
        self.Typical_solar = self.TypicalDay[:,2]
        self.Typical_water = self.TypicalDay[:,3]


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
    def __init__(self,model,Para,x_line,x_conv,x_sub,x_gen,y_line):
        self.x_line = GurobiValue(x_line,'integer')
        self.x_conv = GurobiValue(x_conv,'integer')
        self.x_sub  = GurobiValue(x_sub, 'integer')
        self.x_gen  = GurobiValue(x_gen, 'integer')
        self.y_line = GurobiValue(y_line,'integer')
        

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
def GurobiValue(var,string):
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


def Planning(Para,Info):
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

    # Set objective
    obj = LinExpr()
    for t in range(Para.N_stage):
        RR = 0  # Reconvery rate in 5 years
        for y in range(Para.N_year):
            RR = RR + (1 + Para.Int_rate) ** (-(t * Para.N_year + y + 1))
        for n in range(Para.N_line):  # line
            obj = obj + RR * x_line[n,t] * Para.Line[n][8] * Para.Dep_line
        for n in range(Para.N_conv):  # converter
            obj = obj + RR * x_conv[n,t] * Para.Conv[n][4] * Para.Dep_conv
        for n in range(Para.N_sub):  # substation
            obj = obj + RR * x_sub [n,t] * Para.Sub [n][4] * Para.Dep_sub
        for n in range(Para.N_gen):  # renewables generation
            obj = obj + RR * x_gen [n,t] * Para.Gen [n][3] * Para.Dep_gen
    model.setObjective(obj, GRB.MINIMIZE)

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
                    bus_temp = int(np.where(i == Para.Sub[:,1])[0])
                    expr = expr - f_sub[bus_temp,s,t]
                if i in Para.Gen[:,1]:
                    bus_temp = int(np.where(i == Para.Gen[:,1])[0])
                    expr = expr + f_gen[bus_temp,s,t]
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

    # Optimize
    model.optimize()
    if model.status == GRB.Status.OPTIMAL:
        result = ResultPlanning(model,Para,x_line,x_conv,x_sub,x_gen,y_line)
    return result
   

# This function creates the reconfiguration worker problem. Dual variables 
# are returned for generating Benders cut. The problem is formulated under
# a given scenario 's' at stage 't'.
#
def Reconfig(Para,Info,Result_Planning,s,t):
    #
    '''
    y_line = Result_Planning.y_line[:,s,t]
    x_conv = Result_Planning.x_conv[:,t]
    x_sub  = Result_Planning.x_sub [:,t]
    x_gen  = Result_Planning.x_gen [:,t]
    '''
    pass


if __name__ == "__main__":

    # Input parameter
    filename = "data/Data-Ninghai.xlsx"
    Data = ReadData(filename)

    # Data formulation
    Para = Parameter(Data)  # System parameter
    Info = BusInfo(Para)

    # Benders decomposition
    Result_Planning = Planning(Para,Info)
    Result_Reconfig = Reconfig(Para,Info,Result_Planning,s,t)

    '''
    # Figure
    Plot = PlotFunc(Para)
    Plot.Planning(Para,Result_Planning,0)
    Plot.Reconfig(Para,Result_Planning,0,0)
    '''

    n = 1
    