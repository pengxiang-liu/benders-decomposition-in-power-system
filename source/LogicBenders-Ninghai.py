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
        self.N_stage = 3
        self.N_year_of_stage = 5
        self.N_day = 4  # number of typical day
        self.N_day_season = [90,91,92,92]  # number of days in each season
        self.N_scenario = 4  # number of reconfiguration in a day
        self.N_hour = int(24/self.N_scenario)  # number of hour
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
def Depreciation(Life,Rate):
    return Rate*((1+Rate)**Life)/((1+Rate)**Life-1)


# This function plots the planning solution in all stages
# The solution is given in three separated sub-plots
#
def PlotPlanning(Para,x_line):
    x = Para.Bus[:,2]
    y = Para.Bus[:,3]
    
    for n in range(Para.N_bus):  # Bus
        plt.text(x[n] + 3, y[n] + 3, '%s'%n)
        if n in Para.Sub[:,1]:
            plt.plot(x[n],y[n],'rs')
        else:
            plt.plot(x[n],y[n],'b.')
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


if __name__ == "__main__":

    # Input parameter
    filename = "data/Data-Ninghai.xlsx"
    Data = ReadData(filename)

    # Data formulation
    Para = Parameter(Data)  # System parameter
    Info = BusInfo(Para)

    # Benders decomposition
    Result_Planning = Planning(Para,Info)

    n = 1
    