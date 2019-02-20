#!/usr/bin/python
#
# Copyright 2019, Southeast University, Liu Pengxiang
#
# A distribution system planning model using logic-based Benders decomposition
# 
# This project develops a multi-stage planning model for AC-DC distribution system
# in Zhejiang province, China. A logic-based Benders decomposition algorithm is 
# developed to deal with the integer variables in the Master-problem


import sys
import math
import xlrd
import time
import numpy as np
import matplotlib.pyplot as plt

from gurobipy import *


# This class builds the system parameter, including data of system, bus, line,
# substation, wind farm, solar station and typical day(load, wind and solar)
# Note: AC => 0, DC => 1, AC/DC => 2
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
        self.Voltage_low = 35 * 0.95
        self.Voltage_upp = 35 * 1.05
        # Bus data
        Bus = Data_origin[0]
        self.Bus = Bus
        self.N_bus = len(Bus)
        self.Coordinate = Bus[:,2:4]  # coordinate
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


class Parameter_AC(object):
    def __init__(self,Para):
        self.Bus_AC = Para.Bus[np.where(Para.Bus[:,10] != 1)]

# This function input data from Excel files. The filtname can be changed to other
# power system for further study
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
    # Data formulation
    Para = Parameter(Data_origin)  # System parameter
    Para_AC = Parameter_AC(Para)
    Para_DC = Parameter_DC(Para)
    #Info = BusInfo(Para)  # Bus Information
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




if __name__ == "__main__":

    # Input parameter
    filename = "data/Data-Ninghai.xlsx"
    [Para,Info] = ReadData(filename)
    