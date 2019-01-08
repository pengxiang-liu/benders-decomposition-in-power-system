#!/usr/bin/python
#
# Copyright 2019, Southeast University, Liu Pengxiang
#
# Read system data from Excel files and reurn two parameters: Para and Info
# Para contains all system data as a "Parameter" class
# Info contains all bus information as a "BusInfo" class


import sys
import math
import xlrd
import numpy as np


# Read data from Excel files
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


# Matrix slice
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


# Depreciation calculator
def Depreciation(Life,Rate):
    return Rate*((1+Rate)**Life)/((1+Rate)**Life-1)


# System parameter
class Parameter(object):
    def __init__(self,Data_origin):
        # System
        self.N_stage = 3
        self.N_year_of_stage = 5
        self.N_day = 4  # number of typical day
        self.N_scenario = 4  # number of reconfiguration in a day
        self.N_hour = 24/self.N_scenario  # number of hour in a scenario
        self.Int_rate = 0.05  # interest rate

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
        self.Cost_cutwind = 150  # cost of wind curtailment
        self.Wind_angle = 0.98  # phase angle of wind farm output
        self.Dep_wind = Depreciation(Year_wind,self.Int_rate)

        # Solar data
        Solar = Data_origin[4]
        Year_solar = 15
        self.Solar = Solar
        self.N_solar = len(Solar)
        self.Cost_cutsolar = 150  # cost of solar curtailment
        self.Solar_angle = 1.00  # phase angle of PV station output
        self.Dep_solar = Depreciation(Year_solar,self.Int_rate)

        # Typical data
        self.Typical_load  = Data_origin[5][:,1:]  # load
        self.Typical_wind  = Data_origin[6][:,1:]  # wind
        self.Typical_solar = Data_origin[7][:,1:]  # solar


# Bus information
class BusInfo(object):
    def __init__(self,Para):
        # line
        Line_head = [[] for i in range(Para.N_bus)]
        Line_tail = [[] for i in range(Para.N_bus)]
        for i in range(Para.N_line):
            head = Para.Line[i][1]
            tail = Para.Line[i][2]
            Line_head[int(head)].append(i) # set of lines whose head-end is bus i
            Line_tail[int(tail)].append(i) # set of lines whose tail-end is bus i
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
        
