#!/usr/bin/python
#
# Copyright 2019, Southeast University, Liu Pengxiang
#
# Algorithm Toolbox
# 
# 1.Branch-and-Bound
# 2.Linear programming


import sys
import math
import numpy as np
from gurobipy import *


'''1. Modified Branch-and-Bound Algorithm'''
#
# This program solves 0-1 programming based on a classical Branch-and-Bound algorithm
# Dual information from the relaxed linear programming at each leaf node is returned
# 
# The model has the following format:
# 
# minimize
#       c * x + d
# subject to 
#       A * x >= B     (u)
#       lb <= x <= ub  (v)
# where x is binary varibale, u and v are dual variables
#
def BranchBound(c,d,A,B,lb,ub,br = 0):
    # Global variables
    global var, obj  # optimization
    global dual, flag, j0, j1, fit  # node of search tree
    if br == 0:  # initialization
        var  = np.zeros(len(c))  # all zero
        obj  = float("inf")  # python infinity
        dual = []  # dual variables 
        flag = []  # optimality flag
        fit  = []  # fitness
        j0   = []  # index of x where branching has set xj to 0
        j1   = []  # index of x where branching has set xj to 1
    # Solve the relaxed linear programming
    [lp_var,lp_obj,lp_flg,lp_dul] = Linprog(c,d,A,B,lb,ub)
    # Update global variabes for the current node in the search tree
    dual.append(lp_dul)
    flag.append(lp_flg)
    fit. append(lp_obj)
    j0.  append(np.where(lb + ub == 0))  # lb == 0 and ub == 0
    j1.  append(np.where(lb + ub == 2))  # lb == 1 and ub == 1
    # Branching
    if lp_flg != 1:  # if problem is infeasible
        var_out = lp_var
        obj_out = lp_obj
        flg_out = -1
    else:  # if problem is feasible
        if lp_obj > obj:  # can't find any solution better than the current one
            var_out = lp_var
            obj_out = lp_obj
            flg_out = -2
        else:  # find a solution better than the current one
            lp_var = np.array(lp_var)  # list to array
            lp_gap = np.abs(lp_var - np.rint(lp_var))  # gap
            if max(lp_gap) == 0:  # integer solution
                var = lp_var  # update global variable
                obj = lp_obj
                var_out = var  # update output
                obj_out = obj
                flg_out = 1
            else:  # real solution
                leaf = np.where(lp_gap > 0)  # index of leaf node for branching
                pick = int(leaf[0][0])  # pick up the first index
                lb_temp = lb  # temporary lower bound
                ub_temp = ub  # temporary upper bound
                # The upper branch calculation
                if ub[pick] >= np.floor(lp_var[pick]) + 1:
                    lb_temp[pick] = np.floor(lp_var[pick]) + 1  # branching
                    BranchBound(c,d,A,B,lb_temp,ub,1)
                # The lower branch calculation
                if lb[pick] <= np.floor(lp_var[pick]):
                    ub_temp[pick] = np.floor(lp_var[pick])  # branching
                    BranchBound(c,d,A,B,lb,ub_temp,1)
                # update output
                var_out = var
                obj_out = obj
                flg_out = 1
    # Return results
    return [var_out,obj_out,flg_out,dual,flag,j0,j1,fit]


'''2. Linear Programming'''
#
# This program solves a simple linear programming by Gurobi 8.1.0
# 
# The model has the following format:
# 
# minimize
#       c * x + d
# subject to 
#       A * x >= B     (u)
#       lb <= x <= ub  (v)
# where u and v are dual variables
#
def Linprog(c,d,A,B,lb,ub):
    # Create a gurobi model
    model = Model()
    # Create variables
    n_var = np.size(A,1) # number of x
    x = model.addVars(n_var)
    # Set objective
    obj = quicksum(c[i] * x[i] for i in range(n_var)) + d
    model.setObjective(obj, GRB.MINIMIZE)
    # Add constraints
    for i in range(np.size(A,0)):
        model.addConstr(quicksum(A[i][j] * x[j] for j in range(n_var)) >= B[i])
    for i in range(n_var):
        model.addConstr(x[i] >= lb[i])
        model.addConstr(x[i] <= ub[i])
    # Solve
    model.Params.OutputFlag = 0  # turn off the display
    model.optimize()
    # Return
    if model.status == GRB.Status.OPTIMAL:
        # Return optimal solution
        lp_var = [x[i].x for i in range(n_var)] # solution
        lp_obj = obj.getValue() # objective
        lp_flg = 1 # feasible
        # Return dual variables
        constrs = model.getConstrs() # get constraints
        lp_dul  = [constrs[i].pi for i in range(np.size(A,0))] # dual variable
    else:
        # Return feasibility solution
        n_eye = np.size(A,0) # number of new-added variables (eye matrix)
        c  = np.append(np.zeros(n_var), np.ones(n_eye), axis = 0)
        A  = np.append(A,  np.eye(n_eye), axis = 1)
        lb = np.append(lb, np.zeros(n_eye))
        ub = np.append(ub, np.ones(n_eye) * float("inf"))
        [lp_var,lp_obj,_,lp_dul] = Linprog(c,d,A,B,lb,ub)
        lp_flg = -1
    return [lp_var,lp_obj,lp_flg,lp_dul]
