#!/usr/bin/python

# Copyright 2019, Southeast University, Liu Pengxiang
#
# Modified Branch-and-Bound Algorithm
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


# Import packages
import sys
import math
import numpy as np
# Import Gurobi solver
from gurobipy import *


# Main function of Branch-and-Bound algorithm
def BranchBound(c,d,A,B,lb,ub,var_in,obj_in):
    # Global variables
    global var
    global obj
    var = var_in
    obj = obj_in
    # Solve the relaxed linear programming
    [lp_var,lp_obj,lp_flg,lp_dul] = Linprog(c,d,A,B,lb,ub)
    

# Solve the relaxed linear programming
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
        c  = np.append(np.zeros(n_var), np.ones(n_eye))
        A  = np.append(A,  np.eye(n_eye), axis = 1)
        lb = np.append(lb, np.zeros(n_eye))
        ub = np.append(ub, np.ones(n_eye) * 1e6)
        [lp_var,lp_obj,lp_flg,lp_dul] = Linprog(c,d,A,B,lb,ub)
        lp_flg = -1
    return lp_var,lp_obj,lp_flg,lp_dul


c  = np.array([4,2,5,1,1,1])
d  = np.array([0])
A  = np.array([[2,1,0,0,2,1],[0,0,3,2,0,1]])
B  = np.array([5,4])
lb = np.array([0,0,0,0,0,0])
ub = np.array([1,1,1,1,1,1])

[_,obj,_,_] = Linprog(c,d,A,B,lb,ub)

print(obj)