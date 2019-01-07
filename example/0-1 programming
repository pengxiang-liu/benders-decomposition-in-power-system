#!/usr/bin/python

# Copyright 2019, Southeast University, Liu Pengxiang
#
# Logic-based benders decomposition (DEMO)
#
# This program solves a simple 0-1 problem where both master problem
# and sub-problem are integer programming
#
# The detailed algorithm can be found in paper: "Logic-based Benders
# decomposition ---- J.N.Hooker(2003)" p46 - p51

import os
import sys
import math
import numpy as np
from gurobipy import *
import matplotlib.pyplot as plt


# Solve a relax linear programming problem
# min c*x    s.t. (A*x >= B, lb <= x <= ub)
def Linprog(y,c,A,B,a,lb,ub):
    # Create a gurobi model
    model = Model()
    # Create variables
    num_x = np.size(A,1) # number of x
    x = model.addVars(num_x)
    # Set objective
    obj = LinExpr()
    for i in range(num_x):
        obj = obj + c[i] * x[i]
    model.setObjective(obj,GRB.MINIMIZE)
    # Add constraints
    for i in range(np.size(A,0)):
        expr = LinExpr() # expression
        for j in range(num_x):
            expr = expr + A[i][j] * x[j]
        rhs = a[i] - B[i].dot(y.T) # right hand side
        model.addConstr(expr >= rhs)
    for i in range(num_x):
        model.addConstr(x[i] >= lb[i])
        model.addConstr(x[i] <= ub[i])
    # Solve
    model.optimize()
    # Return
    if model.status == GRB.Status.OPTIMAL:
        # Return optimal solution
        res_var = [x[i].x for i in range(num_x)] # solution
        res_fit = [obj.getValue()] # objective
        res_flg = [1] # feasible
        constrs = model.getConstrs() # get constraints
        res_dul  = [constrs[i].pi for i in range(np.size(A,0))] # dual variable
    else:
        # Return feasibility solution
        num_e = np.size(A,0) # number of new-added variables
        c  = np.append(np.zeros(num_x), np.ones(num_e))
        A  = np.append(A,  np.eye(num_e), axis = 1)
        lb = np.append(lb, np.zeros(num_e))
        ub = np.append(ub, np.ones(num_e)*100)
        [res_var,res_fit,_,res_dul] = Linprog(y,c,A,B,a,lb,ub)
        res_flg = [-1]
    return [res_var,res_fit,res_flg,res_dul]


# Branch and bound
def BranchAndBound(y,c,A,B,a,lb,ub,OptVar,OptFit,dual,flag,beta,J0,J1):
    # Global variables
    global Global_Var # incumbent solution
    global Global_Fit # incumbent fitness
    global Flag
    Global_Var = OptVar
    Global_Fit = OptFit
    # Solve the relax linear programming
    [res_var,res_fit,res_flg,res_dul] = Linprog(y,c,A,B,a,lb,ub)
    # Update Benders cut information
    dual.extend(res_dul)
    flag.extend(res_flg)
    beta.extend(res_fit)
    for i in range(np.size(A,1)):
        if lb[i] != ub[i]:
            J0.extend([0])
            J1.extend([0])
        else:
            if lb[i] == 0 and ub[i] == 0:
                J0.extend([1]) # j where branching has set xj to 0
                J1.extend([0])
            if lb[i] == 1 and ub[i] == 1:
                J0.extend([0])
                J1.extend([1]) # j where branching has set xj to 1
    # No optimal solution
    if res_flg[0] != 1:
        Output_Var = res_var
        Output_Fit = [100]
        Output_Flg = -1
    else:
        # The following programs are based on the condition that flag = 1
        res_var = np.array(res_var)
        if max(np.abs(res_var - np.rint(res_var))) == 0: # integer solution
            if res_fit > Global_Fit: # can't find any integer solution better than the "Global_Fit"
                Output_Var = res_var
                Output_Fit = res_fit
                Output_Flg = -2
            else: # update the current optimal solution
                Global_Var = res_var
                Global_Fit = res_fit
                Flag = 1
                Output_Var = res_var
                Output_Fit = res_fit
                Output_Flg =  1
        else: # real solution
            if res_fit > Global_Fit: # can't find any real solution better than the "Global_Fit"
                Output_Var = res_var
                Output_Fit = res_fit
                Output_Flg = -2
            else: # keep branching
                gap_var = np.abs(res_var - np.rint(res_var))
                index_real = np.where(gap_var > 0) # find the leaf node
                pick = int(index_real[0][0]) # pick up the first one
                lb_temp = lb
                ub_temp = ub
                # The up branch calculation
                if ub[pick] >= np.floor(res_var[pick]) + 1:
                    lb_temp[pick] = np.floor(res_var[pick]) + 1
                    BranchAndBound(y,c,A,B,a,lb_temp,ub,OptVar,OptFit,dual,flag,beta,J0,J1)
                # The down branch calculation
                if lb[pick] <= np.floor(res_var[pick]):
                    ub_temp[pick] = np.floor(res_var[pick])
                    BranchAndBound(y,c,A,B,a,lb,ub_temp,OptVar,OptFit,dual,flag,beta,J0,J1)
                Output_Var = Global_Var
                Output_Fit = Global_Fit
                Output_Flg = Flag
    return [Output_Var,Output_Fit,Output_Flg]


# Form optimality cut
def OptimalityInequality(c,A,B,a,u,b,j0,j1,y):
    # The following formulation is based on expression (35) in the paper
    temp_uB = u.dot(B) # u*B
    temp_ua = u.dot(a) # u*a
    temp_bt = b # beta
    temp_sum_0 = 0 # initialize the first sum term
    temp_sum_1 = 0 # initialize the second sum term
    for i in range(len(j1)):
        if j1[i] == 1: # if i belongs to J1
            temp_sum_0 = temp_sum_0 + u.dot(A[:,i]) - c[i]
        if j1[i] == 0 and j0[i] == 0: # if i doesn't belong to J0 and J1
            if u.dot(A[:,i]) - c[i] >= 0:
                temp_sum_1 = temp_sum_1 + u.dot(A[:,i]) - c[i]
            else:
                temp_sum_1 = temp_sum_1 + 0
    # Formulate expression
    expr = LinExpr()
    for i in range(len(y)):
        expr = expr + temp_uB[i] * y[i]
    expr = expr - temp_ua + temp_bt + temp_sum_0 + temp_sum_1
    # Return
    return expr


# Form feasibility cut
def FeasibilityInequality(A,B,a,u,j0,j1,y):
    # The following formulation is based on expression (35) in the paper
    temp_uB = u.dot(B) # u*B
    temp_ua = u.dot(a) # u*a
    temp_sum_0 = 0 # initialize the first sum term
    temp_sum_1 = 0 # initialize the second sum term
    for i in range(len(j1)):
        if j1[i] == 1: # if i belongs to J1
            temp_sum_0 = temp_sum_0 + u.dot(A[:,i])
        if j1[i] == 0 and j0[i] == 0: # if i doesn't belong to J0 and J1
            if u.dot(A[:,i]) >= 0:
                temp_sum_1 = temp_sum_1 + u.dot(A[:,i])
            else:
                temp_sum_1 = temp_sum_1 + 0
    # Formulate expression
    expr = LinExpr()
    for i in range(len(y)):
        expr = expr + temp_uB[i] * y[i]
    expr = expr - temp_ua + temp_sum_0 + temp_sum_1
    # Return
    return expr


# Master problem
def MasterProblem(c,d,A,B,a,lb,ub,bd_dual,bd_flag,bd_beta,bd_J0,bd_J1,bd_obj,itr):
    # Create a gurobi model
    model = Model()
    y = model.addVars(len(d), vtype = GRB.BINARY) # Create variables
    # Add constraints
    if itr > 0:
        Alpha = model.addVar()
        for i in range(itr):
            # Reshape for each iteration
            ru  = (np.array(bd_dual[i])).reshape((-1,2))
            rf  = (np.array(bd_flag[i])).reshape((-1,1))
            rb  = (np.array(bd_beta[i])).reshape((-1,1))
            rj0 = (np.array(bd_J0[i])).reshape((-1,np.size(A,1)))
            rj1 = (np.array(bd_J1[i])).reshape((-1,np.size(A,1)))
            rob = (np.array(bd_obj[i])).reshape((-1,1))
            # Inequality initialization
            n_newvar = len(rf) # number of new added logic variable
            logic_var = model.addVars(n_newvar, vtype = GRB.BINARY) # logic variable
            logic_and = model.addVar(vtype = GRB.BINARY) # logic AND variable
            # Formulate Inequality
            Inequality = []
            for t in range(n_newvar):
                if rf[t] == 1: # optimality inequality
                    expr = OptimalityInequality(c,A,B,a,ru[t],rb[t],rj0[t],rj1[t],y)
                    Inequality.append(expr)
                    model.addConstr((logic_var[t] == 1) >> (expr <= 0.001))
                    model.addConstr((logic_var[t] == 0) >> (expr >= 0.001))
                else: # feasibility inequality
                    expr = FeasibilityInequality(A,B,a,ru[t],rj0[t],rj1[t],y)
                    Inequality.append(expr)
                    model.addConstr((logic_var[t] == 1) >> (expr <= -0.001))
                    model.addConstr((logic_var[t] == 0) >> (expr >= -0.001))
            # Add Benders cut
            model.addConstr(logic_and == and_(logic_var)) # logic and
            sum_cj = 0 # sum of cj
            for t in range(len(c)):
                if c[i] > 0:
                    sum_cj = sum_cj + 0
                else:
                    sum_cj = sum_cj + c[i]
            model.addConstr(Alpha >= sum_cj + (rob[0][0]-sum_cj) * logic_and)
    # Set objective
    obj = LinExpr()
    for i in range(len(d)):
        obj = obj + d[i] * y[i]
    if itr > 0:
        obj = obj + Alpha
    model.setObjective(obj,GRB.MINIMIZE)
    # Solve
    model.optimize()
    # Return
    if model.status == GRB.Status.OPTIMAL:
        res_var = [y[i].x for i in range(len(d))]
        res_obj = obj.getValue()
        if itr > 0:
            res_Alpha = Alpha.x
        else:
            res_Alpha = 0
    else:
        os.system("pause")
    return [res_var,res_obj,res_Alpha]


# Sub problem
def SubProblem(y,c,A,B,a,lb,ub):
    # Initialization
    OptVar = np.array([0,0,0,0])
    OptFit = np.array([100])
    y = np.array([y]) # list to array
    # y = np.array([1,1]) # list to array
    # Global variables for each leaf node
    global dual
    global flag
    global beta
    global J0
    global J1
    dual = [] # dual variables
    flag = [] # status flag
    beta = [] # objective at leaf node
    J0   = [] # set of all j where branching has set xj to 0
    J1   = [] # set of all j where branching has set xj to 1
    # Use branch-and-bound to solve sub-problem
    c  = np.array([4,2,5,1])
    d  = np.array([1,1])
    A  = np.array([[2,1,0,0],[0,0,3,2]])
    B  = np.array([[2,1],[0,1]])
    a  = np.array([5,4])
    lb = np.array([0,0,0,0])
    ub = np.array([1,1,1,1])
    [Var,Fit,Flg] = BranchAndBound(y,c,A,B,a,lb,ub,OptVar,OptFit,dual,flag,beta,J0,J1)
    return [dual,flag,beta,J0,J1,Var,Fit,Flg]


# Main function
try:
    # Set coeficient matrix
    # min  c*x + d*y
    # s.t. A*x + B*y >= a
    #      x,y is {0,1}
    c  = np.array([4,2,5,1])
    d  = np.array([1,1])
    A  = np.array([[2,1,0,0],[0,0,3,2]])
    B  = np.array([[2,1],[0,1]])
    a  = np.array([5,4])
    lb = np.array([0,0,0,0])
    ub = np.array([1,1,1,1])

    # Benders cut coeficient initialization
    bd_dual = [] # dual variables
    bd_flag = [] # status flag
    bd_beta = [] # objective at leaf node
    bd_J0   = [] # set of all j where branching has set xj to 0
    bd_J1   = [] # set of all j where branching has set xj to 1
    bd_obj  = [] # set of objective

    # Benders decomposition
    n_iter = 20 # number of iteration
    lower_bound = []
    upper_bound = []
    i = 0 # index of iteration
    while True:
        [y_var,y_obj,Alpha] = MasterProblem(c,d,A,B,a,lb,ub,bd_dual,bd_flag,bd_beta,bd_J0,bd_J1,bd_obj,i)
        lower_bound.append(y_obj)
        [dual,flag,beta,J0,J1,x_var,x_obj,flg] = SubProblem(y_var,c,A,B,a,lb,ub)
        upper_bound.append(y_obj - Alpha + x_obj[0])
        # Append each variable for Benders decomposition information set
        # Each row represents the corresponding information for each iteration
        # Note that this part can be further simplified by "lazy constraint callback"
        bd_dual.append(dual) # set of dual variables
        bd_flag.append(flag) # set of status flag
        bd_beta.append(beta) # set of objective at leaf node
        bd_J0.append(J0) # set of all j where branching has set xj to 0
        bd_J1.append(J1) # set of all j where branching has set xj to 1
        bd_obj.append(x_obj) # set of objective
        gap = (upper_bound[i]-lower_bound[i])/upper_bound[i]
        if gap <= 1e-5:
            break
        else:
            i = i + 1
    
    # Output
    print('')
    print('Result:')
    print('Var: %s' % str(np.r_[x_var,y_var]))
    print('Obj: %s' % str(upper_bound[i]))
    print('')

    # Plot
    plt.plot(lower_bound)
    plt.plot(upper_bound)
    plt.show()

except:
    print('Error')
