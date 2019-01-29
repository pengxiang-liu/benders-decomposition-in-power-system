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


class ResultMaster(object):
    def __init__(self,model,y,obj_con,obj_opr):
        self.y  = np.array([int(y [i].x) for i in range(len(d))])
        self.obj_con = obj_con.getValue()
        self.obj_opr = obj_opr.x
        self.obj = (model.getObjective()).getValue()  # objective


# This class restores the results of reconfiguration sub-problem
class ResultBranchBound(object):
    def __init__(self,var_out,obj_out,flg_out,dual,flag,j0,j1,fit):
        # Final solution
        self.var = var_out  # variable
        self.obj = obj_out  # objective
        self.flg = flg_out  # optimality status
        # Dual information
        self.d_var = dual   # dual variables
        self.d_flg = flag   # flag
        self.d_j0  = j0     # j0
        self.d_j1  = j1     # j1
        self.d_fit = fit    # fitness


# This program solves 0-1 programming based on a classical Branch-and-Bound algorithm
# Dual information from the relaxed linear programming at each leaf node is returned
# 
def BranchBound(c,e,A,rhs,lb,ub,br = 0):
    # 
    # The model has the following format:
    # 
    # minimize
    #       c * x + e
    # subject to 
    #       A * x >= rhs   (u)
    #           x >= lb    (v)
    #           x <= ub    (v)
    # where x is binary varibale, dual and v are dual variables
    #
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
    [lp_var,lp_obj,lp_flg,lp_dul] = Linprog(c,e,A,rhs,lb,ub)
    # Update global variabes for the current node in the search tree
    dual.append(lp_dul)
    flag.append(lp_flg)
    fit. append(lp_obj)
    j0.  append(np.where(lb + ub == 0))  # lb == 0 and ub == 0
    j1.  append(np.where(lb + ub == 2))  # lb == 1 and ub == 1
    # Branching
    if lp_flg != 1:  # if problem is infeasible
        var_out = lp_var
        obj_out = lp_obj + 1e2
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
                lb_temp = np.copy(lb)  # temporary lower bound
                ub_temp = np.copy(ub)  # temporary upper bound
                # The upper branch calculation
                if ub[pick] >= np.floor(lp_var[pick]) + 1:
                    lb_temp[pick] = np.floor(lp_var[pick]) + 1  # branching
                    BranchBound(c,e,A,rhs,lb_temp,ub,1)
                # The lower branch calculation
                if lb[pick] <= np.floor(lp_var[pick]):
                    ub_temp[pick] = np.floor(lp_var[pick])  # branching
                    BranchBound(c,e,A,rhs,lb,ub_temp,1)
                # update output
                var_out = var
                obj_out = obj
                flg_out = 1
    result = ResultBranchBound(var_out,obj_out,flg_out,dual,flag,j0,j1,fit)
    # Return results
    return result


# This program solves a simple linear programming by Gurobi 8.1.0
# 
def Linprog(c,e,A,rhs,lb,ub):
    # 
    # minimize
    #       c * x + e
    # subject to 
    #       A * x >= rhs   (u)
    #           x >= lb    (v)
    #           x <= ub    (v)
    # where dual and v are dual variables
    #
    model = Model()
    # Create variables
    n_var = np.size(A,1) # number of x
    x = model.addVars(n_var)
    # Set objective
    obj = quicksum(c[i] * x[i] for i in range(n_var)) + e
    model.setObjective(obj, GRB.MINIMIZE)
    # Add constraints
    for i in range(np.size(A,0)):
        model.addConstr(quicksum(A[i,j] * x[j] for j in range(n_var)) >= rhs[i])
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
        [lp_var,lp_obj,_,lp_dul] = Linprog(c,e,A,rhs,lb,ub)
        lp_flg = -1
    # List to array
    lp_var = np.array(lp_var)
    lp_obj = np.array(lp_obj)
    lp_flg = np.array(lp_flg)
    lp_dul = np.array(lp_dul)
    return [lp_var,lp_obj,lp_flg,lp_dul]


# Form optimality cut
def OptimalityInequality(c,e,A,B,a,dual,beta,j0,j1,y):
    # The following formulation is based on expression (35) in the paper
    temp_uB = dual.dot(B) # dual*B
    temp_ua = dual.dot(a) # dual*a
    temp_bt = beta # beta
    temp_sum_0 = 0 # initialize the first sum term
    temp_sum_1 = 0 # initialize the second sum term
    for i in range(len(c)):
        if i in j1: # if i belongs to J1
            temp_sum_0 = temp_sum_0 + dual.dot(A[:,i]) - c[i]
        if (i not in j0) and (i not in j1): # if i doesn't belong to J0 and J1
            if dual.dot(A[:,i]) - c[i] >= 0:
                temp_sum_1 = temp_sum_1 + dual.dot(A[:,i]) - c[i]
            else:
                temp_sum_1 = temp_sum_1 + 0
    # Formulate expression
    expr = LinExpr()
    for i in range(len(y)):
        expr = expr + temp_uB[i] * y[i]
    expr = expr - temp_ua + temp_bt - e + temp_sum_0 + temp_sum_1
    # Return
    return expr


# Form feasibility cut
def FeasibilityInequality(c,e,A,B,a,dual,beta,j0,j1,y):
    # The following formulation is based on expression (35) in the paper
    temp_uB = dual.dot(B) # dual*B
    temp_ua = dual.dot(a) # dual*a
    temp_sum_0 = 0 # initialize the first sum term
    temp_sum_1 = 0 # initialize the second sum term
    for i in range(len(y)):
        if i in j1: # if i belongs to J1
            temp_sum_0 = temp_sum_0 + dual.dot(A[:,i])
        if (i not in j0) and (i not in j1): # if i doesn't belong to J0 and J1
            if dual.dot(A[:,i]) >= 0:
                temp_sum_1 = temp_sum_1 + dual.dot(A[:,i])
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
def MasterProblem(d,logic,n_iter):
    # Create a gurobi model
    model = Model()
    y = model.addVars(len(d), vtype = GRB.BINARY)  # Create variables
    obj_opr = model.addVar()  # Create variables
    obj_con = quicksum(d[i] * y[i] for i in range(len(d))) # objective of construction

    # Add constraints
    if n_iter == 0:
        model.addConstr(obj_opr == 0)
    else:
        for i in range(n_iter):
            dual = logic[i].d_var  # dual variables
            flag = logic[i].d_flg  # status flag
            beta = logic[i].d_fit  # objective at leaf node
            j0   = logic[i].d_j0   # set of all j where branching has set xj to 0
            j1   = logic[i].d_j1   # set of all j where branching has set xj to 1
            # Inequality initialization
            n_logic = np.size(flag,0)  # number of new-added logic variables
            logic_var = model.addVars(n_logic, vtype = GRB.BINARY) # logic variable
            logic_and = model.addVar(vtype = GRB.BINARY) # logic AND variable
            # Formulate Inequality
            Inequality = []
            for t in range(n_logic):
                if flag[t] == 1: # optimality inequality
                    expr = OptimalityInequality(c,e,A,B,a,dual[t],beta[t],j0[t][0],j1[t][0],y)
                    Inequality.append(expr)
                    model.addConstr((logic_var[t] == 1) >> (expr <= 0.001))
                    model.addConstr((logic_var[t] == 0) >> (expr >= 0.001))
                else: # feasibility inequality
                    expr = FeasibilityInequality(c,e,A,B,a,dual[t],beta[t],j0[t][0],j1[t][0],y)
                    Inequality.append(expr)
                    model.addConstr((logic_var[t] == 1) >> (expr <= -0.001))
                    model.addConstr((logic_var[t] == 0) >> (expr >= -0.001))
            # Add Benders cut
            model.addConstr(logic_and == and_(logic_var)) # logic and
            sum_cj = 0 # sum of cj
            for t in range(len(c)):
                if c[t] > 0:
                    sum_cj = sum_cj + 0
                else:
                    sum_cj = sum_cj + c[t]
            model.addConstr(obj_opr >= sum_cj + (logic[i].obj-sum_cj) * logic_and)
    # Set objective
    obj = obj_con + obj_opr
    model.setObjective(obj,GRB.MINIMIZE)
    # Solve
    model.optimize()
    # Return
    if model.status == GRB.Status.OPTIMAL:
        result = ResultMaster(model,y,obj_con,obj_opr)
    else:
        os.system("pause")
    return result


# Sub problem
def WorkerProblem(c,e,A,B,a,lb,ub,Master):
    lb = np.array([0,0,0,0])
    ub = np.array([1,1,1,1])
    rhs = a - np.inner(B,Master.y)
    result = BranchBound(c,e,A,rhs,lb,ub)
    return result


# Main function

# Set coeficient matrix
# min  c*x + d*y
# s.t. A*x + B*y >= a
#      x,y is {0,1}
c  = np.array([4,2,5,1])
d  = np.array([1,1])
e  = np.array([10])
A  = np.array([[2,1,0,0],[0,0,3,2]])
B  = np.array([[2,1],[0,1]])
a  = np.array([5,4])
lb = np.array([0,0,0,0])
ub = np.array([1,1,1,1])

# Benders decomposition
logic = []
lower_bound = []
upper_bound = []
n_iter = 0 # index of iteration
while True:
    Master = MasterProblem(d,logic,n_iter)
    Worker = WorkerProblem(c,e,A,B,a,lb,ub,Master)
    # logic benders
    logic.append(Worker)
    lower_bound.append(Master.obj)
    upper_bound.append(Master.obj_con + Worker.obj)
    # Append each variable for Benders decomposition information set
    gap = (upper_bound[n_iter]-lower_bound[n_iter])/upper_bound[n_iter]
    if gap <= 1e-5 or n_iter > 20:
        break
    else:
        n_iter = n_iter + 1

# Output
print('')
print('Result:')
print('Var: %s' % str(np.r_[Worker.var,Master.y]))
print('Obj: %s' % str(upper_bound[-1]))
print('')

# Plot
plt.plot(lower_bound)
plt.plot(upper_bound)
plt.show()
