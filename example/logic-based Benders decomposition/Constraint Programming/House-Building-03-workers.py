'''
Chapter 3. Adding workers and transition times to the house building problem
'''
# This chapter introduces workers and transition times to the house building 
# problem described in the previous chapters.
#
# The problem to be solved is the scheduling of tasks involved in building 
# multiple houses in a manner that minimizes the costs associated with 
# completing each house after a given due date and with the length of time it 
# takes to build each house. Some tasks must necessarily take place before 
# other tasks, and each task has a predefined duration. Each house has an 
# earliest starting date. Moreover, there are two workers, each of whom must 
# perform a given subset of the necessary tasks, and there is a transition 
# time associated # with a worker transferring from one house to another house.
# A task, once started, cannot be interrupted.
# 
# The objective is to minimize the cost, which is composed of tardiness costs
# for # certain tasks as well as a cost associated with the length of time it 
# takes to complete each house.


import sys
import docplex.cp.utils_visu as visu
import matplotlib.pyplot as plt

from docplex.cp.model import *
from pylab import rcParams

# -----------------------------------------------------------------------------
# Prepare data
# -----------------------------------------------------------------------------

NbHouses = 5

WorkerNames = ["Joe", "Jim"]

TaskNames = ["masonry",  "carpentry", "plumbing", "ceiling", "roofing", 
             "painting", "windows",   "facade",   "garden",  "moving"]

Duration =  [35, 15, 40, 15, 5, 10, 5, 10, 5, 5]

Worker = {"masonry"  : "Joe" , 
          "carpentry": "Joe" , 
          "plumbing" : "Jim" , 
          "ceiling"  : "Jim" , 
          "roofing"  : "Joe" , 
          "painting" : "Jim" , 
          "windows"  : "Jim" , 
          "facade"   : "Joe" , 
          "garden"   : "Joe" , 
          "moving"   : "Jim"}

ReleaseDate = [  0,     0,   151,    59,   243]
DueDate     = [120,   212,   304,   181,   425]
Weight      = [100.0, 100.0, 100.0, 200.0, 100.0]

Precedences = [("masonry", "carpentry"),("masonry",   "plumbing"),
               ("masonry", "ceiling"),  ("carpentry", "roofing"),
               ("ceiling", "painting"), ("roofing",   "windows"),  
               ("roofing", "facade"),   ("plumbing",  "facade"),
               ("roofing", "garden"),   ("plumbing",  "garden"),
               ("windows", "moving"),   ("facade",    "moving"),  
               ("garden",  "moving"),   ("painting",  "moving")]

Houses = range(NbHouses)

# Create the transition times

# An optional transition matrix M (in the form of a non-negative integer tuple 
# set) can be passed to the noOverlap constraint meaning that if ai appears 
# before aj in the sequence, then a minimal distance M[typei,typej] must be 
# respected between the end of ai and the start of aj (typei and typej denote 
# the types of ai and aj in the sequence).
transitionTimes = transition_matrix(NbHouses)
for i in Houses:
    for j in Houses:
        transitionTimes.set_value(i, j, int(abs(i - j)))

# -----------------------------------------------------------------------------
# Build the model
# -----------------------------------------------------------------------------

# 1. Creation of the model
mdl = CpoModel()

# 2. Declarations of decision variables

# One part of the objective is based on the time it takes to build a house. To 
# model this, one interval variable is used for each house, and is later 
# constrained to span the tasks associated with the given house. As each house 
# has an earliest starting date, and each house interval variable is declared 
# to have a start date no earlier than that release date. The ending date of 
# the task is not constrained, so the upper value of the range for the variable 
# is maxint.

# Create the house interval variables
houses = [mdl.interval_var(start=(ReleaseDate[i], INTERVAL_MAX), 
                           name="house"+str(i))
          for i in Houses]

# Create the task interval variables
TaskNames_ids = {}
itvs = {}
for h in Houses:
    for i,t in enumerate(TaskNames):
        _name = str(h)+"_"+str(t)
        itvs[(h,t)] = mdl.interval_var(size=Duration[i], name=_name)
        TaskNames_ids[_name] = i

# Create the sequence variables

# A sequence variable represents the order in which the workers perform the 
# tasks

# Using the decision variable type sequence, variable can be created to 
# represent a sequence of interval variables. The sequence can contain a subset 
# of the variables or be empty. In a solution, the sequence will represent a 
# total order over all the intervals in the set that are present in the 
# solution. The assigned order of interval variables in the sequence does not 
# necessarily determine their relative positions in time in the schedule. The 
# sequence variable takes an array of interval variables as well as the 
# transition types for each of those variables. Interval sequence variables are 
# created for Jim and Joe, using the arrays of their tasks and the task 
# locations.
workers = {w : mdl.sequence_var([itvs[(h,t)] for h in Houses for t in TaskNames 
                                    if Worker[t]==w ], 
                                types = [h for h in Houses for t in TaskNames 
                                    if Worker[t]==w ], 
                                name  = "workers_"+w)
            for w in WorkerNames}

# 3. Adding the constraints

# Add the precedence constraints
for h in Houses:
    for p in Precedences:
        mdl.add(mdl.end_before_start(itvs[(h,p[0])], itvs[(h,p[1])]))

# Add the span constraints

# To model the cost associated with the length of time it takes to build a 
# single house, the interval variable associated with the house is constrained 
# to start at the start of the first task of the house and end at the end of 
# the last task. This interval variable must span the tasks.
for h in Houses:
    mdl.add(mdl.span(houses[h], [itvs[(h,t)] for t in TaskNames]))

# Add the no overlap constraint

# Each sequence must be constrained such that the interval variables do not 
# overlap in the solution, that the transition times are respected, and that 
# the sequence represents the relations of the interval variables in time.
# 
# The constraint no_overlap allows to constrain an interval sequence variable 
# to define a chain of non-overlapping intervals that are present in the 
# solution. If a set of transition tuples is specified, it defines the minimal 
# time that must elapse between two intervals in the chain. Note that intervals 
# which are not present in the solution are automatically removed from the 
# sequence. One no overlap constraint is created for the sequence interval 
# variable for each worker.

for w in WorkerNames:
    mdl.add(mdl.no_overlap(workers[w], transitionTimes))

# 4. Add the objective

# The cost for building a house is the sum of the tardiness cost and the number 
# of days it takes from start to finish building the house. To model the cost 
# associated with a task being completed later than its preferred latest end 
# date, the expression endOf() can be used to determine the end date of the 
# house interval variable. To model the cost of the length of time it takes to 
# build the house, the expression lengthOf() can be used, which returns an 
# expression representing the length of an interval variable.
#
# The objective of this problem is to minimize the cost as represented by the 
# cost expression.
mdl.add( 
    mdl.minimize( 
        mdl.sum(Weight[h] * mdl.max([0, mdl.end_of(houses[h])-DueDate[h]]) + 
                mdl.length_of(houses[h]) 
            for h in Houses) 
    ) 
)

# -----------------------------------------------------------------------------
# Solve the model and display the result
# -----------------------------------------------------------------------------

# 1. Calling the solve
print("\nSolving model....")
msol = mdl.solve(url=None, key=None, FailLimit=30000)
print("done")

# 2. Displaying the objective and solution
print("Cost will be " + str(msol.get_objective_values()[0]))

# 3. Viewing the results of sequencing problems in a Gantt chart
rcParams['figure.figsize'] = 15, 3

def showsequence(msol, s, setup, tp):
    seq = msol.get_var_solution(s)
    visu.sequence(name=s.get_name())
    vs = seq.get_value()
    for v in vs:
        nm = v.get_name()
        visu.interval(v, tp[TaskNames_ids[nm]], nm)
    for i in range(len(vs) - 1):
        end = vs[i].get_end()
        tp1 = tp[TaskNames_ids[vs[i].get_name()]]
        tp2 = tp[TaskNames_ids[vs[i + 1].get_name()]]
        visu.transition(end, end + setup.get_value(tp1, tp2))

visu.timeline("Solution for SchedSetup")
for w in WorkerNames:
    types=[h for h in Houses for t in TaskNames if Worker[t]==w]
    showsequence(msol, workers[w], transitionTimes, types)
visu.show()