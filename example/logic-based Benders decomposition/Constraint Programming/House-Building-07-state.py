'''
Chapter 7. Using state functions: house building with state incompatibilities
'''
# This chapter describes how to use state functions to take into account 
# incompatible states as tasks finish.
# There are two workers, and each task requires either one of the two workers. 
# A subset of the tasks require that the house be clean, whereas other tasks 
# make the house dirty. A transition time is needed to change the state of the 
# house from dirty to clean.


import sys
import docplex.cp.utils_visu as visu
import matplotlib.pyplot as plt

from docplex.cp.model import *
from pylab import rcParams

# -----------------------------------------------------------------------------
# Prepare data
# -----------------------------------------------------------------------------

# In the related data, the data provided includes the number of houses 
# (NbHouses), the number of workers (NbWorkers), the names of the tasks 
# (TaskNames), the sizes of the tasks (Duration), the precedence relations 
# (Precedences), and the cleanliness state of each task (AllStates).

NbHouses = 5
NbWorkers = 2
AllStates = ["clean", "dirty"]

TaskNames = ["masonry",  "carpentry", "plumbing", "ceiling", "roofing",
             "painting", "windows",   "facade",   "garden",  "moving"]

Duration = [35, 15, 40, 15, 5, 10, 5, 10, 5, 5]

States = [("masonry","dirty"), ("carpentry","dirty"),("plumbing","clean"),
          ("ceiling","clean"), ("roofing",  "dirty"),("painting","clean"),
          ("windows","dirty")]

Precedences = [("masonry", "carpentry"),("masonry",   "plumbing"),
               ("masonry", "ceiling"),  ("carpentry", "roofing"),
               ("ceiling", "painting"), ("roofing",   "windows"),
               ("roofing", "facade"),   ("plumbing",  "facade"),
               ("roofing", "garden"),   ("plumbing",  "garden"),
               ("windows", "moving"),   ("facade",    "moving"),
               ("garden",  "moving"),   ("painting",  "moving")]

Houses = range(NbHouses)

# -----------------------------------------------------------------------------
# Build the model
# -----------------------------------------------------------------------------

# 1. Creation of the model
mdl = CpoModel()

# 2. Declarations of decision variables

# Create the interval variables

# Each house has a list of tasks that must be scheduled. The duration, or size, 
# of each task t is Duration[t]. Using this information, a matrix task of 
# interval variables can be built.
task = {}
for h in Houses:
    for i,t in enumerate(TaskNames):
        task[(h,t)] = mdl.interval_var(size = Duration[i])

# Declare the worker usage functions

workers = step_at(0, 0)
for h in Houses:
    for t in TaskNames:
        workers += mdl.pulse(task[h,t], 1)

# Create the transition times

# The transition time from a dirty state to a clean state is the same for all 
# houses. As in the example Chapter 3, “Adding workers and transition times to 
# the house building problem”, a tupleset ttime is created to represent the 
# transition time between cleanliness states.

Index = {s : i for i,s in enumerate(AllStates)}

ttime = CpoTransitionMatrix(name = 'TTime', size = 2)
ttime.set_value(Index["dirty"], Index["clean"], 1)
ttime.set_value(Index["clean"], Index["dirty"], 0)

# Declare the state function

# Certain tasks require the house to be clean, and other tasks cause the house 
# to be dirty. To model the possible states of the house, the state_function()
# function is used to represent the disjoint states through time.
# 
# A state function is a function describing the evolution of a given feature of 
# the environment. The possible evolution of this feature is constrained by 
# interval variables of the problem. For example, a scheduling problem may 
# contain a resource whose state changes over time. The resource state can 
# change because of scheduled activities or because of exogenous events; some 
# activities in the schedule may need a particular resource state in order to 
# execute. Interval variables have an absolute effect on a state function, 
# requiring the function value to be equal to a particular state or in a set of 
# possible states.

state = { h : state_function(ttime, name = "house" + str(h)) for h in Houses}

# 3. Adding the constraints

# To model the state required or imposed by a task, a constraint is created to 
# specifies the state of the house throughout the interval variable 
# representing that task.
# 
# The constraint always_equal(), specifies the value of a state function over 
# the interval variable. The constraint takes as parameters a state function, 
# an interval variable, and a state value. Whenever the interval variable is 
# present, the state function is defined everywhere between the start and the 
# end of the interval variable and remains equal to the specified state value 
# over this interval. The state function is constrained to take the appropriate 
# values during the tasks that require the house to be in a specific state. To 
# add the constraint that there can be only two workers working at a given 
# time, the cumulative function expression representing worker usage is 
# constrained to not be greater than the value NbWorkers.

for h in Houses:
    for p in Precedences:
        mdl.add(mdl.end_before_start(task[h,p[0]], task[h,p[1]]))
    for s in States:
        mdl.add(mdl.always_equal(state[h], task[h,s[0]], Index[s[1]]))

mdl.add( workers <= NbWorkers )

# 4. Add the objective

# The objective of this problem is to minimize the overall completion date (the 
# completion date of the house that is completed last).
mdl.add(
    mdl.minimize(mdl.max(mdl.end_of(task[h,"moving"]) for h in Houses))
)

# -----------------------------------------------------------------------------
# Solve the model and display the result
# -----------------------------------------------------------------------------

# 1. Calling the solve
print("\nSolving model....")
msol = mdl.solve(url = None, key = None, FailLimit = 30000)
print("done")

# 2. Displaying the objective and solution
print("Cost will be "+str(msol.get_objective_values()[0]))

# 3. Viewing the results of sequencing problems in a Gantt chart
rcParams['figure.figsize'] = 15, 3

workers_function = CpoStepFunction()
for h in Houses:
    for t in TaskNames:
        itv = msol.get_var_solution(task[h,t])
        workers_function.add_value(itv.get_start(), itv.get_end(), 1)

visu.timeline('Solution SchedState')
visu.panel(name="Schedule")
for h in Houses:
    for t in TaskNames:
        visu.interval(msol.get_var_solution(task[h,t]), h, t)
    
visu.panel(name="Houses state")
for h in Houses:
    f = state[h]
    visu.sequence(name=f.get_name(), segments=msol.get_var_solution(f))
visu.panel(name="Nb of workers")
visu.function(segments=workers_function, style='line')
visu.show()

