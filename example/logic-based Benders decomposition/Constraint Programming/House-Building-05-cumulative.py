'''
Chapter 5. Using cumulative functions in the house building problem
'''
# Some tasks must necessarily take place before other tasks, and each task has 
# a predefined duration. Moreover, there are three workers, and each task 
# requires any one of the three workers. A worker can be assigned to at most 
# one task at a time. In addition, there is a cash budget with a starting 
# balance. Each task consumes a certain amount of the budget at the start of 
# the task, and the cash balance is increased every 60 days.


import sys
import docplex.cp.utils_visu as visu
import matplotlib.pyplot as plt

from docplex.cp.model import *
from pylab import rcParams

# -----------------------------------------------------------------------------
# Prepare data
# -----------------------------------------------------------------------------

NbWorkers = 3
NbHouses  = 5

TaskNames = {"masonry","carpentry","plumbing",
             "ceiling","roofing","painting",
             "windows","facade","garden","moving"}

Duration =  [35, 15, 40, 15, 5, 10, 5, 10, 5, 5]

ReleaseDate = [31, 0, 90, 120, 90]

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
itvs = {}
for h in Houses:
    for i,t in enumerate(TaskNames):
        itvs[h,t] = mdl.interval_var(
            start = [ReleaseDate[h], INTERVAL_MAX], 
            size = Duration[i]
        )

# As the workers are equivalent in this problem, it is better to represent them 
# as one pool of workers instead of as individual workers with no overlap 
# constraints as was done in the earlier examples.

# 3. Adding the constraints

# Declare the worker usage function
workers_usage = step_at(0, 0)
for h in Houses:
    for t in TaskNames:
        workers_usage += mdl.pulse(itvs[h,t],1)

# Declare the cash budget function
cash = step_at(0, 0)
for p in Houses:
    cash += mdl.step_at(60*p, 30000)

for h in Houses:
    for i,t in enumerate(TaskNames):
        cash -= mdl.step_at_start(itvs[h,t], 200*Duration[i])

# Add the temporal constraints
for h in Houses:
    for p in Precedences:
        mdl.add( mdl.end_before_start(itvs[h,p[0]], itvs[h,p[1]]) )

# Add the worker usage constraint
mdl.add( workers_usage <= NbWorkers )

# Add the cash budget constraint
mdl.add( cash >= 0 )

# 4. Add the objective

# The objective of this problem is to minimize the overall completion date (the 
# completion date of the house that is completed last). The maximum completion 
# date among the individual house projects is determined using the expression 
# end_of() on the last task in building each house (here, it is the moving 
# task) and minimize the maximum of these expressions.

mdl.add(
    mdl.minimize( 
        mdl.max( mdl.end_of(itvs[h,"moving"]) for h in Houses)
    )
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

workersF = CpoStepFunction()
cashF = CpoStepFunction()
for p in range(5):
    cashF.add_value(60 * p, INT_MAX, 30000)
for h in Houses:
    for i,t in enumerate(TaskNames):
        itv = msol.get_var_solution(itvs[h,t])
        workersF.add_value(itv.get_start(), itv.get_end(), 1)
        cashF.add_value(itv.start, INT_MAX, -200 * Duration[i])

visu.timeline('Solution SchedCumul')
visu.panel(name="Schedule")
for h in Houses:
    for i,t in enumerate(TaskNames):
        visu.interval(msol.get_var_solution(itvs[h,t]), h, t)
visu.panel(name="Workers")
visu.function(segments=workersF, style='area')
visu.panel(name="Cash")
visu.function(segments=cashF, style='area', color='gold')
visu.show()