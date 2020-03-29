'''
Chapter 4. Adding calendars to the house building problem
'''
# This chapter introduces calendars into the house building problem, a problem 
# of scheduling the tasks involved in building multiple houses in such a manner 
# that minimizes the overall completion date of the houses.
# 
# There are two workers, each of whom must perform a given subset of the 
# necessary tasks. Each worker has a calendar detailing on which days he does 
# not work, such as weekends and holidays. On a worker’s day off, he does no 
# work on his tasks, and his tasks may not be scheduled to start or end on 
# these days. Tasks that are in process by the worker are suspended during his 
# days off.


import sys
import docplex.cp.utils_visu as visu
import matplotlib.pyplot as plt

from docplex.cp.model import *
from collections import namedtuple
from pylab import rcParams

# -----------------------------------------------------------------------------
# Prepare data
# -----------------------------------------------------------------------------

# declaration of the engine
NbHouses = 5

WorkerNames = ["Joe", "Jim" ]

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

Precedences = [("masonry", "carpentry"),("masonry",   "plumbing"),
               ("masonry", "ceiling"),  ("carpentry", "roofing"),
               ("ceiling", "painting"), ("roofing",   "windows"),  
               ("roofing", "facade"),   ("plumbing",  "facade"),
               ("roofing", "garden"),   ("plumbing",  "garden"),
               ("windows", "moving"),   ("facade",    "moving"),  
               ("garden",  "moving"),   ("painting",  "moving")]

Houses = range(NbHouses)

# Add the intensity step functions

Breaks ={
    "Joe" : [
        (  5, 14),( 19, 21),( 26, 28),( 33, 35),( 40, 42),( 47, 49),( 54, 56),
        ( 61, 63),( 68, 70),( 75, 77),( 82, 84),( 89, 91),( 96, 98),(103,105),
        (110,112),(117,119),(124,133),(138,140),(145,147),(152,154),(159,161),
        (166,168),(173,175),(180,182),(187,189),(194,196),(201,203),(208,210),
        (215,238),(243,245),(250,252),(257,259),(264,266),(271,273),(278,280),
        (285,287),(292,294),(299,301),(306,308),(313,315),(320,322),(327,329),
        (334,336),(341,343),(348,350),(355,357),(362,364),(369,378),(383,385),
        (390,392),(397,399),(404,406),(411,413),(418,420),(425,427),(432,434),
        (439,441),(446,448),(453,455),(460,462),(467,469),(474,476),(481,483),
        (488,490),(495,504),(509,511),(516,518),(523,525),(530,532),(537,539),
        (544,546),(551,553),(558,560),(565,567),(572,574),(579,602),(607,609),
        (614,616),(621,623),(628,630),(635,637),(642,644),(649,651),(656,658),
        (663,665),(670,672),(677,679),(684,686),(691,693),(698,700),(705,707),
        (712,714),(719,721),(726,728)
    ],
    "Jim" : [
        (  5,  7),( 12, 14),( 19, 21),( 26, 42),( 47, 49),( 54, 56),( 61, 63),
        ( 68, 70),( 75, 77),( 82, 84),( 89, 91),( 96, 98),(103,105),(110,112),
        (117,119),(124,126),(131,133),(138,140),(145,147),(152,154),(159,161),
        (166,168),(173,175),(180,182),(187,189),(194,196),(201,225),(229,231),
        (236,238),(243,245),(250,252),(257,259),(264,266),(271,273),(278,280),
        (285,287),(292,294),(299,301),(306,315),(320,322),(327,329),(334,336),
        (341,343),(348,350),(355,357),(362,364),(369,371),(376,378),(383,385),
        (390,392),(397,413),(418,420),(425,427),(432,434),(439,441),(446,448),
        (453,455),(460,462),(467,469),(474,476),(481,483),(488,490),(495,497),
        (502,504),(509,511),(516,518),(523,525),(530,532),(537,539),(544,546),
        (551,553),(558,560),(565,581),(586,588),(593,595),(600,602),(607,609),
        (614,616),(621,623),(628,630),(635,637),(642,644),(649,651),(656,658),
        (663,665),(670,672),(677,679),(684,686),(691,693),(698,700),(705,707),
        (712,714),(719,721),(726,728)
    ]
}

Break = namedtuple('Break', ['start', 'end'])

Calendar = {}
mymax = max(max(v for k,v in Breaks[w]) for w in WorkerNames)
for w in WorkerNames:
    step = CpoStepFunction()
    step.set_value(0, mymax, 100)  # (start, end, value)
    for b in Breaks[w]:
        t = Break(*b)
        step.set_value(t.start, t.end, 0)
    Calendar[w] = step

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
        _name = str(h) + "_" + str(t)
        itvs[(h,t)] = mdl.interval_var(
            size = Duration[i], intensity = Calendar[Worker[t]], name = _name
        )

# 3. Adding the constraints

# Add the precedence constraints
    for p in Precedences:
        mdl.add(mdl.end_before_start(itvs[h,p[0]], itvs[h,p[1]]))

# Add the no overlap constraint

# This form is a shortcut that avoids the need to explicitly define the
# interval sequence variable when no additional constraints are required on the
# sequence variable.
for w in WorkerNames:
    mdl.add(
        mdl.no_overlap(
            [itvs[h,t] for h in Houses for t in TaskNames if Worker[t]==w]
        )
    )

# Create the forbidden start and end constraints

# When an intensity function is set on an interval variable, the tasks which 
# overlap weekends and/or holidays will be automatically prolonged. A task 
# could still be scheduled to start or end in a weekend, but, in this problem, 
# a worker’s tasks cannot start or end during the worker’s days off. CP 
# Optimizer provides the constraints forbid_start and forbid_end to model 
# these types of constraints.
for h in Houses:
    for t in TaskNames:
        mdl.add(mdl.forbid_start(itvs[h,t], Calendar[Worker[t]]))
        mdl.add(mdl.forbid_end  (itvs[h,t], Calendar[Worker[t]]))

# 4. Add the objective
mdl.add(mdl.minimize(mdl.max(mdl.end_of(itvs[h,"moving"]) for h in Houses)))

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

# Allocate tasks to workers
tasks = {w : [] for w in WorkerNames}
for k,v in Worker.items():
    tasks[v].append(k)

types = {t : i for i,t in enumerate(TaskNames)}

visu.timeline('Solution SchedCalendar')
for w in WorkerNames:
    visu.panel()
    visu.pause(Calendar[w])
    visu.sequence(name=w,
        intervals=[
            (msol.get_var_solution(itvs[h,t]), types[t], t) 
            for t in tasks[w] for h in Houses
        ]
    )
visu.show()