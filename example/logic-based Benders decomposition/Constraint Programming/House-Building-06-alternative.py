'''
Chapter 6. Using alternative resources in the house building problem
'''
# This chapter presents how to use alternative resources in the house building 
# problem.
# Each house has a maximal completion date. Moreover, there are three workers, 
# and one of the three is required for each task. The three workers have 
# varying levels of skills with regard to the various tasks; if a worker has no 
# skill for a particular task, he may not be assigned to the task. For some 
# pairs of tasks, if a particular worker performs one of the pair on a house, 
# then the same worker must be assigned to the other of the pair for that 
# house. The objective is to find a solution that maximizes the task associated 
# skill levels of the workers assigned to the tasks.


import sys
import docplex.cp.utils_visu as visu
import matplotlib.pyplot as plt

from docplex.cp.model import *
from pylab import rcParams

# -----------------------------------------------------------------------------
# Prepare data
# -----------------------------------------------------------------------------

NbHouses = 5
Deadline =  318

Workers = ["Joe", "Jack", "Jim"]

Tasks = ["masonry",  "carpentry", "plumbing", "ceiling", "roofing",
         "painting", "windows",   "facade",   "garden",  "moving"]

Durations =  [35, 15, 40, 15, 5, 10, 5, 10, 5, 5]

# The data also includes a tupleset, Skills. Each tuple in the set consists of 
# a worker, a task, and the skill level that the worker has for the task.
Skills = [("Joe",  "masonry",  9), ("Joe",  "carpentry", 7),
          ("Joe",  "ceiling",  5), ("Joe",  "roofing",   6), 
          ("Joe",  "windows",  8), ("Joe",  "facade",    5),
          ("Joe",  "garden",   5), ("Joe",  "moving",    6),
          ("Jack", "masonry",  5), ("Jack", "plumbing",  7), 
          ("Jack", "ceiling",  8), ("Jack", "roofing",   7),
          ("Jack", "painting", 9), ("Jack", "facade",    5),
          ("Jack", "garden",   5), ("Jim",  "carpentry", 5), 
          ("Jim",  "painting", 6), ("Jim",  "windows",   5),
          ("Jim",  "garden",   9), ("Jim",  "moving",    8)]

Precedences = [("masonry", "carpentry"),("masonry",   "plumbing"),
               ("masonry", "ceiling"),  ("carpentry", "roofing"),
               ("ceiling", "painting"), ("roofing",   "windows"),
               ("roofing", "facade"),   ("plumbing",  "facade"),
               ("roofing", "garden"),   ("plumbing",  "garden"),
               ("windows", "moving"),   ("facade",    "moving"),
               ("garden",  "moving"),   ("painting",  "moving")]

# In addition, there is a tupleset, Continuities, which is a set of triples (a 
# pair of tasks and a worker). If one of the two tasks in a pair is performed 
# by the worker for a given house, then the other task in the pair must be 
# performed by the same worker for that house.
Continuities = [("Joe",  "masonry",   "carpentry"),
                ("Jack", "roofing",   "facade"), 
                ("Joe",  "carpentry", "roofing"),
                ("Jim",  "garden",    "moving")]

nbWorkers = len(Workers)
Houses = range(NbHouses)

# -----------------------------------------------------------------------------
# Build the model
# -----------------------------------------------------------------------------

# 1. Creation of the model
mdl = CpoModel()

# 2. Declarations of decision variables

# Create the interval variables

# Two matrices of interval variables are created in this model. The first, 
# tasks, is indexed on the houses and tasks and must be scheduled in the 
# interval [0..Deadline]. The other matrix of interval variables is indexed on 
# the houses and the Skills tupleset. These interval variables are optional and 
# may or may not be present in the solution. The intervals that are performed 
# will represent which worker performs which task.
tasks = {}
wtasks = {}
for h in Houses:
    for i,t in enumerate(Tasks):
        tasks [(h,t)] = mdl.interval_var(start=[0,Deadline], size=Durations[i])
    for s in Skills:
        wtasks[(h,s)] = mdl.interval_var(optional=True)

# 3. Adding the constraints

# Add the temporal constraints
for h in Houses:
    for p in Precedences:
        mdl.add(mdl.end_before_start(tasks[h,p[0]], tasks[h,p[1]]))

# Add the alternative constraints

# the specialized constraint alternative() is used to constrain the solution so 
# that exactly one of the interval variables tasks associated with a given task 
# of a given house is to be present in the solution.
# The constraint alternative() creates a constraint between an interval and a 
# set of intervals that specifies that if the given interval is present in the 
# solution, then exactly one interval variable of the set is present in the 
# solution.
# In other words, consider an alternative constraint created with an interval 
# variable a and an array of interval variables bs. If a is present in the 
# solution, then exactly one of the interval variables in bs will be present, 
# and a starts and ends together with this chosen interval.
for h in Houses:
    for t in Tasks:
        mdl.add(
            mdl.alternative(
                tasks[h,t], [wtasks[h,s] for s in Skills if s[1]==t]
            )
        )

# For each house and each given pair of tasks and worker that must have 
# continuity, a constraint states that if the interval variable for one of the 
# two tasks for the worker is present, the interval variable associated with 
# that worker and the other task must also be present.
# The expression presence_of() is used to represent whether a task is performed 
# by a worker. The constraint presence_of() is true if the interval variable is 
# present in and is false if the interval variable is absent from the solution.
for h in Houses:
    for c in Continuities:
        for (worker1, task1, l1) in Skills:
            if worker1 == c[0] and task1 == c[1]:
                for (worker2, task2, l2) in Skills:
                    if worker2 == c[0] and task2 == c[2]:
                        mdl.add(
                            mdl.presence_of(wtasks[h,(c[0], task1, l1)]) 
                            == 
                            mdl.presence_of(wtasks[h,(c[0], task2, l2)])
                        )

# Add the no overlap constraints
for w in Workers:
    mdl.add(
        mdl.no_overlap(
            [wtasks[h,s] for h in Houses for s in Skills if s[0]==w]
        )
    )

# 4. Add the objective

# The presence of an interval variable in the solution must be accounted in the 
# objective. Thus for each of these possible tasks, the cost is incremented by 
# the product of the skill level and the expression representing the presence 
# of the interval variable in the solution. The objective of this problem is to 
# maximize the skill levels used for all the tasks, then to maximize the 
# expression.
mdl.add(
    mdl.maximize(
        mdl.sum(
            s[2] * mdl.presence_of(wtasks[h,s]) for h in Houses for s in Skills
        )
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

worker_idx = {w : i for i,w in enumerate(Workers)}
worker_tasks = [[] for w in range(nbWorkers)]
for h in Houses:
    for s in Skills:
        worker = s[0]
        wt = wtasks[(h,s)]
        worker_tasks[worker_idx[worker]].append(wt)
visu.timeline('Solution SchedOptional', 0, Deadline)
for i,w in enumerate(Workers):
    visu.sequence(name=w)
    for t in worker_tasks[worker_idx[w]]:
        wt = msol.get_var_solution(t)
        if wt.is_present():
            # if desc[t].skills[w] == max(desc[t].skills):
            # # Green-like color when task is using the most skilled worker
            #     color = 'lightgreen'
            # else:
            # # Red-like color when task does not use the most skilled worker
            #     color = 'salmon'
            color = 'salmon'
            visu.interval(wt, color, wt.get_name())
visu.show()