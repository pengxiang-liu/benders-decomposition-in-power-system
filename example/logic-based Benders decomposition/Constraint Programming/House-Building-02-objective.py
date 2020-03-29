'''
Chapter 2. Modeling and solving house building with an objective
'''
# This chapter presents the same house building example in such a manner that 
# minimizes an objective.


import sys
import docplex.cp.utils_visu as visu
import matplotlib.pyplot as plt

from docplex.cp.model import *
from pylab import rcParams

# -----------------------------------------------------------------------------
# Build the model
# -----------------------------------------------------------------------------

# 1. Creation of the model
mdl = CpoModel()

# 2. Declarations of decision variables

# The declaration of necessary interval variables is done as follows:
masonry   = mdl.interval_var(name='masonry',   size=35)
carpentry = mdl.interval_var(name='carpentry', size=15)
plumbing  = mdl.interval_var(name='plumbing',  size=40)
ceiling   = mdl.interval_var(name='ceiling',   size=15)
roofing   = mdl.interval_var(name='roofing',   size=5)
painting  = mdl.interval_var(name='painting',  size=10)
windows   = mdl.interval_var(name='windows',   size=5)
facade    = mdl.interval_var(name='facade',    size=10)
garden    = mdl.interval_var(name='garden',    size=5)
moving    = mdl.interval_var(name='moving',    size=5)

# 3. Adding the constraints

# In this example, certain tasks can start only after other tasks have been 
# completed. CP Optimizer allows to express constraints involving temporal 
# relationships between pairs of interval variables using precedence 
# constraints. Precedence constraints are used to specify when an interval 
# variable must start or end with respect to the start or end time of 
# another interval variable.

mdl.add(mdl.end_before_start(masonry,   carpentry))
mdl.add(mdl.end_before_start(masonry,   plumbing))
mdl.add(mdl.end_before_start(masonry,   ceiling))
mdl.add(mdl.end_before_start(carpentry, roofing))
mdl.add(mdl.end_before_start(ceiling,   painting))
mdl.add(mdl.end_before_start(roofing,   windows))
mdl.add(mdl.end_before_start(roofing,   facade))
mdl.add(mdl.end_before_start(plumbing,  facade))
mdl.add(mdl.end_before_start(roofing,   garden))
mdl.add(mdl.end_before_start(plumbing,  garden))
mdl.add(mdl.end_before_start(windows,   moving))
mdl.add(mdl.end_before_start(facade,    moving))
mdl.add(mdl.end_before_start(garden,    moving))
mdl.add(mdl.end_before_start(painting,  moving))

# 4. Add the objective

# To model the cost for starting a task earlier than the preferred starting 
# date, the expression start_of() can be used. It represents the start of an 
# interval variable as an integer expression.

# For each task that has an earliest preferred start date, the number of days 
# before the preferred date it is scheduled to start can be determined using 
# the expression start_of(). This expression can be negative if the task starts 
# after the preferred date. Taking the maximum of this value and 0 using max() 
# allows to determine how many days early the task is scheduled to start. 
# Weighting this value with the cost per day of starting early determines the 
# cost associated with the task.

# The cost for ending a task later than the preferred date is modeled in a 
# similar manner using the expression endOf(). The earliness and lateness costs 
# can be summed to determine the total cost.

obj = mdl.minimize(  400 * mdl.max([mdl.end_of(moving) - 100, 0]) 
                    + 200 * mdl.max([25 - mdl.start_of(masonry), 0]) 
                    + 300 * mdl.max([75 - mdl.start_of(carpentry), 0]) 
                    + 100 * mdl.max([75 - mdl.start_of(ceiling), 0]) )
mdl.add(obj)

# -----------------------------------------------------------------------------
# Solve the model and display the result
# -----------------------------------------------------------------------------

# 1. Calling the solve
print("\nSolving model....")
msol = mdl.solve(url=None, key=None, TimeLimit=10)
print("done")

# 2. Displaying the objective and solution
print("Cost will be " + str(msol.get_objective_values()[0]))

var_sol = msol.get_var_solution(masonry)
print("Masonry :   {}..{}".format(var_sol.get_start(), var_sol.get_end()))
var_sol = msol.get_var_solution(carpentry)
print("Carpentry : {}..{}".format(var_sol.get_start(), var_sol.get_end()))
var_sol = msol.get_var_solution(plumbing)
print("Plumbing :  {}..{}".format(var_sol.get_start(), var_sol.get_end()))
var_sol = msol.get_var_solution(ceiling)
print("Ceiling :   {}..{}".format(var_sol.get_start(), var_sol.get_end()))
var_sol = msol.get_var_solution(roofing)
print("Roofing :   {}..{}".format(var_sol.get_start(), var_sol.get_end()))
var_sol = msol.get_var_solution(painting)
print("Painting :  {}..{}".format(var_sol.get_start(), var_sol.get_end()))
var_sol = msol.get_var_solution(windows)
print("Windows :   {}..{}".format(var_sol.get_start(), var_sol.get_end()))
var_sol = msol.get_var_solution(facade)
print("Facade :    {}..{}".format(var_sol.get_start(), var_sol.get_end()))
var_sol = msol.get_var_solution(moving)
print("Moving :    {}..{}".format(var_sol.get_start(), var_sol.get_end()))

# 3. Graphical view
rcParams['figure.figsize'] = 15, 3

wt = msol.get_var_solution(masonry)   
visu.interval(wt, 'lightblue', 'masonry')   
wt = msol.get_var_solution(carpentry)   
visu.interval(wt, 'lightblue', 'carpentry')
wt = msol.get_var_solution(plumbing)   
visu.interval(wt, 'lightblue', 'plumbing')
wt = msol.get_var_solution(ceiling)   
visu.interval(wt, 'lightblue', 'ceiling')
wt = msol.get_var_solution(roofing)   
visu.interval(wt, 'lightblue', 'roofing')
wt = msol.get_var_solution(painting)   
visu.interval(wt, 'lightblue', 'painting')
wt = msol.get_var_solution(windows)   
visu.interval(wt, 'lightblue', 'windows')
wt = msol.get_var_solution(facade)   
visu.interval(wt, 'lightblue', 'facade')
wt = msol.get_var_solution(moving)   
visu.interval(wt, 'lightblue', 'moving')
visu.show()