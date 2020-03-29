'''
Chapter 1. Introduction to Scheduling
'''
# This is a basic problem that involves building a house. The masonry, roofing,
# painting, etc.  must be scheduled. Some tasks must necessarily take place 
# before others, and these requirements are expressed through precedence 
# constraints.
# 
# Please refer to documentation for appropriate setup of solving configuration.


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

# This section provides a completed example model that can be tested. The 
# problem is a house building problem. There are ten tasks of fixed size, and 
# each of them needs to be assigned a starting time. The statements for 
# creating the interval variables that represent the tasks are:

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

# The constraints in this problem are precedence constraints; some tasks cannot 
# start until other tasks have ended. For example, the ceilings must be completed 
# before painting can begin.
# The set of precedence constraints for this problem can be added to the model with 
# the block:

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

# Here, the special constraint end_before_start() ensures that one interval variable 
# ends before the other starts. If one of the interval variables is not present, the 
# constraint is automatically satisfied.

# -----------------------------------------------------------------------------
# Solve the model and display the result
# -----------------------------------------------------------------------------

# 1. Calling the solve
print("\nSolving model....")
msol = mdl.solve(url=None, key=None, TimeLimit=10)
print("done")

# 2. Displaying the solution
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