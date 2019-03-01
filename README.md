# Benders-DSEP
A distribution system planning model using modern Benders decomposition. This project develops a multistage planning model for distribution system where investments in the distribution network and distributed generations are jointly considered. The model is the planning model is applied to a real distribution system in Zhejiang province, China.


# Introduction

The original Benders decomposition from the ‘60s uses two distinct ingredients for solving a Mixed-Integer Linear Program (MILP). The modern Benders decomposition uses callback functions in the modern commercial solvers such as IBM ILOG Cplex, Gurobi, XPRESS etc. Callback functions are just entry points in the Branch-and-cut code where an advanced user (you!) can add his/her customizations.

The main advantage of the callback approach is that it is likely to avoid considerable rework. In the original approach, each time you add a cut to the master problem, you have to solve it anew. Although the new cuts may change the structure of the solution tree (by changing the solver's branching decisions), you are probably going to spend time revisiting candidate solutions that you had already eliminated earlier. Moreover, you may actually encounter the optimal solution to the original problem and then discard it, because a superoptimal solution that is either infeasible in the original problem or has an artificially superior value, causes the true optimum to appear suboptimal. With the callback approach, you use a single search tree, never revisit a node, and never overlook a truly superior solution.
