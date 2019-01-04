# LogicBenders
A distribution system planning model using logic-based Benders decomposition

# Introduction
1. Distribution system planning

This project develops a multistage planning model for distribution system where investments in the distribution network and distributed generations are jointly considered. The planning model is decomposed into three layers, i.e. installation, reconfiguration and operation. 

2. Logic-based Benders decomposition

Note that the proposed planning model is a complicated mixed-integer programming (MIP), which is hard to solve, even for the state-of-the-art commercial solver such as CPLEX and GUROBI. Therefore, Benders decomposition is used. The master problem determines the optimal installation plans, while the sub-problem minimizes the costs of reconfiguration and operation at each scenario.

One of the most serious problem is that sub-problem contains integer variables. The traditional approach is to relax the binary variables as continuous variables under a nested Benders decomposition structure. However, the weak Benders cut makes the problem hard to converge.

In this context, a logic-based Benders decomposition algorithm is developed to solve the problem.

# Reference

[1] J.N. Hooker, G. Ottosson, "Logic-based Benders decomposition," Mathematical Programming, no.96 vol.1 pp.33-60
