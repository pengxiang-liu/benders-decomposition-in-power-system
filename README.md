# Power-Benders

Application of Benders decomposition in power system.

## 1. Introduction

Benders decomposition (or Benders' decomposition) is a technique in mathematical programming that allows the solution of very large linear programming problems that have a special block structure. This block structure often occurs in applications such as stochastic programming as the uncertainty is usually represented with scenarios. Therefore, Benders decomposition has a wide range of applications in the field of power systems

## 2. Our work

In this repository, we explore the application of Benders decomposition in
the field of power systems.
The first one is a hybrid AC-DC distribution system expansion planning model using modern Benders decomposition (see: DSEP-Ninghai.py).
The seconde one is a dispatch and scheduling model for EV charging using logic-based Benders decomposition (see: EV-charging.py).
We found that the use of Benders decomposition achieves 10 to 100 times acceleration in solving the model.

## 3. Recommendation

The best expert in the community of Benders decomposition: [Matteo Fischetti](http://www.dei.unipd.it/~fisch/).

The inventor of logic-based Benders decomposition: [J. N. Hooker](https://www.cmu.edu/tepper/faculty-and-research/faculty-by-area/profiles/hooker-john.html).
