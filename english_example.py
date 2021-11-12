# -*- coding: utf-8 -*-
"""
Created on Fri Nov 12 15:30:54 2021

@author: adria
"""

import numpy as np
import matplotlib.pyplot as plt
import gurobipy as gp


rnd = np.random
rnd.seed(0)


n = 10  # numbre of clients
xc = rnd.rand(n+1)*200
yc = rnd.rand(n+1)*100


plt.plot(xc[0], yc[0], c='r', marker='s')
plt.scatter(xc[1:], yc[1:], c='b')



N = [i for i in range(1, n+1)]
V = [0] + N
A = [(i, j) for i in V for j in V if i != j]
c = {(i, j): np.hypot(xc[i]-xc[j], yc[i]-yc[j]) for i, j in A}
Q = 20
q = {i: rnd.randint(1, 10) for i in N}


# Model
mdl = gp.Model('CVRP')


x = mdl.addVars(A, vtype=gp.GRB.BINARY)
u = mdl.addVars(N, vtype=gp.GRB.CONTINUOUS)


mdl.modelSense = gp.GRB.MINIMIZE
mdl.setObjective(gp.quicksum(x[i, j]*c[i, j] for i, j in A))



mdl.addConstrs(gp.quicksum(x[i, j] for j in V if j != i) == 1 for i in N)
mdl.addConstrs(gp.quicksum(x[i, j] for i in V if i != j) == 1 for j in N)
mdl.addConstrs((x[i, j] == 1) >> (u[i]+q[j] == u[j])
               for i, j in A if i != 0 and j != 0)
mdl.addConstrs(u[i] >= q[i] for i in N)
mdl.addConstrs(u[i] <= Q for i in N)

mdl.Params.MIPGap = 0.1
mdl.Params.TimeLimit = 30  # seconds
mdl.optimize()

active_arcs = [a for a in A if x[a].x > 0.99]

for i, j in active_arcs:
    plt.plot([xc[i], xc[j]], [yc[i], yc[j]], c='g', zorder=0)
plt.plot(xc[0], yc[0], c='r', marker='s')
plt.scatter(xc[1:], yc[1:], c='b')

plt.show()























