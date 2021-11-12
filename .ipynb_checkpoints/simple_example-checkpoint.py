# -*- coding: utf-8 -*-
"""
Created on Thu Nov 11 11:35:33 2021
"""

import numpy as np
import os
import pandas as pd
import time
import gurobipy as gp
from gurobipy import GRB
import pickle
from copy import deepcopy

# Nodes
n = 7
clients = [ i for i in range(n-1) if i !=0]
nodes = [0]+clients+[n-1]


arcs = [(0,1),(0,2), (0,5), (1,2),(1,3),(1,4),(1,5),(1,6),(2,3),(2,6),(3,4),(4,5),(5,6),(2,1),(3,1),(4,1),(5,1),(3,2),(4,3),(5,4)]
cost = [11,10,10,5,15,15,5,10,15,10,20,15,10,5,15,15,5,15,20,15]
c = { arcs[i]:  cost[i] for i in range(len(arcs))}


# Create Model
m = gp.Model('CVRP')

# Adding variables
x = m.addVars(arcs,vtype = GRB.BINARY,name='x')
print(x[0,1])
u = m.addVars(clients,vtype = GRB.CONTINUOUS,name='u')

# Objective Function
m.setObjective(gp.quicksum(c[i,j]*x[i,j] for i,j in arcs),GRB.MINIMIZE)

#Constrains
#m.addConstrs(gp.quicksum(x[i,j] for i in range(len(arcs)) if j==arcs[i][1]) == 1 for j in clients)






