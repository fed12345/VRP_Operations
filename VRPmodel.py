#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 12 15:33:55 2021

@author: youssef
"""

import gurobipy as gp
from gurobipy import GRB
import pickle
from copy import deepcopy
import random 
import numpy as np

# Nodes
n = 7
clients = [ i for i in range(n-1) if i !=0]
nodes = [0]+clients+[n-1]

# not connected!
#arcs = [(0,1),(0,2), (0,5), (1,2),(1,3),(1,4),(1,5),(1,6),(2,3),(2,6),(3,4),(4,5),(5,6),(2,1),(3,1),(4,1),(5,1),(3,2),(4,3),(5,4)]
#cost = [11,10,10,5,15,15,5,10,15,10,20,15,10,5,15,15,5,15,20,15]
#c = { arcs[i]:  cost[i] for i in range(len(arcs))}

# fully connected
arcs = [(i,j) for i in nodes for j in nodes if i!=j]
cost = random.sample(range(20,200),len(arcs))
c = { arcs[i]:  cost[i] for i in range(len(arcs))}

np.random.seed(0)
q = {n:np.random.randint(10,15) for n in clients}
#q[0] = 0
#q[6] = 0
Q = 50

# Create Model
m = gp.Model('CVRP')

# Adding variables
x = m.addVars(arcs,vtype = GRB.BINARY,name='x')         # x = arcs
u = m.addVars(clients,vtype = GRB.CONTINUOUS,name='u')  # u = clients


# Objective Function
m.setObjective(gp.quicksum(c[i,j]*x[i,j] for i,j in arcs),GRB.MINIMIZE)


#Constraints

# first constraint: check that each customer is visited once by a vehicle
m.addConstrs(gp.quicksum(x[i,j] for j in nodes if j!= i) == 1 for i in clients)

# second constraint: check that if a vehicle visits a client, the vehicle leaves the client
m.addConstrs(gp.quicksum(x[i,j] for i in nodes if j!= i) == 1 for j in clients)

# third constraint: avoid subtours (TO TAKE OUT LATER)
m.addConstrs((x[i,j]==1) >> (u[i]+q[j]== u[j]) for i,j in arcs if j!=0 and i!=0)

print(len(arcs),clients)