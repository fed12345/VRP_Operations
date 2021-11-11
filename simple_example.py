# -*- coding: utf-8 -*-
"""
Created on Thu Nov 11 11:35:33 2021
"""



import numpy as np
import os
import pandas as pd
import time
import gurobipy as gp
import pickle
from copy import deepcopy

# Nodes
n = 7
clients = [ i for i in range(n-1) if i !=0]
nodes = [0]+clients+[n-1]

# Weights
depot = 10
inner_loop = 5

x = {'x01','x02','x05',
     'x12','x13','x14',
     'x15','x16','x23','x26',
     'x34','x45','x56'}
c = {'10','10','10','5','15','15','5','10','15','10','20','15','10'}

# Create Model
m = gp.Model('CVRP')

# Adding variables
x = m.addVars(x,vtype = gp.GRB.BINARY,name='x')
u = m.addVars(clients,vtype = gp.GRB.CONTINUOUS,name='u')

# Objective Function
m.setObjective(gp.quicksum(c[i]*x[i]for i in nodes),gp.GBR.MINIMIZE)










