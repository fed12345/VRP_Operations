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
x = [(i,j) for i in nodes for j in nodes if i!=j]

# Demand
q = {n:n for n in clients}

# Distance
distance = {(i,j): i+j for i in nodes  for j in nodes if i!=j}

# Create Model
m = gp.Model('CVRP')

# Adding variables
x = m.addVars(x,vtype = gp.GRB.BINARY,name='x')
u = m.addVars(clients,vtype = gp.GRB.CONTINUOUS,name='u')





































