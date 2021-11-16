import numpy as np
import matplotlib.pyplot as plt
import gurobipy as gp
from gurobipy import GRB


rnd = np.random
rnd.seed(0)


### Parameters ###

# Basic problem variables
n = 15 # nodes
clients = [ i for i in range(n-1) if i !=0]
nodes = [0]+clients
N_N_0 = [(i,j) for i in nodes for j in clients if i!=j]
xc = rnd.rand(n-1)*200 # customer x locations
yc = rnd.rand(n-1)*100 # customer y locations
print(xc,yc)
T = 60 # [s] total delivery duration

# Drone parameters
M = 4 # drones
K = 100 # upper bound payload weight
v = 10 # drone speed [m/s]
Q = 10# max drone payload [kg] 

# Decision variables
arcs = [(i,j) for i in nodes for j in nodes if i!=j] # fully connected links
sigma_var = [(i,j) for i in nodes[1:] for j in nodes[1:]] # going through depot
y = arcs # payload weight between paths
t = [i for i in nodes] # time at node i
a = [i for i in clients] # time between node i and depot

# Costs
s = {(i, j): np.hypot(xc[i]-xc[j], yc[i]-yc[j]) for i, j in arcs} # euclidean distances
D = {i: rnd.randint(1,5) for i in clients} # demand of client i


### Creating the Model ###
m = gp.Model('CVRP')

# Adding decision variables
x = m.addVars(arcs,vtype = GRB.BINARY,name='x') # x = arcs 
sigma = m.addVars(sigma_var,vtype = GRB.BINARY,name='sigma') 
y = m.addVars(y,vtype = GRB.CONTINUOUS,name='y')
t = m.addVars(t,vtype = GRB.CONTINUOUS,name='t')
a = m.addVars(a,vtype = GRB.CONTINUOUS,name='a')

# Objective function
m.setObjective(gp.quicksum(s[i,j]*x[i,j] for i,j in arcs),GRB.MINIMIZE)

# Constraints
m.addConstrs(gp.quicksum(x[i,j] for j in nodes if j!= i) == 1 for i in clients) # (4a)
m.addConstrs(gp.quicksum(x[i,j] for j in nodes if j!= i)-gp.quicksum(x[j,i] for j in nodes if j!= i)== 0 for i in nodes ) #(4b)
#Reusability Constrains
m.addConstrs(gp.quicksum(sigma[i,j] for j in clients) <= x[i,0] for i in clients) # (5a)
m.addConstrs(gp.quicksum(sigma[j,i] for j in clients) <= x[0,i] for i in clients) # (5b)
m.addConstr(gp.quicksum(x[0,i] for i in clients) - gp.quicksum(sigma[i,j] for i,j in sigma_var if i!=j) <= M) # (5c)
#Demand Contrains
m.addConstrs(gp.quicksum(y[j,i] for j in nodes if j!=i) - gp.quicksum(y[i,j] for j in nodes if j!=i)==D[i] for i in clients) # (6a)
m.addConstrs(y[i,j] <= K*x[i,j] for i,j in arcs if i!=j) # (6b)
#Time Constrains
m.addConstrs(t[i] - t[j] + s[i,j]/v <= K* (1-x[i,j]) for i,j in N_N_0 if i!=j) # (7a) CHECK THE SUM OF ARCS
m.addConstrs(t[i] - a[i] + s[i,0]/v <= K * (1 - x[i,0]) for i in clients) # (7b)
m.addConstrs(a[i] - t[j] + s[0,j]/v <= K * (1 - sigma[i,j]) for i,j in sigma_var if i!=j) # (7c)
m.addConstrs(t[i] <= T  for i in clients) # (7d)  and (7e) CHECK THIS CONSTRAINT
#
m.addConstrs(y[i,j] <= Q * x[i,j] for i,j in arcs if i!=j) #(8a)


m.Params.timeLimit = 100 #[s]
m.optimize()

#Plotting
active_arcs = [a for a in arcs if x[a].x > 0.99]

for i, j in active_arcs:
    plt.plot([xc[i], xc[j]], [yc[i], yc[j]], c='g', zorder=0)
plt.plot(xc[0], yc[0], c='r', marker='s')
plt.scatter(xc[1:], yc[1:], c='b')

plt.show()




























