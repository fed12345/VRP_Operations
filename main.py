import numpy as np
import matplotlib.pyplot as plt
import gurobipy as gp
from gurobipy import GRB
from data import Drones, Clients
import timeit

#Initialization
rnd = np.random
rnd.seed(0)


#Definitions

def solve_VRP(drones,clients_list,time_limit,Plotting):
    # Basic problem variables
    n = Clients.numeber_of_clients # nodes
    clients = [ i for i in range(1,n+1)]
    nodes = [0]+clients
    N_N_0 = [(i,j) for i in nodes for j in clients if i!=j]
    xc = [0]+[xc.x_coord for xc in clients_list] # customer x locations
    yc = [0]+[yc.y_coord for yc in clients_list] #rnd.rand(n-1)*100 # customer y locations
    print(type(xc))
    T = time_limit # [s] total delivery duration

    # Drone parameters
    M = drones.number_of_drones # Number of drones
    K = 100 # upper bound payload weight
    v = drones.maxspeed # drone speed [m/s]
    Q = drones.maxpayload# max drone payload [kg] 

    # Decision variables
    arcs = [(i,j) for i in nodes for j in nodes if i!=j] # fully connected links
    sigma_var = [(i,j) for i in nodes[1:] for j in nodes[1:]] # going through depot
    y = arcs # payload weight between paths
    t = [i for i in nodes] # time at node i
    a = [i for i in clients] # time between node i and depot

    # Costs
    s = {(i, j): np.hypot(xc[i]-xc[j], yc[i]-yc[j]) for i, j in arcs} # euclidean distances
    D = {i.number: i.demand for i in clients_list}# demand of client rnd.randint(1,5)


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
    m.addConstrs(t[i] - t[j] + s[i,j]/v <= K* (1-x[i,j]) for i,j in N_N_0 if i!=j) # (7a)
    m.addConstrs(t[i] - a[i] + s[i,0]/v <= K * (1 - x[i,0]) for i in clients) # (7b)
    m.addConstrs(a[i] - t[j] + s[0,j]/v <= K * (1 - sigma[i,j]) for i,j in sigma_var if i!=j) # (7c)
    m.addConstrs(t[i] <= T  for i in clients) # (7d)  and (7e) CHECK THIS CONSTRAINT
    #
    m.addConstrs(y[i,j] <= Q * x[i,j] for i,j in arcs if i!=j) #(8a)


    m.Params.timeLimit = 100 #[s]
    m.optimize()
    
    if Plotting == True:

        #Plotting
        active_arcs = [a for a in arcs if x[a].x > 0.99]
    
        for i, j in active_arcs:
            plt.plot([xc[i], xc[j]], [yc[i], yc[j]], c='g', zorder=0)
        plt.plot(xc[0], yc[0], c='r', marker='s')
        plt.scatter(xc[1:], yc[1:], c='b')
    
        plt.show()

# SENSITIVITY ANALYSIS

def sensitivity(clients_range):
    
    # TEST 1: number of clients vs run time
    x_1 = [] # number of clients
    t_1 = [] # run times
    
    for c in range(2,clients_range+1):
        drone = Drones("Amazon Drone", 10, 5, 5)#(name, maxspeed, maxpayload, number_of_drones)
        T = 60 # [s] total delivery duration
        
        client_list = []
        for i in range(1,c):
            client = Clients(i, rnd.rand(1)[0]*100,rnd.rand(1)[0]*200, rnd.randint(1,5))
            client_list.append(client)
        print(len(client_list))    
        
        # Appending run time for c number of clients
        start = timeit.default_timer()
        solve_VRP(drone,client_list, T,False)
        stop = timeit.default_timer()
        time = round(stop-start,3)
        t_1.append(time)
        x_1.append(c)
        client_list = []
        
#SAMPLE DATASET

drone1 = Drones("Amazon Drone", 10, 5, 5)#(name, maxspeed, maxpayload, number_of_drones)

client_list = []
for i in range(1,1):
    client = Clients(i, rnd.rand(1)[0]*100,rnd.rand(1)[0]*200, rnd.randint(1,5))
    client_list.append(client)

T = 60 # [s] total delivery duration



if __name__== "__main__":
    solve_VRP(drone1,client_list, T, Plotting = True)
    























