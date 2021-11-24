import pickle
import numpy as np
import matplotlib.pyplot as plt
import gurobipy as gp
from gurobipy import GRB
from data import Drones, Clients
import timeit
from geopy.distance import geodesic

#Initialization
#https://towardsdatascience.com/easy-steps-to-plot-geographic-data-on-a-map-python-11217859a2db

rnd = np.random
rnd.seed(0)


#Definitions

def solve_VRP(drones,clients_list,time_limit,Plotting):
    # Basic problem variables
    n = Clients.numeber_of_clients # nodes
    clients = [ i for i in range(1,n+1)]
    nodes = [0]+clients
    N_N_0 = [(i,j) for i in nodes for j in clients if i!=j]
    lat= [Clients.depo_location[0]]+[xc.lat for xc in clients_list] # customer x locations
    long = [Clients.depo_location[1]]+[yc.long for yc in clients_list] #rnd.rand(n-1)*100 # customer y locations
    T = time_limit # [s] total delivery duration

    # Drone parameters
    M = drones.number_of_drones # Number of drones
    K = 1000000 # upper bound payload weight
    v = drones.maxspeed # drone speed [m/s]
    Q = drones.maxpayload# max drone payload [kg]
    p = drones.power#Power Consumption [kW]

    # Decision variables
    arcs = [(i,j) for i in nodes for j in nodes if i!=j] # fully connected links
    sigma_var = [(i,j) for i in nodes[1:] for j in nodes[1:]] # going through depot
    y = arcs # payload weight between paths
    t = [i for i in nodes] # time at node i
    a = [i for i in clients] # time between node i and depot
    z = a # The energy consumed from a drone’s battery by the time it arrives at the depot directly after leaving
    f = t #Enegry cosumed at location i

    # Costs
    s = {(i, j): geodesic((lat[i],long[i]),(lat[j],long[j])).m for i, j in arcs} # euclidean distances TODO change km to m
    D = {i.number: i.demand for i in clients_list}# demand of client rnd.randint(1,5)


    ### Creating the Model ###
    m = gp.Model('CVRP')

    # Adding decision variables
    x = m.addVars(arcs,vtype = GRB.BINARY,name='x') # x = arcs
    sigma = m.addVars(sigma_var,vtype = GRB.BINARY,name='sigma')
    y = m.addVars(y,vtype = GRB.CONTINUOUS,name='y')
    t = m.addVars(t,vtype = GRB.CONTINUOUS,name='t')
    a = m.addVars(a,vtype = GRB.CONTINUOUS,name='a')
    f = m.addVars(f,vtype = GRB.CONTINUOUS,name='f')
    z = m.addVars(z,vtype = GRB.CONTINUOUS,name='z')

    # Objective function
    m.setObjective(gp.quicksum(s[i,j]*x[i,j] for i,j in arcs),GRB.MINIMIZE)

    # Constraints
    m.addConstrs(gp.quicksum(x[i,j] for j in nodes if j!= i) == 1 for i in clients) # (4a)
    m.addConstrs(gp.quicksum(x[i,j] for j in nodes if j!= i)-gp.quicksum(x[j,i] for j in nodes if j!= i)== 0 for i in nodes ) #(4b)
    #Reusability Constrains
    m.addConstrs((gp.quicksum(sigma[i,j] for j in clients) <= x[i,0] for i in clients),name = 'Resusability') # (5a)
    m.addConstrs(gp.quicksum(sigma[j,i] for j in clients) <= x[0,i] for i in clients) # (5b)
    m.addConstr(gp.quicksum(x[0,i] for i in clients) - gp.quicksum(sigma[i,j] for i,j in sigma_var if i!=j) <= M) # (5c)
    #Demand Contrains
    m.addConstrs((gp.quicksum(y[j,i] for j in nodes if j!=i) - gp.quicksum(y[i,j] for j in nodes if j!=i)==D[i] for i in clients), name = 'Demand') # (6a)
    m.addConstrs(y[i,j] <= K*x[i,j] for i,j in arcs if i!=j) # (6b)
    #Time Constrains
    m.addConstrs((t[i] - t[j] + s[i,j]/v <= K* (1-x[i,j]) for i,j in N_N_0 if i!=j), name = 'Time') # (7a)
    m.addConstrs(t[i] - a[i] + s[i,0]/v <= K * (1 - x[i,0]) for i in clients) # (7b)
    m.addConstrs(a[i] - t[j] + s[0,j]/v <= K * (1 - sigma[i,j]) for i,j in sigma_var if i!=j) # (7c)
    m.addConstrs(t[i] <= T  for i in clients) # (7d)  and (7e) CHECK THIS CONSTRAINT
    # Capacity Constrains
    m.addConstrs((y[i,j] <= Q * x[i,j] for i,j in arcs if i!=j), name = 'Capacity') #(8a)
    #Energy Contrains
    m.addConstrs((f[i] - f[j] + p*s[i,j]/v <= K*(1-x[i,j]) for i,j in N_N_0 if i!=j), name = 'Enegry')#(9a)
    m.addConstrs(f[i] - z[i] + p*s[i,0]/v <= K * (1 - x[i,0]) for i in clients)#(9b) 
    m.addConstrs(z[i]<= K*x[i,0] for i in clients)


    m.update()
    #Writing LP file
    m.write('model.lp') 

    #m.Params.timeLimit = 200 #[s]
    m.Params.MIPGap = 5

    m.optimize()
    if Plotting == True:
        #Plotting
        BBox = ((min(long),   max(long),  min(lat), max(lat)))
        ruh_m = plt.imread('map.png')

        active_arcs = [a for a in arcs if x[a].x > 0.99]
        fig, ax = plt.subplots(figsize = (8,7))
        for i, j in active_arcs:
            ax.plot([long[i], long[j]], [lat[i], lat[j]], c='g', linestyle= ':', zorder=1)
        ax.plot(long[0], lat[0], c='r', marker='s')
        ax.scatter(long[1:], lat[1:], c='b')
        ax.set_xlim(BBox[0],BBox[1])
        ax.set_ylim(BBox[2],BBox[3])
        
        #ax.imshow(ruh_m, zorder=0, extent = BBox, aspect= 'equal')
        plt.show()
        
    # output objective function
    obj = m.getObjective()
    return obj.getValue() 




def sensitivity(clients_range, maxspeed_range, maxpayload_range):
    
    # CLIENTS LIST
        
    # infile = open('villages_burundi', 'rb')
    # list = pickle.load(infile)
    
    # client_list = []
    # for i in range(1,20):
    #     client = Clients(list[i+20][0],i,list[i+20][1],list[i+20][2],list[i+20][3],list[i+20][4])
    #     client_list.append(client)
               
    # TEST 1: max speed vs objective function
    x_1 = [] # max speed
    y_1 = [] # ojective value
    
    T = 5500 # [s] total delivery duration   
    
    for i in range (30,maxspeed_range+30): #min speed at 30 [m/s]
        drone = Drones("AAI RQ-7 Shadow", i, 10, 4)#(name, maxspeed, maxpayload, number_of_drones) 
        
        # updating plotting lists
        x_1.append(i)
        y_1.append(solve_VRP(drone,client_list, T,False))
        
        
    # TEST 2: max payload vs objective function
    x_2 = []
    y_2 = []
    
    T = 5500 # [s] total delivery duration 
    
    for i in range(10,maxpayload_range+10): # min payload of 5 [kg]
            drone2 = Drones("AAI RQ-7 Shadow", 36, i, 4)#(name, maxspeed, maxpayload, number_of_drones)
            
            # updating plotting lists
            x_2.append(i)
            y_2.append(solve_VRP(drone2,client_list, T,False))
            
        
        
    # PLOTTING
    fig, (ax1, ax2) = plt.subplots(1, 2)
    
    # Test 1
    ax1.plot(x_1,y_1)
    #ax1.xlabel("Drone Max speed [m/s]")
    #ax1.ylabel("Objective Funcntion Value")
    #ax1.title('Objective Function vs Drone Max Speed')
    
    # Test 2
    ax2.plot(x_2,y_2)
    #ax2.xlabel("Drone Max Payload [kg]")
    #ax2.ylabel("Objective Funcntion Value")
    #ax2.title('Objective Function vs Drone Max Speed')
    
    
    plt.show()
    
    
    
    

#SAMPLE DATASET
drone1 = Drones("AAI RQ-7 Shadow", 36.1111, 10, 4, 28.5)#(name, maxspeed, maxpayload, number_of_drones, power consumtion)
infile = open('villages_burundi', 'rb')
list = pickle.load(infile)

client_list = []
for i in range(1,20):
    client = Clients(list[i+20][0],i,list[i+20][1],list[i+20][2],list[i+20][3],list[i+20][4])
    client_list.append(client)

T = 5500 # [s] total delivery duration



if __name__== "__main__":
   solve_VRP(drone1,client_list, T, Plotting = True)
   # sensitivity(20,5,10)
