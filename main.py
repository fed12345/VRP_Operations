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
    tao = 60

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
    m.addConstrs(t[i] - a[i] + + tao + s[i,0]/v <= K * (1 - x[i,0]) for i in clients) # (7b)
    m.addConstrs(a[i] - t[j] + s[0,j]/v <= K * (1 - sigma[i,j]) for i,j in sigma_var if i!=j) # (7c)
    m.addConstrs(t[i] <= T  for i in clients) # (7d)  and (7e) CHECK THIS CONSTRAINT
    # Capacity Constrains
    m.addConstrs((y[i,j] <= Q * x[i,j] for i,j in arcs if i!=j), name = 'Capacity') #(8a)
    #Energy Contrains
    m.addConstrs((f[i] - f[j] + p*s[i,j]/v <= K*(1-x[i,j]) for i,j in N_N_0 if i!=j), name = 'Enegry')#(9a)
    m.addConstrs(f[i] - z[i] + p*(s[i,0]/v + tao)<= K * (1 - x[i,0]) for i in clients)#(9b) 
    m.addConstrs(z[i]<= K*x[i,0] for i in clients)


    m.update()
    #Writing LP file
    m.write('model.lp') 

    #m.Params.timeLimit = 200 #[s]
    m.Params.MIPGap = 0

    m.optimize()
    if Plotting == True:
        #Plotting

        BBox = ((min(long)-0.1,   max(long)+0.1,  min(lat)-0.1, max(lat)+0.1))
        ruh_m = plt.imread('map.png')
        
        active_arcs = [a for a in arcs if x[a].x > 0.99]
        
        sorted_arcs = loop_finder(active_arcs)
        fig, ax = plt.subplots(figsize = (8,7))
        color = ['g','r','b',"y"]
        linestyle = [':' ]
        for k in range(len(sorted_arcs)):
            for i, j in sorted_arcs[k]:
                ax.plot([long[i], long[j]], [lat[i], lat[j]], c=color[k], linestyle= ':', zorder=1)
                ax.annotate(nodes[i], (long[i]+0.01, lat[i]+0.01))
        ax.plot(long[0], lat[0], c='r', marker='s')
        ax.scatter(long[1:], lat[1:], c='b')
        ax.set_xlim(BBox[0],BBox[1])
        ax.set_ylim(BBox[2],BBox[3])


        #ax.imshow(ruh_m, zorder=0, extent = BBox, aspect= 'equal')
        plt.show()
        
    # output objective function
    obj = m.getObjective()
    return obj.getValue() 



def loop_finder(arc):
    starting_list = []
    for i in range(0,len(arc)):
        if arc[i][0] == 0:
            starting_list.append(arc[i])
    sorted_list = []
    for i in range(0, len(starting_list)):
        sorted_list.append(starting_list[i])
        j = 0
        while j < len(arc):
            j += 1
            if sorted_list[-1][1]== arc[j][0]:
                sorted_list.append(arc[j])
                j = 0 
            if sorted_list[-1][1]== 0:
                j = len(arc)
    index = []
    for i in range(0,len(sorted_list)):
        if sorted_list[i][0]== 0:
            index.append(i)
    print(len(index))
    index.append(len(sorted_list))
    loops = []
    for i in range(0,len(index)):
        loops.append(sorted_list[index[i-1]:index[i]])
    loops = loops[1:]
    return loops
    



def sensitivity(min_speed, max_speed, min_payload, max_payload, min_T, max_T, T_step, min_drones, max_drones):
    
    ### TEST 1: max speed vs objective function ###
    # Generating arrays for plotting:
    x_1 = [] # max speed
    y_1 = [] # ojective value   
    
    # Fixed variables
    T1 = 5800 # [s] total delivery duration    
    number_drones1 = 4
    maxpayload1 = 10 # [kg]
    
    # Looping through maxspeed values and appending the corresponding obj function
    for i in np.arange(min_speed,max_speed, 0.5): 
        drone = Drones("AAI RQ-7 Shadow", i, maxpayload1,  number_drones1,28.5)#(name, maxspeed, maxpayload, number_of_drones, power consumtion)         
        # updating plotting arrays
        x_1.append(i)
        y_1.append(solve_VRP(drone,client_list, T1,False))
                
        
    ### TEST 2: max payload vs objective function ###
    # Generating arrays for plotting:
    x_2 = [] # maxpayload
    y_2 = [] # objective function
    
    # Fixed variables
    T2 = 5500 # [s] total delivery duration   
    number_drones2 = 4
    maxspeed2 = 37 # [m/s]
    
    # Looping through maxpayload values and appending the corresponding obj function
    for i in range(min_payload,max_payload): # min payload of 10 [kg]
            drone2 = Drones("AAI RQ-7 Shadow", maxspeed2, i, number_drones2,28.5)#(name, maxspeed, maxpayload, number_of_drones, power consumtion)            
            # updating plotting arrays
            x_2.append(i)
            y_2.append(solve_VRP(drone2,client_list, T2,False))
            
            
    ### TEST 3: T vs objective function ###
    # Generating arrays for later plotting:
    x_3 = [] # T
    y_3 = [] # objective function
    
    # Fixed variables
    number_drones3 = 4
    maxspeed3 = 37
    maxpayload3 = 10 # [kg]
    drone3 = Drones("AAI RQ-7 Shadow", maxspeed3,  maxpayload3, number_drones3, 28.5)#(name, maxspeed, maxpayload, number_of_drones, power consumtion)
    
    # Looping through T values and appending the corresponding obj function
    for i in range(min_T,max_T,T_step): # min T of 5000 [s]
        # updating plotting arrays
        x_3.append(i)
        y_3.append(solve_VRP(drone3,client_list, i,False))
    
    
    ### TEST 4: number of drones vs objective function ###
    # Generating arrays for later plotting:
    x_4 = [] # nuumber of drones
    y_4 = [] # objective function
    
    # Fixed variables
    T4 = 7600 # [s] total delivery duration   
    maxpayload4 = 10 # [kg]
    maxspeed4 = 37 # [m/s]
    
    # Looping through number of drones and appending the corresponding obj function
    for i in range(min_drones,max_drones):
        drone4 = Drones("AAI RQ-7 Shadow", maxspeed4, maxpayload4, i, 28.5)#(name, maxspeed, maxpayload, number_of_drones, power consumtion)
        x_4.append(i)
        y_4.append(solve_VRP(drone4,client_list, T4,False))
    
    ### Plotting results from all tests ###
    plt.style.use('seaborn-darkgrid')
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    fig.subplots_adjust( wspace=0.3, hspace = 0.5)
    size = 20

    
    # Test 1
    ax1.plot(x_1,0.001*np.array(y_1), color = 'r',zorder = 1)
    ax1.scatter(x_1,0.001*np.array(y_1), marker = 'o',s = 30, zorder=2)
    ax1.tick_params(axis='both', which='major', labelsize=size)
    ax1.set_xlabel("Drone Max Speed [m/s]",fontsize=size)
    ax1.set_ylabel("Objective Function [Km]",fontsize=size)
    ax1.set_title('Objective Function vs Drone Max Speed',fontsize=size)
    text1 = f'T = {T1} [s]' + f'\nDrones = {number_drones1}' + f'\nMax Payload = {maxpayload1} [Kg] '
    ax1.text(0.74, 0.8, text1, fontsize = size, multialignment="left",horizontalalignment='center', verticalalignment='center',bbox = dict(facecolor = 'white', alpha = 1), transform=ax1.transAxes)
    
    
    # Test 2
    ax2.plot(x_2,0.001*np.array(y_2), color = 'r',zorder = 1)
    ax2.scatter(x_2,0.001*np.array(y_2), marker = 'o',s = 30, zorder=2)
    ax2.tick_params(axis='both', which='major', labelsize=size)
    ax2.set_xlabel("Drone Max Payload [kg]",fontsize=size)
    ax2.set_ylabel("Objective Function [Km]",fontsize=size)
    ax2.set_title('Objective Function vs Drone Max Payload',fontsize=size)
    text2 = f'T = {T2} [s]' + f'\nDrones = {number_drones2}' + f'\nMax Speed = {maxspeed2} [m/s] '
    ax2.text(2.05, 0.8, text2, fontsize = size, multialignment="left",horizontalalignment='center', verticalalignment='center',bbox = dict(facecolor = 'white', alpha = 1), transform=ax1.transAxes)
    
    
    # Test 3
    ax3.plot(x_3,0.001*np.array(y_3), color = 'r',zorder = 1)
    ax3.scatter(x_3,0.001*np.array(y_3), marker = 'o',s = 30, zorder=2)
    ax3.tick_params(axis='both', which='major', labelsize=size)
    ax3.set_xlabel("Total Delivery Duration [s]",fontsize=size)
    ax3.set_ylabel("Objective Function [Km]",fontsize=size)
    ax3.set_title('Objective Function vs Total Delivery Duration',fontsize=size)
    text3 = f'Drones = {number_drones2}' + f'\nMax Speed = {maxspeed2} [m/s] ' + f'\nMax Payload = {maxpayload3} [kg]'
    ax3.text(0.75, -0.7, text3, fontsize = size, multialignment="left",horizontalalignment='center', verticalalignment='center',bbox = dict(facecolor = 'white', alpha = 1), transform=ax1.transAxes)
    
    
    # Test 4
    ax4.plot(x_4,0.001*np.array(y_4), color = 'r',zorder = 1)
    ax4.scatter(x_4,0.001*np.array(y_4),  marker = 'o',s = 30, zorder=2)
    ax4.tick_params(axis='both', which='major', labelsize=size)
    ax4.set_xlabel("Number of Drones",fontsize=size)
    ax4.set_ylabel("Objective Function [Km]",fontsize=size)
    ax4.set_title('Objective Function vs Number of Drones',fontsize=size)   
    text4 = f'T = {T4} [s]' + f'\nMax Speed = {maxspeed4} [m/s] ' + f'\nMax Payload = {maxpayload4} [kg]'
    ax4.text(2.05, -0.7, text4, fontsize =size, multialignment="left",horizontalalignment='center', verticalalignment='center',bbox = dict(facecolor = 'white', alpha = 1), transform=ax1.transAxes)
    
        
    plt.show()
    
    
    
    

#SAMPLE DATASET
drone1 = Drones("AAI RQ-7 Shadow", 36.1111, 10, 4, 28.5)#(name, maxspeed, maxpayload, number_of_drones, power consumtion)
infile = open('villages_burundi', 'rb')
list = pickle.load(infile)

client_list = []
for i in range(1,10):
    client = Clients(list[i+20][0],i,list[i+20][1],list[i+20][2],list[i+20][3],list[i+20][4])
    client_list.append(client)

T = 10000 # [s] total delivery duration


if __name__== "__main__":
    solve_VRP(drone1,client_list, T, Plotting = True)
    #sensitivity(min_speed = 20, max_speed = 28, min_payload = 4, max_payload = 20, min_T = 3300, max_T = 5000, T_step = 40, min_drones = 1, max_drones = 10)
   





























