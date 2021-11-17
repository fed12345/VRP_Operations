import numpy as np

#DATASET 
#Here we create the dataset
#Initialize
class Drones():
    def __init__(self, name, maxspeed, maxpayload, number_of_drones):
        self.name = name
        self.maxspeed = maxspeed
        self.maxpayload = maxpayload
        self.number_of_drones = number_of_drones

class Clients():
    numeber_of_clients = 0
    def __init__(self, number, x_coord, y_coord, demand):
        self.number = number
        self.x_coord = x_coord
        self.y_coord = y_coord
        self.demand = demand
        Clients.numeber_of_clients += 1