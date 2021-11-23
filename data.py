import numpy as np
import requests
import json
import pickle
#DATASET 
#Here we create the dataset
rnd = np.random
rnd.seed(0)

#Initialize
class Drones():
    def __init__(self, name, maxspeed, maxpayload, number_of_drones):
        self.name = name
        self.maxspeed = maxspeed
        self.maxpayload = maxpayload
        self.number_of_drones = number_of_drones

class Clients():
    numeber_of_clients = 0
    depo_location = (-3.4333, 29.9000)
    def __init__(self,id, number, name, lat, long, demand):
        self.id = id
        self.number = number
        self.name = name
        self.lat = lat
        self.long = long
        self.demand = demand
        Clients.numeber_of_clients += 1

def create_dataset():
    overpass_url = "http://overpass-api.de/api/interpreter"
    overpass_query = """
    [out:json];area[name="Burundi"];(node[place="village"](area););out;
    """
    response = requests.get(
        overpass_url, 
        params={'data': overpass_query}
    )

    coords = []
    if response.status_code == 200:
        data = response.json()
        places = data.get('elements', [])
        for place in places:
            mydic = place['tags']
            try:
                a = mydic['name']
            except KeyError:
                a = 'unnamed'

            coords.append((place['id'],a,place['lat'], place['lon'],2))
        print ("Got %s village coordinates!" % len(coords))
    else:
        print("Error")

    filename = 'villages_ghana'
    outfile = open(filename,'wb')
    pickle.dump(coords,outfile)
    outfile.close()

#create_dataset()