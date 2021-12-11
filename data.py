import numpy as np
import requests
import json
import pickle
from geopy.distance import geodesic
import pandas as pd
#DATASET 
#Here we create the dataset
rnd = np.random
rnd.seed(0)

#Initialize
class Drones():
    def __init__(self, name, maxspeed, maxpayload, number_of_drones, power):
        self.name = name
        self.maxspeed = maxspeed
        self.maxpayload = maxpayload
        self.number_of_drones = number_of_drones
        self.power = power

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

def create_dataset(): #https://wiki.openstreetmap.org/wiki/Overpass_API/Overpass_API_by_Example
    overpass_url = "http://overpass-api.de/api/interpreter"
    overpass_query = """
    [out:json];area[name="Burundi"];(node[place="village"](area););out;
    """
    response = requests.get(
        overpass_url, 
        params={'data': overpass_query}
    )
    depo_lat = -3.4333
    depo_long = 29.9000
    coords = []
    if response.status_code == 200:
        data = response.json()
        places = data.get('elements', [])
        
        arr1 = []
        arr2 = []
        arr3 = []
        arr4 = []
        arr5 = []
        arr6 = []
        arr7 = []
        
        
        i = 1
        for place in places:
            mydic = place['tags']
            try:
                a = mydic['name']
            except KeyError:
                a = 'unnamed'
                
                
            dist_from_depo = geodesic((depo_lat,depo_long),(place['lat'],place['lon'])).km
            arr1.append(place['id'])
            arr2.append(a)
            arr3.append(place['lat'])
            arr4.append(place['lon'])
            arr5.append(dist_from_depo)
            arr6.append(2)
            arr7.append(i)
            i = i + 1
            
            coords.append((place['id'],a,place['lat'], place['lon'],2))
        #print(arr1)
        df = pd.DataFrame({'id': arr1,'number': arr7, 'Name':arr2, 'lat': arr3,
                           'lon': arr4, 'dist_from_depo': arr5, 'demand': arr6})
        #df.to_csv('villages.csv',index = False)
        print (" %s village" % len(arr1))
    else:
        print("Error")

    filename = 'villages_burundi'
    outfile = open(filename,'wb')
    pickle.dump(coords,outfile)
    outfile.close()

create_dataset()












