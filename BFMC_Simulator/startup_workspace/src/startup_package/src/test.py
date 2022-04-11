

# !/usr/bin/python

"""
These are the libraries you need to import in your project in order to
be able to communicate with the Gazebo simulator
"""
# Only for use in Python 2.6.0a2 and later
from __future__ import print_function
import bs4 as bs
import os
import time
from math import sqrt, hypot
# current_time =  round (time.time() , 2)
# print(current_time)
# current = [3,4]
# another = [4, 4 ]
# print(hypot(current[0]-another[0],current[1]-another[1]))
#

x = input("X is: ")
y = input("Y is: ")

car_postion = [x,y]


# open the map in xml format
map_path = os.getcwd() + "/map.xml"
xml_file = open(map_path, 'r')
# building a soup object to parse
map = bs.BeautifulSoup(xml_file, 'xml')

# extracting all nodes tags and build a dictinary contain all nodes id with
# its related coordination
nodes_dict = {}
for node in map.find_all('node'):
    # get all data tags from the node tag
    x, y = node.find_all('data')
    # get the value of each data tag
    x = eval(str(x.text))
    y = eval(str(y.text))
    # get the id of the each node
    id = eval(str(node['id']))
    # add element to the dictinary contain  { key = id : value = [x,y] }
    nodes_dict[id] = [x, y]

# show the dictinary format
# print(nodes_dict)

# extracting all edges tags and build a dictinary contain all edges and
# the situation of ech edge
dashed_edges = []
edges_dict = {}
for edge in map.find_all('edge'):
    # get  the value of data tag from the edge tag
    status = edge.data.text
    status = str(status)
    # get the value of source and target attrebutes in the edge tag
    source = eval(str(edge['source']))
    target = eval(str(edge['target']))
    # add element to the dictinary contain  { key = (source,target) : value = status }
    edges_dict[(source, target)] = status
    if status == 'True' :
        dashed_edges.extend([source,target])

# print the edge dict to show the format
print(edges_dict)
# delete all repeated values
dashed_edges= list( set(dashed_edges) )
# print(dashed_edges)

# loop over all dashed points
for point in dashed_edges:
    diffrenc_postion = hypot(nodes_dict[point][0]-car_postion[0],nodes_dict[point][1]-car_postion[1])
    print(point, diffrenc_postion)

