#!/usr/bin/env python3
# coding: utf-8


# importing osmnx library
import osmnx as ox
# matplot to draw
import matplotlib.pyplot as plt
import networkx as nx
import geopandas as gpd
import pandas as pd
from pyproj import CRS
import random
import cv2


# tis code to configer the ox library to make it aple to
# load and save the model
# this section uses grph ml files
utn = ox.settings.useful_tags_node
oxna = ox.settings.osm_xml_node_attrs
oxnt = ox.settings.osm_xml_node_tags
utw = ox.settings.useful_tags_way
oxwa = ox.settings.osm_xml_way_attrs
oxwt = ox.settings.osm_xml_way_tags
utn = list(set(utn + oxna + oxnt))
utw = list(set(utw + oxwa + oxwt))
ox.config(all_oneway=True, useful_tags_node=utn, useful_tags_way=utw)
# save a graph
# G = ox.graph_from_place(place_name, network_type='drive')
# ox.save_graphml(G, filepath='miskolc_osmnx_road.osm')
# load a graph
x = ox.load_graphml( filepath='Bosch_updated_osm_map.osm')
# ox.plot_graph(x)
nodes, edges = ox.graph_to_gdfs(x)




# Get origin x and y coordinates
orig_xy = (5 , 5)

# Get target x and y coordinates
target_xy = (9, 9)

# Find the node in the graph that is closest to the origin point (here, we want to get the node id)
orig_node_id = ox.get_nearest_node(x, orig_xy, method='euclidean')
print(orig_node_id)

# Find the node in the graph that is closest to the target point (here, we want to get the node id)
target_node_id = ox.get_nearest_node(x, target_xy, method='euclidean')
print(target_node_id)






# make the back ground balck
plt.rcParams['axes.facecolor'] = 'black'

# # Let’s create a map
fig, ax = plt.subplots(figsize=(2, 2))

# # Plot street edges and nodes
edges.plot(ax=ax, linewidth=1, edgecolor='dimgray')
# plt.plot(5, 5, 'go--', linewidth=0.5, markersize=2)
# nodes.plot(ax=ax, linewidth=1, edgecolor='yellow')

# # define the limits of the plot on y and x
ax.set_ylim(bottom=15)
ax.set_ylim(top=0)
ax.set_xlim(left=0)
ax.set_xlim(right=15)
# set the surround color to black
fig.patch.set_facecolor('xkcd:black')
plt.tight_layout(pad=0)
# Calculate the shortest path
source= random.choice([9, 8, 141])
route = nx.shortest_path(G=x, source=orig_node_id, target=target_node_id, weight='length' )
# Plot the shortest path
fig, ax = ox.plot_graph_route(x, route, ax=ax, route_linewidth=1,show=False,
                              route_alpha=0.5 , save=True, filepath='image.png', orig_dest_size=25)
# Show what we have
print(type(route))
route = [ str(x) for x in route]
print(route)
# for number in len(route):
#     route[number] = str(route[number])

client_data = ' '.join(route)
print(client_data)

img = cv2.imread('image.png')
img= cv2.resize(img,(350, 400))
# show the image on the previw
cv2.imshow("Path Planing", img)
key = cv2.waitKey(1)



# Show what we have
print(route)

