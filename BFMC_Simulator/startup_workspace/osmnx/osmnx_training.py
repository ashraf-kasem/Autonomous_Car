#!/usr/bin/env python
# coding: utf-8

# In[35]:


# importing osmnx library
import osmnx as ox
# matplot to draw
import matplotlib.pyplot as plt
import networkx as nx
import geopandas as gpd
import pandas as pd
from pyproj import CRS


# In[36]:


# Specify the name that is used to seach for the data
place_name = "miskolc university"


# In[37]:


# Fetch OSM street network from the location
# name have to be  geocodable which means that
# the place name should exist in the OpenStreetMap database 
# using function named graph_from_place
# return graph object
graph = ox.graph_from_place(place_name)


# In[38]:


type(graph)


# In[39]:


# Plot the streets
# using the plot graph function from the osmnx library
# return fig, axes object
fig, ax = ox.plot_graph(graph)


# In[40]:


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
# # save a graph
# G = ox.graph_from_place('Piedmont, CA, USA', network_type='drive')
# ox.save_graphml(graph, filepath='miskolc_osmnx.osm')
# # load a graph
# x = ox.load_graphml( filepath='miskolc_osmnx_road.osm')
# ox.plot_graph(x)


# In[44]:


# you can load an xml model after dowload it from openstreetmaps
# create graph from .osm extract file
G = ox.graph_from_xml("highway.osm")
fig, ax = ox.plot_graph(G)


# In[43]:


ox.save_graphml(G, filepath='highwaygraph.osm')


# In[183]:


# OSMnx provides a convenient function graph_to_gdfs()
# that can convert the graph into two separate GeoDataFrames where the first one contains the information about
# the nodes and the second one about the edge.
# Retrieve nodes and edges
nodes, edges = ox.graph_to_gdfs(graph)


# In[184]:


# see the structure of the geodataframes
# return objects from geopandas 
print(type(nodes))
print(nodes.head())
print(edges.head())


# In[185]:


# Get place boundary related to the place name as a geodataframe
# we can use this function to retrieve an area in gdf format directly
area = ox.geocode_to_gdf(place_name)


# In[186]:


# you can see shape and type of this gdf object
print(type(area))
print(area.shape)
print(area)


# In[187]:


# Plot the area:
area.plot()


# In[188]:


# It is also possible to retrieve other
# types of OSM data features with OSMnx such as buildings
# List key-value pairs for tags
tags = {'building': True}
buildings = ox.geometries_from_place(place_name, tags)


# In[189]:


# print info
print(type(buildings))
print(buildings.shape)
print(buildings.head())


# In[190]:


# Let’s check how many building footprints we received:
print(len(buildings))


# In[191]:


# you can know how manny columns
print(buildings.columns)


# In[192]:


# It is also possible to retrieve other types of geometries 
# from OSM using the geometries_from_place by passing different tags.
# Point-of-interest (POI) is a generic concept that describes point locations 
# that represent places of interest.

# List key-value pairs for tags
tags = {'amenity': 'restaurant'}

# Retrieve restaurants
restaurants = ox.geometries_from_place(place_name, tags)

# How many restaurants do we have?
print(len(restaurants))

# print some data to show the structure
print(restaurants.columns)
print(type(restaurants))
print(restaurants.shape)
print(restaurants.head())


# In[193]:


# Available columns
restaurants.columns.values


# In[194]:


# Select some useful cols and print
cols = ['name', 'outdoor_seating', 'smoking', 'type']

# Print only selected cols
restaurants[cols]


# In[34]:


# Let’s create a map out of the streets, buildings,
# restaurants, and the area Polygon but let’s exclude the nodes 
fig, ax = plt.subplots(figsize=(12,12))
# Plot the footprint
area.plot(ax=ax, facecolor='black')

# Plot street edges
edges.plot(ax=ax, linewidth=1, edgecolor='dimgray')

# Plot buildings
buildings.plot(ax=ax, facecolor='silver', alpha=0.6)

# Plot restaurants
restaurants.plot(ax=ax, color='yellow', alpha=0.4, markersize=10)
plt.tight_layout()


# In[ ]:





# In[196]:


# get park places
# List key-value pairs for tags
tags = {'leisure': 'park'}

# Get the data
parks = ox.geometries_from_place(place_name, tags)

# Check the result
print("Retrieved", len(parks), "objects")


# In[197]:


# print the info
print(parks)
print(parks.columns.values)


# In[198]:


# Let’s create a map out of the streets, buildings, parks
fig, ax = plt.subplots(figsize=(12,12))

# Plot the footprint
area.plot(ax=ax, facecolor='black')

# Plot street edges
edges.plot(ax=ax, linewidth=1, edgecolor='dimgray')

# Plot buildings
buildings.plot(ax=ax, facecolor='silver', alpha=0.6)

# Plot parks
parks.plot(ax=ax, color='yellow', alpha=0.4, markersize=10)
plt.tight_layout()


# In[199]:


#  only the drivable network
graph = ox.graph_from_place(place_name, network_type='drive')

# Plot the graph:
fig, ax = ox.plot_graph(graph)


# In[200]:


# Retrieve only edges from the graph
edges = ox.graph_to_gdfs(graph, nodes=False, edges=True)


# In[201]:


# some info about the roads
# Check columns
print(edges.shape)
print(edges.columns)
print(edges.head())


# In[202]:


# to know what are the types of the roads
edges['highway'].value_counts()


# In[203]:


# As the data is in WGS84 format, we might want to reproject
# our data into a metric system before proceeding to the shortest
# path analysis. We can re-project the graph from latitudes and longitudes
# to an appropriate UTM zone using the project_graph() function from OSMnx.

# I think that step is not nececcary if I worked with bosch map
# because it already uses two deminsional coordinate system like UTM

# Project the data
graph_proj = ox.project_graph(graph) 


# In[204]:


print(type(graph_proj))
ox.plot_graph(graph_proj)


# In[205]:


# Get Edges and Nodes from the projected graph
nodes_proj, edges_proj = ox.graph_to_gdfs(graph_proj, nodes=True, edges=True)
# you can notice that the coordination have been changed to the UTM system
print(edges_proj.head())
print(nodes_proj.head())


# In[206]:


# show some info
print("Coordinate system:", edges_proj.crs)


# In[207]:


# to see epsg code
CRS(edges_proj.crs).to_epsg()


# In[208]:


# Calculate network statistics
stats = ox.basic_stats(graph_proj, circuity_dist='euclidean')
stats


# In[209]:


# the surrounding area
# Get the Convex Hull of the network
convex_hull = edges_proj.unary_union.convex_hull

# Show output
convex_hull


# In[210]:


# Here, we combine the basic and extended statistics
# into one pandas Series to keep things in more compact form.

# Calculate the area
area = convex_hull.area

# Calculate statistics with density information
stats = ox.basic_stats(graph_proj, area=area)
extended_stats = ox.extended_stats(graph_proj, ecc=True, cc=True)

# Add extened statistics to the basic statistics
for key, value in extended_stats.items():
    stats[key] = value
    
# Convert the dictionary to a Pandas series for a nicer output
pd.Series(stats)


# In[211]:


# Set place name to be the source to calculate the shortest path to the 
# distination

place = "Uni Hotel Miskolc"

# Geocode the place name
geocoded_place = ox.geocode_to_gdf(place)

# Check the result
geocoded_place


# In[212]:


#  From here, we can get the centroid as the source location of our shortest path analysis. 
# However, we first need to project the data into the correct crs:

# I think this step is not neccessary in bosch map
# because the map already in two deminsional coordination system

# Re-project 
geocoded_place.to_crs(CRS(edges_proj.crs), inplace=True)


# In[213]:


# Get centroid as shapely point
origin = geocoded_place["geometry"].centroid.values[0]

print(origin)


# In[214]:


# the distination point
# I choosed a point in an intersection

place2 = "miskolc university A/6"

# Geocode the place name
geocoded_place2 = ox.geocode_to_gdf(place2)

# Check the result
geocoded_place2

# Re-project 
geocoded_place2.to_crs(CRS(edges_proj.crs), inplace=True)

# Get centroid as shapely point
destination = geocoded_place2["geometry"].centroid.values[0]

print(destination)


# In[215]:


# According to the documentation of this function, we need to
# parse Point coordinates as coordinate-tuples in this order:
#     latitude, longitude(or y, x). 

# Get origin x and y coordinates
orig_xy = (origin.y, origin.x)

# Get target x and y coordinates
target_xy = (destination.y, destination.x)

# Find the node in the graph that is closest to the origin point (here, we want to get the node id)
orig_node_id = ox.get_nearest_node(graph_proj, orig_xy, method='euclidean')
print(orig_node_id)

# Find the node in the graph that is closest to the target point (here, we want to get the node id)
target_node_id = ox.get_nearest_node(graph_proj, target_xy, method='euclidean')
print(target_node_id)


# In[216]:


# Retrieve the rows from the nodes GeoDataFrame based on the node id (node id is the index label)
orig_node = nodes_proj.loc[orig_node_id]
target_node = nodes_proj.loc[target_node_id]

print(type(orig_node))


# In[217]:


# Let’s also create a GeoDataFrame that contains these points
# these two points now are geo frames
# Create a GeoDataFrame from the origin and target points
od_nodes = gpd.GeoDataFrame([orig_node, target_node], geometry='geometry', crs=nodes_proj.crs)
od_nodes.head()


# In[218]:


# find the shortest path between the 
# origin and target locations by using the shortest_path()
# function of networkx. With weight -parameter we can specify that 'length'

# Calculate the shortest path
route = nx.shortest_path(G=graph_proj, source=orig_node_id, target=target_node_id, weight='length')

# Show what we have
print(route)


# In[219]:


# Plot the shortest path
fig, ax = ox.plot_graph_route(graph_proj, route)


# In[ ]:




