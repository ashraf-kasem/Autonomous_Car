#!/usr/bin/env python3

# importing osmnx library
import osmnx as ox
# matplot to draw
import matplotlib.pyplot as plt
import networkx as nx
import geopandas as gpd
import pandas as pd
from pyproj import CRS
import os
import socket
import cv2

# to make sure port 8000 is free
os.system("sudo fuser -k 8000/tcp")




"""                                                    Loading the OSM MAP                             """
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
# make the edges and nodes a data frames
nodes, edges = ox.graph_to_gdfs(x)



"""                                                    Server program                           """
# server program
def server_program():
	# global conn
	# get the hostname
	host = socket.gethostname()
	# initiate port no below 1024
	port = 8000
	# build TCP object
	server_socket = socket.socket()
	# look closely. The bind() function takes tuple as argument
	# bind host address and port together
	server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	server_socket.bind((host, port))

	# configure how many client the server can listen simultaneously
	server_socket.listen(2)
	print("Waiting for a car to connect ... ")
	conn, address = server_socket.accept()  # accept new connection
	print("Connection from: " + str(address))


	# the first data received is the destination coordinates
	client_distination =  conn.recv(1024).decode()
	# process the received data
	client_postion = client_distination.split()
	distination_x = eval(client_postion[0])
	distination_y = eval(client_postion[1])
	# print the car position in the server terminal
	print("car destination is: ", distination_x, distination_y)
	client_data = "HI FROM SERVER"
	# send client_data to the client
	conn.send(client_data.encode())


	# the main loop
	while True:
		# receive client_data stream. it won't accept client_data packet greater than 1024 bytes
		client_data = conn.recv(1024).decode()
		if not client_data:
			# if client_data is not received break
			break
		# process the received data
		client_postion = client_data.split()
		car_x = eval(client_postion[0])
		car_y = eval(client_postion[1])







		"""                 the part of showing the Map in real time                   """

		# Get origin x and y coordinates
		orig_xy = (car_y, car_x)
		# Get target x and y coordinates
		target_xy = (distination_y, distination_x)
		# Find the node in the graph that is closest to the origin point (here, we want to get the node id)
		orig_node_id = ox.get_nearest_node(x, orig_xy, method='euclidean')
		# Find the node in the graph that is closest to the target point (here, we want to get the node id)
		target_node_id = ox.get_nearest_node(x, target_xy, method='euclidean')


		# make the back ground balck
		plt.rcParams['axes.facecolor'] = 'black'
		# # Let’s create a map
		fig, ax = plt.subplots(figsize=(2, 2))
		# # Plot street edges and nodes
		edges.plot(ax=ax, linewidth=1, edgecolor='dimgray')

		plt.plot(car_x, car_y, 'go--', linewidth=0.5, markersize=2)
		plt.text(car_x - 0.3, car_y + 0.5, 'car', fontsize=3, color="white")
		plt.plot(distination_x, distination_y, 'rx--', linewidth=0.5, markersize=3)
		plt.text(distination_x - 0.3, distination_y + 0.5, 'dist', fontsize=3, color="white")

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
		route = nx.shortest_path(G=x, source=orig_node_id, target=target_node_id, weight='length')
		# Plot the shortest path
		fig, ax = ox.plot_graph_route(x, route, ax=ax, route_linewidth=1, show=False,
									  route_alpha=0.5, save=True, filepath='image.png', orig_dest_size=25)
		# Show what we have
		# print(route)
		img = cv2.imread('image.png')
		img = cv2.resize(img, (350, 400))
		# show the image on the previw
		cv2.imshow("Path Planing", img)
		key = cv2.waitKey(1)









		# print the car postion in the server terminal
		print("car postion is: " + client_postion[0], client_postion[1] )
		route = [ str(x) for x in route]
		client_data = ' '.join(route)
		# send client_data to the client
		conn.send(client_data.encode())
	conn.close()  # close the connection




if __name__ == '__main__':
	server_program()

