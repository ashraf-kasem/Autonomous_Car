#!/usr/bin/env python
# coding: utf-8

# In[107]:


import bs4 as bs
import math


# In[108]:


import lxml


# In[368]:


# open the map in xml format
map_path = "new 1 - Copy.osm"
xml_file = open(map_path,'r')
# building a soup object to parse
map = bs.BeautifulSoup( xml_file , 'xml')

# extracting all nodes tags and build a dictinary contain all nodes id with
# its related coordination
nodes_dict = {}
for node in map.find_all('node'):
    # get all data tags from the node tag
    x,y = node.find_all('data')
    # get the value of each data tag
    x  =  eval(str(x.text))
    y  =  eval(str(y.text))
    # get the id of the each node
    id =  eval(str(node['id']))
    # add element to the dictinary contain  { key = id : value = [x,y] }
    nodes_dict[ id ]   =  [ x , y ]

# show the dictinary format
# print(nodes_dict)


for k,v in nodes_dict.items():
    print(v[0],v[1], end=', ')
    
print() 
# code to caculate the legnth between many points
total_distance = 0
# pust the first node in a variable
previos_node = list(nodes_dict.values())[0]
# iterate over all values and get the distance between every two points 
for v in nodes_dict.values():
    # calculate the distance between the current point and the previos one 
    total_distance = total_distance + math.dist(v, previos_node)
#     print(math.dist(v, previos_node))
    # put the current valuse as a previos value for the next iteration
    previos_node = v

# check values
print(len(nodes_dict))   
print()
print(total_distance)


# In[240]:





# In[297]:



# this code is to change d0 to d4 and d1 to d5


#input file
fin = open(map_path, "rt")
#output file to write the result to
fout = open("updated" + map_path, "wt")
#for each line in the input file
for line in fin:
        #read replace the string and write to output file
        if 'd0' in line:
            fout.write(line.replace('"d0"', '"d5"'))
        elif 'd1' in line:
            fout.write(line.replace('"d1"', '"d4"'))
        else:
            fout.write(line)
        


#close input and output files
fin.close()
fout.close()


# In[ ]:




