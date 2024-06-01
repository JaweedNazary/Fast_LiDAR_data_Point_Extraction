#!/usr/bin/env python
# coding: utf-8

# # Five Methods to Extract LiDAR Elevation DATA Along a Line 

# In[1]:


import laspy
import numpy as np
from shapely.geometry import LineString
from shapely.geometry import Point
import time
import rasterio


# In[2]:


LiDAR_DEM = "1640_1120.img"


# In[3]:


las = laspy.read('1640_1120.las')


# In[4]:


## some information about the LiDAR file if defined
las.header.__dict__


# In[57]:


# getting only the ground points, we can also get trees, low vegitaions, 
# higher vegitaion and so on like wires and else. we just need to 
# know their class. for ground point the classificaiton number is 2
classifications = las.classification
ground_points = las.points[classifications==2]
x = ground_points.x
y = ground_points.y
z = ground_points.z


# In[59]:


x = las.points.x
y = las.points.y
z = las.points.z


# In[6]:


## Copyright (c) 2023, Mohammad Jaweed Nazary
## All tights reserved 
## University of Missouri-Columbia
## Department of Civil and Environmental Engineering


## Description: The code create randomly located and randomly orientated stright lines for the porpuse of 
## assessing thier cross sectional information. 

## NOTE: This code is written to decect levee like structures. You can adjust the number of lines and the lenght of 
## these lines to use for other porpsues.  

## Author: Mohammad Jaweed Nazary
## Contact: mjwd1991@gmail.com
## Date: 30 October 2023

import geopandas as gpd
from shapely.geometry import LineString
import matplotlib.pyplot as plt
import random

# Number of random lines to create
num_lines = 100

start_time = time.time()

# Create an empty GeoDataFrame to store the random lines
gdf = gpd.GeoDataFrame(columns=['geometry'])

# Define the bounding box for random centroid locations (adjust as needed)
min_x, max_x = np.min(x),np.max(x)
min_y, max_y = np.min(y),np.max(y)

# Generate random lines
for _ in range(num_lines):
    centroid_x = random.uniform(min_x, max_x)
    centroid_y = random.uniform(min_y, max_y)
    line_length = 300
    orientation_angle_degrees = random.uniform(0, 360)

    orientation_angle_radians = orientation_angle_degrees * (3.14159265359 / 180.0)

    line_endpoint_x1 = centroid_x - (line_length/2)*np.cos(orientation_angle_radians)
    line_endpoint_y1 = centroid_y - (line_length/2)*np.sin(orientation_angle_radians)
    line_endpoint_x2 = centroid_x + (line_length/2)*np.cos(orientation_angle_radians)
    line_endpoint_y2 = centroid_y + (line_length/2)*np.sin(orientation_angle_radians)
    
    line = LineString([(line_endpoint_x1, line_endpoint_y1), (line_endpoint_x2, line_endpoint_y2)])

    gdf = gdf.append({'geometry': line}, ignore_index=True)
end_time = time.time()
elapsed_time = end_time-start_time
print(" Elapsed time = %.3f sec" %elapsed_time)


# ## First Method
# #### Creating shapely points and then using the within distance to get the points 

# In[7]:


start_time = time.time()

# line created using Shapely and stored in GeoPandas
cross_section_line = gdf["geometry"][0]

# Converting all the LAS points to Shapely Points
shapely_points = [Point(x,y,z) for x,y,z in zip(x,y,z)]

# Using Within Distance method in Shapely to get the points along the line 
extracted_points = [point for point in shapely_points if point.dwithin(cross_section_line, 3.0)]

end_time = time.time()
elapsed_time = end_time-start_time

print("Number of points generated =  %s" %len(extracted_points))
print(" Elapsed time = %.3f sec" %elapsed_time)


# ## Second Method
# #### Using Raster files and masking the elevation raster using the Shapely line and extracting the elevation data
# 

# In[21]:


from rasterio.features import geometry_mask
# Create Shapely Point objects from LAS points
start_time = time.time()

cross_section_line = gdf["geometry"][0]



# Open the raster file
with rasterio.open(LiDAR_DEM) as src:
    # Create a mask for the LineString geometry
    mask = geometry_mask([cross_section_line], out_shape=src.shape, transform=src.transform, invert=True)

    # Read the raster data only for the pixels along the line
    raster_data_along_line = src.read(1, masked=True)[mask]



end_time = time.time()
elapsed_time = end_time-start_time
print("Number of points generated =  %s" %len(raster_data_along_line))
print(elapsed_time)


# ## Third Method
# ##### Simply stacking the las points uing numpy and get the dot product of those 

# In[120]:


start_time = time.time()
cross_section_line = gdf["geometry"][10]
# line start and end point 
x1 = cross_section_line.coords[0][0]
y1 = cross_section_line.coords[0][1]
x2 = cross_section_line.coords[1][0]
y2 = cross_section_line.coords[1][1]


# having three points on a plane we can define the plane
# as we have 2 points in 2D space we assume a third point in 
# an arbitray elevation z = 5. This will enforce a vertical plane 
# passes throght our line. 
P0 = np.array([x1, y1, 0])
P1 = np.array([x1, y1, 5])
P2 = np.array([x2, y2, 0])

# We can find its normal vector by applying cross product rule. 
N = np.cross(P1-P0, P2-P1)
unit_N = N/np.linalg.norm(N)  # unit normal vector 

# stacking all the points from the LAS file 
points = np.vstack([x, y, z]).T

# choosing on point on our sampling line
plane_point = np.array([x2, y2, 0.0])  # Example point on the plane

# vector from each elevation point to starting point on the sampling line. 
V = points - plane_point

# The distance between each elevation point and the verticl plane passing from our sampling line 
distance = np.dot(V, unit_N)

# to what distance of sampling plane we want to get elevation points 
thickness = 6.0  # Cross section thickness


# Extract cross section points
cross_section_points = points[np.abs(distance) < thickness/2]


# this section filters points that are between the start and end point of the sampling line
def is_between(point, P2, P0 ):
    D = P2- P0  # direction of normal vector is from P2 to P0
    unit_D = D/np.linalg.norm(D)
    V2 = point - P2    # distance to point on start plane
    V0 = point - P0    # distance to point on end plane

    d2 = np.dot(V2, unit_D)  # prependicular distance to start plane
    d0 = np.dot(V0, unit_D)  # prependicular distance to end plane

    return(d0>=0 and d2<=0)

#using the filter we defined above
cross_section_points = np.array([point for point in cross_section_points if is_between(point, P2, P0)])


                              
end_time = time.time()
elapsed_time = end_time-start_time
print("Number of points generated =  %s" %len(cross_section_points))
print("Elapsed time = %.3f sec" %elapsed_time)


# ## 4th Method
# Choosing few hundreds points on the sampling line and calcutate the difference vector of each point with 3d elevation points. Then choosing one point from elevation points that has the minimum distance 

# In[119]:


start_time = time.time()
cross_section_line = gdf["geometry"][10]


x1 = cross_section_line.coords[0][0]
y1 = cross_section_line.coords[0][1]
x2 = cross_section_line.coords[1][0]
y2 = cross_section_line.coords[1][1]

y_on_line = np.linspace(y1, y2, 300)
x_on_line = np.linspace(x1, x2, 300)
z_on_line = np.zeros(300)

points_on_line = np.array([x_on_line,y_on_line,z_on_line]).T

# stacking all the points from the LAS file 
points = np.vstack([x, y, z]).T
nearest = []
for point in points_on_line:
    # vector from points to each point on the line
    V =  np.abs(points - point)
    # getting values smaller than 3 ft distance
    V = V[(V[:, 0] < 3.0) | (V[:, 1] < 3.0)]
    # distance between elevation points and the point on our sampling line. 
    L = V[:, 0]**2 + V[:, 1]**2
    nearest.append(list(V[np.argmin(L)]))
end_time = time.time()
elapsed_time = end_time-start_time
print("Number of points generated =  %s" %len(nearest))
print("Elapsed time = %.3f sec" %elapsed_time)


# ## This is a Test to see if our between the point code works properly

# In[562]:


import matplotlib.pyplot as plt

get_ipython().run_line_magic('matplotlib', 'inline')

for line_No in range(num_lines):
    
    # line start and end point 
    x1 = gdf["geometry"][line_No].coords[0][0]
    y1 = gdf["geometry"][line_No].coords[0][1]
    x2 = gdf["geometry"][line_No].coords[1][0]
    y2 = gdf["geometry"][line_No].coords[1][1]

    # having three points on a plane we can define the plane
    # as we have 2 points in 2D space we assume a third point in 
    # an arbitray elevation z = 5. This will enforce a vertical plane 
    # passes throght our line. 
    P0 = np.array([x1, y1, 0])
    P1 = np.array([x1, y1, 5])
    P2 = np.array([x2, y2, 0])

    # We can find its normal vector by applying cross product rule. 
    N = np.cross(P1-P0, P2-P1)
    unit_N = N/np.linalg.norm(N)  # unit normal vector 

    # stacking all the points from the LAS file 
    points = np.vstack([x, y, z]).T

    plane_point = np.array([x2, y2, 0.0])  # Example point on the plane

    V = points - plane_point


    distance = np.dot(V, unit_N)

    thickness = 3.0  # Cross section thickness

    # Extract cross section points
    cross_section_points = points[np.abs(distance) < thickness/2]


    
    
    filtered = [point for point in cross_section_points if is_between(point, P2, P0)]
    
    x_ = []
    y_ = []
    for p in filtered:
        x_.append(p[0])
        y_.append(p[1])
        
    # Plot the GeoDataFrame
    fig = plt.plot(figsize=(10, 10))
    # plot the las LiDAR points
    plt.scatter(x,y,c=z, cmap='jet', s=1)

    plt.scatter(x1, y1, color ='r') 
    plt.scatter(x2, y2, color ='r')  
    plt.scatter(x_, y_)  
    
    
    plt.xlim(1640000, 1642800)
    plt.ylim(1120500, 1123300)
    plt.show()
    print("DDDD")


# In[69]:


cross_section_points[:,0]


# In[74]:


def get_points(line):
    
    cross_section_line = gdf["geometry"][line]
    # line start and end point 
    x1 = cross_section_line.coords[0][0]
    y1 = cross_section_line.coords[0][1]
    x2 = cross_section_line.coords[1][0]
    y2 = cross_section_line.coords[1][1]


    # having three points on a plane we can define the plane
    # as we have 2 points in 2D space we assume a third point in 
    # an arbitray elevation z = 5. This will enforce a vertical plane 
    # passes throght our line. 
    P0 = np.array([x1, y1, 0])
    P1 = np.array([x1, y1, 5])
    P2 = np.array([x2, y2, 0])

    # We can find its normal vector by applying cross product rule. 
    N = np.cross(P1-P0, P2-P1)
    unit_N = N/np.linalg.norm(N)  # unit normal vector 

    # stacking all the points from the LAS file 
    points = np.vstack([x, y, z]).T

    # choosing on point on our sampling line
    plane_point = np.array([x2, y2, 0.0])  # Example point on the plane

    # vector from each elevation point to starting point on the sampling line. 
    V = points - plane_point

    # The distance between each elevation point and the verticl plane passing from our sampling line 
    distance = np.dot(V, unit_N)

    # to what distance of sampling plane we want to get elevation points 
    thickness = 6.0  # Cross section thickness


    # Extract cross section points
    cross_section_points = points[np.abs(distance) < thickness/2]


    # this section filters points that are between the start and end point of the sampling line
    def is_between(point, P2, P0 ):
        D = P2- P0  # direction of normal vector is from P2 to P0
        unit_D = D/np.linalg.norm(D)
        V2 = point - P2    # distance to point on start plane
        V0 = point - P0    # distance to point on end plane

        d2 = np.dot(V2, unit_D)  # prependicular distance to start plane
        d0 = np.dot(V0, unit_D)  # prependicular distance to end plane

        return(d0>=0 and d2<=0)

    #using the filter we defined above
    cross_section_points = np.array([point for point in cross_section_points if is_between(point, P2, P0)])

    return(cross_section_points)    


# In[115]:


start_time = time.time()
for line in range(num_lines):
    print(line)
    cross_section_points = get_points(line)
    x__ = cross_section_points[:,0]
    y__ = cross_section_points[:,1]
    z__ = cross_section_points[:,2]
    
    h = max(x__)-min(x__)
    w = max(y__)-min(y__)
    d = max(h, w)  
    
    # Plot the GeoDataFrame
    fig, axes = plt.subplots(2, 2, figsize=(10, 10))
    
    # Plot the first scatter plot in the first subplot
    axes[1][0].scatter(x__, z__, c=z__, cmap='jet', s=1)
    axes[1][0].set_ylim(560, 600)
    axes[1][0].set_title('X-Projection')
    axes[1][0].set_xlim(min(x__), min(x__)+d)

    
    # Plot the first scatter plot in the first subplot
    axes[0][1].scatter(z__, y__, c=z__, cmap='jet', s=1)
    axes[0][1].set_xlim(560, 600)
    axes[0][1].set_title('Y-Projection')
    axes[0][1].set_ylim(min(y__), min(y__)+d)
    
    # Plot the first scatter plot in the first subplot
 
    axes[0][0].scatter(x__, y__, c=z__, cmap='jet', s=1)
    axes[0][0].set_title('TOP')
    axes[0][0].set_xlim(min(x__), min(x__)+d)
    axes[0][0].set_ylim(min(y__), min(y__)+d)

    axes[1][1].scatter(x, y, c=z, cmap='jet', s=1, vmin =560 , vmax =580, alpha = 0.5)
    axes[1][1].scatter(x__, y__,c = 'black', s = 0.4)
    axes[1][1].set_title('Landscape')
    axes[1][1].set_xlim(1640000, 1642800)
    axes[1][1].set_ylim(1120500, 1123300)
    
    
    plt.show()
    
end_time = time.time()
elapsed_time = end_time-start_time
print("Elapsed time = %.3f sec" %elapsed_time)


# ## Fifth Method 
# Using points but this time istead of calculating the nearest hardcore, we can use cKDTree to get the nearest

# In[121]:


start_time = time.time()

import numpy as np
from scipy.spatial import cKDTree

def sample_elevation_along_line(las_x, las_y, las_z, P1, P2, num_points):
    # Unpack points
    x1, y1 = P1
    x2, y2 = P2
    
    # Generate points along the line
    t_values = np.linspace(0, 1, num_points)
    line_x = x1 + t_values * (x2 - x1)
    line_y = y1 + t_values * (y2 - y1)
    
    # Create a k-d tree for fast nearest neighbor search
    las_points = np.column_stack((las_x, las_y))
    kdtree = cKDTree(las_points)
    
    # Query the k-d tree for the nearest neighbors
    _, indices = kdtree.query(np.column_stack((line_x, line_y)), k=1)
    
    # Get the corresponding elevations
    sampled_z = las_z[indices]
    
    return line_x, line_y, sampled_z


cross_section_line = gdf["geometry"][10]
# line start and end point 
x1 = cross_section_line.coords[0][0]
y1 = cross_section_line.coords[0][1]
x2 = cross_section_line.coords[1][0]
y2 = cross_section_line.coords[1][1]


# having three points on a plane we can define the plane
# as we have 2 points in 2D space we assume a third point in 
# an arbitray elevation z = 5. This will enforce a vertical plane 
# passes throght our line. 
P1 = np.array([x1, y1])
P2 = np.array([x2, y2])
# Example usage
las_x = x  # x-coordinates of LAS data
las_y = y  # y-coordinates of LAS data
las_z = z  # z-coordinates (elevation) of LAS data
num_points = 300  # Number of points to sample

line_x, line_y, sampled_z = sample_elevation_along_line(las_x, las_y, las_z, P1, P2, num_points)

# line_x, line_y are the coordinates of the sampled points
# sampled_z contains the elevation data for these points

end_time = time.time()
elapsed_time = end_time-start_time
print("Number of points generated =  %s" %len(sampled_z))
print("Elapsed time = %.3f sec" %elapsed_time)

