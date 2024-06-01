#!/usr/bin/env python
# coding: utf-8

# In[1]:


## Copyright (c) 2024, Mohammad Jaweed Nazary
## All tights reserved 
## University of Missouri-Columbia
## Department of Civil and Environmental Engineering


## Description: We selected a few hundred points along the sampling line and calculated the difference 
## vectors between these points and the 3D elevation points. For each sample point on the line, we 
## identified the elevation point with the minimum distance by comparing the magnitudes of the difference vectors. 
## This method ensures precise matching of elevation data to the line points. 


## Author: Mohammad Jaweed Nazary
## Contact: jaweedpy@gmail.com
## Date: 01 June 2024


def get_points(line, las_file):
    # line: a line geometry created within the las file extent using Shapely Line String method
    # las_file: an string of the name of las file to use. This file should be in the same directory as this code. 
    
    import numpy as np
    import laspy
    
    # read the las file 
    las = laspy.read(las_file)

    classifications = las.classification
    ground_points = las.points[classifications==2]   # getting the las point that are classified as ground
    
    
    x = ground_points.x
    y = ground_points.y
    z = ground_points.z
    
    
    cross_section_line = line


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
        
    return(nearest)

