#!/usr/bin/env python
# coding: utf-8

# In[1]:


## Copyright (c) 2024, Mohammad Jaweed Nazary
## All tights reserved 
## University of Missouri-Columbia
## Department of Civil and Environmental Engineering


## Description: Here, we stacked the LiDAR points into NumPy arrays and projected to the plane passing from our line. 
## This can be done by computing the dot product of each point vector in 3D space with the unit normal vector of the
## plane. This method is the fasted and is very precise. It takes few milli seconds to exrtrat about 500 points from 
## LiDAR las data


## Author: Mohammad Jaweed Nazary
## Contact: jaweedpy@gmail.com
## Date: 01 June 2024


def get_points_dot_product(line, las_file):
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

