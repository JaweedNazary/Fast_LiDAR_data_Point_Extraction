#!/usr/bin/env python
# coding: utf-8

# In[1]:


## Copyright (c) 2024, M Jaweed Nazary
## All tights reserved 
## University of Missouri-Columbia
## Department of Civil and Environmental Engineering


## Description: This code Shapely points from input las data and uses the within distance method inherent to shapely
## geometry to identify points within a specified distance from the line. 


## Author: Mohammad Jaweed Nazary
## Contact: jaweedpy@gmail.com
## Date: 01 June 2024



def get_points_shapely_method(line, las_file):   
    # line: a line geometry created within the las file extent using Shapely Line String method
    # las_file: an string of the name of las file to use. This file should be in the same directory as this code. 
    
    from shapely.geometry import Point
    import numpy as np
    import laspy
    
    # read the las file 
    las = laspy.read(las_file)

    classifications = las.classification
    ground_points = las.points[classifications==2]   # getting the las point that are classified as ground
    
    
    x = ground_points.x
    y = ground_points.y
    z = ground_points.z
    
    # Converting all the LAS points to Shapely Points
    shapely_points = [Point(x,y,z) for x,y,z in zip(x,y,z)]

    # Using Within Distance method in Shapely to get the points along the line 
    extracted_points = [point for point in shapely_points if point.dwithin(line, 3.0)]

    return(extracted_points)

