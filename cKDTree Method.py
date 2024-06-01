#!/usr/bin/env python
# coding: utf-8

# In[1]:


## Copyright (c) 2024, M Jaweed Nazary
## All tights reserved 
## University of Missouri-Columbia
## Department of Civil and Environmental Engineering


## Description:  Using the cKDTree from the SciPy library, we created a k-dimensional tree to find the 
## nearest neighbor points from the LiDAR data to the points on the line efficiently. This method involves 
## spatial indexing to perform fast nearest neighbor searches, making it highly efficient for large datasets.


## Author: Mohammad Jaweed Nazary
## Contact: jaweedpy@gmail.com
## Date: 01 June 2024



def get_points_cKDTree(line, las_file,num_points):
    # line: a line geometry created within the las file extent using Shapely Line String method
    # las_file: an string of the name of las file to use. This file should be in the same directory as this code. 
    # num_points: an integer, how points to get along the line
    
    from scipy.spatial import cKDTree
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
    
    return np.array([line_x, line_y, sampled_z])



# In[ ]:




