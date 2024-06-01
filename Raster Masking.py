#!/usr/bin/env python
# coding: utf-8

# In[ ]:


## Copyright (c) 2024, M. Jaweed Nazary
## All tights reserved 
## University of Missouri-Columbia
## Department of Civil and Environmental Engineering


## Description: This code uses raster files to represent elevation data. We masked the elevation
## raster using a Shapely line to extract elevation data directly from the raster cells that intersected with the line. 
## This approach is very fast, 250 times faster thatn shapely method, However comes with some problem due indexing 
## errors.



## Author: Mohammad Jaweed Nazary
## Contact: jaweedpy@gmail.com
## Date: 01 June 2024


def get_points_raster_masking(DEM, line):
    
    ## DEM: Name if the raster file to be used for this process.    
    ## line: a line geometry created within the las file extent using Shapely Line String method
    from rasterio.features import geometry_mask
    import rasterio

    cross_section_line = line



    # Open the raster file
    with rasterio.open(DEM) as src:
        # Create a mask for the LineString geometry
        mask = geometry_mask([cross_section_line], out_shape=src.shape, transform=src.transform, invert=True)

        # Read the raster data only for the pixels along the line
        raster_data_along_line = src.read(1, masked=True)[mask]



    return(raster_data_along_line)

