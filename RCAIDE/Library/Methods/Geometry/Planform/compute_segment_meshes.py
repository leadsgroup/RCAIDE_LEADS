# RCAIDE/Library/Methods/Geometry/Planform/compute_segment_meshes.py 
# 
# Created:  Dec 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# package imports 
import numpy as np  
from   copy import deepcopy
import shapely.geometry as geom
from shapely import Polygon, box 
import trimesh


# ----------------------------------------------------------------------------------------------------------------------
#  Compute segment meshes 
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_segment_meshes(x_in,y_in, x_out, y_out, L, spanwise_shift):

    points_out = list(zip(x_out, y_out))
    poly_out = Polygon(points_out)

    points_in = list(zip(x_in, y_in))
    poly_in = Polygon(points_in) 
    
    # STEP 1: Build 3D point clouds for both sections
    x1, y1 = poly_in.exterior.xy
    x2, y2 = poly_out.exterior.xy

    pts1 = np.column_stack((x1[:-1], y1[:-1], np.zeros(len(x1)-1)))   # z = 0
    pts2 = np.column_stack((x2[:-1], y2[:-1], np.full(len(x2)-1, L))) # z = L

    # STEP 2: Combine all points
    all_pts = np.vstack([pts1, pts2])

    # STEP 3: Convex hull → watertight volume mesh
    solid_segment = trimesh.convex.convex_hull(all_pts)

    # Apply spanwise translation AFTER orientation fix
    T = np.eye(4)
    T[0, 3] = 0.0
    T[1, 3] = 0.0
    T[2, 3] = spanwise_shift
    solid_segment.apply_transform(T)
    R = trimesh.transformations.rotation_matrix(np.deg2rad(90), [1, 0, 0], [0, 0, 0])
    solid_segment.apply_transform(R)
    
    return solid_segment