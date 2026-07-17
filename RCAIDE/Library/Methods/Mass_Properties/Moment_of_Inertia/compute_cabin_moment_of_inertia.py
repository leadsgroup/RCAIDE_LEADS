# RCAIDE/Library/Methods/Mass_Properties/Moment_of_Inertia/compute_cabin_moment_of_inertia.py 
# 
# Created:  Dec 2025, M. Clarke  
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
from RCAIDE.Library.Methods.Geometry.LOPA.compute_layout_of_passenger_accommodations import compute_layout_of_passenger_accommodations

# python imports
import numpy as np  
import trimesh
# ----------------------------------------------------------------------------------------------------------------------
#  Compute Cabin Moment of Inertia
# ----------------------------------------------------------------------------------------------------------------------   
def compute_cabin_moment_of_inertia(cabin,fuselage,center_of_gravity = np.array([[0,0,0]])):  
    """
    Computes the moment of inertia tensor for the cabin.
    
    Parameters
    ----------
    center_of_gravity : list, optional
        Reference point coordinates for moment calculation, defaults to [[0, 0, 0]]
    
    Returns
    -------
    I : ndarray
        3x3 moment of inertia tensor in kg*m^2
    
    See Also
    --------
    RCAIDE.Library.Methods.weights.vehicle.moments_of_inertia.compute_fuselage_moment_of_inertia
        Implementation of the moment of inertia calculation
    """

    # Gather x,y points of the cabin
    if fuselage.layout_of_passenger_accommodations == None:
        compute_layout_of_passenger_accommodations(fuselage)
    half_coords = fuselage.layout_of_passenger_accommodations.cabin_area_coordinates # one side of the x,y coordiantes of the cabin
    coordinates = np.vstack([np.hstack([half_coords[:,0], half_coords[::-1,0]]), np.hstack([half_coords[:,1], -1*half_coords[::-1,1]])]) # Full coordinates

    # Create 3D mesh of the cabin area
    L       = max(cabin.height,0.1) # cabin height with arbitrarily small value to avoid 0 thickness error
    pts1    = np.column_stack((coordinates[0], coordinates[1], np.zeros(len(coordinates[0]))))   # z = 0 top surface points
    pts2    = np.column_stack((coordinates[0], coordinates[1], np.full(len(coordinates[0]), L))) # z = L bottom surface points
    all_pts = np.vstack([pts1, pts2]) # Combine all points for the 3D geometry

    solid_segment = trimesh.convex.convex_hull(all_pts) # Convex hull → watertight volume mesh

    R = trimesh.transformations.rotation_matrix(np.deg2rad(90), [1, 0, 0], [0, 0, 0]) # Rotate to match the RCAIDE aircraft axes convention
    solid_segment.apply_transform(R)

    # Calculate MOI of the cabin
    mass                  = cabin.mass_properties.mass
    solid_segment.density = mass / solid_segment.volume # Assign the density of the solid so that the total mass is equal to the assigned mass
    I_compoment           = solid_segment.moment_inertia
    
    cabin.mass_properties.moments_of_inertia.tensor =  I_compoment

    return  cabin.mass_properties.moments_of_inertia.tensor, cabin.mass_properties.mass