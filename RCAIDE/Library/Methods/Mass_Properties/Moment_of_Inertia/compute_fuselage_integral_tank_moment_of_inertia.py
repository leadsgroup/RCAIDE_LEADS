# RCAIDE/Library/Methods/Mass_Properties/Moment_of_Inertia/compute_fuselage_integral_tank_moment_of_inertia.py 
# 
# Created:  Jan 2026, M. Clarke  
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE 

# package imports 
import numpy as np
import trimesh

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Wing Moment of Inertia
# ----------------------------------------------------------------------------------------------------------------------  
def compute_fuselage_integral_tank_moment_of_inertia(fuel_tank,fuselage, center_of_gravity = [[0, 0, 0]]):


    # intialize matrices
    fuel           = fuel_tank.fuel
    tank_mass      = fuel_tank.mass_properties.mass
    fuel_mass      = fuel.mass_properties.mass
    I_local_fuel   = np.zeros((3, 3)) 
    I_global_fuel  = np.zeros((3, 3)) 
    I_local_tank   = np.zeros((3, 3))    
    
    # Collect all segment tags between start and end (inclusive)
    if len(fuselage.segments) > 1:
        collect = False
        seg_tags = []
        seg_bounds =  fuel_tank.segments_bounding_tank

        total_wing_volume = 0
        for segment in fuselage.segments:
            if segment.tag == seg_bounds[0]:
                collect = True
            if collect:
                seg_tags.append(segment.tag) 
                total_wing_volume += segment.volume_properties.gross_volume                
            if segment.tag == seg_bounds[1]:
                break
            
            
        tessellation = 20
        num_fus_segs = len(seg_tags)
        fuselage_points = np.zeros((num_fus_segs*tessellation ,3))
          
        for i_seg, segment in enumerate(seg_tags):
            segment   = fuselage.segments[segment]
            segment_0 = fuselage.segments[seg_tags[0]]
            a         = segment.width/2
            b         = segment.height/2
            n         = segment.curvature
            theta     = np.linspace(0,2*np.pi,tessellation) 
            fus_ypts  =  (abs((np.cos(theta)))**(2/n))*a * ((np.cos(theta)>0)*1 - (np.cos(theta)<0)*1) 
            fus_zpts  =  (abs((np.sin(theta)))**(2/n))*b * ((np.sin(theta)>0)*1 - (np.sin(theta)<0)*1)
            
            
            start_idx  = i_seg * tessellation
            end_idx    = (i_seg + 1 )* tessellation
            fuselage_points[start_idx:end_idx,0] = (segment.percent_x_location - segment_0.percent_x_location) *fuselage.lengths.total  
            fuselage_points[start_idx:end_idx,1] = fus_ypts + segment.percent_y_location*fuselage.lengths.total + fuselage.origin[0][1]
            fuselage_points[start_idx:end_idx,2] = fus_zpts + segment.percent_z_location*fuselage.lengths.total + fuselage.origin[0][2]
   
        # Convex hull → watertight volume mesh
        solid_segment = trimesh.convex.convex_hull(fuselage_points) 
    
        # Rotate to match the RCAIDE aircraft axes convention
        R = trimesh.transformations.rotation_matrix(np.deg2rad(90), [1, 0, 0], [0, 0, 0])
        solid_segment.apply_transform(R)
    
        # Calculate MOI of the fuel within the fuel tank 
        solid_segment.density = fuel.density  
        I_local_fuel          = solid_segment.moment_inertia
        
    # Store moment of inertia tensors of tank and fuel 
    fuel_tank.fuel.mass_properties.moments_of_inertia.tensor                 = I_local_fuel
    fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor = I_local_fuel / tank_mass
    fuel_tank.mass_properties.moments_of_inertia.tensor                      = I_local_tank 
    fuel_tank.mass_properties.moments_of_inertia.non_dimensional_tensor      = I_local_tank  / fuel_mass
    
    return I_global_fuel,  tank_mass 

 