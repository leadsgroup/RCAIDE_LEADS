# RCAIDE/Library/Methods/Mass_Properties/Center_of_Gravity/compute_distributor_center_of_gravity.py 
# 
# Created:  Dec 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE 

# # package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Distributor Center of Gravity
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_distributor_center_of_gravity(component,vehicle, length=0):
    """
    Compute the center of gravity of a distributor
    
    Assumptions
    ___________
         - Lines line on y-z plane (i.e. C.G.z is 0)
         - Interates through all compoments on the distribution lines and computes the manhattan distance
         
       .-------------.
       | compoment i |---<-- line 1 ->----|
       '-------------'                    |
                                          ^ 
                                          |
                                          V
              .------------.              |      
              | compoment j|-<- line 2 ->-|
              '------------'              |
                                          
                                          
     """
     
    redundancy_factor     = 2.0
    valve_unit_mass       = component.valve_unit_mass                      
    fuel_probe_unit_mass  = component.fuel_probe_unit_mass
    boost_pump_unit_mass  = component.boost_pump_unit_mass 
    insulation_rm_density = component.insulation.rigid_material.density
    insulation_fm_density = component.insulation.flexible_material.density
    insulation_fm_ratio   = component.insulation.flexible_material_ratio  
    pipe_rm_density       = component.pipe.rigid_material.density        
    pipe_fm_density       = component.pipe.flexible_material.density   
    pipe_fm_ratio         = component.pipe.flexible_material_ratio  
    
    insulation_cross_sectional_area = (np.pi / 4) *  (component.insulation.diameters.external ** 2 -   component.insulation.diameters.internal** 2)
    pipe_cross_sectional_area       = (np.pi / 4) *  (component.pipe.diameters.external ** 2 -   component.pipe.diameters.internal** 2)
    
    c_list  =  []
    c_symm  =  []
    c_locs  = np.empty((0,3)) 

    total_lat_line_length= 0
    total_line_mass      = 0
    total_line_moment    = 0
    transfer_system_mass = 0 
     
    for network in  vehicle.networks: 
        for propulsor in network.propulsors:
            c_list.append(propulsor.tag)
            c_symm.append(propulsor.xz_plane_symmetric) 
            c_loc  = np.array(propulsor.origin) + np.array(propulsor.mass_properties.center_of_gravity)
            c_locs =  np.concatenate((c_locs,c_loc), axis=0)

        for source in  network.sources: 
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):   
                c_list.append(source.tag)
                c_symm.append(source.xz_plane_symmetric)
                c_loc  = np.array(source.origin) + np.array(source.mass_properties.center_of_gravity)
                c_locs =  np.concatenate((c_locs,c_loc), axis=0)  
    
    # lateral lines running from center of aircraft to sources (fuel tanks. batteries etc) t
    for i in range(len(c_list)):   
        lat_line_length    = abs(c_locs[i][1]) 
        lat_line_centroid  = c_locs[i][0]
        
        lat_line_length *= (c_symm[i]+ 1)
        
        if insulation_cross_sectional_area == 0.0:
            lat_line_insulation_mass   = 0
            lat_line_insulation_moment = 0
        else:
            lat_line_insulation_volume  = lat_line_length *  insulation_cross_sectional_area
            lat_line_insulation_mass    = insulation_fm_ratio * (insulation_rm_density * lat_line_insulation_volume) +  (1 - insulation_fm_ratio) * (insulation_fm_density * lat_line_insulation_volume)  
            lat_line_insulation_moment  = lat_line_insulation_mass *lat_line_centroid     
             
        if pipe_cross_sectional_area == 0.0:
            lat_line_pipe_mass   = 0
            lat_line_pipe_moment = 0
        else:
            lat_line_pipe_volume = lat_line_length *  pipe_cross_sectional_area
            lat_line_pipe_mass   = pipe_fm_ratio * (pipe_rm_density * lat_line_pipe_volume) +  (1 - pipe_fm_ratio) * (pipe_fm_density * lat_line_pipe_volume)  
            lat_line_pipe_moment = lat_line_pipe_mass *lat_line_centroid   
                                 
        # assumulate all masses and  moments 
        total_lat_line_length  += redundancy_factor * lat_line_length 
        total_line_mass        += lat_line_insulation_mass + lat_line_pipe_mass
        transfer_system_mass   += lat_line_insulation_mass + lat_line_pipe_mass +  valve_unit_mass +  fuel_probe_unit_mass  +  boost_pump_unit_mass
        total_line_moment      += lat_line_pipe_moment + lat_line_insulation_moment
    
    
    # longininal
    sorted_indices = c_locs[:, 0].argsort()
    sorted_c_locs = c_locs[sorted_indices]
    
    min_c_loc =  sorted_c_locs[0]
    max_c_loc =  sorted_c_locs[-1]
    
    # distributor distances  
    long_line_length     = abs(max_c_loc[0] - min_c_loc[0] )
    long_line_centroid   = (max_c_loc[0] + min_c_loc[0] )/2
    
    if insulation_cross_sectional_area == 0.0:
        long_line_insulation_mass   = 0
        long_line_insulation_moment = 0
    else:  
        long_line_insulation_volume  = long_line_length *  insulation_cross_sectional_area
        long_line_insulation_mass    = insulation_fm_ratio * (insulation_rm_density * long_line_insulation_volume) +  (1 - insulation_fm_ratio) * (insulation_fm_density * long_line_insulation_volume) 
        long_line_insulation_moment  = long_line_insulation_mass *long_line_centroid
         
    if pipe_cross_sectional_area == 0:
        long_line_pipe_mass = 0
        long_line_pipe_moment = 0
    else:  
        long_line_pipe_volume  = long_line_length *  pipe_cross_sectional_area
        long_line_pipe_mass    = pipe_fm_ratio * (pipe_rm_density * long_line_pipe_volume) +  (1 - pipe_fm_ratio) * (pipe_fm_density * long_line_pipe_volume)  
        long_line_pipe_moment  =  long_line_pipe_mass *long_line_centroid  
                             
    # assumulate all masses and  moments 
    total_lat_line_length  += redundancy_factor * long_line_length  
    total_line_mass        += long_line_insulation_mass + long_line_pipe_mass
    transfer_system_mass   += long_line_insulation_mass + long_line_pipe_mass
    total_line_moment      += long_line_pipe_moment + long_line_insulation_moment
     
     
    # distributor distances  
    for network in  vehicle.networks: 
        for distributor in network.distributor: 
            vent_line_length     = distributor.venting_system_length
            vent_line_centroid   = (max_c_loc[0] + min_c_loc[0] )/2
            
            if insulation_cross_sectional_area == 0.0:
                vent_line_insulation_mass   = 0
                vent_line_insulation_moment = 0
            else:  
                vent_line_insulation_volume  = vent_line_length *  insulation_cross_sectional_area
                vent_line_insulation_mass    = insulation_fm_ratio * (insulation_rm_density * vent_line_insulation_volume) +  (1 - insulation_fm_ratio) * (insulation_fm_density * vent_line_insulation_volume) 
                vent_line_insulation_moment  = vent_line_insulation_mass * vent_line_centroid
                
            if pipe_cross_sectional_area == 0:
                vent_line_pipe_mass = 0
                vent_line_pipe_moment = 0
            else:  
                vent_line_pipe_volume  = vent_line_length *  pipe_cross_sectional_area
                vent_line_pipe_mass    = pipe_fm_ratio * (pipe_rm_density * vent_line_pipe_volume) +  (1 - pipe_fm_ratio) * (pipe_fm_density * vent_line_pipe_volume)  
                vent_line_pipe_moment  =  vent_line_pipe_mass *vent_line_centroid  
            total_lat_line_length  += redundancy_factor * vent_line_length  
            total_line_mass        += vent_line_insulation_mass + vent_line_pipe_mass
            transfer_system_mass   += vent_line_insulation_mass + vent_line_pipe_mass
            total_line_moment      += vent_line_pipe_moment + vent_line_insulation_moment
                                    
    # assumulate all masses and  moments 
    transfer_system_mass   += valve_unit_mass +  fuel_probe_unit_mass  +  boost_pump_unit_mass
    
    if total_line_mass != 0.0: 
        c_g_line =  [[total_line_moment / total_line_mass, 0, 0]]
        component.mass_properties.center_of_gravity = c_g_line 
        component.mass_properties.mass              = transfer_system_mass
    
    return component.mass_properties.center_of_gravity