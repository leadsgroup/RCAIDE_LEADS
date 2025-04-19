# RCAIDE/Library/Missions/Common/Update/weights.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
# Update Weights
# ----------------------------------------------------------------------------------------------------------------------  
def weights(segment): 
    """ Updates the weight of the vehicle 
        
        Assumptions:
        N/A
        
        Inputs:
             segment.state.
                 numerics.time.integrate               [-]
                 conditions.weights.total_mass         [kg]
                 conditions.weights.vehicle_mass_rate  [kg/s]
                 conditions.freestream.gravity         [m/s^2]

                 
        Outputs: 
            segment.state.conditions.
                 weights.total_mass
                 frames.inertial.gravity_force_vector
      
        Properties Used:
        N/A
                    
    """ 
    
    # unpack
    conditions   = segment.state.conditions
    I            = segment.state.numerics.time.integrate  
    m0           = conditions.weights.total_mass[0,0]
    mdot         = conditions.weights.vehicle_mass_rate
    g            = conditions.freestream.gravity   
    
    networks = segment.analyses.energy.vehicle.networks
    for network in networks:
        if 'fuel_lines' in network:
            for fuel_line in network.fuel_lines:  
                fuel_line_results   = conditions.energy[fuel_line.tag] 
                for fuel_tank in fuel_line.fuel_tanks: 
                    fuel_line_results[fuel_tank.tag].mass[:,0]  =  fuel_line_results[fuel_tank.tag].mass[0,0]  + np.dot(I, -fuel_line_results[fuel_tank.tag].mass_flow_rate[:,0])   
            
    # calculate
    m = m0 + np.dot(I, -mdot)

    # weight
    W = m*g

    # pack
    conditions.weights.total_mass[1:,0]                  = m[1:,0]  
    conditions.frames.inertial.gravity_force_vector[:,2] = W[:,0]

    return
 