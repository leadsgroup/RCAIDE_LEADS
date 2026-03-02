# RCAIDE/Library/Missions/Common/Update/weights.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE 
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity.update_center_of_gravity import update_center_of_gravity
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.update_moments_of_inertia import update_moments_of_inertia

# package imports 
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
                 conditions.weights.vehicle.mass         [kg]
                 conditions.weights.vehicle.mass_rate  [kg/s]
                 conditions.freestream.gravity         [m/s^2]

                 
        Outputs: 
            segment.state.conditions.
                 weights.vehicle.mass
                 frames.inertial.gravity_force_vector
      
        Properties Used:
        N/A
                    
    """ 
    
    # unpack
    conditions     = segment.state.conditions
    I              = segment.state.numerics.time.integrate 
    m_0_vehicle    = conditions.weights.vehicle.mass[0,0]
    m_dot_vehicle  = conditions.weights.vehicle.mass_rate
    g              = conditions.freestream.gravity
    vehicle        = segment.analyses.vehicle
     
    if (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude) or\
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_AVL_Trimmed) or \
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_No_Propulsion) or \
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Throttle):
 
        W = m_0_vehicle*g 
        conditions.frames.inertial.gravity_force_vector[:,2] = W[:,0] 
    else:
    
        # --------------------------------------------------------------------------  
        # update center of gravity  
        # --------------------------------------------------------------------------  
        if segment.analyses.weights.settings.run_center_of_gravity_analysis:
            # loop through battery modules in networks 
            for network in vehicle.networks: 
                for source in  network.sources: 
                    if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):   
                        fuel =  source.fuel
                        mass_flow_rate = conditions.energy.sources[source.tag].mass_flow_rate              
                        m_0_fuel       = conditions.weights.components.mass[fuel.tag][0,0]     
                        conditions.weights.components.mass[fuel.tag][:,0]  = m_0_fuel +  np.dot(I, -mass_flow_rate).flatten() 
                
            update_center_of_gravity(segment.state, vehicle)
            
        # --------------------------------------------------------------------------        
        # update moment of inertia 
        # --------------------------------------------------------------------------  
        if segment.analyses.weights.settings.run_moments_of_inertia_analysis:
            update_moments_of_inertia(segment.state, vehicle) 
    
        # --------------------------------------------------------------------------                  
        # update mass 
        # --------------------------------------------------------------------------  
        m = m_0_vehicle + np.dot(I, -m_dot_vehicle) 
        W = m*g 
        conditions.weights.vehicle.mass[1:,0]                = m[1:,0]  
        conditions.frames.inertial.gravity_force_vector[:,2] = W[:,0]
                
    return
 