# RCAIDE/Methods/Energy/Propulsors/Constant_Speed_ICE_Propulsor/compute_cs_ice_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Framework.Core import Units  
from RCAIDE.Library.Methods.Powertrain.Converters.Engine import compute_throttle_from_power
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.compute_rotor_performance import  compute_rotor_performance
 
# pacakge imports  
from copy import deepcopy
import numpy as np  

# ----------------------------------------------------------------------------------------------------------------------
# internal_combustion_engine_constant_speed_propulsor
# ----------------------------------------------------------------------------------------------------------------------  
def compute_constant_speed_internal_combustion_engine_performance(propulsor,state,fuel_line=None,bus=None,center_of_gravity= [[0.0, 0.0,0.0]]):  
    ''' Computes the perfomrance of one propulsor
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    conditions           - operating conditions data structure           [-]  
    fuel_line            - fuelline                                      [-] 
    propulsor        - propulsor data structure            [-] 
    total_thrust         - thrust of propulsor group              [N]
    total_power          - power of propulsor group               [W] 

    Outputs:  
    total_thrust         - thrust of propulsor group              [N]
    total_power          - power of propulsor group               [W] 
    stored_results_flag  - boolean for stored results                    [-]     
    stored_propulsor_tag - name of propulsor with stored results  [-]
    
    Properties Used: 
    N.A.        
    '''  
    conditions              = state.conditions  
    ice_cs_conditions       = conditions.energy.propulsors[propulsor.tag] 
    engine                  = propulsor.engine 
    propeller               = propulsor.propeller 

    # Run the propeller to get the power
    conditions.energy.converters[engine.tag].rpm                     = ice_cs_conditions.rpm 
    conditions.energy.converters[propeller.tag].omega                = conditions.energy.converters[engine.tag].rpm * Units.rpm
    conditions.energy.converters[propeller.tag].blade_pitch_command  = ice_cs_conditions.throttle - 0.5
    conditions.energy.converters[propeller.tag].throttle             = ice_cs_conditions.throttle
    compute_rotor_performance(propeller,conditions)

    # Compute moment 
    moment_vector           = 0*state.ones_row(3)
    moment_vector[:,0]      = propeller.origin[0][0]  -  center_of_gravity[0][0] 
    moment_vector[:,1]      = propeller.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2]      = propeller.origin[0][2]  -  center_of_gravity[0][2]
    moment                  =  np.cross(moment_vector, conditions.energy.converters[propeller.tag].thrust)       
        

    # Run the engine to calculate the throttle setting and the fuel burn
    conditions.energy.converters[engine.tag].power        = conditions.energy.converters[propeller.tag].power 
    compute_throttle_from_power(engine,conditions) 
    
    # Create the outputs
    ice_cs_conditions.fuel_flow_rate         = conditions.energy.converters[engine.tag].fuel_flow_rate  
    stored_results_flag                      = True
    stored_propulsor_tag                     = propulsor.tag  

    # compute total forces and moments from propulsor (future work would be to add moments from motors)
    ice_cs_conditions.thrust      = conditions.energy.converters[propeller.tag].thrust 
    ice_cs_conditions.moment      = moment
    ice_cs_conditions.power       = conditions.energy.converters[propeller.tag].power  

    # currently, no hybridization
    power_elec =  0*state.ones_row(3)
    
    return ice_cs_conditions.thrust ,ice_cs_conditions.moment,ice_cs_conditions.power,power_elec,stored_results_flag,stored_propulsor_tag 
    
def reuse_stored_constant_speed_internal_combustion_engine_data(propulsor,state,network,stored_propulsor_tag,center_of_gravity= [[0.0, 0.0,0.0]]):
    '''Reuses results from one propulsor for identical propulsors
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    conditions           - operating conditions data structure    [-]  
    fuel_line            - fuel_line                              [-] 
    propulsor            - propulsor data structure     [-] 
    total_thrust         - thrust of propulsor group       [N]
    total_power          - power of propulsor group        [W] 
     
    Outputs:      
    total_thrust         - thrust of propulsor group       [N]
    total_power          - power of propulsor group        [W] 
    
    Properties Used: 
    N.A.        
    '''
    # unpack 
    conditions                 = state.conditions
    engine                     = propulsor.engine
    propeller                  = propulsor.propeller 
    engine_0                   = network.propulsors[stored_propulsor_tag].engine
    propeller_0                = network.propulsors[stored_propulsor_tag].propeller 
    
    # deep copy results
    conditions.energy.propulsors[propulsor.tag]     = deepcopy(conditions.energy.propulsors[stored_propulsor_tag.tag])
    conditions.energy.converters[engine.tag]        = deepcopy(conditions.energy.converters[engine_0.tag])
    conditions.energy.converters[propeller.tag]     = deepcopy(conditions.energy.converters[propeller_0.tag])

    # compute moment    
    thrust                  = conditions.energy.converters[propeller.tag].thrust 
    power                   = conditions.energy.converters[propeller.tag].power  
    moment_vector           = 0*state.ones_row(3) 
    moment_vector[:,0]      = propeller.origin[0][0]  -  center_of_gravity[0][0] 
    moment_vector[:,1]      = propeller.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2]      = propeller.origin[0][2]  -  center_of_gravity[0][2]
    moment                  =  np.cross(moment_vector, thrust)
    
    # pack results 
    conditions.energy.converters[propeller.tag].moment = moment  
    conditions.energy.propulsors[propulsor.tag].thrust = thrust   
    conditions.energy.propulsors[propulsor.tag].moment = moment  
    conditions.energy.propulsors[propulsor.tag].power  = power
    
    power_elec =  0*state.ones_row(3)  
    return thrust,moment,power, power_elec
 