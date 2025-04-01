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
def compute_cs_ice_performance(propulsor,state,center_of_gravity= [[0.0, 0.0,0.0]]):  
    """
    Computes the performance of a constant speed internal combustion engine (ICE) propulsor.
    
    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.Constant_Speed_ICE_Propulsor
        The constant speed ICE propulsor system containing:
            - engine : ICE
                Internal combustion engine component
            - propeller : Propeller
                Propeller component
            - tag : str
                Unique identifier for the propulsor
    state : RCAIDE.Framework.Mission.Segments.Segment
        Current mission state containing conditions
    center_of_gravity : list of lists, optional
        Aircraft center of gravity coordinates [[x, y, z]], defaults to [[0.0, 0.0, 0.0]]
        
    Returns
    -------
    T : array_like
        Thrust force vector [N]
    M : array_like
        Moment vector [N*m]
    P : array_like
        Power required [W]
    stored_results_flag : bool
        Flag indicating if results are stored
    stored_propulsor_tag : str
        Tag of the propulsor with stored results
        
    Notes
    -----
    This function performs the following operations:
        1. Parses inputs
        2. Runs propeller performance calculations
        3. Computes engine throttle setting based on required power
        4. Calculates fuel flow rate
        5. Determines total forces and moments
    
    **Major Assumptions**
        * Propeller pitch command is derived from a linear relationship with throttle setting
        * Moments are primarily from propeller forces
    
    See Also
    --------
    RCAIDE.Library.Methods.Propulsors.Converters.Engine.compute_throttle_from_power
    RCAIDE.Library.Methods.Propulsors.Converters.Rotor.compute_rotor_performance
    """
    conditions              = state.conditions  
    ice_cs_conditions       = conditions.energy[propulsor.tag] 
    engine                  = propulsor.engine 
    propeller               = propulsor.propeller
    engine_conditions       = ice_cs_conditions[engine.tag]
    engine_conditions.rpm   = conditions.energy[propulsor.tag].rpm 

    # Run the propeller to get the power
    propeller_conditions                      = ice_cs_conditions[propeller.tag]
    propeller_conditions.omega                = engine_conditions.rpm * Units.rpm
    propeller_conditions.blade_pitch_command  = ice_cs_conditions.throttle - 0.5
    propeller_conditions.throttle             = ice_cs_conditions.throttle
    compute_rotor_performance(propulsor,state,center_of_gravity)

    # Run the engine to calculate the throttle setting and the fuel burn
    engine_conditions.power        = conditions.energy[propulsor.tag][propeller.tag].power 
    compute_throttle_from_power(engine,engine_conditions,conditions) 
    
    # Create the outputs
    ice_cs_conditions.fuel_flow_rate         = engine_conditions.fuel_flow_rate  
    stored_results_flag                      = True
    stored_propulsor_tag                     = propulsor.tag  

    # compute total forces and moments from propulsor (future work would be to add moments from motors)
    conditions.energy[propulsor.tag].thrust      = conditions.energy[propulsor.tag][propeller.tag].thrust 
    conditions.energy[propulsor.tag].moment      = conditions.energy[propulsor.tag][propeller.tag].moment
    conditions.energy[propulsor.tag].power       = conditions.energy[propulsor.tag][propeller.tag].power 
    T  = conditions.energy[propulsor.tag].thrust 
    M  = conditions.energy[propulsor.tag].moment 
    P  = conditions.energy[propulsor.tag].power 
    
    return T,M,P,stored_results_flag,stored_propulsor_tag 
    
def reuse_stored_ice_cs_prop_data(propulsor,state,network,stored_propulsor_tag,center_of_gravity= [[0.0, 0.0,0.0]]):
    """Reuses results from one propulsor for identical propulsors
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    conditions           - operating conditions data structure    [-]  
    fuel_line            - fuel_line                              [-] 
    propulsor        - propulsor data structure     [-] 
    total_thrust         - thrust of propulsor group       [N]
    total_power          - power of propulsor group        [W] 
     
    Outputs:      
    total_thrust         - thrust of propulsor group       [N]
    total_power          - power of propulsor group        [W] 
    
    Properties Used: 
    N.A.        
    """ 
    conditions                 = state.conditions
    engine                     = propulsor.engine
    propeller                  = propulsor.propeller 
    engine_0                   = network.propulsors[stored_propulsor_tag].engine
    propeller_0                = network.propulsors[stored_propulsor_tag].propeller

    conditions.energy[propulsor.tag][engine.tag]        = deepcopy(conditions.energy[stored_propulsor_tag][engine_0.tag])
    conditions.energy[propulsor.tag][propeller.tag]     = deepcopy(conditions.energy[stored_propulsor_tag][propeller_0.tag])    
  
    thrust                  = conditions.energy[propulsor.tag][propeller.tag].thrust 
    power                   = conditions.energy[propulsor.tag][propeller.tag].power 
    
    moment_vector           = 0*state.ones_row(3) 
    moment_vector[:,0]      = propeller.origin[0][0]  -  center_of_gravity[0][0] 
    moment_vector[:,1]      = propeller.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2]      = propeller.origin[0][2]  -  center_of_gravity[0][2]
    moment                  =  np.cross(moment_vector, thrust)
    
    conditions.energy[propulsor.tag][propeller.tag].moment = moment  
    conditions.energy[propulsor.tag].thrust            = thrust   
    conditions.energy[propulsor.tag].moment            = moment  
    conditions.energy[propulsor.tag].power             = power
 
    return thrust,moment,power
 