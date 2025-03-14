# RCAIDE/Library/Methods/Powertrain/Propulsors/Internal_Combustion_Engine/compute_internal_combustion_engine_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Framework.Core import Units  
from RCAIDE.Library.Methods.Powertrain.Converters.Engine import compute_power_from_throttle
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.compute_rotor_performance import  compute_rotor_performance

# pacakge imports  
from copy import deepcopy
import numpy as np  

# ----------------------------------------------------------------------------------------------------------------------
# compute_internal_combustion_engine_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_internal_combustion_engine_performance(propulsor,state,fuel_line=None,bus=None,center_of_gravity= [[0.0, 0.0,0.0]]):  
    ''' Computes the perfomrance of one propulsor
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    conditions           - operating conditions data structure    [-]  
    fuel_line            - fuelline                               [-] 
    propulsor            - propulsor data structure               [-] 
    total_thrust         - thrust of propulsor group              [N]
    total_power          - power of propulsor group               [W] 

    Outputs:  
    total_thrust         - thrust of propulsor group              [N]
    total_power          - power of propulsor group               [W] 
    stored_results_flag  - boolean for stored results             [-]     
    stored_propulsor_tag - name of propulsor with stored results  [-]
    
    Properties Used: 
    N.A.        
    '''  
    conditions              = state.conditions  
    ice_conditions          = conditions.energy.propulsors[propulsor.tag]
    engine                  = propulsor.engine 
    propeller               = propulsor.propeller
    eta                     = ice_conditions.throttle
    engine_conditions       = conditions.energy.converters[engine.tag]
    propeller_conditions    = conditions.energy.converters[propeller.tag]
    RPM                     = engine_conditions.rpm

    # Throttle the engine
    engine_conditions.speed           = RPM * Units.rpm
    engine_conditions.throttle        = eta 
    compute_power_from_throttle(engine,engine_conditions,conditions)        
     
    # Run the propeller to get the power
    propeller_conditions.omega          = RPM * Units.rpm 
    propeller_conditions.throttle       = engine_conditions.throttle
    compute_rotor_performance(propulsor,state,center_of_gravity)
    
    # Create the outputs
    ice_conditions.fuel_flow_rate            = engine_conditions.fuel_flow_rate  
    stored_results_flag                      = True
    stored_propulsor_tag                     = propulsor.tag 


    # compute total forces and moments from propulsor (future work would be to add moments from motors)
    ice_conditions.thrust      = propeller_conditions.thrust 
    ice_conditions.moment      = propeller_conditions.moment
    ice_conditions.power       = propeller_conditions.power  
    
    # ADD CODE FOR SHAFT OFFTAKE AND MOTORS
    power_elec =  0*state.ones_row(3)
    
    return ice_conditions.thrust,ice_conditions.moment,ice_conditions.power,power_elec,stored_results_flag,stored_propulsor_tag  
    
    
def reuse_stored_internal_combustion_engine_data(propulsor,state,network,fuel_line,bus,stored_propulsor_tag,center_of_gravity= [[0.0, 0.0,0.0]]):
    '''Reuses results from one propulsor for identical propulsors
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    conditions           - operating conditions data structure        [-]  
    fuel_line            - fuel line                                  [-] 
    propulsor            - propulsor data structure               [-] 
    total_thrust         - thrust of propulsor group              [N]
    total_power          - power of propulsor group               [W] 

    Outputs:  
    total_thrust         - thrust of propulsor group              [N]
    total_power          - power of propulsor group               [W] 
    
    Properties Used: 
    N.A.        
    ''' 

    # unpack
    conditions   = state.conditions 
    engine       = propulsor.engine
    propeller    = propulsor.propeller 
    engine_0     = network.propulsors[stored_propulsor_tag].engine
    propeller_0  = network.propulsors[stored_propulsor_tag].propeller  
    
    # deepcopy results 
    conditions.energy.converters[engine.tag]        = deepcopy(conditions.energy.converters[engine_0.tag])
    conditions.energy.converters[propeller.tag]     = deepcopy(conditions.energy.converters[propeller_0.tag])
   
    # compoment 
    thrust_vector           = conditions.energy.converters[propeller.tag].thrust 
    power                   = conditions.energy.converters[propeller.tag].power   
    moment_vector           = 0*state.ones_row(3) 
    moment_vector[:,0]      = propeller.origin[0][0]  -  center_of_gravity[0][0] 
    moment_vector[:,1]      = propeller.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2]      = propeller.origin[0][2]  -  center_of_gravity[0][2]
    moment                  =  np.cross(moment_vector, thrust_vector)
    
    # pack 
    conditions.energy.propulsors[propulsor.tag][propeller.tag].moment = moment  
    conditions.energy.propulsors[propulsor.tag].thrust                = thrust_vector   
    conditions.energy.propulsors[propulsor.tag].moment                = moment  
    conditions.energy.propulsors[propulsor.tag].power                 = power
  
    power_elec =  0*state.ones_row(3)
    
    return thrust_vector,moment,power, power_elec

            
               