# RCAIDE/Methods/Energy/Propulsors/Electric_Rotor_Propulsor/compute_electric_rotor_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports   
from RCAIDE.Library.Methods.Powertrain.Modulators.Electronic_Speed_Controller.compute_esc_performance  import * 
from RCAIDE.Library.Methods.Powertrain.Converters.Motor.compute_motor_performance                      import *
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.compute_rotor_performance                      import * 

# pacakge imports  
import numpy as np 
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------
# compute_electric_rotor_performance
# ----------------------------------------------------------------------------------------------------------------------  
def compute_electric_rotor_performance(propulsor,state,center_of_gravity= [[0.0, 0.0,0.0]]):   
    ''' Computes the perfomrance of one propulsor
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    conditions           - operating conditions data structure    [-] 
    voltage              - system voltage                         [V]
    bus                  - bus                                    [-] 
    propulsor            - propulsor data structure               [-] 
    total_thrust         - thrust of propulsor group              [N]
    total_power          - power of propulsor group               [W]
    total_current        - current of propulsor group             [A]

    Outputs:  
    total_thrust         - thrust of propulsor group              [N]
    total_power          - power of propulsor group               [W]
    total_current        - current of propulsor group             [A]
    stored_results_flag  - boolean for stored results             [-]     
    stored_propulsor_tag - name of propulsor with stored results  [-]
    
    Properties Used: 
    N.A.        
    '''
     
    conditions                 = state.conditions    
    motor                      = propulsor.motor 
    rotor                      = propulsor.rotor 
    esc                        = propulsor.electronic_speed_controller   
    electric_rotor_conditions  = conditions.energy.propulsors[propulsor.tag]
    eta                        = electric_rotor_conditions.throttle
     
    conditions.energy.modulators[esc.tag].throttle         = eta 
    compute_voltage_out_from_throttle(esc,conditions)

    # Assign conditions to the rotor
    conditions.energy.converters[motor.tag].inputs.voltage = conditions.energy.modulators[esc.tag].outputs.voltage  
    compute_motor_performance(motor,conditions) 
    
    # Spin the rotor 
    conditions.energy.converters[rotor.tag].omega           = conditions.energy.converters[motor.tag].outputs.omega
    conditions.energy.converters[rotor.tag].throttle        = conditions.energy.modulators[esc.tag].throttle      
    conditions.energy.converters[rotor.tag].commanded_thrust_vector_angle =  conditions.energy.propulsors[propulsor.tag].commanded_thrust_vector_angle
    compute_rotor_performance(rotor,conditions)
 
    # Compute moment 
    moment_vector           = 0*state.ones_row(3)
    moment_vector[:,0]      = rotor.origin[0][0]  -  center_of_gravity[0][0] 
    moment_vector[:,1]      = rotor.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2]      = rotor.origin[0][2]  -  center_of_gravity[0][2]
    moment                  =  np.cross(moment_vector, conditions.energy.converters[rotor.tag].thrust)     
    
    # Detemine esc current 
    conditions.energy.modulators[esc.tag].outputs.current = conditions.energy.converters[motor.tag].inputs.current
    compute_current_in_from_throttle(esc,conditions)
    
    stored_results_flag     = True
    stored_propulsor_tag    = propulsor.tag 
    
    # compute total forces and moments from propulsor (future work would be to add moments from motors)
    electric_rotor_conditions.thrust      = conditions.energy.converters[rotor.tag].thrust 
    electric_rotor_conditions.power       = conditions.energy.converters[rotor.tag].power 
    electric_rotor_conditions.moment      = moment
    
    return electric_rotor_conditions.thrust,electric_rotor_conditions.moment,electric_rotor_conditions.power,conditions.energy.modulators[esc.tag].inputs.power,stored_results_flag,stored_propulsor_tag 
                
def reuse_stored_electric_rotor_data(propulsor,state,network,stored_propulsor_tag,center_of_gravity= [[0.0, 0.0,0.0]]):
    '''Reuses results from one propulsor for identical propulsors
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    conditions           - operating conditions data structure    [-] 
    voltage              - system voltage                         [V]
    bus                  - bus                                    [-] 
    propulsors           - propulsor data structure               [-] 
    total_thrust         - thrust of propulsor group              [N]
    total_power          - power of propulsor group               [W]
    total_current        - current of propulsor group             [A]

    Outputs:  
    total_thrust         - thrust of propulsor group              [N]
    total_power          - power of propulsor group               [W]
    total_current        - current of propulsor group             [A] 
    
    Properties Used: 
    N.A.        
    ''' 
    conditions                 = state.conditions 
    motor                      = propulsor.motor 
    rotor                      = propulsor.rotor 
    esc                        = propulsor.electronic_speed_controller  
    motor_0                    = network.propulsors[stored_propulsor_tag].motor 
    rotor_0                    = network.propulsors[stored_propulsor_tag].rotor 
    esc_0                      = network.propulsors[stored_propulsor_tag].electronic_speed_controller
    
    conditions.energy.converters[motor.tag]        = deepcopy(conditions.energy.converters[motor_0.tag])
    conditions.energy.converters[rotor.tag]        = deepcopy(conditions.energy.converters[rotor_0.tag])
    conditions.energy.modulators[esc.tag]          = deepcopy(conditions.energy.modulators[esc_0.tag])
  
    thrust_vector           = conditions.energy.converters[rotor.tag].thrust 
    P_mech                  = conditions.energy.converters[rotor.tag].power 
    P_elec                  = conditions.energy.modulators[esc.tag].inputs.power    
    
    moment_vector           = 0*state.ones_row(3) 
    moment_vector[:,0]      = rotor.origin[0][0]  -  center_of_gravity[0][0] 
    moment_vector[:,1]      = rotor.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2]      = rotor.origin[0][2]  -  center_of_gravity[0][2]
    moment                  =  np.cross(moment_vector, thrust_vector)
     
    conditions.energy.propulsors[propulsor.tag].power             = P_mech  
    conditions.energy.propulsors[propulsor.tag].thrust            = thrust_vector  
    conditions.energy.propulsors[propulsor.tag].moment            = moment          
    return thrust_vector,moment,P_mech,P_elec