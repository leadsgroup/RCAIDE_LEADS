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
def compute_electric_rotor_performance(propulsor,state,voltage,center_of_gravity= [[0.0, 0.0,0.0]]):   
    """
    Computes the performance characteristics of an electric rotor propulsion system by evaluating 
    the electronic speed controller (ESC), DC motor, and rotor in sequence.

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.Electric_Rotor
        The electric rotor propulsion system
            - tag : str
                Identifier for the propulsor
            - motor : Component
                DC motor component
            - rotor : Component
                Rotor component
            - electronic_speed_controller : Component
                ESC component
    state : RCAIDE.Framework.Mission.Common.State
        Contains flight conditions and operating state
            - conditions : Conditions
                Flight condition parameters including energy and throttle settings
    voltage : float
        System input voltage [V]
    center_of_gravity : list, optional
        Aircraft center of gravity coordinates [[x, y, z]] [m]
        Default is [[0.0, 0.0, 0.0]]

    Returns
    -------
    T : array_like
        Thrust force vector [N]
    M : array_like
        Moment vector [N-m]
    P : float
        Total electrical power consumption [W]
    stored_results_flag : bool
        Indicator that results have been stored
    stored_propulsor_tag : str
        Identifier of the propulsor with stored results

    Notes
    -----
    The function performs a sequential computation through the propulsion system:
        1. Unpacks conditions and propulsor components
        2. ESC modulates input voltage based on throttle setting
        3. Motor converts electrical power to mechanical rotation
        4. Rotor converts mechanical rotation to thrust
        5. System performance metrics are computed

    **Major Assumptions**
        * Components are connected in series: ESC → Motor → Rotor
        * Power losses occur at each conversion stage
        * Moments from motor rotation are not included (noted as future work)
        * Perfect mechanical coupling between motor and rotor

    **Definitions**

    'Throttle'
        Control input ranging from 0 to 1 that modulates power output

    'Stored Results'
        Performance data saved for reuse with identical propulsors
    """ 
    conditions                 = state.conditions    
    electric_rotor_conditions  = conditions.energy[propulsor.tag] 
    motor                      = propulsor.motor 
    rotor                      = propulsor.rotor 
    esc                        = propulsor.electronic_speed_controller  
    esc_conditions             = electric_rotor_conditions[esc.tag]
    motor_conditions           = electric_rotor_conditions[motor.tag]
    rotor_conditions           = electric_rotor_conditions[rotor.tag]
    eta                        = conditions.energy[propulsor.tag].throttle
    
    esc_conditions.inputs.voltage   = voltage
    esc_conditions.throttle         = eta 
    compute_voltage_out_from_throttle(esc,esc_conditions,conditions)

    # Assign conditions to the rotor
    motor_conditions.voltage  = esc_conditions.outputs.voltage 
    compute_motor_performance(motor,motor_conditions,conditions) 
    
    # Spin the rotor 
    rotor_conditions.omega           = motor_conditions.omega
    rotor_conditions.throttle        = esc_conditions.throttle
    rotor_conditions.motor_torque    = motor_conditions.torque
    compute_rotor_performance(propulsor,state,center_of_gravity)   
    
    # Detemine esc current 
    esc_conditions.outputs.current = motor_conditions.current
    compute_current_in_from_throttle(esc,esc_conditions,conditions)
    
    stored_results_flag     = True
    stored_propulsor_tag    = propulsor.tag 
    
    # compute total forces and moments from propulsor (future work would be to add moments from motors)
    electric_rotor_conditions.thrust      = conditions.energy[propulsor.tag][rotor.tag].thrust 
    electric_rotor_conditions.moment      = conditions.energy[propulsor.tag][rotor.tag].moment 
    
    T  = conditions.energy[propulsor.tag][rotor.tag].thrust 
    M  = conditions.energy[propulsor.tag][rotor.tag].moment 
    P  = esc_conditions.inputs.power
    
    return T,M,P, stored_results_flag,stored_propulsor_tag 
                
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
    
    conditions.energy[propulsor.tag][motor.tag]        = deepcopy(conditions.energy[stored_propulsor_tag][motor_0.tag])
    conditions.energy[propulsor.tag][rotor.tag]        = deepcopy(conditions.energy[stored_propulsor_tag][rotor_0.tag])
    conditions.energy[propulsor.tag][esc.tag]          = deepcopy(conditions.energy[stored_propulsor_tag][esc_0.tag])
  
    thrust                  = conditions.energy[propulsor.tag][rotor.tag].thrust 
    power                   = conditions.energy[propulsor.tag][esc.tag].inputs.power 
    
    moment_vector           = 0*state.ones_row(3) 
    moment_vector[:,0]      = rotor.origin[0][0]  -  center_of_gravity[0][0] 
    moment_vector[:,1]      = rotor.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2]      = rotor.origin[0][2]  -  center_of_gravity[0][2]
    moment                  =  np.cross(moment_vector, thrust)
    
    conditions.energy[propulsor.tag].moment            = moment  
    conditions.energy[propulsor.tag].thrust            = thrust   
    conditions.energy[propulsor.tag].moment            = moment  
    
    return thrust,moment,power