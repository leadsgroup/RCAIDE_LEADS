# RCAIDE/Methods/Energy/Propulsors/Electric_Ducted_Fan_Propulsor/compute_ducted_fan_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports    
from RCAIDE.Library.Methods.Powertrain.Modulators.Electronic_Speed_Controller.compute_esc_performance    import * 
from RCAIDE.Library.Methods.Powertrain.Converters.Motor.compute_motor_performance                        import *
from RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.compute_ducted_fan_performance              import * 

# pacakge imports  
import numpy as np 
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------
# compute_electric_ducted_fan_performance
# ----------------------------------------------------------------------------------------------------------------------  
def compute_electric_ducted_fan_performance(propulsor,state,fuel_line=None,bus=None,center_of_gravity= [[0.0, 0.0,0.0]]):   
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
    bus_conditions             = conditions.energy[bus.tag]
    EDF_conditions             = conditions.energy.propulsors[propulsor.tag] 
    eta                        = EDF_conditions.throttle
    motor                      = propulsor.motor 
    ducted_fan                 = propulsor.ducted_fan 
    esc                        = propulsor.electronic_speed_controller
    
    conditions.energy.modulators[esc.tag].inputs.voltage   = bus.voltage * state.ones_row(1)    
    conditions.energy.modulators[esc.tag].throttle         = eta 
    compute_voltage_out_from_throttle(esc,conditions)

    # Assign conditions to the ducted_fan
    conditions.energy.converters[motor.tag].inputs.voltage = conditions.energy.modulators[esc.tag].outputs.voltage
    compute_motor_performance(motor,conditions) 
    
    # Spin the ducted_fan  
    conditions.energy.converters[ducted_fan.tag].omega              = conditions.energy.converters[motor.tag].outputs.omega 
    conditions.energy.converters[ducted_fan.tag].tip_mach           = (conditions.energy.converters[motor.tag].outputs.omega * ducted_fan.tip_radius) / conditions.freestream.speed_of_sound
    conditions.energy.converters[ducted_fan.tag].throttle           = conditions.energy.modulators[esc.tag].throttle 
    conditions.energy.converters[ducted_fan.tag].commanded_thrust_vector_angle =  conditions.energy.propulsors[propulsor.tag].commanded_thrust_vector_angle
    compute_ducted_fan_performance(ducted_fan,conditions)

    # Compute moment 
    moment_vector      = 0*state.ones_row(3)
    moment_vector[:,0] = ducted_fan.origin[0][0]  -  center_of_gravity[0][0] 
    moment_vector[:,1] = ducted_fan.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2] = ducted_fan.origin[0][2]  -  center_of_gravity[0][2]
    moment             =  np.cross(moment_vector, conditions.energy.converters[ducted_fan.tag].thrust) 
    
    # Detemine esc current 
    conditions.energy.modulators[esc.tag].outputs.current = conditions.energy.converters[motor.tag].inputs.current
    compute_current_in_from_throttle(esc,conditions)   
    
    stored_results_flag            = True
    stored_propulsor_tag           = propulsor.tag 
    
    # compute total forces and moments from propulsor (future work would be to add moments from motors)
    EDF_conditions.thrust      = conditions.energy.converters[ducted_fan.tag].thrust  
    EDF_conditions.moment      = moment
    EDF_conditions.power       = conditions.energy.converters[ducted_fan.tag].power 
    bus_conditions.power_draw  +=  conditions.energy.modulators[esc.tag].inputs.power*bus.power_split_ratio /bus.efficiency 
    
    return EDF_conditions.thrust,EDF_conditions.moment,EDF_conditions.power,conditions.energy.modulators[esc.tag].inputs.power,stored_results_flag,stored_propulsor_tag 
                
def reuse_stored_electric_ducted_fan_data(propulsor,state,network,fuel_line,bus,stored_propulsor_tag,center_of_gravity= [[0.0, 0.0,0.0]]):
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
    # unpack
    conditions                 = state.conditions 
    bus_conditions             = conditions.energy[bus.tag]
    motor                      = propulsor.motor 
    ducted_fan                 = propulsor.ducted_fan 
    esc                        = propulsor.electronic_speed_controller  
    motor_0                    = network.propulsors[stored_propulsor_tag].motor 
    ducted_fan_0               = network.propulsors[stored_propulsor_tag].ducted_fan 
    esc_0                      = network.propulsors[stored_propulsor_tag].electronic_speed_controller 
    
    # deep copy results 
    conditions.energy.converters[motor.tag]        = deepcopy(conditions.energy.converters[motor_0.tag])
    conditions.energy.converters[ducted_fan.tag]   = deepcopy(conditions.energy.converters[ducted_fan_0.tag])
    conditions.energy.modulators[esc.tag]          = deepcopy(conditions.energy.modulators[esc_0.tag])
  
    # compute moment 
    thrust_vector           = conditions.energy.converters[ducted_fan.tag].thrust  
    P_mech                  = conditions.energy.converters[ducted_fan.tag].power 
    P_elec                  = conditions.energy.modulators[esc.tag].inputs.power   
    moment_vector           = 0*state.ones_row(3) 
    moment_vector[:,0]      = ducted_fan.origin[0][0]  -  center_of_gravity[0][0] 
    moment_vector[:,1]      = ducted_fan.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2]      = ducted_fan.origin[0][2]  -  center_of_gravity[0][2]
    moment                  =  np.cross(moment_vector, thrust_vector)
    
    # pack results 
    conditions.energy.converters[ducted_fan.tag].moment = moment  
    conditions.energy.propulsors[propulsor.tag].thrust  = thrust_vector  
    conditions.energy.propulsors[propulsor.tag].moment  = moment  
    
    bus_conditions.power_draw  += P_elec*bus.power_split_ratio /bus.efficiency        
    return thrust_vector,moment,P_mech,P_elec