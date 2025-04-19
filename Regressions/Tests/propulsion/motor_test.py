# motor_test.py
# 
# Created:  Jan 2025, M. Clarke, M. Guidotti

#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
from RCAIDE.Framework.Core                              import Units, Data
from RCAIDE.Library.Plots                               import *      
from RCAIDE.Library.Methods.Powertrain.Converters       import Motor
from RCAIDE.Library.Methods.Powertrain.Converters.Motor.design_DC_motor import design_DC_motor
from RCAIDE.Library.Methods.Powertrain                  import setup_operating_conditions 

import os 
import numpy as np 
import sys 

# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles' + os.path.sep + 'Rotors'))
from Test_Propeller    import Test_Propeller   
 
def main(): 
    motor_type  = ['DC_Motor', 'PMSM_Motor']
    omega_truth = [36.455819245889195,37.653244590335106]
    torque_truth = [145.0992694988849,145.7740091588354]
    current_truth = [73.5454727365098,73.0]
    voltage_truth = [120,120.0]

    for i in range(len(motor_type)):
        motor = design_test_motor( motor_type[i])
        
        # set up default operating conditions 
        operating_state,propulsor_tag  = setup_operating_conditions(motor) 
        
        # Assign conditions to the rotor
        motor_conditions = operating_state.conditions.energy[propulsor_tag][motor.tag]
        motor_conditions.voltage[:, 0]   = 120

        if (type(motor) == RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor):

            Q_cond_path_truth               = [375.04333098554946]
            Q_conv_path_truth               = [0.024899697947077856]
            Q_conv_path_cooling_flow_truth  = [0.0011197159981354437]
            Q_conv_airgap_truth             = [0.0003900450642249714]
            Q_conv_endspace_truth           = [0.048254460826049554]
            Loss_cooling_truth              = [4.000000000000001e-08]

            motor_conditions.current[:, 0] =  73.
            Motor.compute_motor_performance(motor,motor_conditions,operating_state.conditions)

            Q_cond_path              = motor_conditions.Q_cond_path                            
            Q_conv_path              = motor_conditions.Q_conv_path                            
            Q_conv_path_cooling_flow = motor_conditions.Q_conv_path_cooling_flow               
            Q_conv_airgap            = motor_conditions.Q_conv_airgap                          
            Q_conv_endspace          = motor_conditions.Q_conv_endspace                        
            Loss_cooling             = motor_conditions.Loss_cooling            
    
        else:
            motor_conditions.rotor_power_coefficient[:, 0] =  0.5
            Motor.compute_motor_performance(motor,motor_conditions,operating_state.conditions)

        # run analysis 
        omega   = motor_conditions.omega
        torque  = motor_conditions.torque
        current = motor_conditions.current
        voltage = motor_conditions.voltage
 
        # Truth values 
        error = Data()
        error.omega_test     = np.max(np.abs(omega_truth[i]   - omega[0][0]  ))
        error.torque_test    = np.max(np.abs(torque_truth[i]  - torque[0][0] ))
        error.current_test   = np.max(np.abs(current_truth[i] - current[0][0])) 
        error.voltage_test   = np.max(np.abs(voltage_truth[i] - voltage[0][0])) 

        if (type(motor) == RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor):
            error.Q_cond_path_test      = np.max(np.abs(Q_cond_path_truth[0] - Q_cond_path))
            error.Q_conv_path_test      = np.max(np.abs(Q_conv_path_truth[0] - Q_conv_path))
            error.Q_conv_path_cooling_flow_test = np.max(np.abs(Q_conv_path_cooling_flow_truth[0] - Q_conv_path_cooling_flow))
            error.Q_conv_airgap_test    = np.max(np.abs(Q_conv_airgap_truth[0] - Q_conv_airgap))
            error.Q_conv_endspace_test  = np.max(np.abs(Q_conv_endspace_truth[0] - Q_conv_endspace))
            error.Loss_cooling_test     = np.max(np.abs(Loss_cooling_truth[0] - Loss_cooling))
        
        print('Errors:')
        print(error)
        
        for k,v in list(error.items()):
            assert(np.abs(v)<1e-6) 
               
    return

def design_test_motor(motor_type): 
    
    if motor_type == 'DC_Motor':
        motor = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    
        prop = Test_Propeller()
        motor.mass_properties.mass = 9. * Units.kg 
        motor.efficiency           = 0.935
        motor.gear_ratio           = 1. 
        motor.gearbox_efficiency   = 1. # Gear box efficiency     
        motor.no_load_current      = 2.0 
        motor.propeller_radius     = prop.tip_radius
        motor.nominal_voltage      = 400
        motor.rotor_radius         = prop.tip_radius
        motor.design_torque        = prop.cruise.design_torque 
        motor.angular_velocity     = prop.cruise.design_angular_velocity # Horse power of gas engine variant  750 * Units['hp']
        design_DC_motor(motor) 
    elif motor_type == 'PMSM_Motor':
        motor = RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor()
        motor.speed_constant            = 3                        # [rpm/V]        speed constant
        motor.stator_inner_diameter     = 0.16                        # [m]            stator inner diameter
        motor.stator_outer_diameter     = 0.348                       # [m]            stator outer diameter

        # Input data from Literature
        motor.winding_factor            = 0.95                        # [-]            winding factor

        # Input data from Assumptions
        motor.motor_stack_length        = 11.40                       # [m]            (It should be around 0.14 m) motor stack length 
        motor.number_of_turns           = 100                          # [-]            number of turns  
        motor.length_of_path            = 0.4                         # [m]            length of the path  
        motor.mu_0                      = 1.256637061e-5              # [N/A**2]       permeability of free space
        motor.mu_r                      = 1005                        # [N/A**2]       relative permeability of the magnetic material 
        
    else:
        raise ValueError('Invalid motor type')
    return motor

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()