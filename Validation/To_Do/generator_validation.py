# generator_validation.py
# 
# Created:  Feb 2025, M. Clarke, M. Guidotti

#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
from RCAIDE.Framework.Core                              import Units, Data
from RCAIDE.Library.Plots                               import *      
from RCAIDE.Library.Methods.Powertrain.Converters       import generator
from RCAIDE.Library.Methods.Powertrain.Converters.generator.design_generator import design_generator
from RCAIDE.Library.Methods.Powertrain                  import setup_operating_conditions 

import os 
import numpy as np 
import sys 

# local imports 
sys.path.append('/Users/matteoguidotti/Documents/Research/LEADS/RCAIDE/Regressions/Vehicles/Rotors')
from Test_Propeller import Test_Propeller   
 
def main(): 
    generator_type    = ['DC_generator', 'PMSM_generator']
    
    omega_truth   = [82.75490115895134,418.53135089164175]
    torque_truth  = [747.685485650496,384.67761377296347]
    current_truth = [370.6683724448575,375.0]
    voltage_truth = [400.0,610.0]

    for i in range(len(generator_type)):
        generator = design_test_generator( generator_type[i])
        
        # set up default operating conditions 
        operating_state,propulsor_tag  = setup_operating_conditions(generator) 
        
        # Assign conditions to the generator
        generator_conditions = operating_state.conditions.energy[propulsor_tag][generator.tag]
        generator_conditions.voltage[:, 0]   = generator.nominal_voltage

        if (type(generator) == RCAIDE.Library.Components.Powertrain.Converters.PMSM_generator):

            Q_cond_path_truth               = [375.04333098554946]
            Q_conv_path_truth               = [0.024899697947077856]
            Q_conv_path_cooling_flow_truth  = [0.0011197159981354437]
            Q_conv_airgap_truth             = [0.0003900450642249714]
            Q_conv_endspace_truth           = [0.048254460826049554]
            Loss_cooling_truth              = [4.000000000000001e-08]

            generator_conditions.current[:, 0] =  generator.nominal_current
            generator.compute_generator_performance(generator,generator_conditions,operating_state.conditions)

            Q_cond_path              = generator_conditions.Q_cond_path                            
            Q_conv_path              = generator_conditions.Q_conv_path                            
            Q_conv_path_cooling_flow = generator_conditions.Q_conv_path_cooling_flow               
            Q_conv_airgap            = generator_conditions.Q_conv_airgap                          
            Q_conv_endspace          = generator_conditions.Q_conv_endspace                        
            Loss_cooling             = generator_conditions.Loss_cooling            
    
        else:
            generator_conditions.rotor_power_coefficient[:, 0] =  0.5
            generator.compute_generator_performance(generator,generator_conditions,operating_state.conditions)

        # run analysis 
        omega   = generator_conditions.omega
        torque  = generator_conditions.torque
        current = generator_conditions.current
        voltage = generator_conditions.voltage
 
        # Truth values 
        error = Data()
        error.omega_test     = np.max(np.abs(omega_truth[i]   - omega[0][0]  ))
        error.torque_test    = np.max(np.abs(torque_truth[i]  - torque[0][0] ))
        error.current_test   = np.max(np.abs(current_truth[i] - current[0][0])) 
        error.voltage_test   = np.max(np.abs(voltage_truth[i] - voltage[0][0])) 

        if (type(generator) == RCAIDE.Library.Components.Powertrain.Converters.PMSM_generator):
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

def design_test_generator(generator_type): 
    
    if generator_type == 'DC_generator':
        generator = RCAIDE.Library.Components.Powertrain.Converters.DC_generator()
    
        prop = Test_Propeller()
        generator.mass_properties.mass = 9. * Units.kg 
        generator.efficiency           = 0.935
        generator.gear_ratio           = 1. 
        generator.gearbox_efficiency   = 1. # Gear box efficiency     
        generator.no_load_current      = 2.0 
        generator.propeller_radius     = prop.tip_radius
        generator.nominal_voltage      = 400
        generator.nominal_current      = generator.no_load_current
        generator.rotor_radius         = prop.tip_radius
        generator.design_torque        = prop.cruise.design_torque 
        generator.angular_velocity     = prop.cruise.design_angular_velocity # Horse power of gas engine variant  750 * Units['hp']
        design_DC_generator(generator) 
    elif generator_type == 'PMSM_generator':
        generator = RCAIDE.Library.Components.Powertrain.Converters.PMSM_generator()
        generator.speed_constant            = 6.56                      # [rpm/V]        speed constant
        generator.nominal_voltage           = 610                       # [V]            nominal voltage
        generator.nominal_current           = 375                       # [A]            nominal current
        generator.stator_inner_diameter     = 0.16                      # [m]            stator inner diameter
        generator.stator_outer_diameter     = 0.348                     # [m]            stator outer diameter

        # Input data from Literature
        generator.winding_factor            = 0.95                      # [-]            winding factor

        # Input data from Assumptions
        generator.generator_stack_length        = 11.40                     # [m]            (It should be around 0.14 m) generator stack length 
        generator.number_of_turns           = 80                       # [-]            number of turns  
        generator.length_of_path            = 0.4                       # [m]            length of the path  
        generator.mu_0                      = 1.256637061e-6            # [N/A**2]       permeability of free space
        generator.mu_r                      = 1005                      # [N/A**2]       relative permeability of the magnetic material 
        
    else:
        raise ValueError('Invalid generator type')
    return generator

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()