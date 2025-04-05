# Generator_test.py
# 
# Created:  Jan 2025, M. Clarke, M. Guidotti

#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
from RCAIDE.Framework.Core                              import Units, Data
from RCAIDE.Library.Plots                               import *      
from RCAIDE.Library.Methods.Powertrain.Converters       import Generator
from RCAIDE.Library.Methods.Powertrain.Converters.Generator.design_optimal_generator import design_optimal_generator
from RCAIDE.Library.Methods.Powertrain                  import setup_operating_conditions 

import os 
import numpy as np 
import sys 

# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles' + os.path.sep + 'Rotors'))
from Test_Propeller    import Test_Propeller   
 
def main():
    
    forward_mode_model()
    
    inverse_mode_model()
    
    return 
    
def forward_mode_model():

    generator_type    = ['DC_Generator', 'PMSM_Generator'] 
    current_truth = [11.733611216369859,111.3693277099695] 
    voltage_truth = [420 , 420] 

    for i in range(len(generator_type)):
        generator = design_test_generator( generator_type[i])
        generator.inverse_calculation = False
        
        # set up default operating conditions 
        operating_state = setup_operating_conditions(generator) 
        
        # Assign conditions to the Generator
        generator_conditions = operating_state.conditions.energy.converters[generator.tag] 
        
        generator_conditions.inputs.omega[:, 0] = 120
        generator_conditions.inputs.power[:, 0] = 500

        generator_conditions.outputs.voltage[:, 0] = 420
        
        Generator.compute_generator_performance(generator,operating_state.conditions)

        # run analysis  
        current = generator_conditions.outputs.current
        voltage = generator_conditions.outputs.voltage
 
        # Truth values 
        error = Data() 
        error.current_test   = np.max(np.abs(current_truth[i] - current[0][0])) 
        error.voltage_test   = np.max(np.abs(voltage_truth[i] - voltage[0][0]))  
        
        print('Errors:')
        print(error)
        
        for k,v in list(error.items()):
            assert(np.abs(v)<1e-6) 
               
    return

def inverse_mode_model():

    generator_type    = ['DC_Generator', 'PMSM_Generator']
    omega_truth   = [89.67260012714557,14.399999999999999]
    torque_truth  = [223.1932139491046,1.646090249033937] 

    for i in range(len(generator_type)):
        generator = design_test_generator( generator_type[i])
        
        generator.inverse_calculation = True
        # set up default operating conditions 
        operating_state = setup_operating_conditions(generator) 
        
        # Assign conditions to the Generator
        generator_conditions = operating_state.conditions.energy.converters[generator.tag]
        generator_conditions.outputs.voltage[:, 0] = 480
        generator_conditions.outputs.current[:, 0] = 70

        Generator.compute_generator_performance(generator,operating_state.conditions)

        # run analysis 
        omega   = generator_conditions.inputs.omega
        torque  = generator_conditions.inputs.torque 
 
        # Truth values 
        error = Data()
        error.omega_test     = np.max(np.abs(omega_truth[i]   - omega[0][0]  ))
        error.torque_test    = np.max(np.abs(torque_truth[i]  - torque[0][0] )) 

        print('Errors:')
        print(error)
        
        for k,v in list(error.items()):
            assert(np.abs(v)<1e-6) 

    return

def design_test_generator(generator_type): 
    
    if generator_type == 'DC_Generator':
        generator = RCAIDE.Library.Components.Powertrain.Converters.DC_Generator()
    
        generator.mass_properties.mass    = 9. * Units.kg 
        generator.efficiency              = 0.98    
        generator.no_load_current         = 1.0 
        generator.nominal_voltage         = 400
        generator.design_torque           = 90
        generator.design_angular_velocity = 100
        generator.design_power            = generator.design_torque * generator.design_angular_velocity
        design_optimal_generator(generator) 
    elif generator_type == 'PMSM_Generator':
        generator = RCAIDE.Library.Components.Powertrain.Converters.PMSM_Generator()
        generator.speed_constant            = 0.03                        # [rpm/V]        speed constant
        generator.stator_inner_diameter     = 0.16                        # [m]            stator inner diameter
        generator.stator_outer_diameter     = 0.348                       # [m]            stator outer diameter

        # Input data from Literature
        generator.winding_factor            = 0.95                        # [-]            winding factor

        # Input data from Assumptions
        generator.generator_stack_length        = 11.40                       # [m]            (It should be around 0.14 m) Generator stack length 
        generator.number_of_turns           = 100                          # [-]            number of turns  
        generator.length_of_path            = 0.4                         # [m]            length of the path  
        generator.mu_0                      = 1.256637061e-5              # [N/A**2]       permeability of free space
        generator.mu_r                      = 1005                        # [N/A**2]       relative permeability of the magnetic material 
        generator.no_load_current           = 1.0 
        generator.stack_length              = 0.14
        generator.inner_diameter            = 0.16
    else:
        raise ValueError('Invalid Generator type')
    return generator

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()