# RCAIDE/Methods/Library/Methods/Powertrain/Converters/compute_turboelectric_generator_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports      
from RCAIDE.Framework.Core import Data    
from RCAIDE.Library.Methods.Powertrain.Converters.Turboshaft         import compute_turboshaft_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Generator          import compute_generator_performance 
 
# python imports 
from copy import deepcopy 
import numpy as np
# ----------------------------------------------------------------------------------------------------------------------
# compute_turboelectric_generator_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_turboelectric_generator_performance(turboelectric_generator,state,fuel_line, bus):    
    ''' Computes the perfomrance of one turboelectric_generator
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    conditions               - operating conditions data structure                  [-]  
    fuel_line                - fuel+line                                            [-] 
    turboelectric_generator  - turboelectric_generator data structure               [-] 
    total_power              - power of turboelectric_generator group               [W] 

    Outputs:  
    total_power              - power of turboelectric_generator group               [W] 
    stored_results_flag      - boolean for stored results                           [-]     
    stored_propulsor_tag     - name of turboelectric_generator with stored results  [-]
    
    Properties Used: 
    N.A.        
    '''

    conditions        = state.conditions
    generator         = turboelectric_generator.generator
    turboshaft        = turboelectric_generator.turboshaft # check if there are more than one turboshaft

    turboelectric_generator_conditions = conditions.energy.fuel_line[turboelectric_generator.tag] 
    state.conditions.energy.fuel_line[turboelectric_generator.tag][turboshaft.tag].throttle = turboelectric_generator_conditions.throttle
    P_mech,stored_results_flag,stored_propulsor_tag = compute_turboshaft_performance(turboshaft,state,turboelectric_generator,fuel_line) 
    #turboshaft.compute_turboshaft_performance()

    omega        = turboshaft.angular_velocity       # MATTEO check this, it doesnt exist!@@@!!!!
    
    generator_conditions    = turboelectric_generator_conditions[generator.tag]
    generator_conditions.inputs.shaft_power      = P_mech    # MATTEO PLEASE VERIFY? 
    generator_conditions.inputs.omega            = omega     # MATTEO PLEASE VERIFY?

    generator_conditions.voltage                 = bus.voltage*np.ones_like(generator_conditions.inputs.shaft_power)  

    compute_generator_performance(generator,generator_conditions,conditions)   
    P_elec                                       = generator_conditions.outputs.power
    
    conditions.energy[bus.tag].power_draw -=  P_elec # MATTEO I MADE THIS NEGATIVE BECAUSE POWER IS ENTERING THE SYSTEM
    
    # Pack results      
    stored_results_flag    = True
    stored_propulsor_tag   = turboelectric_generator.tag
    
    return P_mech,P_elec,stored_results_flag,stored_propulsor_tag

def reuse_stored_turboelectric_generator_data(turboelectric_generator,state,fuel_line,bus,stored_converter_tag,center_of_gravity= [[0.0, 0.0,0.0]]):
    '''Reuses results from one turboelectric_generator for identical propulsors
    
    Assumptions: 
    N/A

    Source:
    N/A

    Inputs:  
    conditions           - operating conditions data structure     [-]  
    fuel_line            - fuelline                                [-] 
    turboelectric_generator           - turboelectric_generator data structure              [-] 
    total_power          - power of turboelectric_generator group               [W] 

    Outputs:  
    total_power          - power of turboelectric_generator group               [W] 
    
    Properties Used: 
    N.A.        
    '''

    conditions                 = state.conditions 
    bus_conditions             = conditions.energy[bus.tag]
    generator                  = turboelectric_generator.generator 
    turboshaft                 = turboelectric_generator.turboshaft  
    generator_0                = fuel_line.turboelectric_generators[stored_converter_tag].generator  
    turboshaft_0               = fuel_line.turboelectric_generators[stored_converter_tag].turboshaft    
    
    conditions.energy[turboelectric_generator.tag][generator.tag]        = deepcopy(conditions.energy[stored_converter_tag][generator_0.tag])
    conditions.energy[turboelectric_generator.tag][turboshaft.tag]       = deepcopy(conditions.energy[stored_converter_tag][turboshaft_0.tag]) 
   
    P_elec         = conditions.energy[turboelectric_generator.tag][generator.tag].power 
    P_mech         = conditions.energy[turboelectric_generator.tag][turboshaft.tag].power   
     
    #bus_conditions.power_draw  -= P_elec*bus.power_split_ratio /bus.efficiency   # MATTEO I MADE THIS NEGATIVE BECAUSE POWER IS ENTERING THE SYSTEM      
    return P_mech, P_elec
 