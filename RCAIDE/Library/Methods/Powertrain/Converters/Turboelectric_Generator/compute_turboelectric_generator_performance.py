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

    conditions                         = state.conditions
    generator                          = turboelectric_generator.generator
    turboshaft                         = turboelectric_generator.turboshaft
    turboshaft.mode                    = turboelectric_generator.mode
    compressor                         = turboshaft.compressor
    turboelectric_generator_conditions = conditions.energy[turboelectric_generator.tag] 
    generator_conditions               = turboelectric_generator_conditions[generator.tag]
    turboshaft_conditions              = turboelectric_generator_conditions[turboshaft.tag]
    compressor_conditions              = turboelectric_generator_conditions[turboshaft.tag][compressor.tag]
    generator.mode                     = turboelectric_generator.mode
    
    if turboelectric_generator.mode == 'forward':
        # here we run the turboshaft first, then run the generator
        turboshaft_conditions.throttle = turboelectric_generator_conditions.throttle
        
        # run the generator 
        P_mech,stored_results_flag,stored_propulsor_tag = compute_turboshaft_performance(turboshaft,state,turboelectric_generator,fuel_line)
        
        # connect properties of the turboshaft to generator 
        generator_conditions.inputs.power      = P_mech     
        generator_conditions.inputs.omega      = compressor_conditions.omega         
        
        # assign voltage across bus 
        generator_conditions.voltage                 = bus.voltage*np.ones_like(generator_conditions.inputs.power)
        
         # run the generator 
        compute_generator_performance(generator,generator_conditions,conditions)   
         
    elif turboelectric_generator.mode == 'reverse':
        # here , we know the electric power produced by the generator and we want to determine how much fuel was used to produce said power
        
        # assign voltage across bus 
        generator_conditions.voltage       = bus.voltage*np.ones_like(generator_conditions.outputs.power)
        
        # run the generator 
        compute_generator_performance(generator,generator_conditions,conditions)
        
        # connect properties of the generator to the turboshaft 
        turboshaft_conditions.power  = generator_conditions.inputs.power
        
        # run the turboshaft 
        P_mech,stored_results_flag,stored_propulsor_tag = compute_turboshaft_performance(turboshaft,state,turboelectric_generator,fuel_line) 
    
    
    P_elec = generator_conditions.outputs.power
    
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
    generator                  = turboelectric_generator.generator 
    turboshaft                 = turboelectric_generator.turboshaft  
    generator_0                = fuel_line.turboelectric_generators[stored_converter_tag].generator  
    turboshaft_0               = fuel_line.turboelectric_generators[stored_converter_tag].turboshaft    
    
    conditions.energy[turboelectric_generator.tag][generator.tag]        = deepcopy(conditions.energy[stored_converter_tag][generator_0.tag])
    conditions.energy[turboelectric_generator.tag][turboshaft.tag]       = deepcopy(conditions.energy[stored_converter_tag][turboshaft_0.tag]) 
   
    P_elec         = conditions.energy[turboelectric_generator.tag][generator.tag].power 
    P_mech         = conditions.energy[turboelectric_generator.tag][turboshaft.tag].power
    
    return P_mech, P_elec
 