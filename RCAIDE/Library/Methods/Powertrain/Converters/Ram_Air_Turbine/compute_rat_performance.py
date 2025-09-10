# RCAIDE/Methods/Library/Methods/Powertrain/Converters/compute_rat_performance.py
# 
# 
# Created:  Sep 2025, M. Guidotti

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
def compute_rat_performance(Ram_Air_Turbine, state, bus=None):
    """

    """

    conditions                         = state.conditions
    generator                          = Ram_Air_Turbine.generator
    rotor                              = Ram_Air_Turbine.rotor 
    rat_conditions                     = conditions.energy.converters[Ram_Air_Turbine.tag] 
    generator_conditions               = conditions.energy.converters[generator.tag]
    rotor_conditions                   = conditions.energy.converters[rotor.tag]
        
    # connect properties of the turboshaft to generator 
    rat_conditions.power               = rotor_conditions.power
    generator_conditions.inputs.power  = rotor_conditions.P_mech     
    generator_conditions.inputs.omega  = rotor_conditions.omega         
    
    # assign voltage across bus 
    generator_conditions.outputs.voltage = bus.voltage*np.ones_like(generator_conditions.inputs.power)
    
     # run the generator 
    compute_generator_performance(generator,conditions)   
    
    P_elec                      = generator_conditions.outputs.power       
    
    # Pack results      
    stored_results_flag    = True
    stored_converter_tag   = rotor.tag
    
    return P_mech,P_elec,stored_results_flag,stored_converter_tag

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
 
    conditions                  = state.conditions 
    generator                   = turboelectric_generator.generator
    rotor                       = turboelectric_generator.rotor

    # deep copy results 
    conditions.energy.converters[generator.tag]                = deepcopy(conditions.energy.converters[generator_0.tag]) 
    conditions.energy.converters[rotor.tag]                    = deepcopy(conditions.energy.converters[rotor_0.tag])
    conditions.energy.converters[rat.tag]                      = deepcopy(conditions.energy.converters[stored_converter_tag]) 
    
    P_elec         = conditions.energy.converters[generator.tag].outputs.power 
    P_mech         = conditions.energy.converters[rotor.tag].outputs.power  
    
    return P_mech, P_elec
 