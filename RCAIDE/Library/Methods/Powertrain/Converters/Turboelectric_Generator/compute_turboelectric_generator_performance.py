# RCAIDE/Methods/Library/Methods/Powertrain/Converters/compute_turboelectric_generator_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import Data      
 
# python imports 
from copy import deepcopy 
import numpy as np
# ----------------------------------------------------------------------------------------------------------------------
# compute_turboelectric_generator_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_turboelectric_generator_performance(turboelectric_generator,state,network=None):
    """
    Computes the performance of a turboelectric generator system.
    
    Parameters
    ----------
    turboelectric_generator : RCAIDE.Components.Energy.Converters.Turboelectric_Generator
        The turboelectric generator component for which performance is being computed
    state : RCAIDE.Framework.Mission.Common.State
        Container for mission segment conditions
    fuel_line : RCAIDE.Components.Energy.Distribution.Fuel_Line, optional
        Fuel distribution system connected to the turboelectric generator
    bus : RCAIDE.Components.Energy.Distribution.Bus, optional
        Electrical bus connected to the generator output
        
    Returns
    -------
    P_mech : float
        Mechanical power produced by the turboshaft engine [W]
    P_elec : float
        Electrical power produced by the generator [W]
    stored_results_flag : bool
        Flag indicating that results have been stored for potential reuse
    stored_propulsor_tag : str
        Tag identifier of the turboelectric generator with stored results
        
    Notes
    -----
    This function handles both direct and inverse calculations for the turboelectric generator:
        - Direct calculation (reverse_mode_computation=False): Computes generator output based on 
        turboshaft throttle setting
        - Inverse calculation (reverse_mode_computation=True): Determines turboshaft fuel consumption 
        based on required generator output power
    
    The function coordinates the operation of the turboshaft engine and generator components,
    ensuring proper power flow and electrical characteristics.
    
    **Major Assumptions**
        * The turboshaft and generator are properly connected and compatible
        * Bus voltage is constant across all operating conditions
        * Mechanical power from turboshaft is directly coupled to generator input
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Turboshaft.compute_turboshaft_performance
    RCAIDE.Library.Methods.Powertrain.Converters.Generator.compute_generator_performance
    """

    conditions                         = state.conditions
    generator                          = turboelectric_generator.generator
    turboshaft                         = turboelectric_generator.turboshaft  
    turboelectric_generator_conditions = conditions.energy.converters[turboelectric_generator.tag] 
    generator_conditions               = conditions.energy.converters[generator.tag]
    turboshaft_conditions              = conditions.energy.converters[turboshaft.tag]
    generator.reverse_mode_computation      = turboelectric_generator.reverse_mode_computation
    turboshaft.reverse_mode_computation     = turboelectric_generator.reverse_mode_computation  

    # Determine what electrical distributor is connected to the electric powertrain 
    for d_tag in turboelectric_generator.assigned_distributors[0]:
        if type(network.distributors[d_tag]) == RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus:
            distributor = network.distributors[d_tag]
            
    if turboelectric_generator.reverse_mode_computation == False:
        # here we run the turboshaft first, then run the generator
        turboshaft_conditions.throttle = turboelectric_generator_conditions.throttle
        
        # run the turboshaft 
        _,_,_,_ =  turboshaft.compute_performance(state)
        turboelectric_generator_conditions.fuel_mass_flow_rate =  turboshaft_conditions.fuel_mass_flow_rate  
        
        # connect electrical power produced by of the turboshaft to generator  
        generator_conditions.inputs.omega             = turboelectric_generator_conditions.omega
        generator_conditions.inputs.power.mechanical  = turboshaft_conditions.outputs.power.mechanical # efficiency 
        generator_conditions.outputs.voltage          = conditions.energy.distributors[distributor.tag].voltage 
        
         # run the generator 
        _,_,_,_ =  generator.compute_performance(generator,conditions)  
        turboelectric_generator_conditions.outputs.power.electrical = generator_conditions.outputs.power.electrical  
         
    else: 
        # link turboelectric generator outputs to generator outputs 
        generator_conditions.outputs.power.electrical = turboelectric_generator_conditions.outputs.power.electrical
        generator_conditions.outputs.voltage          = conditions.energy.distributors[distributor.tag].voltage 
        generator_conditions.outputs.current          = generator_conditions.outputs.power.electrical / generator_conditions.outputs.voltage 
        generator.reverse_mode_computation = True
        
        # run the generator 
        _,_,_,_  = generator.compute_performance(generator)
        
        # connect properties of the generator to the turboshaft 
        turboshaft_conditions.outputs.power.mechanical  = generator_conditions.inputs.power.mechanical # /efficiency
        
        # run the turboshaft 
        _,_,_,_ = turboshaft.compute_performance(state) 
        turboelectric_generator_conditions.fuel_mass_flow_rate =  turboshaft_conditions.fuel_mass_flow_rate   
    
    stored_results_flag            = True
    stored_converter_tag           = turboelectric_generator.tag   

    return  turboelectric_generator_conditions.power, stored_results_flag, stored_converter_tag

def reuse_stored_turboelectric_generator_data(turboelectric_generator,state,network,stored_converter_tag):
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
    turboshaft                  = turboelectric_generator.turboshaft
    ram                         = turboelectric_generator.turboshaft.ram 
    inlet_nozzle                = turboelectric_generator.turboshaft.inlet_nozzle 
    compressor                  = turboelectric_generator.turboshaft.compressor
    low_pressure_turbine        = turboelectric_generator.turboshaft.low_pressure_turbine
    high_pressure_turbine       = turboelectric_generator.turboshaft.high_pressure_turbine 
    combustor                   = turboelectric_generator.turboshaft.combustor
    core_nozzle                 = turboelectric_generator.turboshaft.core_nozzle

    generator_0                = network.converters[stored_converter_tag].generator 
    turboshaft_0               = network.converters[stored_converter_tag].turboshaft
    ram_0                      = network.converters[stored_converter_tag][turboshaft_0.tag].ram
    inlet_nozzle_0             = network.converters[stored_converter_tag][turboshaft_0.tag].inlet_nozzle 
    compressor_0               = network.converters[stored_converter_tag][turboshaft_0.tag].compressor
    high_pressure_turbine_0    = network.converters[stored_converter_tag][turboshaft_0.tag].high_pressure_turbine
    combustor_0                = network.converters[stored_converter_tag][turboshaft_0.tag].combustor
    low_pressure_turbine_0     = network.converters[stored_converter_tag][turboshaft_0.tag].low_pressure_turbine
    core_nozzle_0              = network.converters[stored_converter_tag][turboshaft_0.tag].core_nozzle

    # deep copy results 
    conditions.energy.converters[generator.tag]             = deepcopy(conditions.energy.converters[generator_0.tag]            )
    conditions.energy.converters[turboshaft.tag]            = deepcopy(conditions.energy.converters[turboshaft_0.tag]           )
    conditions.energy.converters[ram.tag]                   = deepcopy(conditions.energy.converters[ram_0.tag]                  )
    conditions.energy.converters[inlet_nozzle.tag]          = deepcopy(conditions.energy.converters[inlet_nozzle_0.tag]         )
    conditions.energy.converters[compressor.tag]            = deepcopy(conditions.energy.converters[compressor_0.tag]           )
    conditions.energy.converters[low_pressure_turbine.tag]  = deepcopy(conditions.energy.converters[low_pressure_turbine_0.tag] )
    conditions.energy.converters[high_pressure_turbine.tag] = deepcopy(conditions.energy.converters[high_pressure_turbine_0.tag])
    conditions.energy.converters[combustor.tag]             = deepcopy(conditions.energy.converters[combustor_0.tag]            )
    conditions.energy.converters[core_nozzle.tag]           = deepcopy(conditions.energy.converters[core_nozzle_0.tag]          )
  
    
    return conditions.energy.converters[turboelectric_generator.tag].inputs, conditions.energy.converters[turboelectric_generator.tag].outputs
 