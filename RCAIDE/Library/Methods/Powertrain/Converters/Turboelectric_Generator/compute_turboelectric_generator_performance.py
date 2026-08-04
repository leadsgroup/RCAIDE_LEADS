# RCAIDE/Library/Methods/Powertrain/Converters/Turboelectric_Generator/compute_turboelectric_generator_performance.py
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
    turboelectric_generator : RCAIDE.Library.Components.Powertrain.Converters.Turboelectric_Generator
        The turboelectric generator component for which performance is being computed
    state : RCAIDE.Framework.Mission.Common.State
        Container for mission segment conditions
    network : RCAIDE.Framework.Networks.Network, optional
        The network this generator belongs to, used to resolve its assigned
        distributor(s)

    Returns
    -------
    inputs : Data
        Generator input conditions (power.mechanical from the turboshaft, etc.)
    outputs : Data
        Generator output conditions (power.electrical, etc.)
    stored_results_flag : bool
        Flag indicating that results have been stored for potential reuse
    stored_converter_tag : str
        Tag identifier of the turboelectric generator with stored results
        
    Notes
    -----
    ``Network.evaluate()`` always sets ``reverse_mode_computation = True`` before calling a
    converter's ``compute_performance``, so this always runs in reverse (inverse) mode: the
    electrical power this generator must supply is treated as a known target, and the
    generator/turboshaft chain is solved backward from it to determine fuel consumption.

    The electrical target is computed the same way a fuel cell computes its own power share:
    its own electrical distributor's total demand, split between battery/generator sources
    by that distributor's ``battery_fuel_cell_power_split_ratio`` (psi) and between multiple
    identical generators on the same bus by this generator's own ``power_split_ratio``. The
    distributor is looked up from ``assigned_distributors`` rather than assumed, since this
    generator may share the vehicle with other electrically-isolated buses resolved to a
    different psi.

    **Major Assumptions**
        * The turboshaft and generator are properly connected and compatible
        * Bus voltage is constant across all operating conditions
        * Mechanical power from turboshaft is directly coupled to generator input

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Turboshaft.compute_turboshaft_performance
    RCAIDE.Library.Methods.Powertrain.Converters.Generator.compute_generator_performance
    """

    conditions                           = state.conditions
    generator                            = turboelectric_generator.generator
    turboshaft                           = turboelectric_generator.turboshaft
    turboelectric_generator_conditions   = conditions.energy.converters[turboelectric_generator.tag]
    generator_conditions                 = conditions.energy.converters[generator.tag]
    turboshaft_conditions                = conditions.energy.converters[turboshaft.tag]
    generator.reverse_mode_computation   = turboelectric_generator.reverse_mode_computation
    turboshaft.reverse_mode_computation  = turboelectric_generator.reverse_mode_computation

    for d_tag in turboelectric_generator.assigned_distributors[0]:
        if type(network.distributors[d_tag]) == RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus:
            distributor = network.distributors[d_tag]

    psi = state.conditions.energy.battery_fuel_cell_power_split_ratio[distributor.tag]
    total_electrical_demand = state.conditions.energy.distributors[distributor.tag].outputs.power.electrical
    turboelectric_generator_conditions.outputs.power.electrical = total_electrical_demand * turboelectric_generator.power_split_ratio * (1. - psi)

    # link turboelectric generator outputs to generator outputs
    generator_conditions.outputs.power.electrical = turboelectric_generator_conditions.outputs.power.electrical
    generator_conditions.outputs.voltage          = conditions.energy.distributors[distributor.tag].voltage
    generator_conditions.outputs.current          = generator_conditions.outputs.power.electrical / generator_conditions.outputs.voltage

    # run the generator
    _,_,_,_  = generator.compute_performance(state)

    # connect properties of the generator to the turboshaft. compute_power's reverse-mode
    # branch reads this as its target power, then overwrites it with the (same) computed
    # value as its normal output -- same field serves as both, like every other converter.
    turboshaft_conditions.outputs.power.mechanical = generator_conditions.inputs.power.mechanical

    # run the turboshaft
    _,_,_,_ = turboshaft.compute_performance(state)
    turboelectric_generator_conditions.fuel_mass_flow_rate =  turboshaft_conditions.fuel_mass_flow_rate

    stored_results_flag            = True
    stored_converter_tag           = turboelectric_generator.tag 
    return  turboelectric_generator_conditions.inputs,  turboelectric_generator_conditions.outputs, stored_results_flag, stored_converter_tag

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
    conditions.energy.converters[turboelectric_generator.tag] = deepcopy(conditions.energy.converters[stored_converter_tag]        )
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
 