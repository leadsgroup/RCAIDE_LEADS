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
def compute_turboelectric_generator_performance(turboelectric_generator, state, fuel_line=None, bus=None):
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
        - Direct calculation (inverse_calculation=False): Computes generator output based on 
        turboshaft throttle setting
        - Inverse calculation (inverse_calculation=True): Determines turboshaft fuel consumption 
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
    compressor                         = turboshaft.compressor
    turboelectric_generator_conditions = conditions.energy.converters[turboelectric_generator.tag] 
    generator_conditions               = conditions.energy.converters[generator.tag]
    turboshaft_conditions              = conditions.energy.converters[turboshaft.tag]
    compressor_conditions              = conditions.energy.converters[compressor.tag]
    
    generator.inverse_calculation      =  turboelectric_generator.inverse_calculation
    turboshaft.inverse_calculation     =  turboelectric_generator.inverse_calculation
    
    if turboelectric_generator.inverse_calculation == False:
        # here we run the turboshaft first, then run the generator
        turboshaft_conditions.throttle = turboelectric_generator_conditions.throttle
        
        # run the generator 
        P_mech,stored_results_flag,stored_propulsor_tag = compute_turboshaft_performance(turboshaft,state,turboelectric_generator,fuel_line)
        
        # connect properties of the turboshaft to generator 
        turboelectric_generator_conditions.power = generator_conditions.outputs.power
        generator_conditions.inputs.power  = P_mech     
        generator_conditions.inputs.omega  = compressor_conditions.omega         
        
        # assign voltage across bus 
        generator_conditions.outputs.voltage = bus.voltage*np.ones_like(generator_conditions.inputs.power)
        
         # run the generator 
        compute_generator_performance(generator,conditions)   
        turboelectric_generator_conditions.fuel_mass_flow_rate =  turboshaft_conditions.fuel_mass_flow_rate  
         
    else:
        # here , we know the electric power produced by the generator and we want to determine how much fuel was used to produce said power
        
        # assign voltage across bus 
        generator_conditions.outputs.voltage = bus.voltage*np.ones_like(generator_conditions.outputs.power)
        generator_conditions.outputs.current = generator_conditions.outputs.power / generator_conditions.outputs.voltage
        
        generator.inverse_calculation = True
        # run the generator 
        compute_generator_performance(generator,conditions)
        
        # connect properties of the generator to the turboshaft 
        turboshaft_conditions.power  = generator_conditions.inputs.power
        
        # run the turboshaft 
        P_mech,stored_results_flag,stored_propulsor_tag = compute_turboshaft_performance(turboshaft,state,turboelectric_generator,fuel_line) 
        turboelectric_generator_conditions.fuel_mass_flow_rate =  turboshaft_conditions.fuel_mass_flow_rate   
    
    stored_results_flag            = True
    stored_converter_tag           = turboelectric_generator.tag  

    turboelectric_generator_conditions.power.propulsive               = 0.0 * state.ones_row(1)
    turboelectric_generator_conditions.power.mechanical               = 0.0 * state.ones_row(1)
    turboelectric_generator_conditions.power.electrical               = generator_conditions.outputs.power       
    turboelectric_generator_conditions.power.chemical                 = 0.0 * state.ones_row(1)
    turboelectric_generator_conditions.power.pneumatic                = 0.0 * state.ones_row(1)
    turboelectric_generator_conditions.power.hydraulic                = 0.0 * state.ones_row(1)
    turboelectric_generator_conditions.power.thermal                  = 0.0 * state.ones_row(1)

    return  turboelectric_generator_conditions.power, stored_results_flag, stored_converter_tag

def reuse_stored_turboelectric_generator_data(turboelectric_generator, state, network,stored_converter_tag,fuel_line=None, bus=None):
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
 
    P_elec         = conditions.energy.converters[generator.tag].outputs.power 
    P_mech         = conditions.energy.converters[turboshaft.tag].power  
    
    return P_mech, P_elec
 