# RCAIDE/Library/Missions/Common/Initialize/energy.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Sep 2024, S. Shekar
# Modified: Sep 2025, M. Guidotti

import RCAIDE

# ----------------------------------------------------------------------------------------------------------------------
#  energy
# ----------------------------------------------------------------------------------------------------------------------  
def energy(segment):
    """
    Initializes energy states for vehicle networks at mission segment start

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function initializes energy-related conditions for all energy networks
    in the vehicle, including batteries, fuel systems, and thermal management
    systems. It handles both electrical and fuel-based energy storage systems.

    The function processes:
    1. Electrical networks with busses
        - Battery module conditions
        - Thermal management systems
            * Battery cooling systems
            * Heat exchangers
            * Coolant reservoirs
    2. Fuel-based networks
        - Fuel tank conditions
        - Fuel mass tracking

    **Required Segment Components**

    segment.analyses.vehicle.networks:
        Network configurations containing:
        - Electrical busses with battery modules
        - Cooling systems and heat exchangers
        - Fuel lines and tanks

    **State Variables**

    conditions.energy:
        For electrical systems:
        - Battery states
        - Thermal conditions
        - Coolant properties

        For fuel systems:
        - Fuel mass
        - Tank conditions

    **Major Assumptions**
        * Well-defined network architecture
        * Valid initial conditions
        * Compatible energy storage systems
        * Proper thermal management setup

    Returns
    -------
    None
        Updates segment conditions directly

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """ 

    energy_conditions  = segment.state.conditions.energy
    initial_conditions = initial_conditions
    vehicle            = segment.analyses.vehicle
    ones_row           = segment.state.ones_row  
 
    for network in vehicle.networks:
        
        # resets the conditions of components each iteration if the mission solver
        for propulsor in  network.propulsors: 
            propulsor.append_segment_conditions(segment)
            energy_conditions.propulsors[propulsor.tag].inputs.power.electrical[0,:]  = 0  # initial_conditions.propulsors[propulsor.tag].inputs.power.electrical[-1,0] 
            energy_conditions.propulsors[propulsor.tag].inputs.power.chemical[0,:]    = 0  # initial_conditions.propulsors[propulsor.tag].inputs.power.chemical[-1,0]   
            energy_conditions.propulsors[propulsor.tag].inputs.power.thermal[0,:]     = 0  # initial_conditions.propulsors[propulsor.tag].inputs.power.thermal[-1,0]    
            energy_conditions.propulsors[propulsor.tag].outputs.power.electrical[0,:] = 0  # initial_conditions.propulsors[propulsor.tag].outputs.power.electrical[-1,0]
            energy_conditions.propulsors[propulsor.tag].outputs.power.chemical[0,:]   = 0  # initial_conditions.propulsors[propulsor.tag].outputs.power.chemical[-1,0]  
            energy_conditions.propulsors[propulsor.tag].outputs.power.thermal[0,:]    = 0  # initial_conditions.propulsors[propulsor.tag].outputs.power.thermal[-1,0]   
         
        for converter in network.non_propulsive_converters:
            converter.append_segment_conditions(segment)
            energy_conditions.converters[converter.tag].inputs.power.electrical[0,:]  = 0  #initial_conditions.converters[converter.tag].inputs.power.electrical[-1,0]  
            energy_conditions.converters[converter.tag].inputs.power.chemical[0,:]    = 0  #initial_conditions.converters[converter.tag].inputs.power.chemical[-1,0]    
            energy_conditions.converters[converter.tag].inputs.power.thermal[0,:]     = 0  #initial_conditions.converters[converter.tag].inputs.power.thermal[-1,0]     
            energy_conditions.converters[converter.tag].outputs.power.electrical[0,:] = 0  #initial_conditions.converters[converter.tag].outputs.power.electrical[-1,0] 
            energy_conditions.converters[converter.tag].outputs.power.chemical[0,:]   = 0  #initial_conditions.converters[converter.tag].outputs.power.chemical[-1,0]   
            energy_conditions.converters[converter.tag].outputs.power.thermal[0,:]    = 0  #initial_conditions.converters[converter.tag].outputs.power.thermal[-1,0]              
        
    
        for modulator in network.modulators:
            modulator.append_segment_conditions(segment)
            energy_conditions.modulators[modulator.tag].inputs.power.electrical[0,:]  = 0  #initial_conditions.modulators[modulator.tag].inputs.power.electrical[-1,0] 
            energy_conditions.modulators[modulator.tag].inputs.power.chemical[0,:]    = 0  #initial_conditions.modulators[modulator.tag].inputs.power.chemical[-1,0]   
            energy_conditions.modulators[modulator.tag].inputs.power.thermal[0,:]     = 0  #initial_conditions.modulators[modulator.tag].inputs.power.thermal[-1,0]    
            energy_conditions.modulators[modulator.tag].outputs.power.electrical[0,:] = 0  #initial_conditions.modulators[modulator.tag].outputs.power.electrical[-1,0]
            energy_conditions.modulators[modulator.tag].outputs.power.chemical[0,:]   = 0  #initial_conditions.modulators[modulator.tag].outputs.power.chemical[-1,0]  
            energy_conditions.modulators[modulator.tag].outputs.power.thermal[0,:]    = 0  #initial_conditions.modulators[modulator.tag].outputs.power.thermal[-1,0]           
        
        for source in network.sources:
            source.append_segment_conditions(segment)
            energy_conditions.sources[source.tag].inputs.power.electrical[0,:]   = 0
            energy_conditions.sources[source.tag].inputs.power.chemical[0,:]     = 0
            energy_conditions.sources[source.tag].inputs.power.thermal[0,:]      = 0
            energy_conditions.sources[source.tag].outputs.power.electrical[0,:]  = 0
            energy_conditions.sources[source.tag].outputs.power.chemical[0,:]    = 0
            energy_conditions.sources[source.tag].outputs.power.thermal[0,:]     = 0
            
        for system in network.systems:
            energy_conditions.systems[system.tag].inputs.power.electrical[0,:]  = 0
            energy_conditions.systems[system.tag].inputs.power.chemical[0,:]    = 0
            energy_conditions.systems[system.tag].inputs.power.thermal[0,:]     = 0
            energy_conditions.systems[system.tag].outputs.power.electrica[0,:]  = 0
            energy_conditions.systems[system.tag].outputs.power.chemical[0,:]   = 0
            energy_conditions.systems[system.tag].outputs.power.therma[0,:]     = 0
        
        for distributor in network.distributors:
            distributor.append_segment_conditions(segment)
            
