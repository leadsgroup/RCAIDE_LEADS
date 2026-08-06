# RCAIDE/Methods/Powertrain/Sources/Batteries/Common/append_battery_unknown_and_residual.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Sep 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  append_battery_unknown_and_residual
# ----------------------------------------------------------------------------------------------------------------------
def append_battery_unknown_and_residual(module, battery,segment):
    """
    Appends the cell temperature and state-of-charge unknowns and residuals for a
    battery module to be solved by the mission solver.

    Parameters
    ----------
    module : RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Module
        Battery module for which unknowns and residuals are being appended
    battery : RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack
        Battery pack the module belongs to
    segment : RCAIDE.Framework.Mission.Segments.Segment
        Mission segment with the following attributes:
            - initial_battery_conditions : Data
                Initial cell_temperature and state_of_charge, if specified for this
                segment (e.g. by a prior segment's final state); otherwise the
                unknowns are seeded from ambient temperature and zero, respectively
            - state : Data
                Segment state
                    - ones_row : function
                        Function to create array of ones with specified length

    Returns
    -------
    None

    Notes
    -----
    This function adds two unknowns (cell temperature, state of charge) and two
    matching residuals per battery module, each unbounded (+/- inf), for the
    mission solver to drive to zero. It increments
    segment.state.number_of_network_unknowns and number_of_network_residuals by 2
    each to match.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common.compute_battery_performance
    """
    ones_row  = segment.state.ones_row
    segment.state.number_of_network_unknowns  += 2 
    segment.state.number_of_network_residuals += 2
    
    # -----------------------------------
    # Temperature Unknown
    # -----------------------------------
    if segment.initial_battery_conditions.cell_temperature is not None:
        cell_temperature  = segment.cell_temperature  
    else: 
        # compute ambient conditions
        atmosphere    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        alt           = -segment.conditions.frames.inertial.position_vector[:,2] 
        if segment.temperature_deviation != None:
            temp_dev = segment.temperature_deviation    
        atmo_data    = atmosphere.compute_values(altitude = alt,temperature_deviation=temp_dev)         
        cell_temperature = atmo_data.temperature[0,0] 
    segment.state.unknowns.network[battery.tag + '_' + module.tag + '_cell_temperature']  = ones_row(1) * cell_temperature
    segment.state.residuals.network[battery.tag + '_' + module.tag + '_cell_temperature'] = ones_row(1)* 0 
    segment.state.unknowns_lower_bounds.network[battery.tag + '_' + module.tag  + '_cell_temperature']     = -np.inf * ones_row(1)
    segment.state.unknowns_upper_bounds.network[battery.tag + '_' + module.tag  + '_cell_temperature']     = np.inf * ones_row(1)    

    # -----------------------------------   
    # State of Charge Unknown 
    # -----------------------------------
    if segment.initial_battery_conditions.state_of_charge is not None: 
        initial_battery_energy = segment.initial_battery_conditions.state_of_charge
        segment.state.unknowns.network[battery.tag + '_' + module.tag + '_cell_state_of_charge'] = ones_row(1) * initial_battery_energy
    else:
        segment.state.unknowns.network[battery.tag + '_' + module.tag + '_cell_state_of_charge'] = ones_row(1) * 0
    segment.state.residuals.network[battery.tag + '_' + module.tag + '_cell_state_of_charge']    = ones_row(1)* 0
    segment.state.unknowns_lower_bounds.network[battery.tag + '_' + module.tag  + '_cell_state_of_charge'] = -np.inf * ones_row(1)
    segment.state.unknowns_upper_bounds.network[battery.tag + '_' + module.tag  + '_cell_state_of_charge'] = np.inf * ones_row(1) 

    return