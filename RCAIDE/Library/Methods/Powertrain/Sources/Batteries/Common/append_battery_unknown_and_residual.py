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
def append_battery_unknown_and_residual(battery_module,segment):
    
    # compute ambient conditions
    atmosphere    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    alt           = -segment.conditions.frames.inertial.position_vector[:,2] 
    if segment.temperature_deviation != None:
        temp_dev = segment.temperature_deviation    
    atmo_data    = atmosphere.compute_values(altitude = alt,temperature_deviation=temp_dev)  
    ones_row    = segment.state.ones_row
    
    segment.state.number_of_network_unknowns  += 2 
    segment.state.number_of_network_residuals += 2 

    if segment.initial_battery_conditions.cell_temperature is not None:
        cell_temperature  = segment.cell_temperature  
    else:
        cell_temperature = atmo_data.temperature[0,0] 
    segment.state.unknowns.network[battery_module.tag+ '_cell_temperature']  = ones_row(1) * cell_temperature
    segment.state.residuals.network[battery_module.tag+ '_cell_temperature'] = ones_row(1)* 0

    if segment.initial_battery_conditions.state_of_charge is not None: 
        initial_battery_energy                                                      = segment.initial_battery_conditions.state_of_charge
        segment.state.unknowns.network[battery_module.tag+ '_cell_state_of_charge'] = ones_row(1) * initial_battery_energy
    else:
        segment.state.unknowns.network[battery_module.tag+ '_cell_state_of_charge'] = ones_row(1) * 0
    segment.state.residuals.network[battery_module.tag+ '_cell_state_of_charge']    = ones_row(1)* 0
 
    segment.state.unknowns_lower_bounds.network[battery_module.tag + '_cell_temperature']     = -np.inf * ones_row(1)
    segment.state.unknowns_upper_bounds.network[battery_module.tag + '_cell_temperature']     = np.inf * ones_row(1)    
    segment.state.unknowns_lower_bounds.network[battery_module.tag + '_cell_state_of_charge'] = -np.inf * ones_row(1)
    segment.state.unknowns_upper_bounds.network[battery_module.tag + '_cell_state_of_charge'] = np.inf * ones_row(1)
  

    return