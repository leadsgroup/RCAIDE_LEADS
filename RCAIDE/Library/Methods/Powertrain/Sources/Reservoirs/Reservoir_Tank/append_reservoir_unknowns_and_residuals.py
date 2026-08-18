# RCAIDE/Library/Methods/Powertrain/Sources/Reservoirs/Reservoir_Tank/append_reservoir_unknowns_and_residuals.py
#
# Created: Jul 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  append_reservoir_unknowns_and_residuals
# ----------------------------------------------------------------------------------------------------------------------
def append_reservoir_unknowns_and_residuals(reservoir,coolant_line,segment):
    """
    Registers the coolant temperature unknown and residual for a reservoir.

    The reservoir's coolant temperature is solved implicitly alongside every other
    network unknown (cell temperature, state of charge, ...) using the same
    control-point collocation scheme, rather than being explicitly time-marched.
    """
    ones_row = segment.state.ones_row
    key      = coolant_line.tag + '_' + reservoir.tag + '_coolant_temperature'

    atmosphere    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    alt           = -segment.conditions.frames.inertial.position_vector[:,2]
    if segment.temperature_deviation != None:
        temp_dev = segment.temperature_deviation
    atmo_data     = atmosphere.compute_values(altitude = alt,temperature_deviation=temp_dev)
    coolant_temperature = atmo_data.temperature[0,0]

    segment.state.number_of_network_unknowns  += 1
    segment.state.number_of_network_residuals += 1

    segment.state.unknowns.network[key]              = ones_row(1) * coolant_temperature
    segment.state.residuals.network[key]             = ones_row(1) * 0
    segment.state.unknowns_lower_bounds.network[key] = -np.inf * ones_row(1)
    segment.state.unknowns_upper_bounds.network[key] =  np.inf * ones_row(1)

    return
