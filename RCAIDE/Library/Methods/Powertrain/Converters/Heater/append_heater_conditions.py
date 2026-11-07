# RCAIDE/Library/Methods/Powertrain/Converters/Heater/append_heater_conditions.py
#
# Created:  Aug 2026, M. Clarke

from RCAIDE.Framework.Mission.Common     import   Conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Common.append_converter_power_conditions import append_converter_power_conditions

# ----------------------------------------------------------------------------------------------------------------------
#  append_heater_conditions
# ----------------------------------------------------------------------------------------------------------------------
def append_heater_conditions(heater,segment):

    """
    Initializes empty condition containers for heater analysis in the propulsion system.

    Parameters
    ----------
    heater : Heater
        The heater component being analyzed
    segment : Segment
        The mission segment being analyzed

    Returns
    -------
    None

    Notes
    -----
    Mirrors ``append_pump_conditions`` -- creates the same full set of power-domain
    fields (propulsive/mechanical/electrical/chemical/pneumatic/hydraulic/thermal)
    so the generic per-distributor bookkeeping in ``RCAIDE.Framework.Networks.
    Network.evaluate`` (which indexes ``inputs.power[distributor.domain]`` /
    ``outputs.power[distributor.domain]`` generically) and the electrical bus's own
    ``compute_distribution_losses`` both work without special-casing this converter,
    even though a heater only ever populates the ``electrical`` (input) and
    ``thermal`` (output) fields.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Heater.compute_heater_performance
    """

    ones_row                         = segment.state.ones_row
    heater_conditions                = append_converter_power_conditions(heater, segment)
    heater_conditions.fuel_mass_flow_rate = 0. * ones_row(1)
    return
