# RCAIDE/Library/Methods/Powertrain/Converters/Heater/append_heater_conditions.py
#
# Created:  Aug 2026, M. Clarke

from RCAIDE.Framework.Mission.Common     import   Conditions

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

    ones_row                                                          = segment.state.ones_row
    segment.state.conditions.energy.converters[heater.tag]                          = Conditions()
    segment.state.conditions.energy.converters[heater.tag].fuel_mass_flow_rate      = 0. * ones_row(1)
    segment.state.conditions.energy.converters[heater.tag].inputs                   = Conditions()
    segment.state.conditions.energy.converters[heater.tag].outputs                  = Conditions()
    segment.state.conditions.energy.converters[heater.tag].inputs.power             = Conditions()
    segment.state.conditions.energy.converters[heater.tag].inputs.power.propulsive  = 0 * ones_row(1)
    segment.state.conditions.energy.converters[heater.tag].inputs.power.mechanical  = 0 * ones_row(1)
    segment.state.conditions.energy.converters[heater.tag].inputs.power.electrical  = 0 * ones_row(1)
    segment.state.conditions.energy.converters[heater.tag].inputs.power.chemical    = 0 * ones_row(1)
    segment.state.conditions.energy.converters[heater.tag].inputs.power.pneumatic   = 0 * ones_row(1)
    segment.state.conditions.energy.converters[heater.tag].inputs.power.hydraulic   = 0 * ones_row(1)
    segment.state.conditions.energy.converters[heater.tag].inputs.power.thermal     = 0 * ones_row(1)
    segment.state.conditions.energy.converters[heater.tag].outputs.power            = Conditions()
    segment.state.conditions.energy.converters[heater.tag].outputs.power.propulsive = 0 * ones_row(1)
    segment.state.conditions.energy.converters[heater.tag].outputs.power.mechanical = 0 * ones_row(1)
    segment.state.conditions.energy.converters[heater.tag].outputs.power.electrical = 0 * ones_row(1)
    segment.state.conditions.energy.converters[heater.tag].outputs.power.chemical   = 0 * ones_row(1)
    segment.state.conditions.energy.converters[heater.tag].outputs.power.pneumatic  = 0 * ones_row(1)
    segment.state.conditions.energy.converters[heater.tag].outputs.power.hydraulic  = 0 * ones_row(1)
    segment.state.conditions.energy.converters[heater.tag].outputs.power.thermal    = 0 * ones_row(1)
    return
