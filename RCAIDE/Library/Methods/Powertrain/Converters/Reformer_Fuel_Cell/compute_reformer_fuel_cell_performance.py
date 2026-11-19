# RCAIDE/Library/Methods/Powertrain/Converters/Reformer_Fuel_Cell/compute_reformer_fuel_cell_performance.py
#
# Created:  Aug 2026, RCAIDE Team

# ----------------------------------------------------------------------------------------------------------------------
#  compute_reformer_fuel_cell_performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_reformer_fuel_cell_performance(reformer_fuel_cell,state,network):
    """
    Computes the performance of a reformer/fuel-cell system: a fuel cell that draws its
    hydrogen from an onboard reformer converting a hydrocarbon fuel, modeled as one
    reformer + solid oxide fuel cell + balance-of-plant system.

    Parameters
    ----------
    reformer_fuel_cell : RCAIDE.Library.Components.Powertrain.Converters.Reformer_Fuel_Cell
        The reformer/fuel-cell system for which performance is being computed
    state : RCAIDE.Framework.Mission.Common.State
        Mission segment state
    network : RCAIDE.Framework.Networks.Network
        The network this system belongs to, used to resolve its electrical distributor

    Returns
    -------
    inputs : Data
        System input conditions (power.chemical, the fuel drawn from the fuel line)
    outputs : Data
        System output conditions (power.electrical, delivered to the bus)
    stored_results_flag : bool
        Flag indicating that results have been stored for potential reuse
    stored_converter_tag : str
        Tag identifier of the reformer/fuel-cell system with stored results

    Notes
    -----
    ``Network.evaluate()`` runs electrical converters backward from a known electrical
    target. The system supplies its share of the bus demand at system_efficiency (net
    electrical output over fuel LHV input), so the fuel drawn is
    demand / (system_efficiency x LHV) [1].

    **Major Assumptions**
        * Constant system efficiency over the operating range (the reference reports
          0.45 to 0.48 across cruise power levels and 0.32 to 0.36 on the ground [1])

    References
    ----------
    [1] Mak, A., and Meier, J., "Fuel Cell Auxiliary Power Study, Volume 1: RASER Task Order 5,"
        NASA/CR-2007-214461/VOL1, Honeywell Engines, Systems & Services, 2007, Table 27.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Reformer_Fuel_Cell
    """
    rfc_conditions = state.conditions.energy.converters[reformer_fuel_cell.tag]

    electrical_distributor_tag = None
    for d_tag in reformer_fuel_cell.assigned_distributors[0]:
        if network.distributors[d_tag].domain == 'electrical':
            electrical_distributor_tag = d_tag

    psi              = state.conditions.energy.battery_fuel_cell_power_split_ratio[electrical_distributor_tag]
    demand           = state.conditions.energy.distributors[electrical_distributor_tag].outputs.power.electrical
    electrical_power = demand * reformer_fuel_cell.power_split_ratio * (1. - psi)
    fuel_power       = electrical_power / reformer_fuel_cell.system_efficiency

    rfc_conditions.inputs.power.chemical    = fuel_power
    rfc_conditions.outputs.power.electrical = electrical_power
    rfc_conditions.fuel_mass_flow_rate      = fuel_power / reformer_fuel_cell.working_fluid.lower_heating_value

    stored_results_flag  = True
    stored_converter_tag = reformer_fuel_cell.tag
    return rfc_conditions.inputs, rfc_conditions.outputs, stored_results_flag, stored_converter_tag
