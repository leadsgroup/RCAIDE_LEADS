# RCAIDE/Library/Methods/Powertrain/Converters/Reformer_Fuel_Cell/compute_reformer_fuel_cell_performance.py
#
# Created:  Aug 2026, RCAIDE Team

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from RCAIDE.Library.Methods.Powertrain.Converters.Reformer.compute_reformer_performance import compute_reformer_performance

# ----------------------------------------------------------------------------------------------------------------------
#  compute_reformer_fuel_cell_performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_reformer_fuel_cell_performance(reformer_fuel_cell,state,network):
    """
    Computes the performance of a reformer/fuel-cell composite: a fuel cell that draws
    its hydrogen from an onboard reformer converting a hydrocarbon fuel, rather than
    from a hydrogen fuel tank.

    Parameters
    ----------
    reformer_fuel_cell : RCAIDE.Library.Components.Powertrain.Converters.Reformer_Fuel_Cell
        The composite converter for which performance is being computed
    state : RCAIDE.Framework.Mission.Common.State
        Mission segment state
    network : RCAIDE.Framework.Networks.Network
        The network this composite belongs to, used to resolve its assigned
        distributor(s)

    Returns
    -------
    inputs : Data
        Composite input conditions (power.chemical, the Jet-A drawn from the fuel line)
    outputs : Data
        Composite output conditions (power.electrical, delivered to the bus)
    stored_results_flag : bool
        Flag indicating that results have been stored for potential reuse
    stored_converter_tag : str
        Tag identifier of the reformer/fuel-cell composite with stored results

    Notes
    -----
    ``Network.evaluate()`` always sets ``reverse_mode_computation = True`` before
    calling a converter's ``compute_performance``, so -- like Turboelectric_Generator
    and every other electrical converter in this network -- this always runs
    backward from a known electrical target: the fuel cell's own
    ``compute_performance`` is called directly (it is self-sufficient and resolves
    its electrical bus from its own ``assigned_distributors``, which this function
    sets to this composite's electrical distributor each call), and reports the
    hydrogen mass flow rate it needs. The reformer's fuel feed rate that produces
    exactly that much hydrogen is then solved for directly.

    That solve is a single linear inversion, not an iterative search: steam and air
    are metered as fixed volumetric ratios of the fuel feed rate
    (``design_steam_to_fuel_volumetric_ratio``, ``design_air_to_fuel_volumetric_ratio``),
    so hydrogen production scales linearly with fuel feed rate. A single probe
    evaluation of the reformer's forward model at a unit fuel feed rate gives that
    proportionality constant directly, which is then inverted and used to re-run the
    forward model at the actual solved feed rate so the reformer's own reported
    conditions (efficiencies, space velocities, etc.) are self-consistent.

    **Major Assumptions**
        * The reformer and fuel cell are properly connected and compatible
        * Steam and air feed rates are held at the reformer's fixed design ratios
          to the fuel feed rate (see Reformer.design_steam_to_fuel_volumetric_ratio,
          Reformer.design_air_to_fuel_volumetric_ratio)

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Reformer.compute_reformer_performance
    RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Larminie_Model.compute_fuel_cell_performance
    """

    reformer  = reformer_fuel_cell.reformer
    fuel_cell = reformer_fuel_cell.fuel_cell

    rfc_conditions       = state.conditions.energy.converters[reformer_fuel_cell.tag]
    reformer_conditions  = state.conditions.energy.converters[reformer.tag]
    fuel_cell_conditions = state.conditions.energy.converters[fuel_cell.tag]

    # Resolve this composite's electrical bus from its own assigned_distributors,
    # and propagate it down to the fuel cell subcomponent so it can resolve its
    # own demand (Generic_Fuel_Cell_Stack / Proton_Exchange_Membrane_Fuel_Cell
    # do this lookup themselves, unlike Generator/Turboshaft).
    electrical_distributor_tag = None
    for d_tag in reformer_fuel_cell.assigned_distributors[0]:
        if network.distributors[d_tag].domain == 'electrical':
            electrical_distributor_tag = d_tag
    fuel_cell.assigned_distributors = [[electrical_distributor_tag]]
    fuel_cell.power_split_ratio     = reformer_fuel_cell.power_split_ratio

    # run the fuel cell to determine the hydrogen mass flow rate it requires
    _,_,_,_ = fuel_cell.compute_performance(state,network)
    target_H2_mass_flow_rate = fuel_cell_conditions.H2_mass_flow_rate

    # Probe the reformer's forward model at a unit fuel feed rate to get the
    # (design-ratio-fixed) linear proportionality between fuel feed rate and
    # hydrogen production rate -- see Notes.
    probe_Q_F                                  = 1.0 * state.ones_row(1)
    reformer_conditions.fuel_volume_flow_rate  = probe_Q_F
    reformer_conditions.steam_volume_flow_rate = reformer.design_steam_to_fuel_volumetric_ratio * probe_Q_F
    reformer_conditions.air_volume_flow_rate   = reformer.design_air_to_fuel_volumetric_ratio   * probe_Q_F
    compute_reformer_performance(reformer,reformer_conditions,state)
    H2_per_unit_Q_F = reformer_conditions.hydrogen_mass_flow_rate

    # Invert to find the fuel feed rate that produces the required hydrogen
    # flow, then re-run the forward model at that feed rate so the reformer's
    # own conditions are self-consistent with the solved operating point.
    Q_F_needed = np.where(H2_per_unit_Q_F > 0., target_H2_mass_flow_rate / np.maximum(H2_per_unit_Q_F,1e-30), 0.)
    reformer_conditions.fuel_volume_flow_rate  = Q_F_needed
    reformer_conditions.steam_volume_flow_rate = reformer.design_steam_to_fuel_volumetric_ratio * Q_F_needed
    reformer_conditions.air_volume_flow_rate   = reformer.design_air_to_fuel_volumetric_ratio   * Q_F_needed
    compute_reformer_performance(reformer,reformer_conditions,state)

    # Fuel mass flow rate drawn from the fuel line (working_fluid.density is kg/m**3, SI)
    fuel_mass_flow_rate = Q_F_needed * reformer.working_fluid.density

    rfc_conditions.inputs.power.chemical    = reformer_conditions.inputs.power.chemical
    rfc_conditions.outputs.power.electrical = fuel_cell_conditions.outputs.power.electrical
    rfc_conditions.fuel_mass_flow_rate       = fuel_mass_flow_rate

    stored_results_flag  = True
    stored_converter_tag = reformer_fuel_cell.tag

    return rfc_conditions.inputs, rfc_conditions.outputs, stored_results_flag, stored_converter_tag
