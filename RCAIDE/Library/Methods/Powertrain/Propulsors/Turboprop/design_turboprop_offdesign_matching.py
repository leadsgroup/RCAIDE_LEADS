# RCAIDE/Library/Methods/Powertrain/Propulsors/Turboprop/design_turboprop_offdesign_matching.py
#
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                                                      import Data
from RCAIDE.Library.Methods.Powertrain                                          import setup_operating_conditions

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  design_turboprop_offdesign_matching
# ----------------------------------------------------------------------------------------------------------------------
def design_turboprop_offdesign_matching(turboprop):
    """
    Builds the fixed design constants and reference operating point
    `solve_turboprop_offdesign` needs, by reading them directly off an
    already-`design_turboprop`-ed turboprop's own design-point solution.

    Parameters
    ----------
    turboprop : RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop
        An already-designed turboprop (`design_turboprop(turboprop)` already
        called).

    Returns
    -------
    design_constants : Data
    reference_point : Data
        See the schema comment atop `Turboprop_OffDesign_Matching.py` for
        the full field list.

    Notes
    -----
    RCAIDE's `Turboprop` is genuinely single-spool: one `compressor`, driven
    by one `high_pressure_turbine` (the gas-generator turbine); the
    `low_pressure_turbine` is a *free* turbine -- mechanically decoupled
    from the compressor entirely (`lpt_conditions.inputs.compressor.
    work_done=0.0` in `design_turboprop.py`), driving only the propeller
    through a gearbox. This is a materially different architecture from both
    `Turbofan` and `Turbojet` (neither has a shaft with no compressor on it
    at all) -- see `Turboprop_OffDesign_Matching.py`'s own module docstring
    for how the matching equations handle that.

    `design_constants.eta_prop`/`eta_gearbox` are read directly from
    `turboprop.propeller.design_efficiency`/`turboprop.gearbox.efficiency`
    and held constant, matching `compute_thrust.py`'s own assumption
    ("Propeller efficiency is constant") -- there is no full propeller
    aerodynamic model coupled into this solver, same as RCAIDE's existing
    analytical dispatch.

    `design_constants.shaft_work_specific_design` is read from `turboprop.
    design_shaft_work_specific` (set by `design_turboprop` from the
    gas-generator accessory tap, `design_power_offtake` -- NOT the free-
    turbine's own `design_power`, which drives the propeller on a separate
    shaft and has no matching-solver power-balance equation at all, see
    `Turboprop_OffDesign_Matching.py`'s own module docstring). Zero for an
    engine with no `integrated_drive_generator`/`integrated_drive_motor`.

    Component adiabatic efficiencies are back-derived from RCAIDE's own
    converged pressure/temperature ratios, gas properties are read
    separately for the cold/hot sides -- same reasons as `design_turbofan_
    offdesign_matching`, whose docstring covers them in full.

    `reference_point.M9`/`F` are read directly from `turboprop`'s own
    converged state (`Expansion_Nozzle` output, `turboprop_conditions.
    thrust`) -- unlike the turbojet case, no self-consistency workaround is
    needed here: the core nozzle is the same convergent type already
    validated for `Turbofan`, and the thrust formula is reused from RCAIDE's
    own `compute_thrust.py` directly rather than re-derived, so both are
    already consistent with the design point by construction.

    References
    ----------
    [1] Mattingly, J. D., Heiser, W. H., and Pratt, D. T., "Aircraft Engine
        Design", 2nd ed., AIAA Education Series, 2002, Appendix K.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.design_turboprop
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.Turboprop_OffDesign_Matching.solve_turboprop_offdesign
    """
    if turboprop.sealevel_static_thrust is None:
        raise RuntimeError(
            "design_turboprop_offdesign_matching: turboprop has not been designed yet -- "
            "call design_turboprop(turboprop) first."
        )

    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(turboprop.design_altitude, turboprop.design_isa_deviation)
    speed_of_sound = float(np.ravel(atmo_data.speed_of_sound)[0])
    design_velocity = speed_of_sound * turboprop.design_mach_number

    fuel_line = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    state = setup_operating_conditions(turboprop, fuel_line, velocity_range=np.array([design_velocity]),
                                        altitude=turboprop.design_altitude, angle_of_attack=0,
                                        temperature_deviation=turboprop.design_isa_deviation)
    state.conditions.energy.propulsors[turboprop.tag].throttle[:, 0] = 1.0
    turboprop.compute_performance(state)

    conditions = state.conditions
    converters = conditions.energy.converters
    compressor_c = converters[turboprop.compressor.tag]
    combustor_c = converters[turboprop.combustor.tag]
    hpt_c   = converters[turboprop.high_pressure_turbine.tag]
    lpt_c   = converters[turboprop.low_pressure_turbine.tag]
    core_nozzle_c = converters[turboprop.core_nozzle.tag]
    turboprop_c = conditions.energy.propulsors[turboprop.tag]

    gamma_c = float(np.ravel(conditions.freestream.isentropic_expansion_factor)[0])
    cpc     = float(np.ravel(conditions.freestream.Cp)[0])
    gamma_t = float(np.ravel(hpt_c.outputs.gamma)[0])
    cpt     = float(np.ravel(hpt_c.outputs.cp)[0])

    # compressor
    pi_c  = float(np.ravel(compressor_c.outputs.stagnation_pressure)[0] / np.ravel(compressor_c.inputs.stagnation_pressure)[0])
    tau_c = float(np.ravel(compressor_c.outputs.stagnation_temperature)[0] / np.ravel(compressor_c.inputs.stagnation_temperature)[0])
    eta_c = (pi_c ** ((gamma_c - 1) / gamma_c) - 1) / (tau_c - 1)

    # HP (gas-generator) turbine -- RCAIDE computes pressure_ratio/temperature_ratio directly
    tau_tH = float(np.ravel(hpt_c.outputs.temperature_ratio)[0])
    pi_tH  = float(np.ravel(hpt_c.outputs.pressure_ratio)[0])

    # LP (free/power) turbine, likewise, then back out its adiabatic efficiency
    tau_tL = float(np.ravel(lpt_c.outputs.temperature_ratio)[0])
    pi_tL  = float(np.ravel(lpt_c.outputs.pressure_ratio)[0])
    eta_tL = (1 - tau_tL) / (1 - pi_tL ** ((gamma_t - 1) / gamma_t))

    pi_b = float(np.ravel(combustor_c.outputs.stagnation_pressure)[0] / np.ravel(combustor_c.inputs.stagnation_pressure)[0])

    design_constants = Data()
    design_constants.gamma_c = gamma_c
    design_constants.gamma_t = gamma_t
    design_constants.cpc     = cpc
    design_constants.cpt     = cpt
    design_constants.fuel_heating_value = turboprop.combustor.fuel_data.specific_energy
    design_constants.pi_dmax = turboprop.inlet_nozzle.pressure_ratio
    design_constants.pi_b    = pi_b
    design_constants.pi_n    = turboprop.core_nozzle.pressure_ratio
    design_constants.tau_tH  = tau_tH
    design_constants.pi_tH   = pi_tH
    design_constants.eta_c   = eta_c
    design_constants.eta_b   = turboprop.combustor.efficiency
    design_constants.eta_mH  = turboprop.high_pressure_turbine.mechanical_efficiency
    design_constants.eta_mL  = turboprop.low_pressure_turbine.mechanical_efficiency
    design_constants.eta_tL  = eta_tL
    design_constants.eta_prop    = turboprop.propeller.design_efficiency
    design_constants.eta_gearbox = turboprop.gearbox.efficiency
    design_constants.shaft_work_specific_design = float(turboprop.design_shaft_work_specific)

    reference_point = Data()
    reference_point.M0  = float(np.ravel(turboprop.design_mach_number)[0])
    reference_point.T0  = float(np.ravel(conditions.freestream.temperature)[0])
    reference_point.P0  = float(np.ravel(conditions.freestream.pressure)[0])
    reference_point.Tt4 = float(np.ravel(combustor_c.outputs.stagnation_temperature)[0])
    reference_point.pi_c, reference_point.tau_c   = pi_c, tau_c
    reference_point.tau_tL, reference_point.pi_tL = tau_tL, pi_tL
    reference_point.M9  = float(np.ravel(core_nozzle_c.outputs.mach_number)[0])
    reference_point.m0  = float(np.ravel(turboprop_c.core_mass_flow_rate)[0])
    reference_point.F   = float(np.ravel(turboprop_c.thrust)[0])

    return design_constants, reference_point
