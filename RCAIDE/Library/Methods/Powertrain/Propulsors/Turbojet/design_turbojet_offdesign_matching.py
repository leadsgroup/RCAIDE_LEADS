# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbojet/design_turbojet_offdesign_matching.py
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
#  design_turbojet_offdesign_matching
# ----------------------------------------------------------------------------------------------------------------------
def design_turbojet_offdesign_matching(turbojet):
    """
    Builds the fixed design constants and reference operating point
    `solve_turbojet_offdesign` needs, by reading them directly off an
    already-`design_turbojet`-ed turbojet's own design-point solution.

    Parameters
    ----------
    turbojet : RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet
        An already-designed turbojet (`design_turbojet(turbojet)` already
        called).

    Returns
    -------
    design_constants : Data
    reference_point : Data
        See the schema comment atop `Turbojet_OffDesign_Matching.py` for the
        full field list.

    Notes
    -----
    RCAIDE's `Turbojet` has the same two-spool layout as its `Turbofan` (LP
    compressor+turbine, HP compressor+turbine) but no bypass stream at all --
    the LP compressor alone plays the role a turbofan's fan+LPC combination
    plays, with no separate fan-alone/bypass-split bookkeeping needed (there
    is nothing to split).

    `design_constants.shaft_work_specific_design` is read from `turbojet.
    design_shaft_work_specific` (set by `design_turbojet`), same convention
    as `design_turbofan_offdesign_matching`. Zero for an engine with no
    `integrated_drive_generator`/`integrated_drive_motor`.

    Component adiabatic efficiencies are back-derived from RCAIDE's own
    converged pressure/temperature ratios, gas properties are read
    separately for the cold/hot sides, and afterburner state is not modeled
    at all -- all for the same reasons as `design_turbofan_offdesign_
    matching`, whose docstring covers them in full.

    `reference_point.M9` is read directly from `turbojet`'s own converged
    `Supersonic_Nozzle` output (`RCAIDE.Library.Methods.Powertrain.Converters.
    Supersonic_Nozzle.compute_supersonic_nozzle_performance`, a fully-
    expanded nozzle model, P9=P0 always) -- unlike the turbofan case, this
    *is* consistent with what `solve_turbojet_offdesign` itself computes:
    both use the same fully-expanded Mach relation (see `Turbojet_OffDesign_
    Matching.py`'s own module docstring for the two independent primary
    sources -- Cantwell and Mattingly -- confirming that assumption is the
    textbook-standard one for a turbojet's convergent-divergent nozzle, not
    a convergent-only choked one). `reference_point.F` is likewise read
    directly from `turbojet.compute_performance`'s own converged thrust --
    validated to reproduce it to within a bounded ~4-6% across the full
    tested pressure-ratio range (see `Turbojet_OffDesign_Matching.py`'s own
    module docstring for the two RCAIDE `Supersonic_Nozzle` findings that
    got it there).

    References
    ----------
    [1] Mattingly, J. D., "Elements of Gas Turbine Propulsion", 2nd ed., AIAA
        Education Series, 2005, Sec. 8.3.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.design_turbojet
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.Turbojet_OffDesign_Matching.solve_turbojet_offdesign
    """
    if turbojet.sealevel_static_thrust is None:
        raise RuntimeError(
            "design_turbojet_offdesign_matching: turbojet has not been designed yet -- "
            "call design_turbojet(turbojet) first."
        )

    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(turbojet.design_altitude, turbojet.design_isa_deviation)
    speed_of_sound = float(np.ravel(atmo_data.speed_of_sound)[0])
    design_velocity = speed_of_sound * turbojet.design_mach_number

    fuel_line = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    state = setup_operating_conditions(turbojet, fuel_line, velocity_range=np.array([design_velocity]),
                                        altitude=turbojet.design_altitude, angle_of_attack=0,
                                        temperature_deviation=turbojet.design_isa_deviation)
    state.conditions.energy.propulsors[turbojet.tag].throttle[:, 0] = 1.0
    turbojet.compute_performance(state)

    conditions = state.conditions
    converters = conditions.energy.converters
    lpc_c   = converters[turbojet.low_pressure_compressor.tag]
    hpc_c   = converters[turbojet.high_pressure_compressor.tag]
    combustor_c = converters[turbojet.combustor.tag]
    hpt_c   = converters[turbojet.high_pressure_turbine.tag]
    lpt_c   = converters[turbojet.low_pressure_turbine.tag]
    core_nozzle_c = converters[turbojet.core_nozzle.tag]
    turbojet_c = conditions.energy.propulsors[turbojet.tag]

    gamma_c = float(np.ravel(conditions.freestream.isentropic_expansion_factor)[0])
    cpc     = float(np.ravel(conditions.freestream.Cp)[0])
    gamma_t = float(np.ravel(hpt_c.outputs.gamma)[0])
    cpt     = float(np.ravel(hpt_c.outputs.cp)[0])

    # LP compressor alone -- see Notes
    pi_c  = float(np.ravel(lpc_c.outputs.stagnation_pressure)[0] / np.ravel(lpc_c.inputs.stagnation_pressure)[0])
    tau_c = float(np.ravel(lpc_c.outputs.stagnation_temperature)[0] / np.ravel(lpc_c.inputs.stagnation_temperature)[0])

    # HP compressor alone
    pi_cH  = float(np.ravel(hpc_c.outputs.stagnation_pressure)[0] / np.ravel(hpc_c.inputs.stagnation_pressure)[0])
    tau_cH = float(np.ravel(hpc_c.outputs.stagnation_temperature)[0] / np.ravel(hpc_c.inputs.stagnation_temperature)[0])

    eta_c  = (pi_c ** ((gamma_c - 1) / gamma_c) - 1) / (tau_c - 1)
    eta_cH = (pi_cH ** ((gamma_c - 1) / gamma_c) - 1) / (tau_cH - 1)

    # HP turbine -- RCAIDE computes pressure_ratio/temperature_ratio directly
    tau_tH = float(np.ravel(hpt_c.outputs.temperature_ratio)[0])
    pi_tH  = float(np.ravel(hpt_c.outputs.pressure_ratio)[0])

    # LP turbine, likewise, then back out its adiabatic efficiency for the same reason as eta_c/eta_cH above
    tau_tL = float(np.ravel(lpt_c.outputs.temperature_ratio)[0])
    pi_tL  = float(np.ravel(lpt_c.outputs.pressure_ratio)[0])
    eta_tL = (1 - tau_tL) / (1 - pi_tL ** ((gamma_t - 1) / gamma_t))

    pi_b = float(np.ravel(combustor_c.outputs.stagnation_pressure)[0] / np.ravel(combustor_c.inputs.stagnation_pressure)[0])

    design_constants = Data()
    design_constants.gamma_c = gamma_c
    design_constants.gamma_t = gamma_t
    design_constants.cpc     = cpc
    design_constants.cpt     = cpt
    design_constants.fuel_heating_value = turbojet.combustor.fuel_data.specific_energy
    design_constants.pi_dmax = turbojet.inlet_nozzle.pressure_ratio
    design_constants.pi_b    = pi_b
    design_constants.pi_n    = turbojet.core_nozzle.pressure_ratio
    design_constants.tau_tH  = tau_tH
    design_constants.pi_tH   = pi_tH
    design_constants.eta_c   = eta_c
    design_constants.eta_cH  = eta_cH
    design_constants.eta_b   = turbojet.combustor.efficiency
    design_constants.eta_mH  = turbojet.high_pressure_turbine.mechanical_efficiency
    design_constants.eta_mL  = turbojet.low_pressure_turbine.mechanical_efficiency
    design_constants.eta_tL  = eta_tL
    design_constants.shaft_work_specific_design = float(turbojet.design_shaft_work_specific)

    reference_point = Data()
    reference_point.M0  = turbojet.design_mach_number
    reference_point.T0  = float(np.ravel(conditions.freestream.temperature)[0])
    reference_point.P0  = float(np.ravel(conditions.freestream.pressure)[0])
    reference_point.Tt4 = float(np.ravel(combustor_c.outputs.stagnation_temperature)[0])
    reference_point.pi_c, reference_point.tau_c   = pi_c, tau_c
    reference_point.pi_cH, reference_point.tau_cH = pi_cH, tau_cH
    reference_point.tau_tL, reference_point.pi_tL = tau_tL, pi_tL
    reference_point.M9  = float(np.ravel(core_nozzle_c.outputs.mach_number)[0])
    reference_point.m0  = float(np.ravel(turbojet_c.core_mass_flow_rate)[0])
    reference_point.F   = float(np.ravel(turbojet_c.thrust)[0])

    return design_constants, reference_point
