# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbofan/design_turbofan_offdesign_matching.py
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
#  design_turbofan_offdesign_matching
# ----------------------------------------------------------------------------------------------------------------------
def design_turbofan_offdesign_matching(turbofan):
    """
    Builds the fixed design constants and reference operating point
    `solve_turbofan_offdesign` needs, by reading them directly off an already-
    `design_turbofan`-ed turbofan's own design-point solution -- not by
    re-deriving the design cycle independently. Requires `design_turbofan`
    (and therefore `turbofan.sealevel_static_thrust`, etc.) to already have
    been called; raises if it hasn't.

    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan
        An already-designed turbofan (`design_turbofan(turbofan)` already
        called).

    Returns
    -------
    design_constants : Data
        Fixed engine constants -- see the schema comment atop
        Turbofan_OffDesign_Matching.py for the full field list.
    reference_point : Data
        Design-point flight condition and converged cycle state -- see the
        same schema comment.

    Notes
    -----
    Mattingly's off-design equations (`solve_turbofan_offdesign`) treat the
    turbofan as a *two*-spool machine: a combined "fan" (Ref. [1], Fig. 8.46 --
    the fan exit state is assumed identical to the low-pressure compressor
    exit state) driven by the LP turbine, and an HP compressor driven by the
    HP turbine alone. RCAIDE's own component wiring already matches this
    exactly -- the low-pressure turbine's inputs bundle both the fan's and the
    low-pressure compressor's outputs (`bypass_ratio` term), while the high-
    pressure turbine's inputs carry only the high-pressure compressor's
    (`bypass_ratio=0`) -- see `design_turbofan.py`. So the fan+LPC are combined
    here (pressure/temperature ratios multiplied together) to form
    `reference_point.pi_f`/`tau_f`, and the HP compressor is used alone for
    `reference_point.pi_cH`/`tau_cH`; no assumption is introduced beyond what
    RCAIDE's own network already assumes.

    Component adiabatic efficiencies are not read from
    `polytropic_efficiency` attributes directly (RCAIDE's compressor/fan/
    turbine components are polytropic-efficiency models, matching Ref. [2]'s
    convention, not Ref. [1]'s adiabatic-efficiency convention `solve_
    turbofan_offdesign` uses) -- they are back-derived from the actual
    converged design-point pressure/temperature ratios RCAIDE itself computed
    (`eta = (pi**((gamma-1)/gamma) - 1) / (tau - 1)` for compressors,
    `eta = (1 - tau) / (1 - pi**((gamma-1)/gamma))` for turbines), so the
    reference point is exactly self-consistent with RCAIDE's own design
    solve, not a fresh, potentially-diverging model of it.

    Gas properties (cp, gamma) are read separately for the cold side
    (freestream, from `conditions.freestream`) and hot side (turbine, from
    the high-pressure turbine's own outputs) rather than assumed constant --
    matching Mattingly's own cold/hot two-constant convention (Ref. [1],
    Assumption 1/2 of Sec. 7-4), sampled from RCAIDE's continuous,
    temperature-dependent `Air` model instead of fixed textbook constants.

    References
    ----------
    [1] Mattingly, J. D., "Elements of Gas Turbine Propulsion", 2nd ed., AIAA
        Education Series, 2005, Sec. 8.5, Fig. 8.46.
    [2] Mattingly, J. D., Heiser, W. H., and Pratt, D. T., "Aircraft Engine
        Design", 2nd ed., AIAA Education Series, 2002.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.design_turbofan
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.solve_turbofan_offdesign
    """
    if turbofan.sealevel_static_thrust is None:
        raise RuntimeError(
            "design_turbofan_offdesign_matching: turbofan has not been designed yet -- "
            "call design_turbofan(turbofan) first."
        )

    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(turbofan.design_altitude, turbofan.design_isa_deviation)
    speed_of_sound = float(np.ravel(atmo_data.speed_of_sound)[0])
    design_velocity = speed_of_sound * turbofan.design_mach_number

    fuel_line = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    state = setup_operating_conditions(turbofan, fuel_line, velocity_range=np.array([design_velocity]),
                                        altitude=turbofan.design_altitude, angle_of_attack=0,
                                        temperature_deviation=turbofan.design_isa_deviation)
    state.conditions.energy.propulsors[turbofan.tag].throttle[:, 0] = 1.0
    turbofan.compute_performance(state, fuel_line)

    conditions = state.conditions
    converters = conditions.energy.converters
    fan_c   = converters[turbofan.fan.tag]
    lpc_c   = converters[turbofan.low_pressure_compressor.tag]
    hpc_c   = converters[turbofan.high_pressure_compressor.tag]
    combustor_c = converters[turbofan.combustor.tag]
    hpt_c   = converters[turbofan.high_pressure_turbine.tag]
    lpt_c   = converters[turbofan.low_pressure_turbine.tag]
    turbofan_c = conditions.energy.propulsors[turbofan.tag]

    gamma_c = float(np.ravel(conditions.freestream.isentropic_expansion_factor)[0])
    cpc     = float(np.ravel(conditions.freestream.Cp)[0])
    gamma_t = float(np.ravel(hpt_c.outputs.gamma)[0])
    cpt     = float(np.ravel(hpt_c.outputs.cp)[0])

    # combined fan+LPC (the LP spool -- see Notes) pressure/temperature ratios
    pi_f  = float(np.ravel(fan_c.outputs.stagnation_pressure)[0] / np.ravel(fan_c.inputs.stagnation_pressure)[0]) * \
            float(np.ravel(lpc_c.outputs.stagnation_pressure)[0] / np.ravel(lpc_c.inputs.stagnation_pressure)[0])
    tau_f = float(np.ravel(fan_c.outputs.stagnation_temperature)[0] / np.ravel(fan_c.inputs.stagnation_temperature)[0]) * \
            float(np.ravel(lpc_c.outputs.stagnation_temperature)[0] / np.ravel(lpc_c.inputs.stagnation_temperature)[0])

    # HP compressor alone
    pi_cH  = float(np.ravel(hpc_c.outputs.stagnation_pressure)[0] / np.ravel(hpc_c.inputs.stagnation_pressure)[0])
    tau_cH = float(np.ravel(hpc_c.outputs.stagnation_temperature)[0] / np.ravel(hpc_c.inputs.stagnation_temperature)[0])

    eta_f  = (pi_f ** ((gamma_c - 1) / gamma_c) - 1) / (tau_f - 1)
    eta_cH = (pi_cH ** ((gamma_c - 1) / gamma_c) - 1) / (tau_cH - 1)

    # fan ALONE (not combined with LPC) -- needed for the fan nozzle/bypass-stream
    # equations specifically, since the bypass stream never passes through the LPC; see
    # the design_constants schema comment atop Turbofan_OffDesign_Matching.py for why
    # this split exists.
    pi_f_alone  = float(np.ravel(fan_c.outputs.stagnation_pressure)[0] / np.ravel(fan_c.inputs.stagnation_pressure)[0])
    tau_f_alone = float(np.ravel(fan_c.outputs.stagnation_temperature)[0] / np.ravel(fan_c.inputs.stagnation_temperature)[0])
    eta_f_alone = (pi_f_alone ** ((gamma_c - 1) / gamma_c) - 1) / (tau_f_alone - 1)
    fan_temperature_rise_fraction = (tau_f_alone - 1) / (tau_f - 1)

    # HP turbine -- RCAIDE computes pressure_ratio/temperature_ratio directly
    tau_tH = float(np.ravel(hpt_c.outputs.temperature_ratio)[0])
    pi_tH  = float(np.ravel(hpt_c.outputs.pressure_ratio)[0])

    # LP turbine, likewise, then back out its adiabatic efficiency for the same reason as eta_f/eta_cH above
    tau_tL = float(np.ravel(lpt_c.outputs.temperature_ratio)[0])
    pi_tL  = float(np.ravel(lpt_c.outputs.pressure_ratio)[0])
    eta_tL = (1 - tau_tL) / (1 - pi_tL ** ((gamma_t - 1) / gamma_t))

    pi_b = float(np.ravel(combustor_c.outputs.stagnation_pressure)[0] / np.ravel(combustor_c.inputs.stagnation_pressure)[0])

    design_constants = Data()
    design_constants.gamma_c = gamma_c
    design_constants.gamma_t = gamma_t
    design_constants.cpc     = cpc
    design_constants.cpt     = cpt
    design_constants.fuel_heating_value = turbofan.combustor.fuel_data.specific_energy
    design_constants.pi_dmax = turbofan.inlet_nozzle.pressure_ratio
    design_constants.pi_b    = pi_b
    design_constants.pi_n    = turbofan.core_nozzle.pressure_ratio
    design_constants.pi_fn   = turbofan.fan_nozzle.pressure_ratio
    design_constants.tau_tH  = tau_tH
    design_constants.pi_tH   = pi_tH
    design_constants.eta_f   = eta_f
    design_constants.eta_cH  = eta_cH
    design_constants.eta_b   = turbofan.combustor.efficiency
    design_constants.eta_mH  = turbofan.high_pressure_turbine.mechanical_efficiency
    design_constants.eta_mL  = turbofan.low_pressure_turbine.mechanical_efficiency
    design_constants.eta_tL  = eta_tL
    design_constants.eta_f_alone = eta_f_alone
    design_constants.fan_temperature_rise_fraction = fan_temperature_rise_fraction
    # Read from turbofan.design_shaft_work_specific (set by design_turbofan itself), not
    # from this function's own re-verification hpt_c.inputs.external_shaft.work_done: that
    # comes from a fresh compute_performance() call on a synthetic single-point state, and
    # compute_turbofan_performance.py only computes motor/generator power when
    # state.numerics.time.differentiate is non-empty (a real mission-segment time
    # discretization) -- absent here, so it would silently read back as zero. Zero for an
    # engine with no IDG/motor either way.
    design_constants.shaft_work_specific_design = float(turbofan.design_shaft_work_specific)

    reference_point = Data()
    reference_point.M0  = turbofan.design_mach_number
    reference_point.T0  = float(np.ravel(conditions.freestream.temperature)[0])
    reference_point.P0  = float(np.ravel(conditions.freestream.pressure)[0])
    reference_point.Tt4 = float(np.ravel(combustor_c.outputs.stagnation_temperature)[0])
    reference_point.pi_f, reference_point.tau_f   = pi_f, tau_f
    reference_point.pi_cH, reference_point.tau_cH = pi_cH, tau_cH
    reference_point.tau_tL, reference_point.pi_tL = tau_tL, pi_tL
    reference_point.alpha = turbofan.bypass_ratio
    reference_point.M9  = float(np.ravel(converters[turbofan.core_nozzle.tag].outputs.mach_number)[0])
    reference_point.M19 = float(np.ravel(converters[turbofan.fan_nozzle.tag].outputs.mach_number)[0])
    reference_point.m0  = float(np.ravel(turbofan_c.core_mass_flow_rate)[0]) * (1 + turbofan.bypass_ratio)
    reference_point.F   = float(np.ravel(turbofan_c.thrust)[0])

    return design_constants, reference_point
