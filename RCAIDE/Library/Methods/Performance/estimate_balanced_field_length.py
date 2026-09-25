# estimate_balanced_field_length.py
#
# Created: Aug 2026, M. Clarke
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE
from RCAIDE.Framework.Core import Data, Units
from RCAIDE.Library.Methods.Aerodynamics.Common.Lift import compute_max_lift_coeff
from RCAIDE.Library.Mission.Common.Pre_Process import geometry_preprocess_routine
from .estimate_take_off_field_length import estimate_take_off_field_length

# package imports
import numpy as np
from scipy.optimize import brentq

# ----------------------------------------------------------------------
#  Compute balanced field length
# ----------------------------------------------------------------------
def estimate_balanced_field_length(analyses=None, altitude=0., delta_isa=0., obstacle_height=35.*Units.ft,
                                    rolling_friction_coefficient=0.025, braking_friction_coefficient=0.4,
                                    recognition_time=2.0, lift_off_speed_ratio=1.1, time_step=0.1):
    """
    Computes the true balanced field length (BFL): the runway length required to take off and
    clear a specified obstacle when one engine fails at the decision speed V1, where V1 is chosen
    such that the accelerate-stop distance equals the accelerate-go (continued takeoff) distance.

    Parameters
    ----------
    analyses : Analyses
        Container with atmosphere and aerodynamic analyses
    altitude : float, optional
        Airport altitude [m], default 0
    delta_isa : float, optional
        Temperature offset from ISA conditions [K], default 0
    obstacle_height : float, optional
        Screen height that must be cleared on a continued (one-engine-inoperative) takeoff [m],
        default 35 ft per FAR 25.113
    rolling_friction_coefficient : float, optional
        Ground roll rolling-resistance coefficient (brakes off), default 0.025
    braking_friction_coefficient : float, optional
        Effective braking-resistance coefficient (max braking, brakes on), default 0.4
    recognition_time : float, optional
        Pilot recognition/transition time [s] credited at V1 before deceleration begins during an
        accelerate-stop, default 2.0 s per FAR 25.109(f)
    lift_off_speed_ratio : float, optional
        Ratio of liftoff speed to stall speed (V_LOF/Vs), default 1.1
    time_step : float, optional
        Time-marching integration step [s] used for the ground-roll simulation, default 0.1 s

    Returns
    -------
    balanced_field_length : float
        Balanced field length [m]
    decision_speed : float
        Balanced V1 (decision speed) [m/s]

    Notes
    -----
    Distinct from ``estimate_take_off_field_length``, which is a purely empirical (all-engines
    correlation + static second-segment climb-gradient check) estimate that never actually
    accounts for engine-failure ground dynamics. This function instead time-marches the point-mass
    ground-roll equations of motion through three flight phases and root-finds V1:

    1. All-engines-operating (AEO) ground roll from a standstill to V1.
    2. **Accelerate-go**: one engine fails at V1; ground roll continues on one-engine-inoperative
       (OEI) thrust to liftoff speed, followed by a straight-line OEI climb-out (using the
       second-segment climb gradient from ``estimate_take_off_field_length``) to clear the
       obstacle.
    3. **Accelerate-stop**: one engine fails at V1; speed is held for ``recognition_time`` before
       maximum braking is applied, decelerating the aircraft to a stop.

    V1 is found via bisection such that the accelerate-go distance equals the accelerate-stop
    distance (both phases start from the identical AEO ground roll, so this is a monotonic,
    well-posed root-finding problem: accelerate-go distance decreases with V1, accelerate-stop
    distance increases with V1).

    **Major Assumptions**
        * 1-DOF point-mass ground roll; lift and drag scale with V^2 at a constant ground-attitude
          lift coefficient, set so that lift equals weight exactly at liftoff speed
          (:math:`C_{L,ground} = W / (0.5 \\rho V_{LOF}^2 S)`)
        * Takeoff-configuration lift-to-drag ratio during the ground roll follows the same
          empirical correlation used for the second-segment climb check (Obert's chart, as in
          ``estimate_take_off_field_length``)
        * One-engine-inoperative ground thrust is the remaining engines' share of all-engines
          thrust at the same speed, i.e. :math:`T_{OEI} = T_{AEO} \\cdot (N-1)/N`
        * No reverse thrust credit during the accelerate-stop braking phase (brakes/spoilers only)
        * The OEI airborne segment from liftoff to obstacle height is approximated as a straight
          climb at the constant second-segment climb gradient (small-angle approximation)
        * Dry, level, paved runway; no wind

    **Theory**
    Ground-roll equation of motion:

    .. math::
        m \\frac{dV}{dt} = T(V) - D(V) - \\mu (W - L(V))

    with :math:`\\mu = \\mu_{roll}` while accelerating and :math:`\\mu = \\mu_{brake}` while braking.
    Distance is accumulated by time-marching this ODE (explicit midpoint integration) from V=0.

    References
    ----------
    [1] Federal Aviation Regulations, Part 25.109 (Accelerate-stop distance), 25.111 (Takeoff
        path), 25.113 (Takeoff distance and takeoff run).
    [2] Raymer, D.P., "Aircraft Design: A Conceptual Approach", AIAA, Ch. 17 (balanced field
        length / accelerate-stop, accelerate-go construction).
    [3] Torenbeek, E., "Synthesis of Subsonic Airplane Design", Ch. 6 (takeoff ground-roll point
        mass equations of motion).

    See Also
    --------
    RCAIDE.Library.Methods.Performance.estimate_take_off_field_length
    """
    if analyses is None:
        raise AttributeError('RCAIDE analyses must be defined')

    vehicle        = analyses.vehicle
    weight         = vehicle.mass_properties.takeoff
    reference_area = vehicle.reference_area
    sea_level_gravity = RCAIDE.Library.Attributes.Planets.Earth().sea_level_gravity

    # geometry must be preprocessed before CLmax/L-D correlations are evaluated below (mirrors
    # the first step of estimate_take_off_field_length, which this function also calls)
    geometry_preprocess_routine(analyses)

    # ==============================================
    # Atmosphere at the airport
    # ==============================================
    atmo       = analyses.atmosphere
    atmo_values = atmo.compute_values(altitude, delta_isa)
    rho         = float(np.ravel(atmo_values.density)[0])
    a           = float(np.ravel(atmo_values.speed_of_sound)[0])

    # ==============================================
    # Stall / liftoff speed
    # ==============================================
    state                                         = RCAIDE.Framework.Mission.Common.State()
    state.conditions                              = RCAIDE.Framework.Mission.Common.Results()
    state.conditions.freestream.density           = atmo_values.density
    state.conditions.freestream.velocity           = np.atleast_1d(90. * Units.knots)
    state.conditions.freestream.dynamic_viscosity  = atmo_values.dynamic_viscosity
    settings = analyses.aerodynamics.settings

    maximum_lift_coefficient, _ = compute_max_lift_coeff(state, settings, vehicle)
    maximum_lift_coefficient    = float(np.ravel(maximum_lift_coefficient)[0])

    stall_speed   = (2. * weight * sea_level_gravity / (rho * reference_area * maximum_lift_coefficient)) ** 0.5
    v_lift_off    = lift_off_speed_ratio * stall_speed

    # Representative ground-attitude lift coefficient: lift equals weight exactly at liftoff
    cl_ground = weight * sea_level_gravity / (0.5 * rho * v_lift_off ** 2 * reference_area)

    # Takeoff-configuration lift-to-drag ratio correlation (Obert), same source used for the
    # second-segment climb gradient in estimate_take_off_field_length
    aspect_ratio = None
    for wing in vehicle.wings:
        if not (isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing) or
                isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body)):
            continue
        aspect_ratio = wing.aspect_ratio
    lift_drag_ratio_ground = -6.464 * cl_ground + 7.264 * aspect_ratio ** 0.5
    drag_area_ground       = cl_ground * reference_area / lift_drag_ratio_ground  # D = drag_area_ground * q

    # ==============================================
    # Number of engines and AEO static thrust
    # ==============================================
    engine_number = 0.
    for network in vehicle.networks:
        engine_number += len(network.propulsors)
    if engine_number == 0:
        raise ValueError("No engine found in the vehicle")

    # Sample all-engines-operating thrust versus speed (full throttle) and build an interpolant
    speed_samples  = np.linspace(0.5, v_lift_off * 1.05, 12)
    thrust_samples = np.array([_static_thrust(vehicle, altitude, delta_isa, v, a) for v in speed_samples])

    def T_AEO(v):
        return np.interp(v, speed_samples, thrust_samples)

    def T_OEI(v):
        return T_AEO(v) * (engine_number - 1.) / engine_number

    # ==============================================
    # OEI second-segment climb gradient (airborne portion of accelerate-go)
    # ==============================================
    _, second_seg_clb_grad = estimate_take_off_field_length(
        analyses=analyses, altitude=altitude, delta_isa=delta_isa, compute_2nd_seg_climb=True
    )
    if second_seg_clb_grad <= 0:
        raise ValueError("Non-positive OEI second-segment climb gradient; cannot clear obstacle.")
    climb_out_distance = obstacle_height / second_seg_clb_grad

    # ==============================================
    # Ground-roll integration
    # ==============================================
    def acceleration(v, thrust_func, mu):
        lift  = cl_ground * 0.5 * rho * v ** 2 * reference_area
        drag  = drag_area_ground * 0.5 * rho * v ** 2
        force = thrust_func(v) - drag - mu * max(weight * sea_level_gravity - lift, 0.)
        return force / weight

    def integrate(v_start, v_end, thrust_func, mu, dt=time_step):
        """Time-march from v_start to v_end (v_end may be below v_start for braking). Returns
        (time, distance) using explicit midpoint (RK2) integration."""
        v = v_start
        t = 0.
        s = 0.
        direction = 1. if v_end >= v_start else -1.
        max_steps = int(600. / dt)
        for _ in range(max_steps):
            if (direction > 0 and v >= v_end) or (direction < 0 and v <= v_end):
                break
            a_now  = acceleration(v, thrust_func, mu)
            v_mid  = v + 0.5 * dt * a_now
            a_mid  = acceleration(v_mid, thrust_func, mu)
            v_next = v + dt * a_mid
            if direction > 0 and v_next > v_end:
                frac   = (v_end - v) / (v_next - v) if v_next != v else 1.0
                s     += 0.5 * (v + v_end) * dt * frac
                t     += dt * frac
                v      = v_end
                break
            if direction < 0 and v_next < v_end:
                frac   = (v_end - v) / (v_next - v) if v_next != v else 1.0
                s     += 0.5 * (v + v_end) * dt * frac
                t     += dt * frac
                v      = v_end
                break
            s += 0.5 * (v + v_next) * dt
            t += dt
            v  = v_next
        return t, s

    def accelerate_go_distance(v1):
        _, s_aeo   = integrate(0., v1, T_AEO, rolling_friction_coefficient)
        _, s_oei   = integrate(v1, v_lift_off, T_OEI, rolling_friction_coefficient)
        return s_aeo + s_oei + climb_out_distance

    def accelerate_stop_distance(v1):
        _, s_aeo     = integrate(0., v1, T_AEO, rolling_friction_coefficient)
        s_reaction   = v1 * recognition_time
        _, s_braking = integrate(v1, 0., lambda v: 0., braking_friction_coefficient)
        return s_aeo + s_reaction + s_braking

    def balance_residual(v1):
        return accelerate_go_distance(v1) - accelerate_stop_distance(v1)

    v1_low  = 0.85 * stall_speed
    v1_high = v_lift_off

    if balance_residual(v1_low) < 0 or balance_residual(v1_high) > 0:
        raise ValueError(
            "Balanced field length did not bracket a valid V1 between 0.85*Vstall and V_LOF; "
            "check vehicle thrust/weight ratio and friction assumptions."
        )

    decision_speed = brentq(balance_residual, v1_low, v1_high, xtol=1e-2)
    balanced_field_length = accelerate_go_distance(decision_speed)

    return balanced_field_length, decision_speed


def _static_thrust(vehicle, altitude, delta_isa, velocity, speed_of_sound):
    """Evaluates all-engines-operating, full-throttle thrust at a given ground-roll speed and
    altitude by building a throwaway single-point mission segment (mirrors the static-thrust
    evaluation used in estimate_take_off_field_length, sampled at a nonzero speed)."""

    planet          = RCAIDE.Library.Attributes.Planets.Earth()
    atmosphere      = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data       = atmosphere.compute_values(altitude, delta_isa)

    mach = velocity / speed_of_sound

    conditions                                             = RCAIDE.Framework.Mission.Common.Results()
    conditions.freestream.altitude                         = np.atleast_1d(altitude)
    conditions.freestream.mach_number                      = np.atleast_1d(mach)
    conditions.freestream.pressure                         = atmo_data.pressure
    conditions.freestream.temperature                      = atmo_data.temperature
    conditions.freestream.density                          = atmo_data.density
    conditions.freestream.dynamic_viscosity                = atmo_data.dynamic_viscosity
    conditions.freestream.kinematic_viscosity               = atmo_data.kinematic_viscosity
    conditions.freestream.thermal_conductivity              = atmo_data.thermal_conductivity
    conditions.freestream.prandtl_number                    = atmo_data.prandtl_number
    conditions.freestream.gravity                           = np.atleast_2d(planet.sea_level_gravity)
    conditions.freestream.speed_of_sound                    = np.atleast_1d(speed_of_sound)
    conditions.freestream.velocity                          = np.atleast_1d(velocity)
    conditions.freestream.dynamic_pressure                  = np.atleast_1d(0.5 * atmo_data.density[0] * velocity ** 2)
    conditions.freestream.constant_pressure_specific_heat   = atmo_data.constant_pressure_specific_heat
    conditions.freestream.specific_heat                     = atmo_data.specific_heat
    conditions.frames.inertial.position_vector              = np.array([[0, 0, -altitude]])

    analysis           = RCAIDE.Framework.Analyses.Vehicle()
    analysis.vehicle    = vehicle
    energy_analysis     = RCAIDE.Framework.Analyses.Energy.Energy()
    analysis.append(energy_analysis)

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    segment = RCAIDE.Framework.Mission.Segments.Segment()
    segment.hybrid_power_split_ratio            = None
    segment.battery_fuel_cell_power_split_ratio = None
    segment.temperature_deviation                = delta_isa
    segment.initial_battery_conditions            = Data()
    segment.initial_battery_conditions.state_of_charge       = 1.0
    segment.initial_battery_conditions.cell_temperature       = None
    segment.initial_battery_conditions.charge_throughput      = None
    segment.initial_battery_conditions.increment_battery_age  = False
    segment.analyses.extend(analysis)
    segment.state.conditions  = conditions
    segment.conditions        = segment.state.conditions
    segment.state.numerics.time.control_points = np.array([[0.0]])
    segment.state.numerics.time.differentiate  = np.zeros((1, 1))
    segment.state.numerics.time.integrate      = np.zeros((1, 1))
    mission.append_segment(segment)

    from RCAIDE.Library.Mission.Common.Pre_Process.energy import energy
    energy(mission)

    for network in vehicle.networks:
        for source in network.sources:
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack):
                source.append_unknowns_and_residuals(segment)

    thrust = np.array([[0.0, 0.0, 0.0]])
    for network in vehicle.networks:
        for propulsor in network.propulsors:
            segment.state.conditions.energy.propulsors[propulsor.tag].throttle = np.array([[1]])
        for source in network.sources:
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):
                fuel = source.fuel
                segment.state.conditions.weights.components.mass[fuel.tag] = np.array([[0]])
        network.evaluate(segment.state, vehicle)
        thrust = thrust + conditions.energy.total_force_vector

    return float(np.linalg.norm(thrust[0]))
