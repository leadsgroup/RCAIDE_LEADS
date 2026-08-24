# estimate_rate_of_climb.py
#
# Created: Aug 2026, M. Clarke
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE
from RCAIDE.Framework.Core import Units

# ----------------------------------------------------------------------
#  Compute rate of climb capability at a flight condition
# ----------------------------------------------------------------------
def estimate_rate_of_climb(analyses=None, altitude=0., mach_number=0., weight=None, delta_isa=0.):
    """
    Computes the instantaneous rate-of-climb (ROC) capability of a vehicle at a specified
    altitude, Mach number, and weight.

    Parameters
    ----------
    analyses : Analyses
        Container with atmosphere, aerodynamics, energy, and vehicle analyses (e.g. a single
        config's analyses, such as ``analyses.base``)
    altitude : float, optional
        Altitude at which to evaluate the climb capability [m], default 0
    mach_number : float, optional
        Freestream Mach number at which to evaluate the climb capability, default 0
    weight : float, optional
        Vehicle weight at which to evaluate the climb capability [kg]. If None, the vehicle's
        current ``mass_properties.takeoff`` value is used.
    delta_isa : float, optional
        Temperature offset from ISA conditions [K], default 0

    Returns
    -------
    rate_of_climb : float
        Instantaneous rate of climb capability [m/s]
    excess_power : float
        Specific excess power available at the evaluated condition [W]

    Notes
    -----
    This solves a short (0.1 nmi), level, constant-Mach/constant-altitude segment for the
    throttle and pitch angle that balance the vehicle at the requested condition, then infers
    maximum available power by assuming available power scales linearly with throttle. This is
    the same procedure used historically for ICA/service-ceiling sweeps in this codebase
    (``ICA_evaluation_mission_setup`` / ``Test_R_ICA``), refactored into a single-condition,
    reusable Performance function.

    **Major Assumptions**
        * Available power varies linearly with throttle (``P_avail = P_required / throttle``)
        * Quasi-steady, unaccelerated flight at the evaluated condition (rate of climb read off
          as the specific excess power, not a full climbing trajectory)
        * Vehicle mass is held fixed at the requested weight for the evaluation

    **Theory**
    Rate of climb is computed from specific excess power theory:

    .. math::
        R/C = P_s = \\frac{(T - D) V}{W} = \\frac{P_{avail} - P_{req}}{W}

    where :math:`P_{req}` is the power required for level flight at the given weight/altitude/
    Mach (solved for directly), and :math:`P_{avail}` is the maximum power the propulsion system
    can deliver at that same flight condition.

    References
    ----------
    [1] Anderson, J.D., "Aircraft Performance and Design", McGraw-Hill, 1999, Ch. 5.

    See Also
    --------
    RCAIDE.Library.Methods.Performance.estimate_take_off_field_length
    """
    if analyses is None:
        raise AttributeError('RCAIDE analyses must be defined')

    vehicle = analyses.vehicle
    if weight is not None:
        vehicle.mass_properties.takeoff = weight

    # gather every propulsor tag so all engines are throttled together
    propulsor_tags = []
    for network in vehicle.networks:
        for propulsor in network.propulsors:
            propulsor_tags.append(propulsor.tag)
    if len(propulsor_tags) == 0:
        raise ValueError("No engine found in the vehicle")

    # ----------------------------------------------------------------
    # Build a single short level-flight segment at the requested condition
    # ----------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'rate_of_climb_evaluation'

    Segments     = RCAIDE.Framework.Mission.Segments
    base_segment = Segments.Segment()
    base_segment.state.numerics.mission_solver.type = 'root_finder'

    segment                       = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag                   = 'rate_of_climb_point'
    segment.analyses.extend(analyses)
    segment.altitude              = altitude
    segment.mach_number           = mach_number
    segment.temperature_deviation = delta_isa
    segment.distance              = 0.1 * Units.nmi

    # define flight dynamics to model
    segment.flight_dynamics.force_x = True
    segment.flight_dynamics.force_z = True

    # define flight controls
    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.assigned_propulsors  = [propulsor_tags]
    segment.assigned_control_variables.pitch_angle.active            = True

    mission.append_segment(segment)

    # ----------------------------------------------------------------
    # Evaluate and back out excess power / rate of climb
    # ----------------------------------------------------------------
    results         = mission.evaluate()
    conditions      = results.segments[0].conditions

    power = sum(
        conditions.energy.propulsors[tag].outputs.power.propulsive[0, 0]
        for tag in propulsor_tags
    )
    throttle = conditions.energy.propulsors[propulsor_tags[0]].throttle[0, 0]
    if throttle <= 0:
        raise ValueError("Solved throttle is zero or negative; cannot back out available power.")

    power_available = power / throttle
    excess_power     = power_available - power
    vehicle_mass     = conditions.weights.vehicle.mass[0, 0]
    gravity          = conditions.freestream.gravity[0, 0]

    rate_of_climb = excess_power / (vehicle_mass * gravity)

    return rate_of_climb, excess_power
