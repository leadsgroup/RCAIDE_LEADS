# RCAIDE/Library/Mission/Segments/Ground/Refuel.py
#
#
# Created: Aug 2026, M. Clarke

import RCAIDE

# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------
def initialize_conditions(segment):
    """
    Initializes conditions for a ground refueling segment: holds for
    ``segment.time``, during which each Cryogenic_Tank is filled at a constant
    rate sized to reach ``segment.refuel_target_fill_fraction`` of its fixed
    design-basis full-liquid mass (``tank.design_full_liquid_mass``, set in
    ``append_cryogenic_tank_conditions``) by the end of the segment.

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - time : float
                Duration of the refuel operation [s]
            - refuel_target_fill_fraction : float
                Target liquid fill level, as a fraction of design_full_liquid_mass [-]

    Returns
    -------
    None
        Updates segment conditions directly:
            - conditions.frames.inertial.time [s]
            - conditions.energy.sources[tank.tag].refuel_mass_flow_rate [kg/s]

    Notes
    -----
    Incoming fuel is assumed thermally equilibrated with the tank's own bulk
    liquid (see compute_cryogenic_tank_performance's docstring) -- no
    chill-down transient. The target uses a fixed design reference mass, not
    the tank's current (possibly drifted) temperature, the same way a
    battery's recharge target is a fixed design capacity, not a live reading.
    """
    t_initial = segment.state.conditions.frames.inertial.time[0,0]
    t_nondim  = segment.state.numerics.dimensionless.control_points
    time      = t_nondim * segment.time + t_initial
    segment.state.conditions.frames.inertial.time[:,0] = time[:,0]

    vehicle = segment.analyses.vehicle
    for network in vehicle.networks:
        for source in network.sources:
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank):
                tank_conditions = segment.state.conditions.energy.sources[source.tag]

                # append_cryogenic_tank_conditions's own state.initials chaining
                # hasn't run yet at this point (it's under iterate.initials.energy,
                # which follows process.initialize), so read the prior segment's
                # end state directly rather than tank_conditions.fuel_mass[0,0].
                if segment.state.initials.keys():
                    m_l_current = segment.state.initials.conditions.energy.sources[source.tag].fuel_mass[-1,0]
                else:
                    m_l_current = tank_conditions.fuel_mass[0,0]

                target_mass = segment.refuel_target_fill_fraction * source.design_full_liquid_mass
                fill_rate   = max(0.0, target_mass - m_l_current) / segment.time

                tank_conditions.refuel_mass_flow_rate[:,0] = fill_rate

    return
