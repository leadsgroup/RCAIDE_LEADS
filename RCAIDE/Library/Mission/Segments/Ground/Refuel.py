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
    ``segment.time``, during which each fuel tank (any Fuel_Tank subtype --
    Cryogenic_Tank included) fills at ``segment.nominal_fill_rate`` toward
    ``segment.refuel_target_fill_fraction`` of its fixed design-basis
    full-liquid mass (``tank.design_full_liquid_mass``, set generically for
    every tank in ``append_fuel_tank_conditions``; Cryogenic_Tank overwrites
    it with its own temperature-dependent density calc in
    ``append_cryogenic_tank_conditions``). The actual fill-and-cutoff
    integration is tank-type-specific: compute_fuel_tank_performance applies
    an exact linear cutoff (see its own docstring), while
    compute_cryogenic_tank_performance uses a terminal IVP event.

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - time : float
                Duration of the ground hold [s]
            - refuel_target_fill_fraction : float
                Target liquid fill level, as a fraction of design_full_liquid_mass [-]
            - nominal_fill_rate : float or None
                Constant physical fill rate to run at [kg/s]. None sizes it
                automatically as 1.2x the rate that would exactly hit the
                target with zero boil-off, over segment.time -- a margin
                meant to comfortably absorb the boil-off loss that the naive
                rate ignores, so the fill still finishes inside the window.

    Returns
    -------
    None
        Updates segment conditions directly:
            - conditions.frames.inertial.time [s]
            - conditions.energy.sources[tank.tag].refuel_mass_flow_rate [kg/s]
            - conditions.energy.sources[tank.tag].refuel_target_mass [kg]

    Notes
    -----
    This only sets the *rate* to run at and the *target* to cut off at.
    compute_cryogenic_tank_performance is what actually detects fuel_mass
    crossing refuel_target_mass and zeroes the fill from that point on --
    mirroring Battery_Recharge/cutoff_SOC, rather than back-solving a rate
    that lands on target only if boil-off happens to match the estimate.

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
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):
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

                if segment.nominal_fill_rate is not None:
                    fill_rate = segment.nominal_fill_rate
                else:
                    naive_rate = max(0.0, target_mass - m_l_current) / segment.time
                    fill_rate  = 1.2 * naive_rate

                tank_conditions.refuel_mass_flow_rate[:,0] = fill_rate
                tank_conditions.refuel_target_mass[:,0]    = target_mass

    return
