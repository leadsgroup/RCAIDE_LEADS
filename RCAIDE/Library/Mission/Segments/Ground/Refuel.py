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
    Initializes a ground refueling segment: a hold of ``segment.time`` during which each fuel tank fills at
    ``nominal_fill_rate`` toward its target. The tank performance methods cut the fill off once the target
    or the volume limit is reached.

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - time : float
                Duration of the ground hold [s]
            - refuel_target_fill_fraction : float or None
                Target as a fraction of the tank's full liquid mass [-]. None targets the tank's mission fuel
                load (``tank.fuel.mass_properties.mass``), which the mass_properties Pre_Process limits to MTOW.
            - nominal_fill_rate : float or None
                Fill rate [kg/s]. None uses 1.2 times the rate that reaches the target over ``time`` without
                boil-off, leaving margin for the boil-off during the fill.

    Returns
    -------
    None
        Updates segment conditions directly:
            - conditions.frames.inertial.time [s]
            - conditions.energy.sources[tank.tag].refuel_mass_flow_rate [kg/s]
            - conditions.energy.sources[tank.tag].refuel_target_mass [kg]

    Notes
    -----
    Incoming fuel is at the tank's bulk liquid temperature, with no chill-down transient.
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

                # Segment chaining runs after initialization, so read the previous segment's end state directly
                if segment.state.initials.keys():
                    m_l_current = segment.state.initials.conditions.energy.sources[source.tag].fuel_mass[-1,0]
                else:
                    m_l_current = tank_conditions.fuel_mass[0,0]

                if segment.refuel_target_fill_fraction is not None:
                    target_mass = segment.refuel_target_fill_fraction * source.design_full_liquid_mass
                elif source.fuel.mass_properties.mass > 0:
                    target_mass = min(source.fuel.mass_properties.mass, source.design_full_liquid_mass)
                else:
                    target_mass = source.design_full_liquid_mass

                if segment.nominal_fill_rate is not None:
                    fill_rate = segment.nominal_fill_rate
                else:
                    naive_rate = max(0.0, target_mass - m_l_current) / segment.time
                    fill_rate  = 1.2 * naive_rate

                tank_conditions.refuel_mass_flow_rate[:,0] = fill_rate
                tank_conditions.refuel_target_mass[:,0]    = target_mass

    return
