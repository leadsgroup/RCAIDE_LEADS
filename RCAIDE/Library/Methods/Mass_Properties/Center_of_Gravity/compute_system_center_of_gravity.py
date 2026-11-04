# RCAIDE/Library/Methods/Mass_Properties/Center_of_Gravity/compute_system_center_of_gravity.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE

# python imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Compute System Center of Gravity
# ----------------------------------------------------------------------------------------------------------------------
def compute_system_center_of_gravity(system, vehicle):
    """
    Places a system at its generic location when the user has not defined its origin.

    Parameters
    ----------
    system : RCAIDE.Library.Components.Powertrain.Systems.Systems
        System to place
    vehicle : RCAIDE.Vehicle
        Vehicle providing the fuselage, wings, landing gear and cabins

    Notes
    -----
    Locations are the midpoints of the ranges in Table 4 of Chai et al. Groups split between two locations
    are placed at their weighted mean. Systems with a non-zero origin, and system types not listed below, are
    left unchanged.

        * Avionics, instruments: between the nose and the nose-wheel well; at the front of the seated cabin
          (instrument panel) for an unpressurized fuselage, which has no forward pressure bulkhead or equipment bay
        * Environmental controls: between the wing spars on the aircraft centerline
        * Electrical: half with the avionics, half between the wing spars
        * Hydraulics: wing and empennage groups split by surface area; empennage group in the tail cone
        * Flight controls: between the rear spar and trailing edge of each surface, split by surface area
        * Auxiliary power unit: in the tail cone
        * Furnishings: 52.5 percent of the seated cabin length

    References
    ----------
    Chai, S., Crisafulli, P., and Mason, W. H., "Aircraft Center of Gravity Estimation in Conceptual/Preliminary
    Design," AIAA Paper 95-3882, 1995.
    """
    if np.any(np.array(system.origin) != 0):
        return

    Systems = RCAIDE.Library.Components.Powertrain.Systems
    Wings   = RCAIDE.Library.Components.Wings
    placed  = (Systems.Avionics, Systems.Instruments, Systems.Environmental_Controls, Systems.Electrical,
               Systems.Hydraulics, Systems.Flight_Controls, Systems.Auxiliary_Power_Unit, Systems.Furnishings)
    if type(system) not in placed:
        return

    # body carrying the systems: longest fuselage, otherwise the blended wing body
    BWBs = [wing for wing in vehicle.wings if isinstance(wing, Wings.Blended_Wing_Body)]
    if len(vehicle.fuselages) > 0:
        body     = max(vehicle.fuselages, key=lambda fuselage: fuselage.lengths.total)
        x_nose   = body.origin[0][0]
        length   = body.lengths.total
        x_tail   = x_nose + length - body.lengths.tail / 2 if body.lengths.tail > 0 else x_nose + 0.95 * length
    elif len(BWBs) > 0:
        body     = BWBs[0]
        x_nose   = body.origin[0][0]
        length   = body.chords.root
        x_tail   = x_nose + length - body.aft_center_body.length / 2 if body.aft_center_body.length > 0 else x_nose + 0.95 * length
    else:
        return
    z_body = body.origin[0][2]

    # seated cabin extent
    LOPA = body.layout_of_passenger_accommodations
    if LOPA is not None and np.any(LOPA.object_coordinates[:, 10] == 1):
        seat_x = LOPA.object_coordinates[LOPA.object_coordinates[:, 10] == 1, 2]
        x_cabin_front, x_cabin_rear = np.min(seat_x), np.max(seat_x)
    else:
        x_cabin_front, x_cabin_rear = x_nose + 0.2 * length, x_nose + 0.8 * length

    # nose bay between the nose and the nose-wheel well; instrument panel at the cabin front if unpressurized
    nose_gears  = [gear for gear in vehicle.landing_gears if isinstance(gear, RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear)]
    x_nose_gear = nose_gears[0].origin[0][0] if len(nose_gears) > 0 else x_cabin_front
    pressurized = body.get('differential_pressure', 1.0) > 0
    x_nose_bay  = (x_nose + x_nose_gear) / 2 if pressurized else x_cabin_front

    # wing box on the centerline (root chord), or on the reference chord for a blended wing body
    main_wings = [wing for wing in vehicle.wings if isinstance(wing, (Wings.Main_Wing, Wings.Blended_Wing_Body))]
    if len(main_wings) == 0:
        return
    main_wing  = main_wings[0]
    spar_mid   = (main_wing.structural.front_spar_percent_chord + main_wing.structural.rear_spar_percent_chord) / 2
    if isinstance(main_wing, Wings.Blended_Wing_Body):
        x_wing_box = main_wing.LEMAC + spar_mid * main_wing.chords.mean_aerodynamic
    else:
        x_wing_box = main_wing.origin[0][0] + spar_mid * main_wing.chords.root

    # surface areas for groups split between the wing and the empennage
    tails       = [wing for wing in vehicle.wings if isinstance(wing, (Wings.Horizontal_Tail, Wings.Vertical_Tail))]
    S_wing      = sum(wing.areas.reference for wing in main_wings)
    S_empennage = sum(wing.areas.reference for wing in tails)

    if type(system) in (Systems.Avionics, Systems.Instruments):
        x = x_nose_bay
    elif type(system) == Systems.Environmental_Controls:
        x = x_wing_box
    elif type(system) == Systems.Electrical:
        x = (x_nose_bay + x_wing_box) / 2
    elif type(system) == Systems.Hydraulics:
        x = (S_wing * x_wing_box + S_empennage * x_tail) / (S_wing + S_empennage)
    elif type(system) == Systems.Flight_Controls:
        surfaces = main_wings + tails
        x_surfaces = [wing.LEMAC + (wing.structural.rear_spar_percent_chord + 1) / 2 * wing.chords.mean_aerodynamic for wing in surfaces]
        S_surfaces = [wing.areas.reference for wing in surfaces]
        x = np.sum(np.array(x_surfaces) * np.array(S_surfaces)) / np.sum(S_surfaces)
    elif type(system) == Systems.Auxiliary_Power_Unit:
        x = x_tail
    else:
        x = x_cabin_front + 0.525 * (x_cabin_rear - x_cabin_front)

    system.mass_properties.center_of_gravity = [[x, 0.0, z_body]]
    return system.mass_properties.center_of_gravity
