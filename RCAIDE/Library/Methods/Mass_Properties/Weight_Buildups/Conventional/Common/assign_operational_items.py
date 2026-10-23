# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Conventional/Common/assign_operational_items.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE

# python imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Assign Operational Items
# ----------------------------------------------------------------------------------------------------------------------
def assign_operational_items(vehicle, operational_items):
    """
    Distributes the operational items mass over the fuselages and blended wing bodies and places it in the cabin.

    Parameters
    ----------
    vehicle : Vehicle
        Vehicle whose fuselages and blended wing bodies carry the operational items
    operational_items : Data
        Operational items weight breakdown with ``total`` and optionally ``flight_crew`` [kg]

    Notes
    -----
    The mass is split across bodies by seat count so it is counted once. The flight crew sits at the front
    of the seated cabin and the remaining items at the seat centroid of all cabins on that body. Bodies without
    seats use their mid-length. A user-defined ``operational_items.origin`` overrides the automatic location.
    """
    hosts = list(vehicle.fuselages) + [wing for wing in vehicle.wings if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body)]
    if len(hosts) == 0:
        return

    seats = []
    for host in hosts:
        LOPA = host.layout_of_passenger_accommodations
        if LOPA is None:
            seats.append(np.empty((0, 14)))
        else:
            seats.append(LOPA.object_coordinates[LOPA.object_coordinates[:, 10] == 1])
    total_seats = sum(len(host_seats) for host_seats in seats)

    W_total = operational_items.total
    W_crew  = operational_items.get('flight_crew', 0.0)
    for host, host_seats in zip(hosts, seats):
        share     = len(host_seats) / total_seats if total_seats > 0 else 1 / len(hosts)
        component = host.operational_items
        component.mass_properties.mass = W_total * share

        # a user-defined origin is kept and the automatic location is skipped
        if np.any(np.array(component.origin) != 0):
            component.mass_properties.center_of_gravity = [[0.0, 0.0, 0.0]]
            continue

        if len(host_seats) > 0:
            x_cabin = np.mean(host_seats[:, 2])
            x_crew  = np.min(host_seats[:, 2])
            x       = (W_crew * x_crew + (W_total - W_crew) * x_cabin) / W_total if W_total > 0 else x_cabin
            z       = np.mean(host_seats[:, 4])
        elif isinstance(host, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            x = host.origin[0][0] + host.chords.root / 2
            z = host.origin[0][2]
        else:
            x = host.origin[0][0] + host.lengths.total / 2
            z = host.origin[0][2]
        component.mass_properties.center_of_gravity = [[x, 0.0, z]]
    return
