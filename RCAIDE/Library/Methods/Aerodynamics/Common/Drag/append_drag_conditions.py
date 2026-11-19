# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/append_drag_conditions.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Mission.Common import Conditions

import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  append_drag_conditions
# ----------------------------------------------------------------------------------------------------------------------
def append_drag_conditions(segment, vehicle):
    """
    Allocates the drag outputs that depend on the vehicle; NaN until computed.

    Parameters
    ----------
    segment : Segment
        Mission segment whose state.conditions.aerodynamics.coefficients.drag.parasite is filled
    vehicle : Vehicle
        Vehicle flown in the segment

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_total
    """
    rows     = segment.state.ones_row(1).shape[0]
    parasite = segment.state.conditions.aerodynamics.coefficients.drag.parasite
    for tag in parasite_drag_components(vehicle):
        parasite[tag] = Conditions()
        for key in ['total', 'skin_friction', 'compressibility_factor', 'reynolds_factor', 'form_factor']:
            parasite[tag][key] = np.full((rows, 1), np.nan)
    for key in ['total', 'wings', 'fuselages', 'booms', 'nacelles']:
        parasite[key] = np.full((rows, 1), np.nan)
    return


def parasite_drag_components(vehicle):
    """
    Tags of the components that carry parasite drag: wings, fuselages, booms, nacelles and their pylons.

    Parameters
    ----------
    vehicle : Vehicle

    Returns
    -------
    tags : list of str
    """
    tags = [wing.tag for wing in vehicle.wings] + [fuselage.tag for fuselage in vehicle.fuselages] + \
           [boom.tag for boom in vehicle.booms]
    for network in vehicle.networks:
        for propulsor in network.propulsors:
            if propulsor.nacelle is not None:
                tags.append(propulsor.nacelle.tag)
                if propulsor.nacelle.has_pylon:
                    tags.append(propulsor.nacelle.tag + '_pylon')
    return tags
