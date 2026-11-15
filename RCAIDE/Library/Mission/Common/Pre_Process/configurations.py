# RCAIDE/Library/Mission/Common/Pre_Process/configurations.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Data, Diffed_Data, base_vehicle

# python imports
import numpy as np
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------
#  configurations
# ----------------------------------------------------------------------------------------------------------------------
def configurations(mission):
    """
    Gives every segment of a mission the same vehicle and the values of its own configuration.

    Parameters
    ----------
    mission : Mission
        Each segment's analyses hold a vehicle, or a configuration made from the vehicle
        (RCAIDE.Library.Components.Configs.Config).

    Returns
    -------
    None
        segment.analyses.vehicle and the vehicle of each analysis are the one vehicle of the mission.
        segment.configuration.vehicle_values is {path: value}, one entry for every path that any configuration of
        the mission changes, a path being the tuple of keys from the vehicle.

    Notes
    -----
    A mission flies one vehicle. A configuration (flaps, landing gear, rotor angles, active propulsors) is a set of
    values written onto that vehicle with apply_configuration before a segment is set up, solved or read. Every
    configuration holds a value for the same paths, so applying one needs nothing to be undone first.

    See Also
    --------
    RCAIDE.Framework.Core.Diffed_Data
    """
    vehicle = None
    for tag, segment in mission.segments.items():
        set_segment_configuration(segment)
        if vehicle is None:
            vehicle = segment.analyses.vehicle
        elif segment.analyses.vehicle is not vehicle:
            raise ValueError(f"Segment '{tag}' of mission '{mission.tag}' flies another vehicle than the segments before it. "
                             f"Make every configuration of a mission from the same vehicle.")

    # a path changed by one configuration takes the vehicle's own value in the others
    original = {}
    for segment in mission.segments:
        original.update(segment.configuration.original_values)
    for segment in mission.segments:
        for path, value in original.items():
            if path not in segment.configuration.vehicle_values:
                segment.configuration.vehicle_values[path] = value
    return

# ----------------------------------------------------------------------------------------------------------------------
#  set_segment_configuration
# ----------------------------------------------------------------------------------------------------------------------
def set_segment_configuration(segment):
    """
    Points a segment and its analyses at the vehicle its configuration was made from, and keeps the configuration's
    values on the segment.

    Parameters
    ----------
    segment : Segment
        segment.analyses.vehicle is a vehicle, or a configuration made from one.

    Returns
    -------
    None
        segment.analyses.vehicle               the vehicle
        segment.configuration.tag              tag of the configuration
        segment.configuration.vehicle_values   {path: value} the configuration writes onto the vehicle
        segment.configuration.original_values  {path: value} the vehicle holds at those paths without it

    Notes
    -----
    The values are those stored when the configuration was appended to its container. A configuration edited
    after that needs its store_diff() called before the mission is built.
    """
    if 'vehicle' not in segment.analyses.keys() or segment.analyses.vehicle is None:
        return
    vehicle = segment.analyses.vehicle
    if isinstance(vehicle, Diffed_Data):
        config = vehicle
        if config._diff is None:
            config.store_diff()
        if len(config._diff.added) > 0:
            added = ', '.join('.'.join(str(key) for key in path) for path in config._diff.added[:5])
            raise ValueError(f"Configuration '{config.tag}' of segment '{segment.tag}' sets {added}, which the vehicle does not "
                             f"have. A configuration can only change values the vehicle already has.")
        vehicle                                = base_vehicle(config)
        segment.configuration                  = Data()
        segment.configuration.tag              = config.tag
        segment.configuration.vehicle_values   = dict(config._diff.changed)
        segment.configuration.original_values  = dict(config._diff.original)
    elif segment.configuration is None:
        segment.configuration                  = Data()
        segment.configuration.tag              = vehicle.tag
        segment.configuration.vehicle_values   = {}
        segment.configuration.original_values  = {}

    # the segment and its analyses read the one vehicle
    for key, analysis in segment.analyses.items():
        if key == 'vehicle':
            segment.analyses.vehicle = vehicle
        elif isinstance(analysis, Data) and 'vehicle' in analysis.keys():
            analysis.vehicle = vehicle
    return

# ----------------------------------------------------------------------------------------------------------------------
#  apply_configuration
# ----------------------------------------------------------------------------------------------------------------------
def apply_configuration(vehicle, configuration):
    """
    Writes the values of a configuration onto the vehicle.

    Parameters
    ----------
    vehicle : Vehicle
        The vehicle of the mission (segment.analyses.vehicle).
    configuration : Data
        The configuration of a segment (segment.configuration); vehicle_values is the {path: value} to write.
        None leaves the vehicle as it is.

    Returns
    -------
    None
    """
    if configuration is None:
        return
    for path, value in configuration.vehicle_values.items():
        item = vehicle
        for key in path[:-1]:
            item = item[key]
        # lists and arrays are copied, so the vehicle never shares them with the configuration
        if isinstance(value, (list, np.ndarray)):
            value = deepcopy(value)
        item[path[-1]] = value
    return

# ----------------------------------------------------------------------------------------------------------------------
#  copy_segment
# ----------------------------------------------------------------------------------------------------------------------
def copy_segment(segment):
    """
    A copy of a segment that flies the same vehicle as the segment, not a copy of it.

    Parameters
    ----------
    segment : Segment

    Returns
    -------
    segment_copy : Segment
    """
    shared = {}
    if 'vehicle' in segment.analyses.keys() and segment.analyses.vehicle is not None:
        shared[id(segment.analyses.vehicle)] = segment.analyses.vehicle
    return deepcopy(segment, shared)
