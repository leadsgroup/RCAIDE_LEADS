# RCAIDE/Library/Mission/Common/Pre_Process/declared_values.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Data, Container
from RCAIDE.Library.Components.Component import Container as Component_Container

# ----------------------------------------------------------------------------------------------------------------------
#  record_undeclared_values
# ----------------------------------------------------------------------------------------------------------------------
def record_undeclared_values(mission):
    """
    Warns about the values on the mission vehicle that no class declares, and records their paths.

    Parameters
    ----------
    mission : Mission
        Each segment's analyses hold the vehicle, after the configurations step.

    Returns
    -------
    None
        mission.undeclared_vehicle_values is the list of paths.

    Notes
    -----
    A value is declared when a fresh object of its owner's class holds the same key. These values come from the
    vehicle script, so they are kept, but they cannot be design variables.

    See Also
    --------
    RCAIDE.Library.Mission.Common.Pre_Process.check_undeclared_values
    """
    paths = find_undeclared_values(mission)
    if len(paths) > 0:
        print('Warning: values set on the vehicle that no class declares; they cannot be design variables:')
        for path in paths:
            print('  ' + path)
    mission.undeclared_vehicle_values = paths
    return


# ----------------------------------------------------------------------------------------------------------------------
#  check_undeclared_values
# ----------------------------------------------------------------------------------------------------------------------
def check_undeclared_values(mission):
    """
    Raises an error if a setup method added a value to the mission vehicle that no class declares.

    Parameters
    ----------
    mission : Mission
        The mission at the end of its setup, with mission.undeclared_vehicle_values from record_undeclared_values.

    Returns
    -------
    None

    Notes
    -----
    A value a method computes must have a default in its component class, or belong in the results. Otherwise
    restoring the vehicle between design evaluations would miss it.
    """
    recorded = mission.undeclared_vehicle_values
    if recorded is None:
        recorded = []
    added = [path for path in find_undeclared_values(mission) if path not in recorded]
    if len(added) > 0:
        raise AttributeError('setup methods added values that no class declares: ' + ', '.join(added))
    return


# ----------------------------------------------------------------------------------------------------------------------
#  find_undeclared_values
# ----------------------------------------------------------------------------------------------------------------------
def find_undeclared_values(mission):
    """
    Lists the paths of the values on the mission vehicles that no class declares.

    Parameters
    ----------
    mission : Mission

    Returns
    -------
    paths : list of str
        Paths from the vehicle, such as 'vehicle.wings.main_wing.areas.projected'.
    """
    vehicles = {}
    for segment in mission.segments:
        vehicle = segment.analyses.vehicle
        vehicles[id(vehicle)] = vehicle

    paths    = []
    defaults = {}
    for vehicle in vehicles.values():
        walk_declared_values(vehicle, fresh_default(vehicle, defaults), 'vehicle', paths, set(), defaults)
    return sorted(set(paths))


def walk_declared_values(node, default, path, paths, seen, defaults):
    """Adds the paths of the keys of node that default does not hold, and walks into its Data values."""
    if id(node) in seen:
        return
    seen.add(id(node))
    container = isinstance(node, (Container, Component_Container))
    for key, value in dict.items(node):
        child_path = path + '.' + str(key)

        # a key the class declares as None is filled by a method: its contents are that method's output
        if default is not None and key in default and default[key] is None:
            continue

        # entries of a container are named by their tag, each compared with a fresh object of its own class
        if container and (default is None or key not in default):
            if isinstance(value, Data):
                walk_declared_values(value, fresh_default(value, defaults), child_path, paths, seen, defaults)
            continue

        if default is None or key not in default:
            paths.append(child_path)
            continue

        if isinstance(value, Data):
            if type(default[key]) is type(value):
                child_default = default[key]
            else:
                child_default = fresh_default(value, defaults)
            walk_declared_values(value, child_default, child_path, paths, seen, defaults)
    return


def fresh_default(node, defaults):
    """A fresh object of the class of node, made once per class."""
    node_type = type(node)
    if node_type not in defaults:
        try:
            defaults[node_type] = node_type()
        except Exception:
            defaults[node_type] = None
    return defaults[node_type]
