# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/surrogate_inputs.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Data, DataOrdered

# package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  surrogate_inputs
# ----------------------------------------------------------------------------------------------------------------------
def surrogate_inputs(aerodynamics, vehicle):
    """
    The values the vortex lattice surrogates of a vehicle are built from.

    Parameters
    ----------
    aerodynamics : RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method
        Its settings and training conditions are inputs.
    vehicle : RCAIDE.Vehicle
        Its wings and fuselages (without their mass properties), reference area, chord and span, center of gravity
        (the moments are taken about it) and the nacelle and coupling flags are inputs.

    Returns
    -------
    inputs : dict
        {path: value}, a vehicle value under its path from the vehicle, the others under settings and training.
        Two vehicles with equal inputs have the same surrogates.

    Notes
    -----
    The set is wider than what the training reads, so that a value it comes to read later is already covered.
    Control surface deflections are left out: the training sets them itself. The trained tables stored under
    aerodynamics.training are results, not inputs.
    """
    inputs = {}
    collect_values(vehicle.wings,     'wings',     inputs, skip=('mass_properties', 'deflection', 'secondary_deflection'))
    collect_values(vehicle.fuselages, 'fuselages', inputs, skip=('mass_properties',))

    inputs['reference_area']                    = vehicle.reference_area
    inputs['reference_chord']                   = vehicle.reference_chord
    inputs['reference_span']                    = vehicle.reference_span
    inputs['mass_properties.center_of_gravity'] = vehicle.mass_properties.center_of_gravity

    for network in vehicle.networks:
        inputs[f'networks.{network.tag}.wing_on_rotor_coupling'] = network.get('wing_on_rotor_coupling')
        for propulsor in network.propulsors:
            if propulsor.get('nacelle') is not None:
                inputs[f'networks.{network.tag}.propulsors.{propulsor.tag}.nacelle.has_pylon'] = propulsor.nacelle.get('has_pylon')

    collect_values(aerodynamics.settings, 'settings', inputs)
    collect_values(aerodynamics.training, 'training', inputs, skip=('subsonic', 'supersonic', 'transonic'))
    return inputs


def collect_values(item, path, inputs, skip=()):
    """Adds every number, flag, string and array below item to inputs, keyed by its path."""
    for key in item.keys():
        if str(key).startswith('_') or key in skip:
            continue
        value    = item[key]
        key_path = f"{path}.{key}"
        if isinstance(value, (Data, DataOrdered)):
            collect_values(value, key_path, inputs, skip)
        elif isinstance(value, type):
            inputs[key_path] = value.__name__
        elif value is None or isinstance(value, (bool, int, float, str, np.ndarray, np.generic, list, tuple)):
            inputs[key_path] = value
    return


def same_surrogate_inputs(inputs, other):
    """True when two sets of surrogate inputs hold the same paths and equal values."""
    if other is None or inputs.keys() != other.keys():
        return False
    for path, value in inputs.items():
        if not np.array_equal(np.asarray(value, dtype=object), np.asarray(other[path], dtype=object)):
            return False
    return True
