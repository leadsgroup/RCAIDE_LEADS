# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/compiled_lattice_cases.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Data, array_namespace
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM import VLM, LATTICE_FIELDS
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.generate_vortex_distribution import generate_vortex_distribution, repeat_vortex_distribution
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.generate_wing_vortex_distribution import wing_lattice_layout
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.train_VLM_surrogates import (angle_of_attack_conditions, reference_conditions,
    sideslip_and_rate_conditions, control_surface_conditions, high_lift_angle_conditions, stack_conditions)

# results of the vortex lattice that the surrogate training reads
RESULT_KEYS = ('CLift', 'CDrag_induced', 'CX', 'CY', 'CZ', 'CL', 'CM', 'CN', 'sectional_CLift', 'gamma')

# speed of sound of the training cases with rates, as in sideslip_and_rate_conditions
TRAINING_SPEED_OF_SOUND = 343.

# ----------------------------------------------------------------------------------------------------------------------
#  Compiled_Lattice_Cases
# ----------------------------------------------------------------------------------------------------------------------
class Compiled_Lattice_Cases():
    """
    The flow cases of the surrogate training as compiled JAX functions. There are two sets of cases: 'clean', every
    case flown with the control surfaces undeflected, and 'deflected', the two cases flown with a control surface
    deflected.

    The compiled functions take the lattice as their input (the fields of the vortex distribution the solve reads),
    with the reference values, the center of gravity and the Mach number. The lattice is generated outside them, from
    the vehicle as it is at each call. The lattice has the same structure whichever control surface is deflected, so
    each set is compiled once and serves every Mach number and every deflected surface. Keeping the generator out of
    the compiled functions keeps them small: the generator works station by station, and compiled with the solve it
    is most of the program.

    Parameters
    ----------
    aerodynamics : RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method
        Analysis with the vortex lattice settings and the training angles, sideslip angles and rates.
    vehicle : RCAIDE.Vehicle
        Vehicle after the geometry pre-process. Its structure (wings, segments, control surfaces, lattice) is fixed
        here; its values are read at each call.

    Notes
    -----
    The Mach numbers must be subsonic. compilations counts how often each set was traced.
    """
    def __init__(self, aerodynamics, vehicle):
        import jax
        settings          = aerodynamics.settings
        self.settings     = settings
        self.vehicle      = vehicle
        self.layouts      = {wing.tag: wing_lattice_layout(wing, settings.number_of_chordwise_vortices, settings.number_of_spanwise_vortices,
                                                           settings.spanwise_cosine_spacing) for wing in vehicle.wings}
        # the lattice as it is now gives the fields that hold its structure
        self.structure    = generate_vortex_distribution(control_surface_conditions(np.ones(1)), settings, vehicle, self.layouts)
        # the cases of each set at one Mach number, in the blocks the training reads them in
        one               = np.ones(1)
        self.blocks       = {'clean'    : [angle_of_attack_conditions(aerodynamics, one), reference_conditions(aerodynamics, one),
                                           sideslip_and_rate_conditions(aerodynamics, one), control_surface_conditions(one), high_lift_angle_conditions(one)],
                             'deflected': [control_surface_conditions(one), high_lift_angle_conditions(one)]}
        self.compilations = {'clean': 0, 'deflected': 0}
        self.value        = {'clean'    : jax.jit(lambda fields, references, mach: self.evaluate('clean', fields, references, mach)),
                             'deflected': jax.jit(lambda fields, references, mach: self.evaluate('deflected', fields, references, mach))}

    def evaluate(self, name, fields, references, mach):
        """The results of every case of a set for the lattice fields, the reference values and center of gravity, and
        the Mach number (traced by JAX)."""
        import jax.numpy as jnp
        self.compilations[name] += 1
        vehicle  = self.vehicle
        cases    = stack_conditions(self.blocks[name])
        n_cases  = len(cases.freestream.mach_number)
        # cases built with a velocity take the speed of the Mach number; the others the small velocity of the vortex lattice
        with_velocity = np.asarray(cases.freestream.velocity) != 0

        conditions                             = RCAIDE.Framework.Mission.Common.Results()
        conditions.freestream.mach_number      = jnp.ones((n_cases, 1)) * mach
        conditions.freestream.velocity         = jnp.where(with_velocity, mach * TRAINING_SPEED_OF_SOUND, 1e-6)
        conditions.aerodynamics.angles.alpha   = cases.aerodynamics.angles.alpha
        conditions.aerodynamics.angles.beta    = cases.aerodynamics.angles.beta
        conditions.static_stability.pitch_rate = cases.static_stability.pitch_rate
        conditions.static_stability.roll_rate  = cases.static_stability.roll_rate
        conditions.static_stability.yaw_rate   = cases.static_stability.yaw_rate

        lattice = Data()
        for key in self.structure.keys():
            lattice[key] = self.structure[key]
        for key, value in zip(LATTICE_FIELDS, fields):
            lattice[key] = value

        # the vehicle holds the traced reference values and center of gravity only while the cases are traced
        places = reference_inputs(vehicle)
        stored = [container[key] for container, key in places]
        try:
            vehicle.reference_area, vehicle.reference_chord, vehicle.reference_span = references[0], references[1], references[2]
            vehicle.mass_properties.center_of_gravity = [[references[3], references[4], references[5]]]
            results = VLM(conditions, self.settings, vehicle, VD=repeat_vortex_distribution(lattice, n_cases))
            return {key: results[key] for key in RESULT_KEYS}
        finally:
            for (container, key), value in zip(places, stored):
                container[key] = value

    def cases(self, name, vehicle, Mach):
        """
        The results of a set of cases at each Mach number, for the vehicle as it is now.

        Parameters
        ----------
        name : str
            'clean' or 'deflected'.
        vehicle : RCAIDE.Vehicle
            Vehicle with the structure this object was made for; its numbers can be JAX values.
        Mach : numpy.ndarray
            Subsonic Mach numbers.

        Returns
        -------
        results : list of Data
            One per block of the set, with RESULT_KEYS; the rows are the cases of the first Mach number, then those of
            the second, and so on. NumPy arrays unless a number of the vehicle is a JAX value.
        """
        import jax.numpy as jnp
        lattice    = generate_vortex_distribution(control_surface_conditions(np.ones(1)), self.settings, vehicle, self.layouts)
        fields     = tuple(lattice[key] for key in LATTICE_FIELDS)
        center     = vehicle.mass_properties.center_of_gravity
        references = [vehicle.reference_area, vehicle.reference_chord, vehicle.reference_span, center[0][0], center[0][1], center[0][2]]
        xp         = array_namespace(*fields, *references)
        references = jnp.stack([jnp.asarray(value, dtype=float) for value in references])
        outputs    = [self.value[name](fields, references, jnp.asarray(mach, dtype=float)) for mach in Mach]

        results = []
        start   = 0
        for block in self.blocks[name]:
            end  = start + len(block.freestream.mach_number)
            part = Data()
            for key in RESULT_KEYS:
                rows      = jnp.concatenate([output[key][start:end] for output in outputs], axis=0)
                part[key] = np.asarray(rows) if xp is np else rows
            results.append(part)
            start = end
        return results


def reference_inputs(vehicle):
    """Where the reference area, chord and span of a vehicle and its center of gravity are: (container, key) each."""
    return [(vehicle, 'reference_area'), (vehicle, 'reference_chord'), (vehicle, 'reference_span'), (vehicle.mass_properties, 'center_of_gravity')]
