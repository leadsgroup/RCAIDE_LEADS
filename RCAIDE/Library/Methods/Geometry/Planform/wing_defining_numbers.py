# RCAIDE/Library/Methods/Geometry/Planform/wing_defining_numbers.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  wing_defining_numbers
# ----------------------------------------------------------------------------------------------------------------------
def wing_defining_numbers(wing):
    """
    The numbers that define a wing, from which wing_planform computes the rest of its geometry.

    Parameters
    ----------
    wing : Wing

    Returns
    -------
    names : tuple of str
    read : function
        read() gives the numbers as a tuple of float arrays, one per name.
    write : function
        write(*arrays) writes them to the wing; the values can be JAX arrays. Run wing_planform after it.

    Notes
    -----
    A wing given without segments is defined by its area, aspect ratio, taper, quarter-chord sweep, the twist of its
    two segments and its dihedral. A wing given by segments is defined by its span and root chord and, per segment,
    its chord fraction and twist and, for every segment but the tip, its dihedral and quarter-chord sweep.
    """
    segments = list(wing.segments.values())

    if wing.segments_from_planform:
        names = ('area', 'aspect ratio', 'taper', 'quarter-chord sweep', 'twists', 'dihedral')
        def read():
            return (np.array([wing.areas.reference], dtype=float), np.array([wing.aspect_ratio], dtype=float),
                    np.array([wing.taper], dtype=float), np.array([wing.sweeps.quarter_chord], dtype=float),
                    np.array([segment.twist for segment in segments], dtype=float), np.array([wing.dihedral], dtype=float))
        def write(area, aspect_ratio, taper, sweep, twists, dihedral):
            wing.areas.reference      = area[0]
            wing.aspect_ratio         = aspect_ratio[0]
            wing.taper                = taper[0]
            wing.sweeps.quarter_chord = sweep[0]
            for i, segment in enumerate(segments):
                segment.twist = twists[i]
            wing.dihedral = dihedral[0]
        return names, read, write

    names = ('span', 'root chord', 'chord fractions', 'twists', 'dihedrals', 'quarter-chord sweeps')
    def read():
        return (np.array([wing.spans.projected], dtype=float), np.array([wing.chords.root], dtype=float),
                np.array([segment.root_chord_percent for segment in segments], dtype=float),
                np.array([segment.twist for segment in segments], dtype=float),
                np.array([segment.dihedral_outboard for segment in segments[:-1]], dtype=float),
                np.array([segment.sweeps.quarter_chord for segment in segments[:-1]], dtype=float))
    def write(span, root_chord, chord_fractions, twists, dihedrals, sweeps):
        wing.spans.projected = span[0]
        wing.chords.root     = root_chord[0]
        for i, segment in enumerate(segments):
            segment.root_chord_percent = chord_fractions[i]
            segment.twist              = twists[i]
        for i, segment in enumerate(segments[:-1]):
            segment.dihedral_outboard    = dihedrals[i]
            segment.sweeps.quarter_chord = sweeps[i]
    return names, read, write
