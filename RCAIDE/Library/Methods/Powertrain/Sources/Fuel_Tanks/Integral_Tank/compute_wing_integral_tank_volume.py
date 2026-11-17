
# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/Integral_Tank/compute_wing_integral_tank_volume.py
#
#
# Created:  Jul 2023, M. Clarke
# Modified: Aug 2025, S. Shekar
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import  RCAIDE
from RCAIDE.Framework.Core import Data, array_namespace
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry,  compute_naca_4series
from RCAIDE.Library.Methods.Geometry.Solids  import cut_section, loft_properties, combine_properties, mirror_properties, scale_properties

# Python imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------
def compute_wing_integral_tank_volume(fuel_tank,wing,n_points = 101,scale_factor = 0.85, number_of_stations = 101):
    """
    Computes the volume, center of gravity and inertia of an integral fuel tank in a wing.

    Parameters
    ----------
    fuel_tank : Fuel_Tank
        Its segments_bounding_tank (first and last wing segment of the tank) and the chord fractions of the front
        and rear spar at each of those segments (segments_percent_chord_start, segments_percent_chord_end).
    wing : Wing
        Segments with their origins, chord fractions, span locations and airfoils.
    n_points : int
        Points of each airfoil.
    scale_factor : float
        Usable fraction of the volume between the spars.
    number_of_stations : int
        Chordwise stations of each section between the spars, on each surface.

    Notes
    -----
    Each wing segment's airfoil is cut at the spars, and the tank between two segments is the solid blended linearly
    from one cut section to the next, so it stays inside the wing skin. Its volume, center of gravity and inertia are
    computed exactly for that solid (RCAIDE.Library.Methods.Geometry.Solids) and scaled by scale_factor about the
    center of gravity; a wing symmetric about the xz plane holds a mirror image of the tank. The sections are not
    rotated by the segment twist. The center of gravity is relative to the wing origin. Works on NumPy and JAX values.

    **Major Assumptions**
        * Fuel tank geometry follows the wing's airfoil profile between the spars
        * Fuel density is uniform throughout the tank
    """

    # get orgin of fuel tank
    fuel_tank.origin                  = wing.origin
    fuel_tank.fuel.origin             = wing.origin
    fuel_tank.fuel.xz_plane_symmetric = wing.xz_plane_symmetric
    fuel_tank.fuel.xy_plane_symmetric = wing.xy_plane_symmetric
    fuel_tank.fuel.yz_plane_symmetric = wing.yz_plane_symmetric

    seg_bounds =  fuel_tank.segments_bounding_tank
    # Collect all segment tags between start and end (inclusive)
    collect = False
    seg_keys = []
    for segment in wing.segments:
        if segment.tag == seg_bounds[0]:
            collect = True
        if collect:
            seg_keys.append(segment.tag)
        if segment.tag == seg_bounds[1]:
            break

    symm = wing.xz_plane_symmetric

    # each bounding segment's airfoil cut at its spars, placed at the segment's origin and scaled to its chord
    sections = []
    for i, key in enumerate(seg_keys):
        segment  = wing.segments[key]
        airfoil  = airfoil_geometry(segment.airfoil, n_points)
        x, z     = cut_section(airfoil.x_upper_surface, airfoil.y_upper_surface, airfoil.x_lower_surface, airfoil.y_lower_surface,
                               fuel_tank.segments_percent_chord_start[i], fuel_tank.segments_percent_chord_end[i], number_of_stations)
        chord    = wing.chords.root * segment.root_chord_percent
        y        = segment.percent_span_location * wing.spans.projected / 2
        sections.append((x * chord + segment.origin[0][0], z * chord + segment.origin[0][2], y))

    # one solid per pair of segments, blended between their sections
    panels = []
    for i in range(len(sections) - 1):
        inner_x, inner_z, inner_y = sections[i]
        outer_x, outer_z, _      = sections[i + 1]
        # the spanwise length of a panel is its share of the half span of a symmetric wing, of the span otherwise
        length = (wing.segments[seg_keys[i + 1]].percent_span_location - wing.segments[seg_keys[i]].percent_span_location) * wing.spans.projected / (symm + 1)
        panels.append(loft_properties(inner_x, inner_z, outer_x, outer_z, inner_y, inner_y + length))
        tank_end = inner_y + length
    half = combine_properties(panels)

    # external dimensions of the tank before the usable fraction is taken
    xp = array_namespace(*[section[0] for section in sections], *[section[1] for section in sections])
    x_all = xp.concatenate([section[0] for section in sections])
    z_all = xp.concatenate([section[1] for section in sections])
    fuel_tank.widths.external  = xp.max(x_all) - xp.min(x_all)
    fuel_tank.lengths.external = tank_end - sections[0][2]
    fuel_tank.heights.external = xp.max(z_all) - xp.min(z_all)

    # the usable fraction scales the tank about its center of gravity; a symmetric wing holds its mirror image
    tank = scale_properties(half, scale_factor)
    if symm:
        tank = combine_properties([tank, mirror_properties(tank)])

    center = tank.center_of_gravity
    fuel_tank.fuel.mass_properties.center_of_gravity                         = [[center[0], center[1], center[2]]]
    fuel_tank.mass_properties.center_of_gravity                              = [[center[0], center[1], center[2]]]
    fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor = tank.inertia_per_mass
    fuel_tank.volume_properties.gross_volume                                 = tank.volume
    fuel_tank.volume_properties.net_volume                                   = tank.volume
    return


def airfoil_geometry(airfoil, n_points):
    """
    Upper and lower surface of a segment's airfoil per unit chord, a NACA 0012 when the segment has none, taken from
    the airfoil's closed outline (x_coordinates, y_coordinates) split at its leading edge. The outline has its leading
    edge at (0, 0); the separate upper and lower surface arrays of an imported airfoil keep the file's own datum.
    """
    if airfoil is None:
        geometry = compute_naca_4series('0012', n_points)
    elif type(airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
        geometry = compute_naca_4series(airfoil.NACA_4_Series_code, n_points, thickness_multiplier=airfoil.thickness_multiplier)
    else:
        geometry = import_airfoil_geometry(airfoil.coordinate_file, n_points, thickness_multiplier=airfoil.thickness_multiplier)
    x       = np.asarray(geometry.x_coordinates)
    y       = np.asarray(geometry.y_coordinates)
    leading = int(np.argmin(x))
    # the outline runs from the trailing edge along one surface to the leading edge and back along the other
    first   = Data(x=x[:leading + 1], y=y[:leading + 1])
    second  = Data(x=x[leading:], y=y[leading:])
    if np.mean(first.y) > np.mean(second.y):
        upper, lower = first, second
    else:
        upper, lower = second, first
    return Data(x_upper_surface=upper.x, y_upper_surface=upper.y, x_lower_surface=lower.x, y_lower_surface=lower.y)
