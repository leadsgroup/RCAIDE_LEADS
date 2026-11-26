# RCAIDE/Library/Plots/Geometry/generate_3d_wing_points.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry
from RCAIDE.Library.Methods.Geometry.Airfoil import compute_naca_4series 
import numpy as np     

# ----------------------------------------------------------------------------------------------------------------------
#  generate_3d_wing_points
# ----------------------------------------------------------------------------------------------------------------------   
def generate_3d_wing_points(wing, n_points,plot_centerline = False, mirrored = False):
    """
    Generates 3D coordinate points that define a wing surface.

    Parameters
    ----------
    wing : Wing
        RCAIDE wing data structure containing geometry information
        
    n_points : int
        Number of points used to discretize airfoil sections
        
    dim : int
        Number of wing segments plus one
    plot_centerline : bool, optional
        Include the root centerline cap section in the output, default False

    mirrored : bool, optional
        True when the points are for the mirrored side of a symmetric wing, on which ailerons
        and the secondary deflection of compound surfaces are reversed, default False

    Returns
    -------
    G : Data
        Data structure containing generated points with attributes:
            - X, Y, Z : ndarray
                Raw coordinate points
            - PTS : ndarray
                Combined coordinate array
            - XA1, YA1, ZA1, XA2, YA2, ZA2 : ndarray
                Leading edge surface points
            - XB1, YB1, ZB1, XB2, YB2, ZB2 : ndarray
                Trailing edge surface points

    Notes
    -----
    Generates wing geometry by:
        1. Creating airfoil sections at specified span positions
        2. Applying twist, sweep, and dihedral
        3. Scaling sections by local chord
        4. Positioning in aircraft coordinate system
    
    **Definitions**
    
    'Leading Edge Sweep'
        Angle between leading edge and y-axis
    'Quarter Chord Sweep'
        Angle between quarter chord line and y-axis
    'Dihedral'
        Upward angle of wing from horizontal
    """    
    # unpack  
    # obtain the geometry for each segment in a loop            
    segments             = wing.segments
    n_segments           = len(segments.keys())
    origin               = wing.origin
    semispan             = wing.spans.projected / (1+ wing.xz_plane_symmetric)

    pts              = np.zeros((n_segments+2,n_points, 3,1))
    section_twist    = np.zeros((n_segments+2,n_points, 3,3))
    section_twist[:, :, 0, 0] = 1
    section_twist[:, :, 1, 1] = 1
    section_twist[:, :, 2, 2] = 1
    translation        = np.zeros((n_segments+2,n_points, 3,1))
    translation[:, :, 0,:] = origin[0][0]
    translation[:, :, 1,:] = origin[0][1]
    translation[:, :, 2,:] = origin[0][2]
    for i in range(n_segments+2):
        if i == 0:
            current_seg = list(segments.keys())[0]
        elif i == n_segments + 1:
            current_seg = list(segments.keys())[n_segments - 1]
        else:
            current_seg = list(segments.keys())[i-1]

        airfoil = wing.segments[current_seg].airfoil

        if  airfoil !=  None:
            if type(airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                geometry = compute_naca_4series(airfoil.NACA_4_Series_code,n_points, thickness_multiplier = airfoil.thickness_multiplier)
            elif type(airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil:
                geometry     = import_airfoil_geometry(airfoil.coordinate_file,n_points, thickness_multiplier = airfoil.thickness_multiplier)
        else:
            t_c = str(int(wing.segments[current_seg].thickness_to_chord *  100)).zfill(4)
            geometry = compute_naca_4series(t_c,n_points)

        twist    = wing.segments[current_seg].twist
        is_end_cap = (i == 0 or i == n_segments + 1)

        if wing.vertical:
            pts[i,:,0,0]   = geometry.x_coordinates * wing.segments[current_seg].root_chord_percent * wing.chords.root
            pts[i,:,1,0]   = (geometry.y_coordinates + geometry.y_coordinates[::-1]) / 2 if is_end_cap else geometry.y_coordinates * wing.segments[current_seg].root_chord_percent * wing.chords.root
            pts[i,:,2,0]   = np.zeros_like(geometry.y_coordinates)

            section_twist[i,:,0,0] = np.cos(twist)
            section_twist[i,:,0,1] = -np.sin(twist)
            section_twist[i,:,1,0] = np.sin(twist)
            section_twist[i,:,1,1] = np.cos(twist)

        else:
            pts[i,:,0,0]   = geometry.x_coordinates * wing.segments[current_seg].root_chord_percent * wing.chords.root
            pts[i,:,1,0]   = np.zeros_like(geometry.y_coordinates)
            pts[i,:,2,0]   = (geometry.y_coordinates + geometry.y_coordinates[::-1]) / 2 if is_end_cap else geometry.y_coordinates * wing.segments[current_seg].root_chord_percent * wing.chords.root


            section_twist[i,:,0,0] = np.cos(twist)
            section_twist[i,:,0,2] = np.sin(twist)
            section_twist[i,:,2,0] = -np.sin(twist)
            section_twist[i,:,2,2] =  np.cos(twist)

        if i != n_segments + 1:
            translation[i, :, 0,:] += segments[current_seg].origin[0][0]
            translation[i, :, 1,:] += segments[current_seg].origin[0][1]
            translation[i, :, 2,:] += segments[current_seg].origin[0][2]

        if i == n_segments:
            # compute tip translation from sweep and dihedral
            prev_seg = list(segments.keys())[i-2]

            sweep    = wing.segments[prev_seg].sweeps.leading_edge
            dihedral = wing.segments[prev_seg].dihedral_outboard

            segment_percent_span =  wing.segments[current_seg].percent_span_location  -  wing.segments[prev_seg].percent_span_location
            if wing.vertical:
                dz = semispan*segment_percent_span
                dy = dz*np.tan(dihedral)
                l  = dz/np.cos(dihedral)
                dx = l*np.tan(sweep)
            else:
                dy = semispan*segment_percent_span
                dz = dy*np.tan(dihedral)
                l  = dy/np.cos(dihedral)
                dx = l*np.tan(sweep)
            translation[i,:,0,:] = translation[i-1,:,0,:] + dx
            translation[i,:,1,:] = translation[i-1,:,1,:] + dy
            translation[i,:,2,:] = translation[i-1,:,2,:] + dz

        elif i == n_segments + 1:
            translation[i,:,:,:] = translation[i-1,:,:,:]
 
    mat     = translation + np.matmul(section_twist ,pts)

    # trailing-edge control surfaces, each rotated about its hinge line
    span_fractions  = [segments[key].percent_span_location for key in segments.keys()]
    sections        = deflect_control_surfaces(mat[1:n_segments+1,:,:,0], geometry.x_coordinates, span_fractions, wing, mirrored)
    if len(sections) != n_segments:
        tip_cap         = mat[n_segments+1,:,:,0]
        if not np.allclose(sections[-1], mat[n_segments,:,:,0]):
            tip_cap     = (sections[-1] + sections[-1][::-1]) / 2
        mat             = np.concatenate((mat[0:1,:,:,0], sections, tip_cap[None,:,:]))[:,:,:,None]

    if not plot_centerline:
        mat = mat[1:, :, :, :]

    # ---------------------------------------------------------------------------------------------
    # create empty data structure for storing geometry
    G = Data()

    # store node points
    G.X    = mat[:,:,0,0]  
    G.Y    = mat[:,:,1,0]  
    G.Z    = mat[:,:,2,0]
    G.PTS  = mat[:,:,:,0]

    # store points
    G.XA1  = mat[:-1,:-1,0,0] 
    G.YA1  = mat[:-1,:-1,1,0] 
    G.ZA1  = mat[:-1,:-1,2,0] 
    G.XA2  = mat[:-1,1:,0,0]  
    G.YA2  = mat[:-1,1:,1,0]  
    G.ZA2  = mat[:-1,1:,2,0]  
    G.XB1  = mat[1:,:-1,0,0]  
    G.YB1  = mat[1:,:-1,1,0]  
    G.ZB1  = mat[1:,:-1,2,0]  
    G.XB2  = mat[1:,1:,0,0]   
    G.YB2  = mat[1:,1:,1,0]   
    G.ZB2  = mat[1:,1:,2,0]      
    
    return G
     





# ----------------------------------------------------------------------------------------------------------------------
#  deflect_control_surfaces
# ----------------------------------------------------------------------------------------------------------------------
def deflect_control_surfaces(sections, chord_fractions, span_fractions, wing, mirrored):
    """
    Wing sections with each deflected trailing-edge control surface rotated about its hinge line.

    Parameters
    ----------
    sections : ndarray
        Points of the wing sections from root to tip, shape (number of sections, number of points, 3)
    chord_fractions : ndarray
        Chordwise position x/c of each point of a section, from the trailing edge along the lower
        surface to the leading edge and back along the upper surface
    span_fractions : list
        Span fraction of each section
    wing : RCAIDE.Library.Components.Wings.Wing
        Wing whose control surfaces are drawn
    mirrored : bool
        True for the mirrored side of a symmetric wing

    Returns
    -------
    sections : ndarray
        Sections with, for each deflected surface, an undeflected and a deflected section at both of
        its ends, so that the surface is drawn with a clean break from the wing

    Notes
    -----
    A surface is a plain flap: the part of each section aft of the hinge, at x/c = 1 - chord_fraction,
    rotates rigidly about the hinge line by the deflection. The hinge point of a section is midway
    between its upper and lower surfaces, and the hinge line runs between the hinge points at the two
    ends of the surface. A positive deflection follows the right-hand rule about the hinge line
    pointing outboard, as in the vortex lattice method (trailing edge down on the right wing).
    Slotted flaps are drawn as plain flaps. Slats and spoilers are not drawn deflected.

    **Major Assumptions**
        * Control surfaces do not overlap along the span
    """
    Surfaces       = RCAIDE.Library.Components.Wings.Control_Surfaces
    sections       = np.array(sections)
    span_fractions = np.array(span_fractions, dtype=float)
    side           = -1.0 if mirrored else 1.0

    for control_surface in wing.control_surfaces:
        if isinstance(control_surface, (Surfaces.Slat, Surfaces.Spoiler)):
            continue
        if isinstance(control_surface, Surfaces.Aileron):
            deflection = control_surface.deflection * side
        elif isinstance(control_surface, (Surfaces.Flaperon, Surfaces.Elevon, Surfaces.Ruddervator)):
            deflection = control_surface.deflection + control_surface.secondary_deflection * side
        else:
            deflection = control_surface.deflection
        if deflection == 0:
            continue

        start = max(control_surface.span_fraction_start, span_fractions[0])
        end   = min(control_surface.span_fraction_end,   span_fractions[-1])
        if end <= start:
            continue

        # sections of the surface: its two ends and the wing sections between them
        inside         = (span_fractions > start) & (span_fractions < end)
        surface_spans  = np.concatenate(([start], span_fractions[inside], [end]))
        surface        = np.array([section_at(sections, span_fractions, span) for span in surface_spans])

        # hinge line between the hinge points of the two ends, pointing outboard
        hinge_fraction = 1.0 - control_surface.chord_fraction
        hinges         = np.array([hinge_point(section, chord_fractions, hinge_fraction) for section in surface])
        axis           = (hinges[-1] - hinges[0]) / np.linalg.norm(hinges[-1] - hinges[0])
        aft            = chord_fractions > hinge_fraction
        deflected      = surface.copy()
        for k in range(len(surface)):
            deflected[k, aft] = rotate_about_axis(surface[k, aft] - hinges[k], axis, deflection) + hinges[k]

        inboard        = span_fractions < start
        outboard       = span_fractions > end
        sections       = np.concatenate((sections[inboard], surface[0:1], deflected, surface[-1:], sections[outboard]))
        span_fractions = np.concatenate((span_fractions[inboard], [start], surface_spans, [end], span_fractions[outboard]))
    return sections


def section_at(sections, span_fractions, span_fraction):
    """Section at a span fraction, interpolated linearly between the two sections around it, as the surface
    between them is drawn."""
    upper  = int(np.clip(np.searchsorted(span_fractions, span_fraction, side='right'), 1, len(span_fractions) - 1))
    lower  = upper - 1
    width  = span_fractions[upper] - span_fractions[lower]
    weight = 0.0 if width == 0 else (span_fraction - span_fractions[lower]) / width
    return (1 - weight) * sections[lower] + weight * sections[upper]


def hinge_point(section, chord_fractions, hinge_fraction):
    """Point midway between the lower and upper surfaces of a section at the hinge's chord fraction."""
    leading_edge = int(np.argmin(chord_fractions))
    lower_x, lower_points = chord_fractions[:leading_edge + 1][::-1], section[:leading_edge + 1][::-1]
    upper_x, upper_points = chord_fractions[leading_edge:],           section[leading_edge:]
    lower = np.array([np.interp(hinge_fraction, lower_x, lower_points[:, i]) for i in range(3)])
    upper = np.array([np.interp(hinge_fraction, upper_x, upper_points[:, i]) for i in range(3)])
    return (lower + upper) / 2


def rotate_about_axis(vectors, axis, angle):
    """Vectors rotated by angle about a unit axis, by Rodrigues' rotation formula."""
    return (vectors * np.cos(angle) + np.cross(axis, vectors) * np.sin(angle)
            + np.outer(vectors @ axis, axis) * (1 - np.cos(angle)))
