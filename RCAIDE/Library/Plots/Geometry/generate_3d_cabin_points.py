# RCAIDE/Library/Plots/Geometry/generate_3d_cabin_points.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data     
from RCAIDE.Library.Methods.Geometry.Airfoil                import  compute_naca_4series, import_airfoil_geometry  

import numpy as np     

# ----------------------------------------------------------------------------------------------------------------------
#  generate_3d_cabin_points
# ----------------------------------------------------------------------------------------------------------------------   
def generate_3d_cabin_points(component, n_points,plot_centerline = False):
    """
    Generates 3D coordinate points that define a cabin surface.

    Parameters
    ----------
    cabin : Cabin
        RCAIDE cabin data structure containing geometry information
    component : Component
        RCAIDE component data structure containing geometry information 
    n_points : int
        Number of points used to discretize airfoil sections
        
    dim : int
        Number of wing segments plus one

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

    if issubclass(type(component), RCAIDE.Library.Components.Wings.Wing): 
        G = generate_3d_wing_cabin_points(component, n_points, plot_centerline)
    else:
        G = generate_3d_fuselage_cabin_points(component, n_points, plot_centerline)

    return G

def generate_3d_wing_cabin_points(wing, n_points, plot_centerline=False):

    # LOPA gives the seat layout; col 2=X (chord), col 3=Y (span), col 4=Z, col 5=seat pitch in X
    LOPA_data            = wing.layout_of_passenger_accommodations
    LOPA                 = LOPA_data.object_coordinates
    z_min                = LOPA[:, 4].min()        # floor Z: lowest seat position used to cut the semi-cylinder
    cabin_factor         = wing.outer_mold_line_cabin_offset_factor

    # Slice the ordered segment list between the two bounding tags (case-insensitive, RCAIDE lowercases tags).
    # segments_bounding_cabin defaults to [] on Cabin() and must be set explicitly by vehicle_setup();
    # skip cabin geometry entirely (return None) rather than crash on an undefined boundary.
    bounding_tags        = list(wing.cabins.values())[0].segments_bounding_cabin
    if len(bounding_tags) < 2:
        return None
    all_segs             = list(wing.segments)
    seg_tags             = [s.tag.lower() for s in all_segs]
    i_start              = seg_tags.index(bounding_tags[0].lower())
    i_end                = seg_tags.index(bounding_tags[-1].lower())
    cabin_segs           = all_segs[i_start:i_end + 1]

    # Global X (chordwise) extent of the cabin floor plan — used to trim sections fore and aft
    x_cabin_min = np.min(LOPA[:, 2] - LOPA[:, 5] / 2) + LOPA_data.origin[0][0]
    x_cabin_max = np.max(LOPA[:, 2] + LOPA[:, 5] / 2) + LOPA_data.origin[0][0]

    n_segments = len(cabin_segs)
    origin     = wing.origin
    semispan   = wing.spans.projected / (1 + wing.xz_plane_symmetric)

    # pts holds local-frame section coordinates; section_twist is initialised to identity
    pts           = np.zeros((n_segments+2, n_points, 3, 1))
    section_twist = np.zeros((n_segments+2, n_points, 3, 3))
    section_twist[:, :, 0, 0] = 1
    section_twist[:, :, 1, 1] = 1
    section_twist[:, :, 2, 2] = 1
    translation = np.zeros((n_segments+2, n_points, 3, 1))
    translation[:, :, 0, :] = origin[0][0]
    translation[:, :, 1, :] = origin[0][1]
    translation[:, :, 2, :] = origin[0][2]

    for i in range(n_segments+2):
        # i=0 and i=n_segments+1 are flat end-caps that reuse the first/last segment's shape
        if i == 0:
            current_seg = list(cabin_segs)[0]
        elif i == n_segments + 1:
            current_seg = list(cabin_segs)[n_segments - 1]
        else:
            current_seg = list(cabin_segs)[i-1]

        airfoil = current_seg.airfoil
        if airfoil is not None:
            if type(airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                geometry = compute_naca_4series(airfoil.NACA_4_Series_code, n_points)
            elif type(airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil:
                geometry = import_airfoil_geometry(airfoil.coordinate_file, n_points)
        else:
            t_c      = str(int(current_seg.thickness_to_chord * 100)).zfill(4)
            geometry = compute_naca_4series(t_c, n_points)

        twist      = current_seg.twist
        # end-caps average upper and lower surfaces to produce a flat closing face
        is_end_cap = (i == 0 or i == n_segments + 1)

        # shrink thickness by cabin_factor so the surface sits inside the outer mold line
        geometry.y_coordinates *= cabin_factor

        if wing.vertical:
            pts[i,:,0,0] = geometry.x_coordinates * current_seg.root_chord_percent * wing.chords.root
            pts[i,:,1,0] = (geometry.y_coordinates + geometry.y_coordinates[::-1]) / 2 if is_end_cap else geometry.y_coordinates * current_seg.root_chord_percent * wing.chords.root
            pts[i,:,2,0] = np.zeros_like(geometry.y_coordinates)
            section_twist[i,:,0,0] =  np.cos(twist)
            section_twist[i,:,0,1] = -np.sin(twist)
            section_twist[i,:,1,0] =  np.sin(twist)
            section_twist[i,:,1,1] =  np.cos(twist)
        else:
            pts[i,:,0,0] = geometry.x_coordinates * current_seg.root_chord_percent * wing.chords.root
            pts[i,:,1,0] = np.zeros_like(geometry.y_coordinates)
            pts[i,:,2,0] = (geometry.y_coordinates + geometry.y_coordinates[::-1]) / 2 if is_end_cap else geometry.y_coordinates * current_seg.root_chord_percent * wing.chords.root
            section_twist[i,:,0,0] =  np.cos(twist)
            section_twist[i,:,0,2] =  np.sin(twist)
            section_twist[i,:,2,0] = -np.sin(twist)
            section_twist[i,:,2,2] =  np.cos(twist)

        # tip end-cap has no stored origin — its translation is copied from the outermost section below
        if i != n_segments + 1:
            translation[i, :, 0, :] += current_seg.origin[0][0]
            translation[i, :, 1, :] += current_seg.origin[0][1]
            translation[i, :, 2, :] += current_seg.origin[0][2]

        if i == n_segments:
            # outermost section has no segment origin past the last stored one;
            # compute its position by projecting from the previous section using sweep and dihedral
            prev_seg     = list(cabin_segs)[i-2]
            sweep        = prev_seg.sweeps.leading_edge
            dihedral     = prev_seg.dihedral_outboard
            segment_percent_span = current_seg.percent_span_location - prev_seg.percent_span_location
            if wing.vertical:
                dz = semispan * segment_percent_span
                dy = dz * np.tan(dihedral)
                l  = dz / np.cos(dihedral)
                dx = l  * np.tan(sweep)
            else:
                dy = semispan * segment_percent_span
                dz = dy * np.tan(dihedral)
                l  = dy / np.cos(dihedral)
                dx = l  * np.tan(sweep)
            translation[i,:,0,:] = translation[i-1,:,0,:] + dx
            translation[i,:,1,:] = translation[i-1,:,1,:] + dy
            translation[i,:,2,:] = translation[i-1,:,2,:] + dz

        elif i == n_segments + 1:
            # tip end-cap sits at the same position as the outermost section
            translation[i,:,:,:] = translation[i-1,:,:,:]

    mat = translation + np.matmul(section_twist, pts)

    # Trim chord (X) to the fore/aft extent of the cabin floor plan
    mat[:, :, 0, 0] = np.clip(mat[:, :, 0, 0], x_cabin_min, x_cabin_max)

    # Cut the bottom of the cabin at the lowest seat Z, forming a semi-cylinder
    mat[:, :, 2] = np.maximum(mat[:, :, 2], z_min)

    if not plot_centerline:
        mat = mat[1:, :, :, :]

    G      = Data()
    G.X    = mat[:,:,0,0]
    G.Y    = mat[:,:,1,0]
    G.Z    = mat[:,:,2,0]
    G.PTS  = mat[:,:,:,0]
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
     
 
def generate_3d_fuselage_cabin_points(fuselage, n_points, plot_centerline=False):
    cabin_factor  = getattr(fuselage, 'outer_mold_line_cabin_offset_factor', 0.95)

    # Slice the ordered segment list between the two bounding tags (case-insensitive, RCAIDE lowercases tags).
    # segments_bounding_cabin defaults to [] on Cabin() and must be set explicitly by vehicle_setup();
    # skip cabin geometry entirely (return None) rather than crash on an undefined boundary.
    bounding_tags = list(fuselage.cabins.values())[0].segments_bounding_cabin
    if len(bounding_tags) < 2:
        return None
    all_segs      = list(fuselage.segments)
    seg_tags      = [s.tag.lower() for s in all_segs]
    i_start       = seg_tags.index(bounding_tags[0].lower())
    i_end         = seg_tags.index(bounding_tags[-1].lower())
    cabin_segs    = all_segs[i_start:i_end + 1]

    LOPA_data = fuselage.layout_of_passenger_accommodations
    LOPA      = LOPA_data.object_coordinates
    z_min     = LOPA[:, 4].min()   # floor Z: lowest seat position used to cut the semi-cylinder

    theta        = np.linspace(0, 2 * np.pi, n_points)
    cabin_points = np.zeros((len(cabin_segs), n_points, 3))

    for i, segment in enumerate(cabin_segs):
        a = segment.width  / 2 * cabin_factor   # semi-axis in Y
        b = segment.height / 2 * cabin_factor   # semi-axis in Z
        n = segment.curvature                    # superellipse exponent (n=2 → ellipse, n→∞ → rectangle)

        cabin_points[i, :, 0] = (segment.percent_x_location * fuselage.lengths.total
                                  + fuselage.origin[0][0])
        # superellipse: r(θ) = |cos θ|^(2/n) * a * sign(cos θ)  — generalises ellipse to squarer shapes
        cabin_points[i, :, 1] = ((abs(np.cos(theta)) ** (2 / n)) * a
                                  * ((np.cos(theta) > 0) * 1 - (np.cos(theta) < 0) * 1)
                                  + segment.percent_y_location * fuselage.lengths.total
                                  + fuselage.origin[0][1])
        cabin_points[i, :, 2] = ((abs(np.sin(theta)) ** (2 / n)) * b
                                  * ((np.sin(theta) > 0) * 1 - (np.sin(theta) < 0) * 1)
                                  + segment.percent_z_location * fuselage.lengths.total
                                  + fuselage.origin[0][2])

    # Cut the bottom of the cabin at the lowest seat Z, forming a semi-cylinder
    cabin_points[:, :, 2] = np.maximum(cabin_points[:, :, 2], z_min)

    G     = Data()
    G.PTS = cabin_points
    return G