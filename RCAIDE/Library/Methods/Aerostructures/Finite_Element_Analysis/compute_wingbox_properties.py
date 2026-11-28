# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/compute_wingbox_properties.py
#
# Created: Mar 2026, M. Clarke, S. Sharma
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry, compute_naca_4series

import numpy as np
import warnings

# ----------------------------------------------------------------------
#  Wingbox Assembly Function
# ----------------------------------------------------------------------
def compute_wingbox_properties(wing, discretized_params, spar_cutouts=False):
    """
    Section properties of the wingbox at every element. The box lies inside the wing: its skins follow the upper
    and lower surfaces of the section between the spars, and its webs are as deep as the section is at each spar.

    With spar_cutouts True, each hole in wing.structural.spar_cutouts reduces the section properties of the
    elements it crosses, without adding nodes (see apply_spar_cutouts).

    Returns:
        Data with, for each element,
          A, Ixx, Izz, J   : area [m^2], vertical and chordwise bending inertia and torsion constant [m^4]
          w_box            : width between the spars [m]
          fiber_distance   : largest vertical distance from the centroid to a skin [m]
          fiber_distance_chordwise : largest chordwise distance from the centroid to a spar [m]
          enclosed_area    : area enclosed by the skins and the webs [m^2]
    """
    x, z_upper, z_lower = wingbox_shape(wing, discretized_params.Y_elems, discretized_params.chord_elems,
                                        discretized_params.spar_f_elems, discretized_params.spar_r_elems, discretized_params.total_span)

    section       = section_properties(wing, x, z_upper, z_lower)
    section.w_box = x[:, -1] - x[:, 0]

    if spar_cutouts and len(wing.structural.spar_cutouts) > 0:
        apply_spar_cutouts(wing, discretized_params, x, z_upper, z_lower, section)

    return section


def wingbox_shape(wing, y, chord, front_spar, rear_spar, semi_span, box_points=21, airfoil_points=201):
    """
    Upper and lower surface of the wing between the spars at spanwise stations y [m], where the chord and the
    spar positions (fractions of chord) are given.

    Each wing segment gives the surface at its station: its airfoil if it has one, and otherwise the NACA
    four-digit symmetric section of its thickness-to-chord ratio, the same rule as the geometry plots. The
    surface heights, as fractions of chord, are interpolated linearly along the span between segments.

    box_points is the number of points along each skin between the spars, and airfoil_points the number of
    points on each surface of the airfoil they are read from.

    Returns:
        x       : chordwise position of box_points points from the front to the rear spar, from the leading edge [m]
        z_upper : height of the upper surface at those points, from the chord line [m]
        z_lower : height of the lower surface [m]
        each of shape (len(y), box_points)
    """
    fraction      = np.linspace(0.0, 1.0, box_points)
    segments      = list(wing.segments)
    stations      = np.array([segment.percent_span_location for segment in segments]) * semi_span
    upper_station = np.zeros((len(segments), box_points))
    lower_station = np.zeros((len(segments), box_points))
    for k, segment in enumerate(segments):
        front, rear      = segment.structural.front_spar_percent_chord, segment.structural.rear_spar_percent_chord
        x_over_c         = front + fraction * (rear - front)
        upper_station[k], lower_station[k] = segment_surfaces(wing, segment, x_over_c, airfoil_points)

    y        = np.abs(np.atleast_1d(y))
    upper    = np.array([np.interp(y, stations, upper_station[:, j]) for j in range(box_points)]).T
    lower    = np.array([np.interp(y, stations, lower_station[:, j]) for j in range(box_points)]).T
    chord    = np.atleast_1d(chord)[:, None]
    x        = (np.atleast_1d(front_spar)[:, None] + fraction[None, :] * (np.atleast_1d(rear_spar) - np.atleast_1d(front_spar))[:, None]) * chord
    return x, upper * chord, lower * chord


def segment_surfaces(wing, segment, x_over_c, airfoil_points=201):
    """Heights of the upper and lower surface of a segment's section at chord fractions x_over_c, as fractions of chord."""
    airfoil = segment.airfoil
    if airfoil is None:
        # NACA four-digit thickness distribution, which is linear in the thickness-to-chord ratio
        t_c      = segment.thickness_to_chord if segment.thickness_to_chord > 0 else wing.thickness_to_chord
        geometry = compute_naca_4series('0010', airfoil_points)
        scale    = t_c / 0.10
    elif isinstance(airfoil, RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil):
        geometry = compute_naca_4series(airfoil.NACA_4_Series_code, airfoil_points, thickness_multiplier=airfoil.thickness_multiplier)
        scale    = 1.0
    else:
        geometry = import_airfoil_geometry(airfoil.coordinate_file, airfoil_points, thickness_multiplier=airfoil.thickness_multiplier)
        scale    = 1.0
    x_u, z_u = np.ravel(geometry.x_upper_surface), np.ravel(geometry.y_upper_surface)
    x_l, z_l = np.ravel(geometry.x_lower_surface), np.ravel(geometry.y_lower_surface)
    return scale * np.interp(x_over_c, x_u, z_u), scale * np.interp(x_over_c, x_l, z_l)


def section_properties(wing, x, z_upper, z_lower, cut_front=0.0, cut_rear=0.0, eta_front=1.0, eta_rear=1.0):
    """
    Area, vertical and chordwise bending inertia, and torsion constant of a wingbox section whose skins are the
    polylines (x, z_upper) and (x, z_lower) and whose webs join their ends. The last axis of x, z_upper and
    z_lower runs from the front to the rear spar; a rectangle is x = [0, w], z_upper = [h/2, h/2],
    z_lower = [-h/2, -h/2].

    Both inertias are taken about the centroid of the section. The stringers are smeared into the skin
    thickness for the area and the bending inertias. The skin thickness is laid on the surface, not inside it.

    cut_front/cut_rear are the heights of a centred hole in each spar web, which remove web material from
    the area and inertias. eta_front/eta_rear scale each web's thickness in the torsion constant (Kuhn's
    effective thickness, see kuhn_shear_stiffness_factor). They broadcast against the leading axes.
    """
    structure   = wing.structural
    skin_t_top  = structure.top_skin_thickness    + structure.stringer_smeared_thickness
    skin_t_bot  = structure.bottom_skin_thickness + structure.stringer_smeared_thickness

    x, z_upper, z_lower = np.asarray(x, dtype=float), np.asarray(z_upper, dtype=float), np.asarray(z_lower, dtype=float)
    h_front     = z_upper[..., 0]  - z_lower[..., 0]
    h_rear      = z_upper[..., -1] - z_lower[..., -1]
    z_front     = 0.5 * (z_upper[..., 0]  + z_lower[..., 0])
    z_rear      = 0.5 * (z_upper[..., -1] + z_lower[..., -1])
    x_front     = x[..., 0]
    x_rear      = x[..., -1]

    # 1. Spars: area, vertical inertia about their own mid-height, lateral inertia about their own thickness
    A_fs, I_fs_xx, I_fs_yy = get_spar_properties(structure.front_spar.type, h_front, structure.front_spar.t_web,
                                                 structure.front_spar.w_cap, structure.front_spar.t_cap, cut_front)
    A_rs, I_rs_xx, I_rs_yy = get_spar_properties(structure.rear_spar.type, h_rear, structure.rear_spar.t_web,
                                                 structure.rear_spar.w_cap, structure.rear_spar.t_cap, cut_rear)

    # 2. Skins, as straight strips between the points of each surface, with their moments about x = 0 and z = 0
    top     = skin_strips(x, z_upper, skin_t_top)
    bottom  = skin_strips(x, z_lower, skin_t_bot)
    A_skins = top.area + bottom.area

    # 3. Area and centroid
    A_total = A_fs + A_rs + A_skins
    z_c     = (A_fs * z_front + A_rs * z_rear + top.first_moment_z + bottom.first_moment_z) / A_total
    x_c     = (A_fs * x_front + A_rs * x_rear + top.first_moment_x + bottom.first_moment_x) / A_total

    # 4. Vertical stiffness (Ixx) about the centroid -> resists lift (skins moved from z = 0 by the parallel axis theorem)
    I_skins   = top.second_moment_z + bottom.second_moment_z - 2 * z_c * (top.first_moment_z + bottom.first_moment_z) + z_c**2 * A_skins
    Ixx_total = I_fs_xx + A_fs * (z_front - z_c)**2 + I_rs_xx + A_rs * (z_rear - z_c)**2 + I_skins

    # 5. Chordwise stiffness (Izz) about the centroid -> resists drag: the skins, and each spar's lateral bending
    #    about its own thickness plus its area at its distance from the centroid
    I_skins   = top.second_moment_x + bottom.second_moment_x - 2 * x_c * (top.first_moment_x + bottom.first_moment_x) + x_c**2 * A_skins
    Izz_total = I_fs_yy + A_fs * (x_front - x_c)**2 + I_rs_yy + A_rs * (x_rear - x_c)**2 + I_skins

    # 6. Torsion (J) - Bredt-Batho closed cell
    # The shear flow is carried by the skins and the spar webs. The stringers carry direct stress only, so their
    # smeared thickness is left out here, while it is counted in the area and the bending inertias above.
    Am            = np.sum(0.5 * ((z_upper - z_lower)[..., 1:] + (z_upper - z_lower)[..., :-1]) * np.diff(x, axis=-1), axis=-1)
    integral_ds_t = top.length / structure.top_skin_thickness + bottom.length / structure.bottom_skin_thickness \
                  + h_front / (eta_front * structure.front_spar.t_web) + h_rear / (eta_rear * structure.rear_spar.t_web)
    J_total       = 4 * Am**2 / integral_ds_t

    section = Data()
    section.A              = A_total
    section.Ixx            = Ixx_total
    section.Izz            = Izz_total
    section.J              = J_total
    section.fiber_distance = np.maximum(np.max(z_upper, axis=-1) - z_c, z_c - np.min(z_lower, axis=-1))
    section.fiber_distance_chordwise = np.maximum(x_rear - x_c, x_c - x_front)
    section.enclosed_area  = Am
    return section


def skin_strips(x, z, thickness):
    """
    A skin of the given thickness along the polyline (x, z), whose points run along the last axis. Each strip
    between two points is straight, and its moments are exact.

    Returns a Data with the area, the length, and the first and second moments of area about x = 0 and z = 0.
    """
    x_1, x_2 = x[..., :-1], x[..., 1:]
    z_1, z_2 = z[..., :-1], z[..., 1:]
    ds       = np.sqrt((x_2 - x_1)**2 + (z_2 - z_1)**2)
    dA       = thickness * ds
    # strip thickness about its own midline, for the part of the strip that lies along x
    local    = thickness**3 * (x_2 - x_1)**2 / np.maximum(ds, 1e-12) / 12

    strips = Data()
    strips.area            = np.sum(dA, axis=-1)
    strips.length          = np.sum(ds, axis=-1)
    strips.first_moment_z  = np.sum(dA * 0.5 * (z_1 + z_2), axis=-1)
    strips.first_moment_x  = np.sum(dA * 0.5 * (x_1 + x_2), axis=-1)
    strips.second_moment_z = np.sum(dA * (z_1**2 + z_1 * z_2 + z_2**2) / 3 + local, axis=-1)
    strips.second_moment_x = np.sum(dA * (x_1**2 + x_1 * x_2 + x_2**2) / 3, axis=-1)
    return strips


def apply_spar_cutouts(wing, discretized_params, x, z_upper, z_lower, section, cutout_samples=32):
    """
    Section properties of the elements crossed by circular spar web holes, on the existing nodes. x, z_upper
    and z_lower are the box shape of each element (see wingbox_shape), and section is updated in place.

    Each cutout is a Data with spar ('front' or 'rear'), percent_span_location (hole centre, fraction of the
    semi-span), diameter [m] and bay_length [m], the rib bay that contains the hole (Kuhn's hole spacing b).
    The hole is centred on the web depth. Its projected spanwise extent equals its diameter, as for a hole
    cut by a chordwise cylinder.

    Each crossed element is sampled at cutout_samples points along its span. Inside the hole, the web keeps only the material above
    and below it, which reduces the area and inertias. Over the bay, the web thickness in the torsion
    constant is scaled by Kuhn's factor. The element takes the mean area (mass) and the harmonic mean of
    each inertia and the torsion constant, since flexibilities add in series along the element; this is
    exact when the element's internal moment or torque is uniform.
    """
    semi_span = discretized_params.total_span
    Y_nodes   = discretized_params.Y_nodes
    Y_left    = np.minimum(Y_nodes[:-1], Y_nodes[1:])
    Y_right   = np.maximum(Y_nodes[:-1], Y_nodes[1:])
    n_elem    = len(Y_left)
    web_depth = {'front': z_upper[:, 0] - z_lower[:, 0], 'rear': z_upper[:, -1] - z_lower[:, -1]}

    # spanwise sample stations in each element, midpoint rule
    fractions = (np.arange(cutout_samples) + 0.5) / cutout_samples
    Y_samples = Y_left[:, None] + fractions[None, :] * (Y_right - Y_left)[:, None]

    cut   = {'front': np.zeros((n_elem, cutout_samples)), 'rear': np.zeros((n_elem, cutout_samples))}
    eta   = {'front': np.ones((n_elem, cutout_samples)),  'rear': np.ones((n_elem, cutout_samples))}
    for cutout in wing.structural.spar_cutouts:
        if cutout.spar not in cut:
            raise ValueError(f"Spar cutout spar must be 'front' or 'rear', not '{cutout.spar}'")
        y_c = cutout.percent_span_location * semi_span
        R   = cutout.diameter / 2

        # web depth at the hole centre, from the element containing it
        e_c = int(np.clip(np.searchsorted(Y_right, y_c), 0, n_elem - 1))
        h_c = web_depth[cutout.spar][e_c]
        eta_c = kuhn_shear_stiffness_factor(cutout.diameter, cutout.bay_length, h_c)

        # hole height across the span: the chord of the circle at each station
        s       = (Y_samples - y_c) / R
        in_hole = np.abs(s) < 1.0
        cut[cutout.spar][in_hole] = np.maximum(cut[cutout.spar][in_hole], cutout.diameter * np.sqrt(1.0 - s[in_hole]**2))

        in_bay = np.abs(Y_samples - y_c) < cutout.bay_length / 2
        eta[cutout.spar][in_bay] = np.minimum(eta[cutout.spar][in_bay], eta_c)

    affected = np.any(cut['front'] > 0, axis=1) | np.any(cut['rear'] > 0, axis=1) | \
               np.any(eta['front'] < 1, axis=1) | np.any(eta['rear'] < 1, axis=1)
    if not np.any(affected):
        return

    if np.any(cut['front'][affected] >= web_depth['front'][affected, None]) or np.any(cut['rear'][affected] >= web_depth['rear'][affected, None]):
        raise ValueError("A spar cutout reaches the full web depth; a severed spar needs a reinforcement model")

    # the box shape of each affected element at every sample along it
    sampled = section_properties(wing, x[affected, None, :], z_upper[affected, None, :], z_lower[affected, None, :],
                                 cut['front'][affected], cut['rear'][affected], eta['front'][affected], eta['rear'][affected])

    section.A[affected]   = np.mean(sampled.A, axis=1)
    section.Ixx[affected] = 1.0 / np.mean(1.0 / sampled.Ixx, axis=1)
    section.Izz[affected] = 1.0 / np.mean(1.0 / sampled.Izz, axis=1)
    section.J[affected]   = 1.0 / np.mean(1.0 / sampled.J, axis=1)
    return


def kuhn_shear_stiffness_factor(D, b, h):
    """
    Initial shear stiffness factor of a web with round holes, eta_0 = (1 - D/b) [1 - (D/h)^3].

    Source:
        Kuhn, P., "The Strength and Stiffness of Shear Webs with Round Lightening Holes Having 45 Degree
        Flanges," NACA WR L-323, 1942, Eq. (1). D is the clear hole diameter, b the hole spacing and h the
        web depth; the web's thickness is replaced by eta_0 times its actual thickness. Kuhn's specimens had
        flanged holes and 0.45 <= D/h <= 0.75, 0.43 <= D/b <= 0.64 (Table 1).
    """
    if D >= b:
        raise ValueError(f"Spar cutout diameter {D:.3f} m must be smaller than its bay length {b:.3f} m")
    if D >= h:
        raise ValueError("A spar cutout reaches the full web depth; a severed spar needs a reinforcement model")
    if not (0.45 <= D / h <= 0.75) or not (0.43 <= D / b <= 0.64):
        warnings.warn(f"Spar cutout D/h = {D/h:.2f}, D/b = {D/b:.2f} is outside Kuhn's tested range "
                      f"D/h 0.45 to 0.75, D/b 0.43 to 0.64")
    return (1.0 - D / b) * (1.0 - (D / h)**3)


def get_spar_properties(spar_type, h, t_web, w_cap=0, t_cap=0, cut_height=0.0):
    """
    Calculates Area, Vertical Inertia (Ixx), and Lateral Inertia (Iyy)
    for a spar component. cut_height is a centred hole in the web, which keeps
    only the web material above and below it.

    Returns: A, I_xx (Vertical), I_yy (Lateral/Chordwise)
    """
    h_web = h - cut_height   # web material left at this section
    if spar_type == 'Rectangular':
        # Simple web
        A = h_web * t_web
        I_xx = t_web * (h**3 - cut_height**3) / 12  # Vertical bending
        I_yy = h_web * t_web**3 / 12  # Lateral bending (very weak)

    elif spar_type == 'I_Beam':
        # Web + 2 Caps (Symmetric)
        A_web = h_web * t_web
        A_cap = w_cap * t_cap
        A = A_web + 2 * A_cap

        # Vertical (Ixx): Web centroid + Caps parallel axis
        I_xx_web = t_web * (h**3 - cut_height**3) / 12
        I_xx_caps = 2 * ((w_cap * t_cap**3 / 12) + A_cap * (h/2)**2)
        I_xx = I_xx_web + I_xx_caps

        # Lateral (Iyy): Caps centroid + Web centroid (Symmetric)
        I_yy_web = h_web * t_web**3 / 12
        I_yy_caps = 2 * (t_cap * w_cap**3 / 12) # Caps bending about their own center
        I_yy = I_yy_web + I_yy_caps

    elif spar_type == 'C_Channel':
        # Web + 2 Caps (Asymmetric Laterally)
        A_web = h_web * t_web
        A_cap = w_cap * t_cap
        A = A_web + 2 * A_cap

        # Vertical (Ixx): Symmetric vertically, so same as I-Beam
        I_xx_web = t_web * (h**3 - cut_height**3) / 12
        I_xx_caps = 2 * ((w_cap * t_cap**3 / 12) + A_cap * (h/2)**2)
        I_xx = I_xx_web + I_xx_caps

        # Lateral (Iyy): Asymmetric! Need Centroid Calculation.
        # Reference datum: Back of the web (x = 0)
        # Centroid of Web is at x = t_web / 2
        # Centroid of Caps is at x = w_cap / 2

        moment_area = (A_web * (t_web/2)) + 2 * (A_cap * (w_cap/2))
        x_centroid = moment_area / A

        # Parallel Axis Theorem for Iyy
        # Web Term
        I_yy_web_local = h_web * t_web**3 / 12
        I_yy_web = I_yy_web_local + A_web * (x_centroid - t_web/2)**2

        # Caps Term (x2)
        I_yy_cap_local = t_cap * w_cap**3 / 12
        I_yy_caps = 2 * (I_yy_cap_local + A_cap * (x_centroid - w_cap/2)**2)

        I_yy = I_yy_web + I_yy_caps

    else:
        A = 0; I_xx = 0; I_yy = 0

    return A, I_xx, I_yy

def get_skin_properties(width, thickness, dist_from_center):
    """
    Calculates Area and contribution to Ixx using Parallel Axis Theorem.
    """
    A = width * thickness
    I_local = width * thickness**3 / 12
    I_parallel = A * dist_from_center**2
    # Total I contribution
    I_total = I_local + I_parallel
    return A, I_total
