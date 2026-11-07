# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/compute_wingbox_properties.py
#
# Created: Mar 2026, M. Clarke, S. Sharma
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np
import warnings

# Kuhn's tested ranges for the shear stiffness factor (NACA WR L-323, Table 1)
KUHN_D_OVER_H_RANGE = (0.45, 0.75)
KUHN_D_OVER_B_RANGE = (0.43, 0.64)

# sample points per element when integrating a cutout along the span
CUTOUT_SAMPLES = 32

# ----------------------------------------------------------------------
#  Wingbox Assembly Function
# ----------------------------------------------------------------------
def compute_wingbox_properties(wing, discretized_params, spar_cutouts=False):
    """
    Assembles the wingbox from components at every element.

    With spar_cutouts True, each hole in wing.structural.spar_cutouts reduces the section properties of the
    elements it crosses, without adding nodes (see apply_spar_cutouts).
    """
    # 1. Extract the element arrays
    chord_arr   = discretized_params.chord_elems
    t_c_arr     = discretized_params.t_c_elems
    spar_f_loc  = discretized_params.spar_f_elems
    spar_r_loc  = discretized_params.spar_r_elems

    # 2. Geometry Arrays
    h_arr       = chord_arr * t_c_arr
    w_box_arr   = chord_arr * (spar_r_loc - spar_f_loc)

    A_total, Ixx_total, Izz_total, J_total = section_properties(wing, h_arr, w_box_arr)

    if spar_cutouts and len(wing.structural.spar_cutouts) > 0:
        A_total, Ixx_total, Izz_total, J_total = apply_spar_cutouts(wing, discretized_params, h_arr, w_box_arr,
                                                                    A_total, Ixx_total, Izz_total, J_total)

    return A_total, Ixx_total, Izz_total, J_total, w_box_arr


def section_properties(wing, h_arr, w_box_arr, cut_front=0.0, cut_rear=0.0, eta_front=1.0, eta_rear=1.0):
    """
    Area, vertical and chordwise bending inertia, and torsion constant of the wingbox section.

    cut_front/cut_rear are the heights of a centred hole in each spar web, which remove web material from
    the area and inertias. eta_front/eta_rear scale each web's thickness in the torsion constant (Kuhn's
    effective thickness, see kuhn_shear_stiffness_factor). All arguments broadcast together.
    """
    skin_t_top  = wing.structural.top_skin_thickness    + wing.structural.stringer_smeared_thickness
    skin_t_bot  = wing.structural.bottom_skin_thickness + wing.structural.stringer_smeared_thickness

    # 1. Front Spar (Get Ixx and Iyy)
    A_fs, I_fs_xx, I_fs_yy = get_spar_properties(wing.structural.front_spar.type,
                                                h_arr,
                                                wing.structural.front_spar.t_web,
                                                wing.structural.front_spar.w_cap ,
                                                wing.structural.front_spar.t_cap,
                                                cut_front)

    # 2. Rear Spar (Get Ixx and Iyy)
    A_rs, I_rs_xx, I_rs_yy = get_spar_properties(wing.structural.rear_spar.type,
                                                h_arr,
                                                wing.structural.rear_spar.t_web,
                                                wing.structural.rear_spar.w_cap,
                                                wing.structural.rear_spar.t_cap,
                                                cut_rear)

    # 3. Skins
    # Top Skin
    A_sk_top, I_sk_top_xx = get_skin_properties(w_box_arr, skin_t_top, h_arr/2)
    # Bottom Skin
    A_sk_bot, I_sk_bot_xx = get_skin_properties(w_box_arr, skin_t_bot, h_arr/2)

    # 4. Total Vertical Stiffness (Ixx) -> Resists Lift
    Ixx_total = I_fs_xx + I_rs_xx + I_sk_top_xx + I_sk_bot_xx

    # 5. Total Chordwise Stiffness (Izz) -> Resists Drag
    # Plus the "Lateral Bending" (Iyy) of the Spars

    # Skin contribution (Deep Beam approximation: 2 * t * w^3 / 12)
    I_skins_chordwise = (skin_t_top * w_box_arr**3 / 12) + (skin_t_bot * w_box_arr**3 / 12)

    # Total Izz (Note: Spar local Iyy acts in the Global Chordwise direction)
    Izz_total = I_skins_chordwise + I_fs_yy + I_rs_yy

    # 6. Torsion (J) - Bredt-Batho Closed Cell
    Am = w_box_arr * h_arr
    integral_ds_t = (w_box_arr / skin_t_top) + \
                    (w_box_arr / skin_t_bot) + \
                    (h_arr / (eta_front * wing.structural.front_spar.t_web)) + \
                    (h_arr / (eta_rear  * wing.structural.rear_spar.t_web))

    J_total = 4 * Am**2 / integral_ds_t

    # 7. Total Area
    A_total = A_fs + A_rs + A_sk_top + A_sk_bot

    return A_total, Ixx_total, Izz_total, J_total


def apply_spar_cutouts(wing, discretized_params, h_arr, w_box_arr, A_total, Ixx_total, Izz_total, J_total):
    """
    Section properties of the elements crossed by circular spar web holes, on the existing nodes.

    Each cutout is a Data with spar ('front' or 'rear'), percent_span_location (hole centre, fraction of the
    semi-span), diameter [m] and bay_length [m], the rib bay that contains the hole (Kuhn's hole spacing b).
    The hole is centred on the web depth. Its projected spanwise extent equals its diameter, as for a hole
    cut by a chordwise cylinder.

    Each crossed element is sampled along its span. Inside the hole, the web keeps only the material above
    and below it, which reduces the area and inertias. Over the bay, the web thickness in the torsion
    constant is scaled by Kuhn's factor. The element takes the mean area (mass) and the harmonic mean of
    each inertia and the torsion constant, since flexibilities add in series along the element; this is
    exact when the element's internal moment or torque is uniform.
    """
    semi_span = discretized_params.total_span
    Y_nodes   = discretized_params.Y_nodes
    Y_left    = np.minimum(Y_nodes[:-1], Y_nodes[1:])
    Y_right   = np.maximum(Y_nodes[:-1], Y_nodes[1:])
    n_elem    = len(h_arr)

    # spanwise sample stations in each element, midpoint rule
    fractions = (np.arange(CUTOUT_SAMPLES) + 0.5) / CUTOUT_SAMPLES
    Y_samples = Y_left[:, None] + fractions[None, :] * (Y_right - Y_left)[:, None]

    cut   = {'front': np.zeros((n_elem, CUTOUT_SAMPLES)), 'rear': np.zeros((n_elem, CUTOUT_SAMPLES))}
    eta   = {'front': np.ones((n_elem, CUTOUT_SAMPLES)),  'rear': np.ones((n_elem, CUTOUT_SAMPLES))}
    for cutout in wing.structural.spar_cutouts:
        if cutout.spar not in cut:
            raise ValueError(f"Spar cutout spar must be 'front' or 'rear', not '{cutout.spar}'")
        y_c = cutout.percent_span_location * semi_span
        R   = cutout.diameter / 2

        # web depth at the hole centre, from the element containing it
        e_c = int(np.clip(np.searchsorted(Y_right, y_c), 0, n_elem - 1))
        h_c = h_arr[e_c]
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
        return A_total, Ixx_total, Izz_total, J_total

    if np.any(cut['front'][affected] >= h_arr[affected, None]) or np.any(cut['rear'][affected] >= h_arr[affected, None]):
        raise ValueError("A spar cutout reaches the full web depth; a severed spar needs a reinforcement model")

    h_s = np.repeat(h_arr[affected, None], CUTOUT_SAMPLES, axis=1)
    w_s = np.repeat(w_box_arr[affected, None], CUTOUT_SAMPLES, axis=1)
    A_s, Ixx_s, Izz_s, J_s = section_properties(wing, h_s, w_s, cut['front'][affected], cut['rear'][affected],
                                                eta['front'][affected], eta['rear'][affected])

    A_total, Ixx_total, Izz_total, J_total = A_total.copy(), Ixx_total.copy(), Izz_total.copy(), J_total.copy()
    A_total[affected]   = np.mean(A_s, axis=1)
    Ixx_total[affected] = 1.0 / np.mean(1.0 / Ixx_s, axis=1)
    Izz_total[affected] = 1.0 / np.mean(1.0 / Izz_s, axis=1)
    J_total[affected]   = 1.0 / np.mean(1.0 / J_s, axis=1)
    return A_total, Ixx_total, Izz_total, J_total


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
    if not (KUHN_D_OVER_H_RANGE[0] <= D / h <= KUHN_D_OVER_H_RANGE[1]) or not (KUHN_D_OVER_B_RANGE[0] <= D / b <= KUHN_D_OVER_B_RANGE[1]):
        warnings.warn(f"Spar cutout D/h = {D/h:.2f}, D/b = {D/b:.2f} is outside Kuhn's tested range "
                      f"D/h {KUHN_D_OVER_H_RANGE}, D/b {KUHN_D_OVER_B_RANGE}")
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
