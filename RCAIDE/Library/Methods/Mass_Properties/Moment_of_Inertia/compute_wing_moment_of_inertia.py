# RCAIDE/Library/Methods/Mass_Properties/Moment_of_Inertia/compute_wing_moment_of_inertia.py
#
# Created:  September 2023, A. Molloy

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# package imports
import numpy as np

# NACA 4-digit thickness distribution constants [1]
a0 = 2.969
a1 = -1.260
a2 = -3.516
a3 = 2.843
a4 = -1.015

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Wing Moment of Inertia
# ----------------------------------------------------------------------------------------------------------------------
def compute_wing_moment_of_inertia(wing, center_of_gravity = [[0, 0, 0]]):
    ''' computes the moment of inertia tensor of a wing about the wing center of gravity.

    Assumptions:
    - Wing is solid
    - Wing has a constant density
    - NACA 4-digit thickness distribution, no twist or camber
    - Wing mass is split between segments in proportion to segment volume

    Source:
    [1] Moulton, B. C., and Hunsaker, D. F., “Simplified Mass and Inertial Estimates for Aircraft with Components
    of Constant Density,” AIAA SCITECH 2023 Forum, January 2023, AIAA-2023-2432 DOI: 10.2514/
    6.2023-2432

    Inputs:
    - Wing

    Outputs:
    - wing moment of inertia tensor about the wing center of gravity, in the wing geometry frame
      (x aft, y right, z up)
    - wing mass

    Properties Used:
    N/A
    '''
    xz_symm  = wing.xz_plane_symmetric
    mass     = wing.mass_properties.mass
    vertical = wing.vertical
    span     = wing.spans.projected

    # geometry of each wing section: tr, tt, cr, ct, b, sweep, dihedral, root leading edge
    sections = []
    if len(wing.segments) > 1:
        seg_keys = list(wing.segments.keys())
        for i in range(len(wing.segments)-1):
            inner_segment = wing.segments[seg_keys[i]]
            outer_segment = wing.segments[seg_keys[i+1]]
            b = span * (outer_segment.percent_span_location - inner_segment.percent_span_location)/(1+xz_symm)
            if b <= 0:
                continue
            sections.append([inner_segment.thickness_to_chord,
                             outer_segment.thickness_to_chord,
                             wing.chords.root * inner_segment.root_chord_percent,
                             wing.chords.root * outer_segment.root_chord_percent,
                             b,
                             inner_segment.sweeps.quarter_chord,
                             inner_segment.dihedral_outboard,
                             np.array(inner_segment.origin[0], dtype=float)])
    else:
        sections.append([wing.thickness_to_chord,
                         wing.thickness_to_chord,
                         wing.chords.root,
                         wing.chords.tip,
                         span/(1+xz_symm),
                         wing.sweeps.quarter_chord,
                         wing.dihedral,
                         np.zeros(3)])

    # section mass from section volume, Eq. (45) of [1]
    volumes     = np.array([compute_wing_section_volume(s[0], s[1], s[2], s[3], s[4]) for s in sections])
    half_mass   = mass / (1 + xz_symm)
    sides       = [1, -1] if xz_symm else [1]

    section_masses = []
    section_cgs    = []
    section_I      = []
    for section, volume in zip(sections, volumes):
        tr, tt, cr, ct, b, sweep, dihedral, root_le = section
        m_section = half_mass * volume / np.sum(volumes)
        for delta in sides:
            I_cg, r_cg = compute_wing_section_moment_of_intertia(m_section, tr, tt, cr, ct, b, sweep, dihedral, delta, vertical, root_le)
            section_masses.append(m_section)
            section_cgs.append(r_cg)
            section_I.append(I_cg)

    # translate each section to the wing center of gravity and sum, Eq. (116) of [1]
    section_masses = np.array(section_masses)
    section_cgs    = np.array(section_cgs)
    wing_cg        = np.sum(section_masses[:, None] * section_cgs, axis=0) / np.sum(section_masses)
    I_local        = np.zeros((3, 3))
    for m_section, r_cg, I_cg in zip(section_masses, section_cgs, section_I):
        s        = r_cg - wing_cg
        I_local += I_cg + m_section * (np.dot(s, s) * np.identity(3) - np.outer(s, s))

    # Store moment of inertia tensor on component
    wing.mass_properties.moments_of_inertia.tensor  = I_local

    return I_local,  mass


def compute_wing_section_volume(tr, tt, cr, ct, b):
    ''' volume of a wing section with a NACA 4-digit thickness distribution, Eq. (45) of [1] '''
    v0 = 1 / 60 * (40 * a0 + 30 * a1 + 20 * a2 + 15 * a3 + 12 * a4)
    ka = tr * (3 * cr ** 2 + 2 * cr * ct + ct ** 2) + tt * (cr ** 2 + 2 * cr * ct + 3 * ct ** 2)
    return b * ka * v0 / 12


def compute_wing_section_moment_of_intertia(m_wing, tr, tt, cr, ct, b, A, dihedral, delta, vertical, root_le):
    ''' inertia tensor of one wing section (one side) about its center of gravity, and the location of that
    center of gravity, both in the wing geometry frame (x aft, y right, z up).

    Inputs:
    - m_wing   : section mass (one side)
    - tr, tt   : root and tip thickness-to-chord ratios
    - cr, ct   : root and tip chords
    - b        : section span (one side)
    - A        : quarter-chord sweep
    - dihedral : section dihedral
    - delta    : 1 for a right wing, -1 for a left wing
    - vertical : True for a vertical surface
    - root_le  : section root leading edge relative to the wing origin (right side)
    '''
    # ----------------------------------------------------------------------------------------------------------------------
    # Constants. These values and equations are defined in Moulton and Hunsaker [1]
    # ----------------------------------------------------------------------------------------------------------------------
    ka = tr * (3 * cr ** 2 + 2 * cr * ct + ct ** 2) + tt * (cr ** 2 + 2 * cr * ct + 3 * ct ** 2)
    kb = (tr * (4 * cr ** 3 + 3 * cr ** 2 * ct + 2 * cr * ct ** 2 + ct ** 3)
          + tt * (cr ** 3 + 2 * cr ** 2 * ct + 3 * cr * ct ** 2 + 4 * ct ** 3))
    kc = tr * (3 * cr ** 2 + 4 * cr * ct + 3 * ct ** 2) + 2 * tt * (cr ** 2 + 3 * cr * ct + 6 * ct ** 2)
    kd = (tr * (ct + cr) * (2 * cr ** 2 + cr * ct + 2 * ct ** 2)
          + tt * (cr ** 3 + 3 * cr ** 2 * ct + 6 * cr * ct ** 2 + 10 * ct ** 3))
    ke = (tr * (5 * cr ** 4 + 4 * cr ** 3 * ct + 3 * cr ** 2 * ct ** 2 + 2 * cr * ct ** 3 + ct ** 4)
          + tt * (cr ** 4 + 2 * cr ** 3 * ct + 3 * cr ** 2 * ct ** 2 + 4 * cr * ct ** 3 + 5 * ct ** 4))
    kf = (tr * (cr ** 2 + 2 * cr * ct + 2 * ct ** 2) + tt * (cr ** 2 + 4 * cr * ct
                                                             + 10 * ct ** 2))
    kg = (tr ** 3 * (35 * cr ** 4 + 20 * cr ** 3 * ct + 10 * cr ** 2 * ct ** 2 + 4 * cr * ct ** 3 + ct ** 4)
          + tr ** 2 * tt * (15 * cr ** 4 + 20 * cr ** 3 * ct + 18 * cr ** 2 * ct ** 2 + 12 * cr * ct ** 3 + 5 * ct ** 4)
          + tr * tt ** 2 * (5 * cr ** 4 + 12 * cr ** 3 * ct + 18 * cr ** 2 * ct ** 2 + 20 * cr * ct ** 3 + 15 * ct ** 4)
          + tt ** 3 * (cr ** 4 + 4 * cr ** 3 * ct + 10 * cr ** 2 * ct ** 2 + 20 * cr * ct ** 3 + 35 * ct ** 4))

    v0 = 1 / 60 * (40 * a0 + 30 * a1 + 20 * a2 + 15 * a3 + 12 * a4) # NACA 4 digit integral of thickness distribution.
    v1 = 1 / 60 * (56 * a0 + 50 * a1 + 40 * a2 + 33 * a3 + 28 * a4)
    v2 = 1 / 980 * (856 * a0 + 770 * a1 + 644 * a2 + 553 * a3 + 484 * a4)
    v3 = (2 / 5 * a0 ** 3 + a0 ** 2 * a1 + 3 / 4 * a0 ** 2 * a2 + 3 / 5 * a0 ** 2 * a3 + 1 / 2 * a0 ** 2 * a4 + 6 / 7 * a0 * a1 ** 2
          + 4 / 3 * a0 * a1 * a2 + 12 / 11 * a0 * a1 * a3 + 12 / 13 * a0 * a1 * a4 + 6 / 11 * a0 * a2 ** 2 + 12 / 13 * a0 * a2 * a3
          + 4 / 5 * a0 * a2 * a4 + 2 / 5 * a0 * a3 ** 2 + 12 / 17 * a0 * a3 * a4 + 6 / 19 * a0 * a4 ** 2 + 1 / 4 * a1 ** 3
          + 3 / 5 * a1 ** 2 * a2 + 1 / 2 * a1 ** 2 * a3 + 3 / 7 * a1 ** 2 * a4 + 1 / 2 * a1 * a2 ** 2 + 6 / 7 * a1 * a2 * a3
          + 3 / 4 * a1 * a2 * a4 + 3 / 8 * a1 * a3 ** 2 + 2 / 3 * a1 * a3 * a4 + 3 / 10 * a1 * a4 ** 2 + 1 / 7 * a2 ** 3
          + 3 / 8 * a2 ** 2 * a3 + 1 / 3 * a2 ** 2 * a4 + 1 / 3 * a2 * a3 ** 2 + 3 / 5 * a2 * a3 * a4 + 3 / 11 * a2 * a4 ** 2
          + 1 / 10 * a3 ** 3 + 3 / 11 * a3 ** 2 * a4 + 1 / 4 * a3 * a4 ** 2 + 1 / 13 * a4 ** 3)

    # ----------------------------------------------------------------------------------------------------------------------
    # Inertia about the section origin (root quarter chord) in the section frame (x forward, y spanwise, z down), Eqs. (77)-(81)
    # ----------------------------------------------------------------------------------------------------------------------
    Ixx   = m_wing * (56 * b ** 2 * kf * v0 + kg * v3) / (280 * ka * v0)
    Iyy   = m_wing * (84 * b * (2 * b * kf * v0 * np.tan(A) ** 2 + kd * v1 * np.tan(A)) + 49 * ke * v2 + 3 * kg * v3) / (840 * ka * v0)
    Izz   = m_wing * (12 * b * (2 * b * (np.tan(A) ** 2 + 1) * kf * v0 + kd * v1 * np.tan(A)) + 7 * ke * v2) / (120 * ka * v0)
    Ixy   = -1 * delta * b * m_wing * (4 * b * kf * v0 * np.tan(A) + kd * v1) / (20 * ka * v0)
    I_o   = np.array([[Ixx, -Ixy, 0], [-Ixy, Iyy, 0], [0, 0, Izz]])

    # center of gravity relative to the section origin, Eqs. (58)-(60)
    x_bar = -(3 * kb * v1 + 4 * b * kc * v0 * np.tan(A)) / (20 * ka * v0)
    y_bar = delta * b * kc / (5 * ka)
    s     = np.array([x_bar, y_bar, 0.0])

    # shift from the section origin to the section center of gravity, Eq. (83)
    I_cg  = I_o - m_wing * (np.dot(s, s) * np.identity(3) - np.outer(s, s))

    # ----------------------------------------------------------------------------------------------------------------------
    # Rotation to the wing geometry frame (x aft, y right, z up), Eq. (115)
    # ----------------------------------------------------------------------------------------------------------------------
    # dihedral: -dihedral for a right wing, +dihedral for a left wing (section z axis points down)
    gamma = -delta * dihedral
    R_dihedral = np.array([[1, 0, 0], [0, np.cos(gamma), -np.sin(gamma)], [0, np.sin(gamma), np.cos(gamma)]])
    if vertical:
        # section span to +z, section thickness to y
        R_frame = np.array([[-1, 0, 0], [0, 0, 1], [0, 1, 0]])
    else:
        R_frame = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
    R = R_frame @ R_dihedral

    I_section = R @ I_cg @ R.T

    # section center of gravity: root quarter chord plus the rotated offset
    root_qc    = np.array([root_le[0] + 0.25 * cr, delta * root_le[1], root_le[2]])
    r_cg       = root_qc + R @ s

    return I_section, r_cg
