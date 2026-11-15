# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/postprocess_vortex_distribution.py
# 
# Created:  Aug 2025, M. Clarke
#           Apr 2026, M. Clarke 
# Modified: Nov 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------------------------------------------------------

# package imports 
import numpy as np
from RCAIDE.Framework.Core import array_namespace

# panel corners and points of the lattice that the derived quantities are computed from
PANEL_FIELDS = ('XA1','YA1','ZA1','XA2','YA2','ZA2','XB1','YB1','ZB1','XB2','YB2','ZB2','XAH','YAH','ZAH','XBH','YBH','ZBH',
                'XAC','YAC','ZAC','XBC','YBC','ZBC','XC','YC','ZC')

# ----------------------------------------------------------------------------------------------------------------------
#  postprocess_vortex_distribution
# ----------------------------------------------------------------------------------------------------------------------
def postprocess_vortex_distribution(VD, settings):
    """ 
    Recomputes data about the VD. Should be called any time VD panel 
    values (e.g. VD.XA1, VD.XCH, etc) are changed.

    Assumptions: 
      - the last VLM_wing in VD.VLM_wings is last to get discretized and thus has the highest surface_ID

    Source:  


    Inputs: 
    VD       - vehicle vortex distribution                    [Unitless] 
    settings - Vortex_Lattice.settings                        [Unitless] 
    
    Outputs:      
    VD       - vehicle vortex distribution                    [Unitless] 


    Properties Used:
    N/A
    """
    xp         = array_namespace(*[VD[name] for name in PANEL_FIELDS])
    LE_ind     = VD.leading_edge_indices
    TE_ind     = VD.trailing_edge_indices

    # the first and last panel of each chordwise strip, and the strip each panel belongs to
    leading_edge   = np.flatnonzero(LE_ind)
    trailing_edge  = np.flatnonzero(TE_ind)
    strip_n_cw     = VD.panels_per_strip[LE_ind]
    strip_of_panel = np.repeat(np.arange(len(leading_edge)), strip_n_cw)

    VD.normals     = compute_unit_normal(VD)

    # Compute Panel Slopes
    X1c   = (VD.XA1+VD.XB1)/2
    X2c   = (VD.XA2+VD.XB2)/2
    Z1c   = (VD.ZA1+VD.ZB1)/2
    Z2c   = (VD.ZA2+VD.ZB2)/2
    SLOPE = (Z2c - Z1c)/(X2c - X1c)
    D     = xp.sqrt((VD.YAH-VD.YBH)**2+(VD.ZAH-VD.ZBH)**2)[leading_edge]

    # Compute Panel Sweeps
    panel_sweeps = arctangent_of_ratio(VD.XA1-VD.XB1, VD.YA1-VD.YB1)
    panel_sweeps = xp.where(VD.YA1 > VD.YB1, -panel_sweeps, panel_sweeps)
    VD.leading_edge_sweeps =  panel_sweeps[leading_edge]

    # Compute Panel Chord Widths
    Del_Y =  xp.abs(VD.YB1-VD.YA1)
    VD.chord_widths  = Del_Y[leading_edge]

    # Chord of each strip and its incidence, repeated for every panel of the strip
    LE_X           = X1c[leading_edge]
    LE_Z           = Z1c[leading_edge]
    TE_X           = X2c[trailing_edge]
    TE_Z           = Z2c[trailing_edge]
    tan_incidence  = ((LE_Z-TE_Z)/(LE_X-TE_X))[strip_of_panel] # ZETA  in vorlax
    strip_chords   = xp.sqrt((TE_X-LE_X)**2 + (TE_Z-LE_Z)**2)
    chord_adjusted = strip_chords[strip_of_panel] # CHORD in vorlax

    # trailing edge of the strip of each panel
    trailing_edge_of_panel = trailing_edge[strip_of_panel]

    VD.SLOPE                   = SLOPE
    VD.D                       = D
    VD.tangent_incidence_angle = tan_incidence
    VD.chord_lengths           = xp.atleast_2d(chord_adjusted)   # per panel
    VD.strip_chord_lengths     = strip_chords                    # per spanwise strip
    VD.Y_SW                    = VD.YC[trailing_edge]
    VD.XA_TE  = VD.XA2[trailing_edge_of_panel]
    VD.YA_TE  = VD.YA2[trailing_edge_of_panel]
    VD.ZA_TE  = VD.ZA2[trailing_edge_of_panel]
    VD.XB_TE  = VD.XB2[trailing_edge_of_panel]
    VD.YB_TE  = VD.YB2[trailing_edge_of_panel]
    VD.ZB_TE  = VD.ZB2[trailing_edge_of_panel]

    return VD


def compute_unit_normal(VD):
    """ Unit normal of each panel, pointing to the upper side of the surface.

        Inputs:
        VD                   vortex distribution with the panel corners and the control points A and B

        Outputs:
        unit_normal          (panels, 3)
    """
    xp   = array_namespace(VD.XA1, VD.XB1, VD.XA2, VD.YA1, VD.YB1, VD.YA2, VD.ZA1, VD.ZB1, VD.ZA2)
    P1P2 = xp.array([VD.XB1 - VD.XA1,VD.YB1 - VD.YA1,VD.ZB1 - VD.ZA1]).T
    P1P3 = xp.array([VD.XA2 - VD.XA1,VD.YA2 - VD.YA1,VD.ZA2 - VD.ZA1]).T

    cross = xp.cross(P1P2,P1P3)

    unit_normal = (cross.T / xp.linalg.norm(cross,axis=1)).T

    # normals point to the upper side of the surface: with the spanwise direction A to B taken outboard, the
    # normal has a positive component along the spanwise vector rotated a quarter turn
    dy   = VD.YBC - VD.YAC
    dz   = VD.ZBC - VD.ZAC
    side = xp.where(dy < 0, -1.0, 1.0)
    flip = unit_normal[:,2]*side*dy - unit_normal[:,1]*side*dz < 0
    return xp.where(flip[:,None], -unit_normal, unit_normal)


def arctangent_of_ratio(numerator, denominator):
    """ arctan(numerator/denominator), with a finite derivative on JAX where the denominator is zero (a vertical panel)."""
    xp = array_namespace(numerator, denominator)
    if xp is np:
        return np.arctan(numerator/denominator)
    sign = xp.where(denominator<0, -1., 1.)
    return xp.arctan2(numerator*sign, denominator*sign)
