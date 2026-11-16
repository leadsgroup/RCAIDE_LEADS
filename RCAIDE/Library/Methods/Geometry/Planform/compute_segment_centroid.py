# RCAIDE/Library/Methods/Geometry/Planform/compute_segment_centroid.py
# 
# 
# Created:  Dec 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# package imports 
from RCAIDE.Framework.Core import array_namespace

# ----------------------------------------------------------------------------------------------------------------------
#  Compute segment centroid 
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_segment_centroid(le_sweep,seg_span,dx,dy,dz,taper,dihedral,root_chord,tip_chord):
    """Computes the centroid of a trapezoidal segment
    
    Assumptions:
    Polygon
    
    Source:
    None
    
    Inputs:
    le_sweep      [rad]
    seg_span      [m]
    dx            [m]
    dy            [m]
    taper         [dimensionless]
    dihedral      [radians]
    root_chord    [m]
    tip_chord     [m]

    Outputs:
    cx,cy,cz      [m,m,m]  one row each

    Properties Used:
    N/A
    """    
    
    xp = array_namespace(le_sweep, seg_span, dx, dy, dz, taper, dihedral, root_chord, tip_chord)
    a = tip_chord
    b = root_chord
    c = xp.tan(le_sweep)*seg_span
    cx = (2*a*c + a**2 + c*b + a*b + b**2) / (3*(a+b))
    cy = seg_span / 3. * (( 1. + 2. * taper ) / (1. + taper))
    cz = cy * xp.tan(dihedral)    
    
    return xp.stack([cx+dx,cy+dy,cz+dz]) 