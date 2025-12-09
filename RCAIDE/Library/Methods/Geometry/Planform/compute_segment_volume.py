# RCAIDE/Library/Methods/Geometry/Planform/compute_segment_volume.py
# 
# 
# Created:  Jul 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Geometry.Airfoil import compute_naca_4series
import numpy as np
from shapely import Polygon
from copy import deepcopy

def compute_segment_volume(wing, inner_segment, outer_segment,n_points=401):
    """
    Estimate the prism volume between two consecutive wing segments using their
    airfoil cross-sectional areas and local span.

    Parameters
    ----------
    wing : Wing
        Wing object containing reference chord and span information.
    inner_segment : WingSegment
        Upstream segment with airfoil definition and percent span/chord scaling.
    outer_segment : WingSegment
        Downstream segment with airfoil definition and percent span/chord scaling.
    n_points : int, optional
        Number of points to generate NACA 4-series airfoil geometry when missing.

    Returns
    -------
    float
        Estimated segment volume [m³] using a frustum (prismoid) approximation.

    Notes
    -----
    * If airfoil geometry is absent, a NACA 4-series shape is generated/assumed.
    * Areas are computed via polygon integration of the airfoil coordinates.
    * Volume uses (1/3)*(A1 + A2 + sqrt(A1*A2))*span for the local segment span.
    """

    # Extract airfoil coordinates
    if hasattr(inner_segment.airfoil,'geometry') is False or hasattr(outer_segment.airfoil,'geometry') is False: # first, if airfoil geometry data not defined, import from geoemtry files
        inner_segment = deepcopy(inner_segment)
        outer_segment = deepcopy(outer_segment)
        if type(inner_segment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil: # check if naca 4 series of airfoil from datafile
            try:
                inner_segment.airfoil.geometry = compute_naca_4series(inner_segment.NACA_4_Series_code,n_points)
            except:
                inner_segment.airfoil.geometry = compute_naca_4series('0012')
        else:
            inner_segment.airfoil= RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil
            inner_segment.airfoil.geometry = compute_naca_4series('0012')
        
        if type(outer_segment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil: # check if naca 4 series of airfoil from datafile
            try:
                outer_segment.airfoil.geometry = compute_naca_4series(outer_segment.NACA_4_Series_code,n_points)
            except:
                outer_segment.airfoil.geometry = compute_naca_4series('0012')
        else:
            outer_segment.airfoil= RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil
            outer_segment.airfoil.geometry = compute_naca_4series('0012')
    
      
    x_in = np.array(inner_segment.airfoil.geometry.x_coordinates)[:-1] * wing.chords.root *inner_segment.root_chord_percent
    y_in = np.array(inner_segment.airfoil.geometry.y_coordinates)[:-1] * wing.chords.root *inner_segment.root_chord_percent
    x_out = np.array(outer_segment.airfoil.geometry.x_coordinates)[:-1] * wing.chords.root *outer_segment.root_chord_percent
    y_out = np.array(outer_segment.airfoil.geometry.y_coordinates)[:-1] * wing.chords.root *outer_segment.root_chord_percent

    points_out = list(zip(x_out, y_out))
    poly_out = Polygon(points_out)

    points_in = list(zip(x_in, y_in))
    poly_in = Polygon(points_in)
    A_1 = poly_in.area
    A_2 = poly_out.area

     # Compute segment span length
    L = (outer_segment.percent_span_location - inner_segment.percent_span_location) * wing.spans.projected
    volume = (1 /3) * ( A_1 + A_2 + np.sqrt(A_1*A_2)) *L

    return volume
