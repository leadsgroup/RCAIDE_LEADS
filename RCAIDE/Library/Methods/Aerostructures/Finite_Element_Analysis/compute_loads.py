# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/compute_loads.py
# 
# Created: Mar 2026, M. Clarke, S. Sharma  

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
# Import Supporting Functions
import RCAIDE 

# Python Imports
import numpy as np 

# ----------------------------------------------------------------------
#  compute_loads
# ----------------------------------------------------------------------
def compute_loads(lift_total, Y_elems, span_len):
    """ 
    Temporary elliptical load generator. 
    Applies forces directly to the neutral axis (0 eccentricity) to prepare for VLM integration.
    """
    L_root = lift_total * 4 / (span_len * np.pi)
    
    # Safety Clip
    term = 1 - (Y_elems / span_len)**2
    term = np.maximum(term, 0.0) 
    
    w_z = L_root * np.sqrt(term)
    
    # ZERO TORSION (Eccentricity removed. VLM will supply M_y directly later)
    t_y = np.zeros_like(w_z) 
    
    return w_z, t_y