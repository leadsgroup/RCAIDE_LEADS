# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/compute_surface_loads.py
# 
# Created: Mar 2026, M. Clarke, S. Sharma  

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
# package imports 
import RCAIDE
from RCAIDE.Framework.Core import Data

# python imports 
from copy    import  deepcopy
import numpy as np

# ----------------------------------------------------------------------
#  compute_surface_loads
# ---------------------------------------------------------------------
def compute_surface_loads(conditions,VLM_results,VD,settings,geometry): 

    spanwise_CL   = VLM_results.sectional_CLift 
    chords        = VD.chord_lengths 
    dy            = VD.chord_widths    
    n_cpts        = len(spanwise_CL)
    
    
    b_sw = np.concatenate(([0],np.cumsum(VD.n_sw[ti]))) 
    for i in range(VD.n_w[ti][0]):
        # Sectional Lift 
        CL_y  = spanwise_CL[ti,b_sw[i]:b_sw[i+1]]
        c_y   =  chords[ti,b_sw[i]:b_sw[i+1]]
        L_y   = 0.5 * rho * (V[ti] ** 2) * c_y * CL_y
        
        # Section Span 
        b_y   = VD.Y_SW[ti,b_sw[i]:b_sw[i+1]]

    return FX,FY,FZ