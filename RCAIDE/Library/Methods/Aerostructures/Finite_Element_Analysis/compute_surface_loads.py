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

    spanwise_CL   = VLM_results.spanwise_CLift
    
    

    results.CLift_wings           
    results.CDrag_induced_wings   
    results.spanwise_wing_lift    
    results.surface_wing_lift     
    results.surface_Fx            
    results.surface_Fy            
    results.surface_Fz            
    
    
    
    
    
    
    
    
    #chords        = VD.chord_lengths 
    #dy            = VD.chord_widths    
    #n_cpts        = len(spanwise_CL)
    
     


    # F_x_surf = np.zeros((n_cpts,n_panels))
    # F_y_surf = np.zeros((n_cpts,n_panels))
    # F_z_surf = np.zeros((n_cpts,n_panels))
    # M_x_surf = np.zeros((n_cpts,n_panels))
    # M_y_surf = np.zeros((n_cpts,n_panels))
    # M_z_surf = np.zeros((n_cpts,n_panels))
    
    # b_sw = np.concatenate(([0],np.cumsum(VD.n_sw[ti]))) 
    # for i in range(VD.n_w[ti][0]):
    #     # Sectional Lift 
    #     CL_y  = spanwise_CL[ti,b_sw[i]:b_sw[i+1]]
    #     c_y   =  chords[ti,b_sw[i]:b_sw[i+1]]
    #     L_y   = 0.5 * rho * (V[ti] ** 2) * c_y * CL_y
        
    #     # Section Span 
    #     b_y   = VD.Y_SW[ti,b_sw[i]:b_sw[i+1]]
    

    # surface_forces_and_moments = Data(
    #                             F_x = F_x_surf,
    #                             F_y = F_y_surf,
    #                             F_z = F_z_surf,
    #                             M_x = M_x_surf,
    #                             M_y = M_y_surf,
    #                             M_z = M_z_surf,
    #                             spanwise_load = L_y, 
    #                         )

    return # surface_forces_and_moments