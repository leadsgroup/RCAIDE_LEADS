# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/discretize_wing.py
# 
# Created: Mar 2026, M. Clarke, S. Sharma  

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.compute_multisegment_geometry import compute_multisegment_geometry

def discretize_wing(wing, num_elements):
    """
    Translates RCAIDE wing geometry into high-resolution FEA nodes.
    """ 
    geom = compute_multisegment_geometry(wing, num_elements)
    
    X_nodes, Y_nodes, Z_nodes = geom['X_nodes'], geom['Y_nodes'], geom['Z_nodes']
    Le = np.sqrt(np.diff(X_nodes)**2 + np.diff(Y_nodes)**2 + np.diff(Z_nodes)**2)
    Le = np.maximum(Le, 1e-6)
    Y_elems = (Y_nodes[:-1] + Y_nodes[1:]) / 2
    
    # Calculate element-centered arrays
    chord_elems     = (geom['chord_nodes'][:-1] + geom['chord_nodes'][1:]) / 2
    spar_f_elems    = (geom['spar_f_nodes'][:-1] + geom['spar_f_nodes'][1:]) / 2
    spar_r_elems    = (geom['spar_r_nodes'][:-1] + geom['spar_r_nodes'][1:]) / 2
    t_c_elems       = (geom['t_c_nodes'][:-1] + geom['t_c_nodes'][1:]) / 2 
    y_local_path    = np.insert(np.cumsum(Le), 0, 0.0)
    
    discretized_params = Data(
        X_nodes             = X_nodes,
        Y_nodes             = Y_nodes,
        Z_nodes             = Z_nodes,
        chord_nodes         = geom['chord_nodes'],
        twist_nodes         = geom['twist_nodes'],
        sweep_nodes         = geom['sweep_nodes'],
        sweep_elems_rad     = geom['sweep_mid_elems'],
        dihedral_elems_rad  = geom['dihedral_elems'],
        total_span          = geom['total_span'],
        spar_f_nodes        = geom['spar_f_nodes'],
        spar_r_nodes        = geom['spar_r_nodes'],
        t_c_nodes           = geom['t_c_nodes'],
        Le                  = Le,
        Y_elems             = Y_elems,
        chord_elems         = chord_elems,
        spar_f_elems        = spar_f_elems,
        spar_r_elems        = spar_r_elems,
        t_c_elems           = t_c_elems,
        y_local              = y_local_path,
    )
    return discretized_params
 
def map_panel_forces_to_fea(vlm_pts, vlm_F, fea_pts):
    """
    Translates 3D VLM panel forces onto 1D FEA beam elements using 
    Rigid Link Equivalent Force/Moment transfer.
    """
    num_fea = len(fea_pts)
    fea_forces = np.zeros((num_fea, 3))
    fea_moments = np.zeros((num_fea, 3))
    
    Y_fea = fea_pts[:, 1]
    
    for i in range(len(vlm_pts)):
        p_vlm = vlm_pts[i]
        f_vlm = vlm_F[i]
        
        # 1. Find closest FEA element along the span
        closest_idx = np.argmin(np.abs(Y_fea - p_vlm[1]))
        p_fea = fea_pts[closest_idx]
        
        # 2. Add Forces
        fea_forces[closest_idx] += f_vlm
        
        # 3. Calculate Moment Arm & Torsion (r x F)
        r = p_vlm - p_fea 
        m_equiv = np.cross(r, f_vlm)
        fea_moments[closest_idx] += m_equiv
        
    return fea_forces, fea_moments