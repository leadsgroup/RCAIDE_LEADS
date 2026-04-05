# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/compute_3d_transformation_matrix.py
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
# compute_3d_transformation_matrix
# ----------------------------------------------------------------------
def compute_3d_transformation_matrix(sweep_arr, dihedral_arr, twist_arr, num_elements):
    """
    Vectorized 12x12 transformation matrix. 
    Accepts arrays for sweep, dihedral, and twist to handle cranked/multi-segment wings.
    """
    c_sw, s_sw = np.cos(sweep_arr), np.sin(sweep_arr)
    c_di, s_di = np.cos(dihedral_arr), np.sin(dihedral_arr)
    
    # Sweep Rotation Stack (Yaw)
    R_sweep = np.zeros((num_elements, 3, 3))
    R_sweep[:, 0, 0] = c_sw;  R_sweep[:, 0, 1] = -s_sw
    R_sweep[:, 1, 0] = s_sw;  R_sweep[:, 1, 1] = c_sw
    R_sweep[:, 2, 2] = 1.0
    
    # Dihedral Rotation Stack (Roll)
    R_dihedral = np.zeros((num_elements, 3, 3))
    R_dihedral[:, 0, 0] = 1.0
    R_dihedral[:, 1, 1] = c_di;  R_dihedral[:, 1, 2] = -s_di
    R_dihedral[:, 2, 1] = s_di;  R_dihedral[:, 2, 2] = c_di
    
    # Batch multiply Base Rotation
    R_base = np.matmul(R_sweep, R_dihedral)
    
    # Twist Rotation Stack (Pitch)
    c_tw, s_tw = np.cos(twist_arr), np.sin(twist_arr)
    R_twist = np.zeros((num_elements, 3, 3))
    R_twist[:, 0, 0] = c_tw;  R_twist[:, 0, 2] = s_tw
    R_twist[:, 1, 1] = 1.0
    R_twist[:, 2, 0] = -s_tw; R_twist[:, 2, 2] = c_tw
    
    # Total 3x3 Rotation
    R_total = np.matmul(R_base, R_twist)
    
    # Expand to 12x12
    T_all = np.zeros((num_elements, 12, 12))
    T_all[:, 0:3, 0:3]   = R_total
    T_all[:, 3:6, 3:6]   = R_total
    T_all[:, 6:9, 6:9]   = R_total
    T_all[:, 9:12, 9:12] = R_total
    
    return T_all