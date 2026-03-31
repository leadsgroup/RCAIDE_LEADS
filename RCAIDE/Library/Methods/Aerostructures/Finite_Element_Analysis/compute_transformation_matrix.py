import numpy as np

# --- 4. Transformation Matrix Function ---
def compute_transformation_matrix(sweep_rad, num_elements):
    """
    Creates the 12x12 transformation matrix T for all elements.
    Relates Global Displacements to Local Displacements: u_local = T * u_global
    """
    c = np.cos(sweep_rad)
    s = np.sin(sweep_rad)
    
    # Rotation Sub-Matrix (3x3) for (u, v, w)
    # Global u (X/Drag), Global v (Y/Span), Global w (Z/Lift)
    # Local u (Drag Bending), Local v (Axial), Local w (Vertical Bending)
    
    # Based on vector projection:
    # v_local (Axial) = v_global * cos(L) + u_global * sin(L)
    # u_local (Norm)  = u_global * cos(L) - v_global * sin(L)
    
    R = np.array([
        [ c, -s,  0],  # Local u (Row 0)
        [ s,  c,  0],  # Local v (Row 1)
        [ 0,  0,  1]   # Local w (Row 2)
    ])
    
    # Full 12x12 T Matrix (Block Diagonal)
    # We apply the same rotation to (u,v,w) and (theta_x, theta_y, theta_z)
    # at both Node i and Node j.
    T = np.zeros((12, 12))
    T[0:3, 0:3] = R   # Node i Translation
    T[3:6, 3:6] = R   # Node i Rotation
    T[6:9, 6:9] = R   # Node j Translation
    T[9:12, 9:12] = R # Node j Rotation
    
    # Broadcast to all elements
    T_all = np.tile(T, (num_elements, 1, 1))
    return T_all  
  