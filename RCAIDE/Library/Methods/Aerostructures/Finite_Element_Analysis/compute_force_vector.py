import numpy as np 

# --- 5. Force Matrix Function ---
def compute_force_vector(w_x, w_y, w_z, t_y, Le, num_elem, T):
    # Local Force Vector
    F_local = np.zeros((num_elem, 12))
    
    # Rotate Global Loads to Local Frame
    c = T[0,0,0]
    s = T[0,1,0]
    
    w_u = w_x * c - w_y * s # Chordwise
    w_v = w_x * s + w_y * c # Axial
    w_w = w_z               # Vertical
    
    # Distributed Load to Nodal Forces
    # u (Chordwise)
    F_local[:, 0] = w_u * Le / 2
    F_local[:, 6] = w_u * Le / 2
    F_local[:, 5] = -w_u * Le**2 / 12
    F_local[:, 11] = w_u * Le**2 / 12
    
    # v (Axial)
    F_local[:, 1] = w_v * Le / 2
    F_local[:, 7] = w_v * Le / 2
    
    # w (Vertical)
    F_local[:, 2] = w_w * Le / 2
    F_local[:, 8] = w_w * Le / 2
    F_local[:, 3] = w_w * Le**2 / 12
    F_local[:, 9] = -w_w * Le**2 / 12
    
    # Torsion (theta_y)
    F_local[:, 4] = t_y * Le / 2
    F_local[:, 10] = t_y * Le / 2
    
    # Rotate Back to Global
    F_global = np.einsum('eji,ej->ei', T, F_local)
    return F_global
 