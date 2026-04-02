import numpy as np

def compute_force_vector(w_x, w_y, w_z, t_y, Le, num_elem, T):
    F_local = np.zeros((num_elem, 12))
    
    # 1. Stack Global Distributed Loads (N, 3)
    # [w_x, w_y, w_z]
    loads_global = np.column_stack((w_x, w_y, w_z))
    
    # 2. Get Rotation Blocks R from T (N, 3, 3)
    # R maps Local -> Global (u_glob = R * u_loc)
    # So Force transformation: F_local = R^T * F_global
    R_blocks = T[:, 0:3, 0:3]
    
    # 3. Rotate Loads
    # einsum: e=element, j=global_axis, i=local_axis
    # F_local[e, i] = sum_j ( R[e, j, i] * F_global[e, j] )
    # Note: R[e, j, i] is the transpose of R (because we want R^T)
    # R[e, row, col] -> we want R^T -> swap row/col
    loads_local = np.einsum('eji,ej->ei', R_blocks, loads_global)
    
    w_u = loads_local[:, 0] # Local Chordwise
    w_v = loads_local[:, 1] # Local Axial
    w_w = loads_local[:, 2] # Local Vertical
    
    # 4. Integrate to Nodal Forces
    F_local[:,0] = w_u*Le/2; F_local[:,6] = w_u*Le/2
    F_local[:,5] = -w_u*Le**2/12; F_local[:,11] = w_u*Le**2/12
    
    F_local[:,1] = w_v*Le/2; F_local[:,7] = w_v*Le/2
    
    F_local[:,2] = w_w*Le/2; F_local[:,8] = w_w*Le/2
    F_local[:,3] = w_w*Le**2/12; F_local[:,9] = -w_w*Le**2/12
    
    F_local[:,4] = t_y*Le/2; F_local[:,10] = t_y*Le/2
    
    # 5. Rotate back to Global F = T * f_local
    # einsum: e=element, i=global_row, j=local_col
    # F_glob[e, i] = sum_j ( T[e, i, j] * F_loc[e, j] )
    F_global_elem = np.einsum('eij,ej->ei', T, F_local)
    
    return F_global_elem