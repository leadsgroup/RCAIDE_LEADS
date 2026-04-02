import numpy as np

def compute_element_stiffness_arrays(E, G, A, J, I_xx, I_zz, Le, num_elem):
    K_e = np.zeros((num_elem, 12, 12))
    
    C_EA_L = E * A / Le
    C_GJ_L = G * J / Le
    C_EIx_L3 = E * I_xx / (Le**3)
    C_EIz_L3 = E * I_zz / (Le**3)
    
    # Axial
    K_e[:, 1, 1] = K_e[:, 7, 7] = C_EA_L
    K_e[:, 1, 7] = K_e[:, 7, 1] = -C_EA_L
    
    # Torsion
    K_e[:, 4, 4] = K_e[:, 10, 10] = C_GJ_L
    K_e[:, 4, 10] = K_e[:, 10, 4] = -C_GJ_L
    
    # Vertical Bending (w) - Ixx
    K_e[:, 2, 2] = K_e[:, 8, 8] = 12 * C_EIx_L3
    K_e[:, 2, 3] = K_e[:, 3, 2] = K_e[:, 2, 9] = K_e[:, 9, 2] = 6 * C_EIx_L3 * Le
    K_e[:, 8, 3] = K_e[:, 3, 8] = K_e[:, 8, 9] = K_e[:, 9, 8] = -6 * C_EIx_L3 * Le
    K_e[:, 2, 8] = K_e[:, 8, 2] = -12 * C_EIx_L3
    K_e[:, 3, 3] = K_e[:, 9, 9] = 4 * C_EIx_L3 * Le**2
    K_e[:, 3, 9] = K_e[:, 9, 3] = 2 * C_EIx_L3 * Le**2
    
    # Chordwise Bending (u) - Izz
    K_e[:, 0, 0] = K_e[:, 6, 6] = 12 * C_EIz_L3
    K_e[:, 0, 5] = K_e[:, 5, 0] = K_e[:, 0, 11] = K_e[:, 11, 0] = -6 * C_EIz_L3 * Le
    K_e[:, 6, 5] = K_e[:, 5, 6] = K_e[:, 6, 11] = K_e[:, 11, 6] = 6 * C_EIz_L3 * Le
    K_e[:, 0, 6] = K_e[:, 6, 0] = -12 * C_EIz_L3
    K_e[:, 5, 5] = K_e[:, 11, 11] = 4 * C_EIz_L3 * Le**2
    K_e[:, 5, 11] = K_e[:, 11, 5] = 2 * C_EIz_L3 * Le**2
    
    return K_e