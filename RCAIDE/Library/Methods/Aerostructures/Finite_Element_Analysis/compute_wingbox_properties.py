# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/compute_wingbox_properties.py
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
#  Wingbox Assembly Function
# ----------------------------------------------------------------------
def compute_wingbox_properties(wing,discretized_params):
    """
    Assembles the wingbox from components at every element.
    """
   
    chord_arr = discretized_params.discretized_chords_elems
    spar_r_loc = 0 # UPDATE SID 
    spar_f_loc  = 0# UPDATE SID 
    h_arr   = 0# UPDATE SID 

    # Geometry Arrays 
    w_box_arr = chord_arr * (spar_r_loc - spar_f_loc)
    
    # 1. Front Spar (Get Ixx and Iyy)
    A_fs, I_fs_xx, I_fs_yy = get_spar_properties(
        f_spar_data['type'], h_arr, 
        f_spar_data['t_web'], f_spar_data['w_cap'], f_spar_data['t_cap']
    )
    
    # 2. Rear Spar (Get Ixx and Iyy)
    A_rs, I_rs_xx, I_rs_yy = get_spar_properties(
        r_spar_data['type'], h_arr, 
        r_spar_data['t_web'], r_spar_data['w_cap'], r_spar_data['t_cap']
    )
    
    # 3. Skins
    # Top Skin
    A_sk_top, I_sk_top_xx = get_skin_properties(w_box_arr, skin_t_top, h_arr/2)
    # Bottom Skin
    A_sk_bot, I_sk_bot_xx = get_skin_properties(w_box_arr, skin_t_bot, h_arr/2)
    
    # 4. Total Vertical Stiffness (Ixx) -> Resists Lift
    Ixx_total = I_fs_xx + I_rs_xx + I_sk_top_xx + I_sk_bot_xx
    
    # 5. Total Chordwise Stiffness (Izz) -> Resists Drag
    # Plus the "Lateral Bending" (Iyy) of the Spars
    
    # Skin contribution (Deep Beam approximation: 2 * t * w^3 / 12)
    I_skins_chordwise = (skin_t_top * w_box_arr**3 / 12) + (skin_t_bot * w_box_arr**3 / 12)
    
    # Total Izz (Note: Spar local Iyy acts in the Global Chordwise direction)
    Izz_total = I_skins_chordwise + I_fs_yy + I_rs_yy
    
    # 6. Torsion (J) - Bredt-Batho Closed Cell
    Am = w_box_arr * h_arr
    integral_ds_t = (w_box_arr / skin_t_top) + \
                    (w_box_arr / skin_t_bot) + \
                    (h_arr / f_spar_data['t_web']) + \
                    (h_arr / r_spar_data['t_web'])
    
    J_total = 4 * Am**2 / integral_ds_t
    
    # 7. Total Area
    A_total = A_fs + A_rs + A_sk_top + A_sk_bot
    
    return A_total, Ixx_total, Izz_total, J_total, w_box_arr, h_arr


def get_spar_properties(spar_type, h, t_web, w_cap=0, t_cap=0):
    """
    Calculates Area, Vertical Inertia (Ixx), and Lateral Inertia (Iyy) 
    for a spar component.
    
    Returns: A, I_xx (Vertical), I_yy (Lateral/Chordwise)
    """
    if spar_type == 'Rectangular':
        # Simple web
        A = h * t_web
        I_xx = t_web * h**3 / 12  # Vertical bending
        I_yy = h * t_web**3 / 12  # Lateral bending (very weak)
        
    elif spar_type == 'I_Beam':
        # Web + 2 Caps (Symmetric)
        A_web = h * t_web
        A_cap = w_cap * t_cap
        A = A_web + 2 * A_cap
        
        # Vertical (Ixx): Web centroid + Caps parallel axis
        I_xx_web = t_web * h**3 / 12
        I_xx_caps = 2 * ((w_cap * t_cap**3 / 12) + A_cap * (h/2)**2)
        I_xx = I_xx_web + I_xx_caps
        
        # Lateral (Iyy): Caps centroid + Web centroid (Symmetric)
        I_yy_web = h * t_web**3 / 12
        I_yy_caps = 2 * (t_cap * w_cap**3 / 12) # Caps bending about their own center
        I_yy = I_yy_web + I_yy_caps
        
    elif spar_type == 'C_Channel':
        # Web + 2 Caps (Asymmetric Laterally)
        A_web = h * t_web
        A_cap = w_cap * t_cap
        A = A_web + 2 * A_cap
        
        # Vertical (Ixx): Symmetric vertically, so same as I-Beam
        I_xx_web = t_web * h**3 / 12
        I_xx_caps = 2 * ((w_cap * t_cap**3 / 12) + A_cap * (h/2)**2)
        I_xx = I_xx_web + I_xx_caps
        
        # Lateral (Iyy): Asymmetric! Need Centroid Calculation.
        # Reference datum: Back of the web (x = 0)
        # Centroid of Web is at x = t_web / 2
        # Centroid of Caps is at x = w_cap / 2
        
        moment_area = (A_web * (t_web/2)) + 2 * (A_cap * (w_cap/2))
        x_centroid = moment_area / A
        
        # Parallel Axis Theorem for Iyy
        # Web Term
        I_yy_web_local = h * t_web**3 / 12
        I_yy_web = I_yy_web_local + A_web * (x_centroid - t_web/2)**2
        
        # Caps Term (x2)
        I_yy_cap_local = t_cap * w_cap**3 / 12
        I_yy_caps = 2 * (I_yy_cap_local + A_cap * (x_centroid - w_cap/2)**2)
        
        I_yy = I_yy_web + I_yy_caps
        
    else:
        A = 0; I_xx = 0; I_yy = 0
        
    return A, I_xx, I_yy

def get_skin_properties(width, thickness, dist_from_center):
    """
    Calculates Area and contribution to Ixx using Parallel Axis Theorem.
    """
    A = width * thickness
    I_local = width * thickness**3 / 12
    I_parallel = A * dist_from_center**2
    # Total I contribution
    I_total = I_local + I_parallel
    return A, I_total
