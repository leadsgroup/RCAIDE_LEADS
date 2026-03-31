import numpy as np 

# --- 2. Wingbox Function ---
def compute_wingbox_stiffness(w, h, t_sk, t_sp):
    """
    Calculates Stiffness for a Rectangular Box.
    w: Box Width array
    h: Box Height array
    """
    # 1. Moments of Inertia (Ixx - Vertical Bending)
    # Spars (Vertical Webs) - Centroidal term dominates
    # I = b*h^3 / 12
    I_spars = 2 * (t_sp * h**3 / 12) 
    
    # Skins (Horizontal Plates) - Parallel Axis term dominates
    # I = Area * distance^2 = (w*t)* (h/2)^2
    Area_skin = w * t_sk
    I_skins = 2 * (Area_skin * (h/2)**2) 
    
    I_xx = I_spars + I_skins
    
    # 2. Moments of Inertia (Izz - Chordwise Bending)
    # Skins (Horizontal Plates) - Centroidal term dominates
    I_skins_z = 2 * (t_sk * w**3 / 12)
    
    # Spars (Vertical Webs) - Parallel Axis term dominates
    Area_spar = h * t_sp
    I_spars_z = 2 * (Area_spar * (w/2)**2)
    
    I_zz = I_skins_z + I_spars_z
    
    # 3. Torsion (J) - Bredt-Batho Formula for Closed Cell
    # Enclosed Area (Am)
    Am = (w - t_sp) * (h - t_sk)
    
    # Perimeter Integral (Integral ds/t)
    # 2 widths / t_skin + 2 heights / t_spar
    perimeter_integral = (2 * (w - t_sp) / t_sk) + (2 * (h - t_sk) / t_sp)
    
    J = 4 * Am**2 / perimeter_integral
    
    # 4. Area (Cross-section)
    A = 2*(w*t_sk) + 2*(h*t_sp)
    
    return A, I_xx, I_zz, J
