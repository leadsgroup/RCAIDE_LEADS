import numpy as np 

# --- 6. Aerodynamic Load Functions ---
def translate_aerodynamic_loads(lift_total, span_len, y_locs, chords, x_spar1, x_spar2, sweep_rad):
    """
    1. Generates Elliptical Lift Distribution (w_z).
    2. Calculates Torsion (t_y) due to offset between Aero Center and Shear Center.
    """
    # A. Elliptical Lift Distribution
    # L(y) = L_root * sqrt(1 - (y/span)^2)
    L_root = lift_total * 4 / (span_len * np.pi)
    
    # Map spar coordinates to spanwise position (0 to Span) for the formula
    y_span = y_locs * np.cos(sweep_rad)
    w_z = L_root * np.sqrt(1 - (y_span/span_len)**2)
    
    # B. Torsion due to Offset (Eccentricity)
    # Assuming Aero Center at 0.25 * chord (c/4)
    x_ac = 0.25 * chords
    
    # Structural Centroid (Midpoint of the box)
    x_box_center = (x_spar1 + x_spar2) / 2 * chords
    
    # Eccentricity e = Centroid - AeroCenter
    # If Centroid is behind AC, Lift creates a "Pitch Down" (Negative) moment on the beam
    # Usually: T = Lift * e
    eccentricity = x_box_center - x_ac
    t_y = w_z * eccentricity
    
    return w_z, t_y