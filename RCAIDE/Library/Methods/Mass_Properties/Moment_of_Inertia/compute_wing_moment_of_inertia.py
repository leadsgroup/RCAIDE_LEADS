# RCAIDE/Library/Methods/Stability/Moment_of_Inertia/compute_wing_moment_of_inertia.py 
# 
# Created:  September 2023, A. Molloy  
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE 
import RCAIDE
from RCAIDE.Framework.Core import Units    

# package imports 
import numpy as np  
# ----------------------------------------------------------------------------------------------------------------------
#  Compute Wing Moment of Intertia
# ----------------------------------------------------------------------------------------------------------------------  
def compute_wing_moment_of_inertia(wing,mass = 0, center_of_gravity = [[0, 0, 0]], fuel_flag = False):
    ''' computes the moment of inertia tensor for a wing about a given center of gravity.
    Includes the ability to modela  wing fuel tank as a condensed wing

    Assumptions:
    - Wing is solid
    - Wing has a constant density

    Source:
    [1] Moulton, B. C., and Hunsaker, D. F., “Simplified Mass and Inertial Estimates for Aircraft with Components
    of Constant Density,” AIAA SCITECH 2023 Forum, January 2023, AIAA-2023-2432 DOI: 10.2514/
    6.2023-2432
    
    [2] Fuel tank references: These were used to estimate the length percentages. 
    - https://assets.publishing.service.gov.uk/media/5422fa1aed915d13710007a1/2-2007_G-YMME.pdf
    - https://oat.aero/2023/03/17/airbus-a380-general-familiarisation-fuel-storage/
    - http://www.b737.org.uk/fuel.htm
    - https://slideplayer.com/slide/3854059/
    
    Inputs:
    - Wing
    - Wing mass
    - Center of gravity
    - Fuel flag (whether the wing is considered a fuel tank or not)

    Outputs:
    - wing moment of inertia tensor

    Properties Used:
    N/A
    '''
    # ----------------------------------------------------------------------------------------------------------------------
    # Setup: 
    # ---------------------------------------------------------------------------------------------------------------------- 
    tr          = wing.thickness_to_chord # root thickness as percent of chord
    tt          = wing.thickness_to_chord #tip thickness as a percent of chord
    ct          = wing.chords.tip # tip chord 
    cr          = wing.chords.root # root chord
    b           = wing.spans.total / 2 # half-span of the wing
    A           = wing.sweeps.quarter_chord # sweep angle (located at quarter chord)
    dihedral    = wing.dihedral # Wing dihedral
    origin_wing = wing.origin + np.array([[cr / 4], [0], [0]]) # moves the origin of the wing to the quarter chord of the root airfoil.
    
    if wing.symmetric: # Splits the wing weight between the two wings if the wing is symmetric.
        m_wing = mass * 0.5
    else:
        m_wing = mass
    
    # ----------------------------------------------------------------------------------------------------------------------
    # Fuel Tank Values. The percentages come from [2]
    # ----------------------------------------------------------------------------------------------------------------------
    if fuel_flag:
        b           = b * 0.8 # Wing fuel tank is 80% span of the entire wing
        ct          = (0.8 * (ct - cr) + cr)* 0.6 # Wing fuel tank has 60% of the chord 80% down the wing. Assumes linear relation ebtween cr and ct 
        cr          = cr * 0.6 # Wing tank has 60% of the chord of the root chord
        origin_wing = origin_wing + np.array([[cr * 0.0125], [0], [0]])  # wing fuel tank is set about 10% back in the wing. This is a correction that considers the quarter chord location of the main wing and the smaller section. 
    
    # ----------------------------------------------------------------------------------------------------------------------
    # Constants. These values and equations are defined in Moulton and Hunsaker [1]
    # ----------------------------------------------------------------------------------------------------------------------
    # Airfoil constants. a0-a4 values help define the thickness distribution for a NACA 4-digit airfoil. This holds for all NACA airfoils 
    a0 = 2.969
    a1 = -1.260
    a2 = -3.516
    a3 = 2.843
    a4 = -1.015
    
    # Integration constants. These constants are defined in [1].
    ka = tr * (3 * cr ** 2 + 2 * cr * ct + ct ** 2) + tt * (cr ** 2 + 2 * cr * ct + 3 * ct ** 2)
    kd = (tr * (ct + cr) * (2 * cr ** 2 + cr * ct + 2 * ct ** 2)
          + tt * (cr ** 3 + 3 * cr ** 2 * ct + 6 * cr * ct ** 2 + 10 * ct ** 3))
    ke = (tr * (5 * cr ** 4 + 4 * cr ** 3 * ct + 3 * cr ** 2 * ct ** 2 + 2 * cr * ct ** 3 + ct ** 4)
          + tt * (cr ** 4 + 2 * cr ** 3 * ct + 3 * cr ** 2 * ct ** 2 + 4 * cr * ct ** 3 + 5 * ct ** 4))    
    kf = (tr * (cr ** 2 + 2 * cr * ct + 2 * ct ** 2) + tt * (cr ** 2 + 4 * cr * ct
                                                             + 10 * ct ** 2))
    kg = (tr ** 3 * (35 * cr ** 4 + 20 * cr ** 3 * ct + 10 * cr ** 2 * ct ** 2 + 4 * cr * ct ** 3 + ct ** 4)
          + tr ** 2 * tt * (15 * cr ** 4 + 20 * cr ** 3 * ct + 18 * cr ** 2 * ct ** 2 + 12 * cr * ct ** 3 + 5 * ct ** 4)
          + tr * tt ** 2 * (5 * cr ** 4 + 12 * cr ** 3 * ct + 18 * cr ** 2 * ct ** 2 + 20 * cr * ct ** 3 + 15 * ct ** 4)
          + tt ** 3 * (cr ** 4 + 4 * cr ** 3 * ct + 10 * cr ** 2 * ct ** 2 + 20 * cr * ct ** 3 + 35 * ct ** 4))
    
    v0 = 1 / 60 * (40 * a0 + 30 * a1 + 20 * a2 + 15 * a3 + 12 * a4) # NACA 4 digit integral of thickness distribution.
    v1 = 1 / 60 * (56 * a0 + 50 * a1 + 40 * a2 + 33 * a3 + 28 * a4)
    v2 = 1 / 980 * (856 * a0 + 770 * a1 + 644 * a2 + 553 * a3 + 484 * a4)
    v3 = (2 / 5 * a0 ** 3 + a0 ** 2 * a1 + 3 / 4 * a0 ** 2 * a2 + 3 / 5 * a0 ** 2 * a3 + 1 / 2 * a0 ** 2 * a4 + 6 / 7 * a0 * a1 ** 2
          + 4 / 3 * a0 * a1 * a2 + 12 / 11 * a0 * a1 * a3 + 12 / 13 * a0 * a1 * a4 + 6 / 11 * a0 * a2 ** 2 + 12 / 13 * a0 * a2 * a3
          + 4 / 5 * a0 * a2 * a4 + 2 / 5 * a0 * a3 ** 2 + 12 / 17 * a0 * a3 * a4 + 6 / 19 * a0 * a4 ** 2 + 1 / 4 * a1 ** 3
          + 3 / 5 * a1 ** 2 * a2 + 1 / 2 * a1 ** 2 * a3 + 3 / 7 * a1 ** 2 * a4 + 1 / 2 * a1 * a2 ** 2 + 6 / 7 * a1 * a2 * a3
          + 3 / 4 * a1 * a2 * a4 + 3 / 8 * a1 * a3 ** 2 + 2 / 3 * a1 * a3 * a4 + 3 / 10 * a1 * a4 ** 2 + 1 / 7 * a2 ** 3
          + 3 / 8 * a2 ** 2 * a3 + 1 / 3 * a2 ** 2 * a4 + 1 / 3 * a2 * a3 ** 2 + 3 / 5 * a2 * a3 * a4 + 3 / 11 * a2 * a4 ** 2
          + 1 / 10 * a3 ** 3 + 3 / 11 * a3 ** 2 * a4 + 1 / 4 * a3 * a4 ** 2 + 1 / 13 * a4 ** 3)
    
    # ----------------------------------------------------------------------------------------------------------------------
    # Moment of Inertia in the Local Wing System
    # ----------------------------------------------------------------------------------------------------------------------
    delta = 1 # 1 for right wing, -1 for left wing. Assumes all non-symmetric wings are right-wings.
    
    Ixx        = m_wing * (56 * b ** 2 * kf * v0 + kg * v3) / (280 * ka * v0)
    Iyy        = m_wing * (84 * b * (2 * b * kf * v0 * np.tan(A) ** 2 + kd * v1 * np.tan(A)) + 49 * ke * v2 + 3 * kg * v3) / (840 * ka * v0)
    Izz        = m_wing * (12 * b * (2 * b * (np.tan(A) ** 2 + 1) * kf * v0 + kd * v1 * np.tan(A)) + 7 * ke * v2) / (120 * ka * v0)
    Ixy        = -1 * delta * b * m_wing * (4 * b * kf * v0 * np.tan(A) + kd * v1) / (20 * ka * v0)
    Ixz        = 0  # Ixz, Iyz are 0
    Iyz        = 0  # Ixz, Iyz are 0
    I_wing_sys = np.array([[Ixx, -Ixy, -Ixz], [-Ixy, Iyy, -Iyz], [-Ixz, -Iyz, Izz]]) # inertia tensor in the wing system
    
    # Dihedral. -1*dihedral for the right wing
    R       = np.array([[1, 0, 0], [0, np.cos(-1*dihedral), -1 * np.sin(-1*dihedral)], [0, np.sin(-1*dihedral), np.cos(-1*dihedral)]])
    I_local = R *I_wing_sys *np.transpose(R) 
      
    # ----------------------------------------------------------------------------------------------------------------------
    # Symmetric Wing
    # ----------------------------------------------------------------------------------------------------------------------
    if wing.symmetric: # wing is symmetric
        
        # Rotation matrix for dihedral. Note no -1*dihedral for the symmetric wing
        R = np.array([[1, 0, 0], [0, np.cos(dihedral), -1 * np.sin(dihedral)], [0, np.sin(dihedral), np.cos(dihedral)]])        
        
        # Inertia matrix in local wing frame
        delta        = -1 # left wing
        Ixx          = m_wing * (56 * b ** 2 * kf * v0 + kg * v3) / (280 * ka * v0)
        Iyy          = m_wing * (84 * b * (2 * b * kf * v0 * np.tan(A) ** 2 + kd * v1 * np.tan(A)) + 49 * ke * v2 + 3 * kg * v3) / (840 * ka * v0)
        Izz          = m_wing * (12 * b * (2 * b * (np.tan(A) ** 2 + 1) * kf * v0 + kd * v1 * np.tan(A)) + 7 * ke * v2) / (120 * ka * v0)
        Ixy          = -1 * delta * b * m_wing * (4 * b * kf * v0 * np.tan(A) + kd * v1) / (20 * ka * v0)
        I_local_left = np.array([[Ixx, -Ixy, -Ixz], [-Ixy, Iyy, -Iyz], [-Ixz, -Iyz, Izz]])
        
        # Dihedral rotation
        I_local_left = R *I_local_left * np.transpose(R)
        I_local      = I_local + I_local_left # Add the left wing inertia tensor if wing is symmetric
    
    # ----------------------------------------------------------------------------------------------------------------------
    # Vertical Surface
    # ----------------------------------------------------------------------------------------------------------------------
    if wing.vertical: # If it is a vertical tail
        R         = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]) # Rotation matrix for a vertical surface
        I_local   = R * I_local * np.transpose(R) # Rotation of inertia matrix to a vertical frame of reference         
    
    # ----------------------------------------------------------------------------------------------------------------------
    # RCAIDE Coordinate system. (Local system is flipped 180 deg from RCAIDE coordinate system convention.)
    # ----------------------------------------------------------------------------------------------------------------------
    Rr       = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
    I_RCAIDE = Rr * I_local * np.transpose(Rr)
    
    # ----------------------------------------------------------------------------------------------------------------------
    # Global Coordinate System
    # ----------------------------------------------------------------------------------------------------------------------
    s        = np.array(center_of_gravity) - np.array(origin_wing) # Vector for the parallel axis theorem
    I_global = np.array(I_RCAIDE) + m_wing * (np.array(np.dot(s[0], s[0])) * np.array(np.identity(3)) - s * np.transpose(s))
    
    return I_global,  mass

