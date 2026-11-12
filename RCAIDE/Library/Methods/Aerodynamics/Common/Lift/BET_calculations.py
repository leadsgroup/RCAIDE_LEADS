# RCAIDE/Methods/Aerodynamics/Common/Lift/BET_calculations.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
 
from RCAIDE.Framework.Core import interp2d, array_namespace

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_airfoil_aerodynamics
# ----------------------------------------------------------------------------------------------------------------------   
def compute_airfoil_aerodynamics(beta,c,r,R,B,Wa,Wt,a,nu,airfoils,airfoil_locations,ctrl_pts,Nr,Na,tc,use_2d_analysis):
    """
    Cl, Cdval = compute_airfoil_aerodynamics( beta,c,r,R,B,
                                              Wa,Wt,a,nu,
                                              airfoils,a_loc
                                              ctrl_pts,Nr,Na,tc,use_2d_analysis )

    Computes the aerodynamic forces at sectional blade locations. If airfoil
    geometry and locations are specified, the forces are computed using the
    airfoil polar lift and drag surrogates, accounting for the local Reynolds
    number and local angle of attack.

    If the airfoils are not specified, an approximation is used.

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
       beta                       blade twist distribution                        [-]
       c                          chord distribution                              [-]
       r                          radius distribution                             [-]
       R                          tip radius                                      [-]
       B                          number of rotor blades                          [-]

       Wa                         axial velocity                                  [-]
       Wt                         tangential velocity                             [-]
       a                          speed of sound                                  [-]
       nu                         viscosity                                       [-]
       airfoil_data               Data structure of airfoil polar information     [-]
       ctrl_pts                   Number of control points                        [-]
       Nr                         Number of radial blade sections                 [-]
       Na                         Number of azimuthal blade stations              [-]
       tc                         Thickness to chord                              [-]
       use_2d_analysis            Specifies 2d disc vs. 1d single angle analysis  [Boolean]

    Outputs:
       Cl                       Lift Coefficients                         [-]
       Cdval                    Drag Coefficients  (before scaling)       [-]
       alpha                    section local angle of attack             [rad]

    """ 
    xp       = array_namespace(beta, Wa, Wt, a, nu)
    alpha    = beta - xp.arctan2(Wa,Wt)
    W        = (Wa*Wa + Wt*Wt)**0.5
    Ma       = W/a
    Re       = (W*c)/nu

    # If rotor airfoils are defined, use airfoil surrogate
    if len(airfoil_locations) != 0:
        a_loc = np.array(airfoil_locations)

        # Compute blade Cl and Cd distribution from the airfoil data, one airfoil at a time on the stations that use it;
        # shape (ctrl_pts, Nr) for the 1D analysis and (ctrl_pts, Nr, Na) for the 2D analysis
        Cl    = xp.zeros_like(alpha)
        Cdval = xp.zeros_like(alpha)
        for jj,airfoil in enumerate(airfoils):
            pd       = airfoil.polars
            Cl_af    = interp2d(Re,alpha,pd.reynolds_numbers, pd.angle_of_attacks, pd.lift_coefficients)
            Cdval_af = interp2d(Re,alpha,pd.reynolds_numbers, pd.angle_of_attacks, pd.drag_coefficients)
            on_station = (a_loc == jj).reshape((1, Nr) + (1,)*(alpha.ndim - 2))   # radial stations with this airfoil
            Cl       = xp.where(on_station, Cl_af, Cl)
            Cdval    = xp.where(on_station, Cdval_af, Cdval)
    else:
        # Estimate Cl max
        tc_1 = tc*100
        Cl_max_ref = xp.maximum(-0.0009*tc_1**3 + 0.0217*tc_1**2 - 0.0442*tc_1 + 0.7005, 0.7)
        Re_ref     = 9.*10**6
        Cl1maxp    = Cl_max_ref * ( Re / Re_ref ) **0.1

        # If not airfoil polar provided, use 2*pi as lift curve slope
        Cl = 2.*np.pi*alpha

        # By 90 deg, it's totally stalled.
        Cl = xp.where(Cl > Cl1maxp, Cl1maxp, Cl)   # This line of code is what changed the regression testing
        Cl = xp.where(alpha >= np.pi/2, 0., Cl)

        # Scale for Mach, this is Karmen_Tsien; supersonic sections are not scaled
        KT_cond = (Ma < 1.) & (Cl > 0)
        M2      = xp.where(KT_cond, Ma*Ma, 0.)       # zero where unused, so the square root stays real
        Cl      = xp.where(KT_cond, Cl/((1-M2)**0.5 + (M2/(1+(1-M2)**0.5))*Cl/2), Cl)

        #This is an atrocious fit of DAE51 data at RE=50k for Cd
        Cdval = (0.108*(Cl*Cl*Cl*Cl)-0.2612*(Cl*Cl*Cl)+0.181*(Cl*Cl)-0.0139*Cl+0.0278)*((50000./Re)**0.2)
        Cdval = xp.where(alpha >= np.pi/2, 2., Cdval)

    # angle of attack and Reynolds number around the disc: the 1D analysis repeats them at every azimuthal station
    if use_2d_analysis:
        alpha_disc = alpha
        Re_disc    = Re
    else:
        alpha_disc = xp.repeat(alpha[:, :, None], Na, axis=2)
        Re_disc    = xp.repeat(Re[:, :, None], Na, axis=2)

    # prevent zero Cl to keep Cd/Cl from breaking in BET
    Cl = xp.where(Cl == 0, 1e-6, Cl)

    return Cl, Cdval, alpha, alpha_disc,Ma,W,Re,Re_disc

# ----------------------------------------------------------------------------------------------------------------------
#  compute_inflow_and_tip_loss
# ----------------------------------------------------------------------------------------------------------------------    
def compute_inflow_and_tip_loss(r,R,Wa,Wt,B,et1=1,et2=1,et3=1):
    """
    Computes the inflow, lamdaw, and the tip loss factor, F.

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
       r          radius distribution                                              [m]
       R          tip radius                                                       [m]
       Wa         axial velocity                                                   [m/s]
       Wt         tangential velocity                                              [m/s]
       B          number of rotor blades                                           [-]
       et1        tuning parameter for tip loss function 
       et2        tuning parameter for tip loss function 
       et3        tuning parameter for tip loss function 
       
    Outputs:               
       lamdaw     inflow ratio                                                     [-]
       F          tip loss factor                                                  [-]
       piece      output of a step in tip loss calculation (needed for residual)   [-]
    """
    xp                 = array_namespace(r, Wa, Wt)
    lamdaw             = r*Wa/(R*Wt)
    lamdaw             = xp.where(lamdaw <= 0., 1e-12, lamdaw)

    tipfactor = B/2.0*(  (R/r)**et1 - 1  )**et2/lamdaw**et3 

    piece = xp.exp(-tipfactor)
    Ftip  = 2.*xp.arccos(piece)/np.pi  

    return lamdaw, Ftip, piece