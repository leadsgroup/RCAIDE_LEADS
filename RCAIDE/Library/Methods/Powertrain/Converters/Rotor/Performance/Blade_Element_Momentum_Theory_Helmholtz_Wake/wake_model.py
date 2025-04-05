# RCAIDE/Library/Framework/Analyses/Propulsion/Blade_Element_Momentum_Theory_Helmholtz_Wake/wake_model.py
#
# Created:  Jan 2022, R. Erhard
# Modified:  Jun 2024, M. Clarke  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Library.Components import Wings 
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.extract_wing_collocation_points import extract_wing_collocation_points
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Blade_Element_Momentum_Theory_Helmholtz_Wake  import compute_wake_induced_velocity
from RCAIDE.Library.Methods.Aerodynamics.Common.Lift.BET_calculations import compute_airfoil_aerodynamics,compute_inflow_and_tip_loss
# Python imports
import numpy as np 
import scipy as sp

# ----------------------------------------------------------------------------------------------------------------------
# wake model
# ---------------------------------------------------------------------------------------------------------------------- 
def evaluate_wake(rotor,wake_inputs,conditions):
    """
    
    Wake evaluation is performed using Helmholtz vortex theory.
    
    Assumptions:
    None

    Source:
    Drela, M. "Qprop Formulation", MIT AeroAstro, June 2006
    http://web.mit.edu/drela/Public/web/qprop/qprop_theory.pdf

    Inputs:
        rotor        - RCAIDE rotor
       wake_inputs.
          Ua        - Axial velocity
          Ut        - Tangential velocity
          r         - radius distribution
       conditions   - conditions
       
       
    Outputs:
       va  - axially-induced velocity from rotor wake
       vt  - tangentially-induced velocity from rotor wake
    
    Properties Used:
    None
    
    
    """
    
    va, vt = wake_convergence(rotor, wake_inputs)
        
    return va, vt

def evaluate_slipstream(rotor,rotor_conditions,geometry,ctrl_pts,wing_instance=None):
    """
    Evaluates the velocities induced by the rotor on a specified wing of the vehicle.
    If no wing instance is specified, uses main wing or last available wing in geometry.
    
    Assumptions:
    None

    Source:
    N/A

    Inputs:
       self         - rotor wake
       rotor        - rotor
       geometry     - vehicle geometry
       
    Outputs:
       wake_V_ind   - induced velocity from rotor wake at (VD.XC, VD.YC, VD.ZC)
    
    Properties Used:
    None
    """
    # Check for wing if wing instance is unspecified
    if wing_instance == None:
        nmw = 0
        # check for main wing
        for i,wing in enumerate(geometry.wings):
            if not isinstance(wing,Wings.Main_Wing): continue
            nmw +=1                
            wing_instance = wing
            wing_instance_idx = i
        if nmw == 1:
            pass
        elif nmw>1:
            print("No wing specified for slipstream analysis. Multiple main wings in vehicle, using the last one.")
        else:
            print("No wing specified for slipstream analysis. No main wing defined, using the last wing in vehicle.")
            wing_instance = wing 
            wing_instance_idx = i
    
    # Isolate the VD components corresponding to this wing instance
    wing_CPs, slipstream_vd_ids = extract_wing_collocation_points(geometry, wing_instance_idx)
    
    # Evaluate rotor slipstream effect on specified wing instance
    rot_V_wake_ind = evaluate_wake_velocities(rotor,rotor_conditions,wing_CPs,ctrl_pts)
    
    # Expand
    wake_V_ind = np.zeros((ctrl_pts,geometry.vortex_distribution.n_cp,3))
    wake_V_ind[:,slipstream_vd_ids,:] = rot_V_wake_ind
    
        
    return wake_V_ind

def evaluate_wake_velocities(rotor,rotor_conditions,evaluation_points,ctrl_pts):
    """
    Links the rotor wake to compute the wake-induced velocities at the specified
    evaluation points.
    
    Assumptions:
    None

    Source:
    N/A

    Inputs:
       self               - rotor wake
       rotor              - rotor
       evaluation_points  - points at which to evaluate the rotor wake-induced velocities 
       
    Outputs:
       prop_V_wake_ind  - induced velocity from rotor wake at (VD.XC, VD.YC, VD.ZC)
    
    Properties Used:
    None
    """  
     
    rot_V_wake_ind = compute_wake_induced_velocity(rotor,rotor_conditions,evaluation_points,ctrl_pts)  
    
    return rot_V_wake_ind


# ---------------------------------------------------------------------------------------------------------------------- 
#  wake_convergence
# ---------------------------------------------------------------------------------------------------------------------- 
def wake_convergence(rotor,wake_inputs):
    """
    Wake evaluation is performed using a simplified vortex wake method for Fidelity Zero, 
    following Helmholtz vortex theory.
    
    Assumptions:
    None

    Source:
    Drela, M. "Qprop Formulation", MIT AeroAstro, June 2006
    http://web.mit.edu/drela/Public/web/qprop/qprop_theory.pdf

    Inputs:
       self         - rotor wake
       rotor        - SUAVE rotor
       wake_inputs.
          Ua        - Axial velocity
          Ut        - Tangential velocity
          r         - radius distribution
       
       
    Outputs:
       va  - axially-induced velocity from rotor wake
       vt  - tangentially-induced velocity from rotor wake
    
    Properties Used:
    None
    
    """        
    
    # Unpack some wake inputs
    ctrl_pts        = wake_inputs.ctrl_pts
    Nr              = wake_inputs.Nr
    Na              = wake_inputs.Na    

    if wake_inputs.use_2d_analysis:
        PSI    = np.ones((ctrl_pts,Nr,Na))
    else:
        PSI    = np.ones((ctrl_pts,Nr))

    PSI_final,infodict,ier,msg = sp.optimize.fsolve(iteration,PSI,args=(wake_inputs,rotor),xtol=rotor.sol_tolerance,full_output = 1,band=(1,0))
    
    # Calculate the velocities given PSI
    va, vt = va_vt(PSI_final, wake_inputs, rotor)

    
    return va, vt

def iteration(PSI, wake_inputs, rotor):
    """
    Computes the BEVW iteration.

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
       B                          number of rotor blades                          [-]
       beta                       blade twist distribution                        [-]
       r                          radius distribution                             [m]
       R                          tip radius                                      [m]
       Wt                         tangential velocity                             [m/s]
       Wa                         axial velocity                                  [m/s]
       U                          total velocity                                  [m/s]
       Ut                         tangential velocity                             [m/s]
       Ua                         axial velocity                                  [m/s]
       cos_psi                    cosine of the inflow angle PSI                  [-]
       sin_psi                    sine of the inflow angle PSI                    [-]
       piece                      output of a step in tip loss calculation        [-]

    Outputs:
       dR_dpsi                    derivative of residual wrt inflow angle         [-]

    """    
    
    # Unpack inputs to rotor wake fidelity zero
    U               = wake_inputs.velocity_total
    Ua              = wake_inputs.velocity_axial
    Ut              = wake_inputs.velocity_tangential
    use_2d_analysis = wake_inputs.use_2d_analysis        
    beta            = wake_inputs.twist_distribution
    c               = wake_inputs.chord_distribution
    r               = wake_inputs.radius_distribution
    a               = wake_inputs.speed_of_sounds
    nu              = wake_inputs.dynamic_viscosities
    ctrl_pts        = wake_inputs.ctrl_pts
    Nr              = wake_inputs.Nr
    Na              = wake_inputs.Na

    # Unpack rotor data        
    R            = rotor.tip_radius
    B            = rotor.number_of_blades
    tc           = rotor.thickness_to_chord
    airfoils     = rotor.airfoils
    a_loc        = rotor.airfoil_polar_stations
    
    # Reshape PSI because the solver gives it flat
    if wake_inputs.use_2d_analysis:
        PSI    = np.reshape(PSI,(ctrl_pts,Nr,Na))
    else:
        PSI    = np.reshape(PSI,(ctrl_pts,Nr))
    
    # compute velocities
    sin_psi      = np.sin(PSI)
    cos_psi      = np.cos(PSI)
    Wa           = 0.5*Ua + 0.5*U*sin_psi
    Wt           = 0.5*Ut + 0.5*U*cos_psi
    vt           = Ut - Wt

    # compute blade airfoil forces and properties
    Cl, Cdval, alpha, alpha_disc,Ma,W,Re,Re_disc = compute_airfoil_aerodynamics(beta,c,r,R,B,Wa,Wt,a,nu,airfoils,a_loc,ctrl_pts,Nr,Na,tc,use_2d_analysis)

    # compute inflow velocity and tip loss factor
    lamdaw, F, piece = compute_inflow_and_tip_loss(r,R,Wa,Wt,B)

    # compute Newton residual on circulation
    Gamma       = vt*(4.*np.pi*r/B)*F*(1.+(4.*lamdaw*R/(np.pi*B*r))*(4.*lamdaw*R/(np.pi*B*r)))**0.5
    Rsquiggly   = Gamma - 0.5*W*c*Cl
    
    return Rsquiggly.flatten()
 
def va_vt(PSI, wake_inputs, rotor):
    """
    Computes the inflow velocities

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
       B                          number of rotor blades                          [-]
       beta                       blade twist distribution                        [-]
       r                          radius distribution                             [m]
       R                          tip radius                                      [m]
       Wt                         tangential velocity                             [m/s]
       Wa                         axial velocity                                  [m/s]
       U                          total velocity                                  [m/s]
       Ut                         tangential velocity                             [m/s]
       Ua                         axial velocity                                  [m/s]
       cos_psi                    cosine of the inflow angle PSI                  [-]
       sin_psi                    sine of the inflow angle PSI                    [-]
       piece                      output of a step in tip loss calculation        [-]

    Outputs:
       dR_dpsi                    derivative of residual wrt inflow angle         [-]

    """    
    
    # Unpack inputs to rotor wake fidelity zero
    U               = wake_inputs.velocity_total
    Ua              = wake_inputs.velocity_axial
    Ut              = wake_inputs.velocity_tangential
    ctrl_pts        = wake_inputs.ctrl_pts
    Nr              = wake_inputs.Nr
    Na              = wake_inputs.Na
    
    # Reshape PSI because the solver gives it flat
    if wake_inputs.use_2d_analysis:
        PSI    = np.reshape(PSI,(ctrl_pts,Nr,Na))
    else:
        PSI    = np.reshape(PSI,(ctrl_pts,Nr))
    
    # compute velocities
    sin_psi      = np.sin(PSI)
    cos_psi      = np.cos(PSI)
    Wa           = 0.5*Ua + 0.5*U*sin_psi
    Wt           = 0.5*Ut + 0.5*U*cos_psi
    va           = Wa - Ua
    vt           = Ut - Wt

    return va, vt
 
def compute_dR_dpsi(PSI,wake_inputs,rotor):
    """
    Computes the analytical derivative for the BEVW iteration.

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
       B                          number of rotor blades                          [-]
       beta                       blade twist distribution                        [-]
       r                          radius distribution                             [m]
       R                          tip radius                                      [m]
       Wt                         tangential velocity                             [m/s]
       Wa                         axial velocity                                  [m/s]
       U                          total velocity                                  [m/s]
       Ut                         tangential velocity                             [m/s]
       Ua                         axial velocity                                  [m/s]
       cos_psi                    cosine of the inflow angle PSI                  [-]
       sin_psi                    sine of the inflow angle PSI                    [-]
       piece                      output of a step in tip loss calculation        [-]

    Outputs:
       dR_dpsi                    derivative of residual wrt inflow angle         [-]

    """
    # Unpack inputs to rotor wake fidelity zero
    U               = wake_inputs.velocity_total
    Ua              = wake_inputs.velocity_axial
    Ut              = wake_inputs.velocity_tangential
    beta            = wake_inputs.twist_distribution
    r               = wake_inputs.radius_distribution
    ctrl_pts        = wake_inputs.ctrl_pts
    Nr              = wake_inputs.Nr
    Na              = wake_inputs.Na    

    # Unpack rotor data        
    R        = rotor.tip_radius
    B        = rotor.number_of_blades      
    
    # Reshape PSI because the solver gives it flat
    if wake_inputs.use_2d_analysis:
        PSI    = np.reshape(PSI,(ctrl_pts,Nr,Na))
    else:
        PSI    = np.reshape(PSI,(ctrl_pts,Nr))    
    
    
    # An analytical derivative for dR_dpsi used in the Newton iteration for the BEVW
    # This was solved symbolically in Matlab and exported
    # compute velocities
    sin_psi      = np.sin(PSI)
    cos_psi      = np.cos(PSI)
    Wa           = 0.5*Ua + 0.5*U*sin_psi
    Wt           = 0.5*Ut + 0.5*U*cos_psi
    
    lamdaw, F, piece = compute_inflow_and_tip_loss(r,R,Wa,Wt,B)
    
    pi          = np.pi
    pi2         = np.pi**2
    BB          = B*B
    BBB         = BB*B
    f_wt_2      = 4*Wt*Wt
    f_wa_2      = 4*Wa*Wa
    arccos_piece = np.arccos(piece)
    Ucospsi     = U*cos_psi
    Usinpsi     = U*sin_psi
    Utcospsi    = Ut*cos_psi
    Uasinpsi    = Ua*sin_psi
    UapUsinpsi  = (Ua + Usinpsi)
    utpUcospsi  = (Ut + Ucospsi)
    utpUcospsi2 = utpUcospsi*utpUcospsi
    UapUsinpsi2 = UapUsinpsi*UapUsinpsi
    dR_dpsi     = ((4.*U*r*arccos_piece*sin_psi*((16.*UapUsinpsi2)/(BB*pi2*f_wt_2) + 1.)**(0.5))/B -
                   (pi*U*(Ua*cos_psi - Ut*sin_psi)*(beta - np.arctan((Wa+Wa)/(Wt+Wt))))/(2.*(f_wt_2 + f_wa_2)**(0.5))
                   + (pi*U*(f_wt_2 +f_wa_2)**(0.5)*(U + Utcospsi  +  Uasinpsi))/(2.*(f_wa_2/(f_wt_2) + 1.)*utpUcospsi2)
                   - (4.*U*piece*((16.*UapUsinpsi2)/(BB*pi2*f_wt_2) + 1.)**(0.5)*(R - r)*(Ut/2. -
                    (Ucospsi)/2.)*(U + Utcospsi + Uasinpsi ))/(f_wa_2*(1. - np.exp(-(B*(Wt+Wt)*(R -
                    r))/(r*(Wa+Wa))))**(0.5)) + (128.*U*r*arccos_piece*(Wa+Wa)*(Ut/2. - (Ucospsi)/2.)*(U +
                    Utcospsi  + Uasinpsi ))/(BBB*pi2*utpUcospsi*utpUcospsi2*((16.*f_wa_2)/(BB*pi2*f_wt_2) + 1.)**(0.5)))

    dR_dpsi[np.isnan(dR_dpsi)] = 0.1
    
    # This needs to be made into a jacobian
    dR_dpsi = dR_dpsi.flatten()
    L       = np.size(PSI)
    jac     = np.eye(L)*dR_dpsi
    
    return jac