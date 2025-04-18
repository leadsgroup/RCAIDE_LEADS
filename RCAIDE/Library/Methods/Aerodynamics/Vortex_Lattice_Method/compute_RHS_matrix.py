# RCAIDE/Methods/Aerodynamics/Common/Lift/compute_RHS_matrix.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
import RCAIDE
from RCAIDE.Framework.Core import Data 

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Compute RHS matrix 
# ----------------------------------------------------------------------------------------------------------------------    
def compute_RHS_matrix(delta,phi,conditions,settings,geometry,propeller_wake_model):

    """ This computes the right hand side matrix for the VLM. In this
    function, induced velocites from propeller wake are also included
    when relevent and where specified

    Source:
    1. Low-Speed Aerodynamics, Second Edition by Joseph Katz, Allen Plotkin
    Pgs. 331-338

    2. VORLAX Source Code

    Inputs:
    geometry
        networks                                 [Unitless]
        vehicle vortex distribution              [Unitless]

    conditions.aerodynamics.angles.alpha      [radians]
    conditions.aerodynamics.angles.beta     [radians]
    conditions.freestream.velocity               [m/s]
    conditions.static_stability.pitch_rate      [radians/s]
    conditions.static_stability.roll_rate       [radians/s]
    conditions.static_stability.yaw_rate        [radians/s]

    sur_flag    - use_surrogate flag             [Unitless]
    slipstream  - propeller_wake_model flag      [Unitless]
    delta, phi  - flow tangency angles           [radians]

    Outputs:
    rhs.
        RHS
        ONSET
        Vx_ind_total
        Vz_ind_total
        V_distribution
        dt
        YGIRO
        ZGIRO
        VX
        SCNTL
        CCNTL
        COD
        SID

    Properties Used:
    N/A
    """

    # unpack
    VD               = geometry.vortex_distribution

    aoa              = conditions.aerodynamics.angles.alpha
    aoa_distribution = np.repeat(aoa, VD.n_cp, axis = 1)
    PSI              = conditions.aerodynamics.angles.beta
    PSI_distribution = np.repeat(PSI, VD.n_cp, axis = 1)
    V_inf            = conditions.freestream.velocity
    V_distribution   = np.repeat(V_inf , VD.n_cp, axis = 1)
    num_ctrl_pts     = len(aoa)  
    num_eval_pts     = len(VD.XC)

    rot_V_wake_ind   = np.zeros((num_ctrl_pts, VD.n_cp,3))
    Vx_ind_total     = np.zeros_like(V_distribution)
    Vy_ind_total     = np.zeros_like(V_distribution)
    Vz_ind_total     = np.zeros_like(V_distribution) 
    dt               = 0
    for network in geometry.networks:
        if propeller_wake_model:
            rot_V_wake_ind = np.zeros((num_ctrl_pts,num_eval_pts,3))
            for propulsor in network.propulsors: 
                if 'rotor' in  propulsor:
                    rotor =  propulsor.rotor
                elif 'propeller' in  propulsor :
                    rotor =  propulsor.propeller
                rotor_conditions =  conditions.energy.converters[rotor.tag]
                if rotor.fidelity == "Blade_Element_Momentum_Theory_Helmholtz_Wake":                 
                    rot_V_wake_ind += RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Blade_Element_Momentum_Theory_Helmholtz_Wake.wake_model.evaluate_slipstream(rotor,rotor_conditions,geometry,num_ctrl_pts) 
                    
            # update the total induced velocity distribution
            Vx_ind_total = Vx_ind_total  + rot_V_wake_ind[:,:,0]
            Vy_ind_total = Vy_ind_total  + rot_V_wake_ind[:,:,1]
            Vz_ind_total = Vz_ind_total  + rot_V_wake_ind[:,:,2]

            rhs = build_RHS(VD, conditions, settings, aoa_distribution, delta, phi, PSI_distribution,
                            Vx_ind_total, Vy_ind_total, Vz_ind_total, V_distribution, dt)           
            
            return  rhs

    rhs = build_RHS(VD, conditions, settings, aoa_distribution, delta, phi, PSI_distribution,
                    Vx_ind_total, Vy_ind_total, Vz_ind_total, V_distribution, dt)
    
    return rhs 

def build_RHS(VD, conditions, settings, aoa_distribution, delta, phi, PSI_distribution,
              Vx_ind_total, Vy_ind_total, Vz_ind_total, V_distribution, dt):
    """ This uses freestream conditions, induced wake velocities, and rotation
    rates (pitch, roll, yaw) to find the boundary conditions (RHS) needed to
    compute gamma. The function defaults to a classical boundary condition
    equation, RHS = V dot N, where V is the unit velocity vector only the panel
    and N is the panel unit normal. However, the user may also define
    settings.use_VORLAX_matrix_calculation to use the boundary condition equation
    from VORLAX, the code on which this VLM is based. This is useful for future
    developers who need to compare numbers 1:1 with VORLAX.

    Note if using VORLAX boundary conditions:
    VORLAX does not take induced wake velocity into account. Additionally,
    VORLAX uses the camber, twist, and dihedral values of a strip's leading
    edge panel for every panel in that strip when calculating panel normals.

    Source:
    1. Low-Speed Aerodynamics, Second Edition by Joseph Katz, Allen Plotkin Pgs. 331-338

    2. VORLAX Source Code

    Inputs:
    settings.use_VORLAX_matrix_calculation  - RHS equation switch               [boolean]
    conditions.static_stability.pitch_rate -                                   [radians/s]
    conditions.static_stability.roll_rate  -                                   [radians/s]
    conditions.static_stability.yaw_rate   -                                   [radians/s]
    aoa_distribution                        - angle of attack                   [radians]
    PSI_distribution                        - sideslip angle                    [radians]
    V[]_ind_total                           - component induced wake velocities [m/s]
    V_distribution                          - freestream velocity magnitude     [m/s]
    delta, phi                              - flow tangency angles              [radians]

    Outputs:
    rhs  -  a Data object used to hold several values including RHS

    Properties Used:
    N/A
    """
    LE_ind      = VD.leading_edge_indices
    RNMAX       = VD.panels_per_strip


    # VORLAX frame RHS calculation---------------------------------------------------------
    #VORLAX subroutine = BOUNDY
    #unpack conditions
    ALFA   = aoa_distribution
    PSIRAD = PSI_distribution
    PITCHQ = conditions.static_stability.pitch_rate
    ROLLQ  = conditions.static_stability.roll_rate
    YAWQ   = conditions.static_stability.yaw_rate
    VINF   = conditions.freestream.velocity

    SINALF = np.sin(ALFA)
    COSIN  = np.cos(ALFA) * np.sin(PSIRAD)
    COSCOS = np.cos(ALFA) * np.cos(PSIRAD)
    PITCH  = PITCHQ / VINF
    ROLL   = ROLLQ  / VINF
    YAW    = YAWQ   / VINF

    #unpack/rename variables from VD
    X      = VD.XCH
    YY     = VD.YCH
    ZZ     = VD.ZCH
    XBAR   = VD.XBAR
    ZBAR   = VD.ZBAR

    CHORD  = VD.chord_lengths[0,:]
    DELTAX = 0.5/RNMAX

    # LOCATE VORTEX LATTICE CONTROL POINT WITH RESPECT TO THE
    # ROTATION CENTER (XBAR, 0, ZBAR). THE RELATIVE COORDINATES
    # ARE XGIRO, YGIRO, AND ZGIRO.
    XGIRO = X + CHORD*DELTAX - np.repeat(XBAR, RNMAX[LE_ind])
    YGIRO = YY
    ZGIRO = ZZ - np.repeat(ZBAR, RNMAX[LE_ind])

    # VX, VY, VZ ARE THE FLOW ONSET VELOCITY COMPONENTS AT THE LEADING
    # EDGE (STRIP MIDPOINT). VX, VY, VZ AND THE ROTATION RATES ARE
    # REFERENCED TO THE FREE STREAM VELOCITY.
    VX = (COSCOS - PITCH*ZGIRO + YAW  *YGIRO)
    VY = (COSIN  - YAW  *XGIRO + ROLL *ZGIRO)
    VZ = (SINALF - ROLL *YGIRO + PITCH*XGIRO)
    
    #COMPUTE DIRECTION COSINES.
    SCNTL  = VD.SLOPE/np.sqrt(1. + VD.SLOPE **2)
    CCNTL  = 1. / np.sqrt(1.0 + SCNTL**2)
    phi_LE = np.repeat(phi[:,LE_ind]  , RNMAX[LE_ind], axis=1)
    COD    = np.cos(phi_LE)
    SID    = np.sin(phi_LE)

    # COMPUTE ONSET FLOW COMPONENT ALONG THE OUTWARD NORMAL TO
    # THE SURFACE AT THE CONTROL POINT, ALOC.
    ALOC  = VX *SCNTL + VY *CCNTL *SID - VZ *CCNTL *COD

    # COMPUTE VELOCITY COMPONENT ALONG X-AXIS INDUCED BY THE RIGID
    # BODY ROTATION, ONSET.
    ONSET = - PITCH *ZGIRO + YAW *YGIRO

    # Body-Frame RHS calculation----------------------------------------------------------
    # Add wake and rotation effects to the freestream
    Vx_rotation       = -PITCHQ*ZGIRO + YAWQ  *YGIRO
    Vy_rotation       = -YAWQ  *XGIRO + ROLLQ *ZGIRO
    Vz_rotation       = -ROLLQ *YGIRO + PITCHQ*XGIRO

    Vx                = V_distribution*np.cos(aoa_distribution)*np.cos(PSI_distribution) + Vx_rotation + Vx_ind_total
    Vy                = V_distribution*np.cos(aoa_distribution)*np.sin(PSI_distribution) + Vy_rotation + Vy_ind_total
    Vz                = V_distribution*np.sin(aoa_distribution)                          + Vz_rotation + Vz_ind_total    
    
    aoa_distribution  = np.arctan(Vz/ np.sqrt(Vx**2 + Vy**2) )
    PSI_distribution  = np.arctan(Vy / Vx)

    # compute RHS: dot(v, panel_normals)
    V_unit_vector    = (np.array([Vx,Vy,Vz])/V_distribution).T
    panel_normals    = VD.normals[:,np.newaxis,:]
    RHS_from_normals = np.sum(V_unit_vector*panel_normals, axis=2).T    

    #pack values--------------------------------------------------------------------------
    use_VORLAX_RHS = settings.use_VORLAX_matrix_calculation

    rhs = Data()
    rhs.RHS            = RHS_from_normals if not use_VORLAX_RHS else ALOC
    rhs.ONSET          = ONSET
    rhs.Vx_ind_total   = Vx_ind_total
    rhs.Vz_ind_total   = Vz_ind_total
    rhs.V_distribution = V_distribution
    rhs.dt             = dt

    #these values will be used later to calculate EFFINC
    rhs.YGIRO  = YGIRO
    rhs.ZGIRO  = ZGIRO
    rhs.VX     = VX
    rhs.SCNTL  = SCNTL
    rhs.CCNTL  = CCNTL
    rhs.COD    = COD
    rhs.SID    = SID

    return rhs
