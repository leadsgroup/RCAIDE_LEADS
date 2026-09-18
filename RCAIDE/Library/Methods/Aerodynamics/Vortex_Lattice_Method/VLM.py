# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/VLM.py
#
# Created: Aug 2025, M. Clarke
# Modified:Apr 2026, S. Shekar, A. Molloy, M. Clarke
#          May 2026  M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# package imports
from RCAIDE.Framework.Core import Data
from .compute_wing_induced_velocity      import compute_wing_induced_velocity
from .generate_vortex_distribution       import generate_vortex_distribution
from .compute_RHS_matrix                 import compute_RHS_matrix

from scipy.integrate import trapezoid
import numpy as np
# ----------------------------------------------------------------------
#  Vortex Lattice
# ----------------------------------------------------------------------

def VLM(conditions,settings,geometry):
    """Uses the vortex lattice method to compute the lift, induced drag and moment coefficients.

    The user should be forwarned that this will cause very slight differences in results for 0 deflection due to
    the slightly different discretization.

    The user has the option to use the boundary conditions and induced velocities from either RCAIDE
    or VORLAX. See build_RHS in compute_RHS_matrix.py for more details.

    By default in Vortex_Lattice, VLM performs calculations based on panel coordinates with float32 precision.
    The user may also choose to use float16 or float64, but be warned that the latter can be memory intensive.

    The user should note that fully capitalized variables correspond to a VORLAX variable of the same name


    Assumptions:
    The user provides either global discretezation (number_spanwise/chordwise_vortices) or
    separate discretization (wing/fuselage_spanwise/chordwise_vortices) in settings, not both.
    The set of settings not being used should be set to None.

    The VLM requires that the user provide a non-zero velocity that matches mach number. For
    surrogate training cases at mach 0, VLM uses a velocity of 1e-6 m/s


    Source:
    1. Miranda, Luis R., Robert D. Elliot, and William M. Baker. "A generalized vortex
    lattice method for subsonic and supersonic flow applications." (1977). (NASA CR)

    2. VORLAX Source Code


    Inputs:
    geometry.
       reference_area                          [m^2]
       wing.
         spans.projected                       [m]
         chords.root                           [m]
         chords.tip                            [m]
         sweeps.quarter_chord                  [radians]
         taper                                 [Unitless]
         twists.root                           [radians]
         twists.tip                            [radians]
         xz_plane_symmetric                    [Boolean]
         aspect_ratio                          [Unitless]
         areas.reference                       [m^2]
         vertical                              [Boolean]
         origin                                [m]
       fuselage.
        origin                                 [m]
        width                                  [m]
        heights.maximum                        [m]
        lengths.nose                           [m]
        lengths.tail                           [m]
        lengths.total                          [m]
        lengths.cabin                          [m]
        fineness.nose                          [Unitless]
        fineness.tail                          [Unitless]

    settings.number_of_spanwise_vortices       [Unitless]
    settings.number_of_chordwise_vortices      [Unitless]

    settings.use_surrogate                     [Unitless]
    settings.propeller_wake_model              [Unitless]
    settings.use_VORLAX_matrix_calculation     [boolean]
    settings.floating_point_precision          [float16/32/64]

    conditions.aerodynamics.angles.alpha       [radians]
    conditions.aerodynamics.angles.beta        [radians]
    conditions.freestream.mach_number          [Unitless]
    conditions.freestream.velocity             [m/s]
    conditions.static_stability.pitch_rate     [radians/s]
    conditions.static_stability.roll_rate      [radians/s]
    conditions.static_stability.yaw_rate       [radians/s]


    Outputs:
    results.
        CL                                     [Unitless], CLTOT in VORLAX
        CDi                                    [Unitless], CDTOT in VORLAX
        CM                                     [Unitless], CMTOT in VORLAX
        CY                                     [Unitless], Total y force coeff
        CRTOT                                  [Unitless], Rolling moment coeff (unscaled)
        CL_mom                                 [Unitless], Rolling moment coeff (scaled by b_ref)
        CNTOT                                  [Unitless], Yawing  moment coeff (unscaled)
        CN                                     [Unitless], Yawing  moment coeff (scaled by b_ref)
        CL_wing                                [Unitless], CL  of each wing
        CDi_wing                               [Unitless], CDi of each wing
        cl_y                                   [Unitless], CL  of each strip
        cdi_y                                  [Unitless], CDi of each strip
        alpha_i                                [radians] , Induced angle of each strip in each wing (array of numpy arrays)
        CP                                     [Unitless], Pressure coefficient of each panel
        gamma                                  [Unitless], Vortex strengths of each panel


    Properties Used:
    N/A
    """

    S_ref = geometry.reference_area
    c_ref = geometry.reference_chord
    b_ref = geometry.reference_span
    x_m   = geometry.mass_properties.center_of_gravity[0][0]
    z_m   = geometry.mass_properties.center_of_gravity[0][2]

    # ---------------------------------------------------------------------------------------
    # Generate Panelization and Vortex Distribution
    # ------------------ --------------------------------------------------------------------
    VD                                                    = generate_vortex_distribution(conditions,settings,geometry)
    settings.vortex_distribution.chord_lengths            = VD.chord_lengths[VD.leading_edge_indices].reshape(len(VD.n_sw),np.sum(VD.n_sw[0]))
    settings.vortex_distribution.n_sw                     = VD.n_sw
    settings.vortex_distribution.n_cw                     = VD.n_cw
    settings.vortex_distribution.n_w                      = VD.n_w
    settings.vortex_distribution.chord_widths             = VD.chord_widths
    settings.vortex_distribution.leading_edge_sweeps      = VD.leading_edge_sweeps
    settings.vortex_distribution.XA1                      = VD.XA1
    settings.vortex_distribution.XA2                      = VD.XA2
    settings.vortex_distribution.XB1                      = VD.XB1
    settings.vortex_distribution.XB2                      = VD.XB2
    settings.vortex_distribution.YA1                      = VD.YA1
    settings.vortex_distribution.YA2                      = VD.YA2
    settings.vortex_distribution.YB1                      = VD.YB1
    settings.vortex_distribution.YB2                      = VD.YB2
    settings.vortex_distribution.ZA1                      = VD.ZA1
    settings.vortex_distribution.ZA2                      = VD.ZA2
    settings.vortex_distribution.ZB1                      = VD.ZB1
    settings.vortex_distribution.ZB2                      = VD.ZB2
    settings.vortex_distribution.X                        = VD.X
    settings.vortex_distribution.Y                        = VD.Y
    settings.vortex_distribution.Z                        = VD.Z
    settings.vortex_distribution.XC                       = VD.XC
    settings.vortex_distribution.YC                       = VD.YC
    settings.vortex_distribution.ZC                       = VD.ZC
    settings.vortex_distribution.Y_SW                     = VD.Y_SW

    # unpack conditions--------------------------------------------------------------
    pwm      = settings.propeller_wake_model
    K_SPC    = settings.leading_edge_suction_multiplier
    aoa      = conditions.aerodynamics.angles.alpha
    mach     = conditions.freestream.mach_number
    len_mach = len(mach)

    # VORLAX's angles are all degrees-to-radians via DTR; RCAIDE's are radians/Units.degrees throughout.
    PSI       = conditions.aerodynamics.angles.beta
    PITCHQ    = conditions.static_stability.pitch_rate
    ROLLQ     = conditions.static_stability.roll_rate
    YAWQ      = conditions.static_stability.yaw_rate
    VINF      = conditions.freestream.velocity

    #freestream 0 velocity safeguard
    if not conditions.freestream.velocity.all():
        if settings.use_surrogate:
            velocity                       = conditions.freestream.velocity
            velocity[velocity==0]          = np.ones(len(velocity[velocity==0])) * 1e-6
            conditions.freestream.velocity = velocity
        else:
            raise AssertionError("VLM requires that conditions.freestream.velocity be specified and non-zero")


    # Unpack vortex distribution
    CHORD        = VD.chord_lengths
    chord_breaks = VD.chordwise_breaks
    span_breaks  = VD.spanwise_breaks
    RNMAX        = VD.panels_per_strip
    LE_ind       = VD.leading_edge_indices
    ZETA         = VD.tangent_incidence_angle
    RK           = VD.chordwise_panel_number

    exposed_leading_edge_flag = VD.exposed_leading_edge_flag

    YAH = VD.YAH*1.
    YBH = VD.YBH*1.
    XA1 = VD.XA1*1.
    XB1 = VD.XB1*1.

    # Compute X and Z BAR ouside of generate_vortex_distribution to avoid requiring x_m and z_m as inputs
    VD.XBAR = np.ones(( len_mach,sum(LE_ind[0]))) * x_m
    VD.ZBAR = np.ones(( len_mach,sum(LE_ind[0]))) * z_m

    # ---------------------------------------------------------------------------------------
    # STEP 10: Generate A and RHS matrices from VD and geometry
    # ------------------ --------------------------------------------------------------------
    # Compute flow tangency conditions
    phi   = np.arctan((VD.ZBC - VD.ZAC)/(VD.YBC - VD.YAC)) # dihedral angle
    delta = np.arctan((VD.ZC - VD.ZCH)/((VD.XC - VD.XCH))) # mean camber surface angle

    # Build the RHS vector
    rhs     = compute_RHS_matrix(VD,delta,phi,conditions,settings,geometry,pwm)
    RHS     = rhs.RHS*1 # this matches numpy=1.26 in terms of dimension
    ONSET   = rhs.ONSET*1

    # Build induced velocity matrix, C_mn
    C_mn, s, RFLAG, EW = compute_wing_induced_velocity(VD,mach,compute_EW=True)

    # Turn off sonic vortices when Mach>1
    RHS = RHS*RFLAG

    # To ensure compatibility for np.linalg.solve across numpy1.0 and numpy2.0
    RHS = np.atleast_3d(RHS)

    # Build Aerodynamic Influence Coefficient Matrix
    use_VORLAX_induced_velocity = settings.use_VORLAX_matrix_calculation
    if not use_VORLAX_induced_velocity:
        A =   np.multiply(C_mn[:,:,:,0],np.atleast_3d(np.sin(delta)*np.cos(phi))) \
            + np.multiply(C_mn[:,:,:,1],np.atleast_3d(np.cos(delta)*np.sin(phi))) \
            - np.multiply(C_mn[:,:,:,2],np.atleast_3d(np.cos(phi)*np.cos(delta)))   # validated from book eqn 7.42
    else:
        A = EW

    # Compute vortex strength
    GAMMA  = np.linalg.solve(A,RHS)

    # To ensure compatibility for np.linalg.solve across numpy1.0 and numpy2.0
    RHS    = RHS.squeeze(axis=2)
    GAMMA  = GAMMA.squeeze(axis=2)

    # ---------------------------------------------------------------------------------------
    # STEP 11: Compute Pressure Coefficient
    # ------------------ --------------------------------------------------------------------
    #VORLAX subroutine = PRESS

    # spanwise strip exposure flag, always 0 for RCAIDE's infinitely thin airfoils. Needs to change if thick airfoils added
    RJTS = 0

    # COMPUTE FREE-STREAM AND ONSET FLOW PARAMETERS. Used throughout the remainder of VLM
    B2     = np.tile((mach**2 - 1),VD.n_cp[0])
    SINALF = np.sin(aoa)
    COSALF = np.cos(aoa)
    SINPSI = np.sin(PSI)
    COPSI  = np.cos(PSI)
    COSIN  = COSALF *SINPSI *2.0
    COSINP = COSALF *SINPSI
    COSCOS = COSALF *COPSI
    PITCH  = PITCHQ /VINF
    ROLL   = ROLLQ  /VINF
    YAW    = YAWQ   /VINF

    # reshape CHORD
    dim_1 = len(np.sum(LE_ind, axis=1))
    dim_2 = np.sum(LE_ind, axis=1)[0]

    # COMPUTE EFFECT OF SIDESLIP on DCP intermediate variables. needs change if cosine chorwise spacing added
    FORAXL = COSCOS
    FORLAT = COSIN
    TAN_LEi= (VD.XB1[:,LE_ind[0]]-VD.XA1[:,LE_ind[0]])/  np.sqrt((VD.ZB1[:,LE_ind[0]]-VD.ZA1[:,LE_ind[0]])**2 +  (VD.YB1[:,LE_ind[0]]-VD.YA1[:,LE_ind[0]])**2)
    TAN_TE = (VD.XB_TE - VD.XA_TE)/ np.sqrt((VD.ZB_TE-VD.ZA_TE)**2 + (VD.YB_TE-VD.YA_TE)**2)
    TAN_LE = np.repeat( TAN_LEi, RNMAX[LE_ind].reshape(dim_1,dim_2)[0] , axis=1)

    TAN_LE = TAN_LE
    TNL    = TAN_LE * 1 # VORLAX's SIGN variable not needed, as these are taken directly from geometry
    TNT    = TAN_TE * 1
    XIA    = np.broadcast_to((RK-1)/RNMAX, np.shape(B2))
    XIB    = np.broadcast_to((RK  )/RNMAX, np.shape(B2))
    TANA   = TNL *(1. - XIA) + TNT *XIA
    TANB   = TNL *(1. - XIB) + TNT *XIB

    # cumsum GANT loop if KTOP > 0 (don't actually need KTOP with vectorized arrays and np.roll)
    GFX    = VD.chord_lengths
    GANT   = strip_cumsum(GFX*GAMMA, chord_breaks[0], RNMAX[LE_ind].reshape(dim_1,dim_2)[0]  )
    GANT   = np.roll(GANT,1)
    GANT[LE_ind] = 0

    GLAT   = GANT *(TANA - TANB) - GFX *GAMMA *TANB
    cos_DL = (YBH-YAH)[LE_ind].reshape(dim_1,dim_2)/VD.D
    COS_DL = np.repeat( cos_DL, RNMAX[LE_ind].reshape(dim_1,dim_2)[0] , axis=1)
    DCPSID = FORLAT * COS_DL *GLAT /(XIB - XIA)
    FACTOR = FORAXL + ONSET

    # COMPUTE LOAD COEFFICIENT
    GNET = GAMMA*FACTOR
    GNET = GNET *RNMAX /CHORD
    DCP  = 2*GNET + DCPSID
    CP   = DCP

    # ---------------------------------------------------------------------------------------
    # STEP 12: Compute aerodynamic coefficients
    # ------------------ --------------------------------------------------------------------
    # Flip coordinates on the other side of the wing
    boolean = YBH<0.
    XA1[boolean], XB1[boolean] = XB1[boolean], XA1[boolean]
    YAH[boolean], YBH[boolean] = YBH[boolean], YAH[boolean]

    # Leading edge sweep. VORLAX does it panel by panel. This will be spanwise.
    TLE   = TAN_LE[LE_ind].reshape(dim_1,dim_2)
    B2_LE = B2[LE_ind].reshape(dim_1,dim_2)
    T2    = TLE*TLE
    STB   = np.zeros_like(B2_LE)
    STB[B2_LE<T2] = np.sqrt(T2[B2_LE<T2]-B2_LE[B2_LE<T2])

    # COD/SID: dihedral angle (x-y plane) of each strip's horseshoe vortices, LE value only.
    COD = np.cos(phi[LE_ind]).reshape(dim_1,dim_2)
    SID = np.sin(phi[LE_ind]).reshape(dim_1,dim_2)

    # Now on to each strip
    PION = 2.0 /RNMAX
    ADC  = 0.5*PION

    # XLE = LOCATION OF FIRST VORTEX MIDPOINT IN FRACTION OF CHORD.
    XLE = 0.125 *PION

    GAF = 0.5 + 0.5 *RJTS**2

    # CORMED: strip-centerline length between load point and trailing edge, used for the
    # sideslip rolling-couple contribution (SICPLE) below.
    X      = VD.XCH                       #x-coord of load point (horseshoe centroid)
    XTE    = (VD.XA_TE + VD.XB_TE)/2   #Trailing edge x-coord behind the control point
    CORMED = XTE - X

    # SINF: IRT-vortex's load contribution to the strip's nominal (constant-span) area.
    SINF = ADC * DCP # The horshoe span lengths have been removed since VST/VSS == 1 always

    # Split into chordwise strengths and sum into strips
    # SICPLE = COUPLE (ABOUT STRIP CENTERLINE) DUE TO SIDESLIP.
    CNC    = np.add.reduceat(SINF       ,chord_breaks[0],axis=1)
    SICPLE = np.add.reduceat(SINF*CORMED,chord_breaks[0],axis=1)

    # TX: chordwise slope at load points, interpolated between control points (incidence included).
    XX   = (RK - .75) *PION /2.0
    TX    = VD.SLOPE - ZETA
    CAXL  = -SINF*TX/(1.0+TX**2) # These are the axial forces on each panel
    BMLE  = (XLE-XX)*SINF        # These are moment on each panel

    # Sum onto the panel
    CAXL = np.add.reduceat(CAXL,chord_breaks[0],axis=1)
    BMLE = np.add.reduceat(BMLE,chord_breaks[0],axis=1)

    # /3: VORLAX's original SICPLE over-predicts dCl/dbeta vs wind tunnel (Takahashi
    # et al., AIAA SciTech 2025, Sec. III.E).
    SICPLE *= (-1/3) * COSIN * COD * GAF
    DCP_LE = DCP[LE_ind].reshape(dim_1,dim_2)

    # CLE: leading-edge thrust coefficient, from the total induced flow at the leading edge.
    # Only strictly valid for cosine chordwise spacing (LAX = 0); RCAIDE only has linear
    # chordwise spacing today, so this underestimates the true magnitude. See
    # RCAIDE_VLM_Physics_and_Cleanup_Review.md section 1.2.
    CLE = compute_rotation_effects(VD, settings, EW, GAMMA, X, CHORD, XLE, VD.XBAR, rhs, COSINP, SINALF,COSCOS, PITCH, ROLL, YAW, STB, RNMAX)

    # Leading edge suction multiplier (negative integer if used). Defaults to 1.
    SPC  = K_SPC*np.ones_like(DCP_LE)

    # Subsonic + vortex lift enabled -> SPC becomes -1
    VL   = np.repeat(VD.vortex_lift,VD.n_sw[0], axis=1)
    m_b  = np.atleast_2d(mach[:,0]<1.)
    SPC_cond      = VL*m_b.T
    SPC[SPC_cond] = -1.
    SPC           = SPC * exposed_leading_edge_flag

    CLE  = CLE + 0.5* DCP_LE *np.sqrt(XLE[LE_ind].reshape(dim_1,dim_2))
    CSUC = 0.5*np.pi*np.abs(SPC)*(CLE**2)*STB

    # TFX/TFZ: leading-edge force vector components along the body x/z axes.
    SLE  = VD.SLOPE[LE_ind].reshape(dim_1,dim_2)
    ZETA = ZETA[LE_ind].reshape(dim_1,dim_2)
    XCOS = np.cos(SLE-ZETA)
    XSIN = np.sin(SLE-ZETA)
    TFX  =  1.*XCOS
    TFZ  = -1.*XSIN

    # Negative SPC: Lan's correction (see VORLAX documentation) applies instead.
    TFX[SPC<0] = XSIN[SPC<0]*np.sign(DCP_LE)[SPC<0]
    TFZ[SPC<0] = np.abs(XCOS)[SPC<0]*np.sign(DCP_LE)[SPC<0]

    CAXL = CAXL - TFX*CSUC

    # Add a dimension into the suction to be chordwise
    CNC   = CNC + CSUC*np.sqrt(1+T2)*TFZ

    # FCOS/FSIN: cosine/sine of the angle between the strip chordline and the x-axis.
    FCOS = np.cos(ZETA)
    FSIN = np.sin(ZETA)

    # BFX/BFY/BFZ: strip force contribution, body-axis components.
    BFX = -  CNC *FSIN + CAXL *FCOS
    BFY = - (CNC *FCOS + CAXL *FSIN) *SID
    BFZ =   (CNC *FCOS + CAXL *FSIN) *COD

    # CONVERT CNC FROM CN INTO CNC (COEFF. *CHORD).
    CHORD_strip = CHORD[LE_ind].reshape(dim_1,dim_2)
    CNC         = CNC  * CHORD_strip
    BMLE        = BMLE * CHORD_strip

    # BMX/BMY/BMZ: strip moment (about the moment reference point), body-axis components.
    X      = VD.XCH[LE_ind].reshape(dim_1,dim_2)  # These are all LE values
    Y      = VD.YCH[LE_ind].reshape(dim_1,dim_2)  # These are all LE values
    Z      = VD.ZCH[LE_ind].reshape(dim_1,dim_2)  # These are all LE values
    BMX    = BFZ * Y - BFY * (Z - VD.ZBAR)
    BMX    = BMX + SICPLE
    BMY    = BMLE * COD + BFX * (Z - VD.ZBAR) - BFZ * (X - VD.XBAR)
    BMZ    = BMLE * SID - BFX * Y + BFY * (X - VD.XBAR)
    CDC    = BFZ * SINALF +  (BFX *COPSI + BFY *SINPSI) * COSALF
    CDC    = CDC * CHORD_strip

    ES     = 2*s[:,0,:][LE_ind].reshape(dim_1,dim_2)
    STRIP  = ES *CHORD_strip
    LIFT   = (BFZ *COSALF - (BFX *COPSI + BFY *SINPSI) *SINALF)*STRIP
    MOMENT = STRIP * (BMY *COPSI - BMX *SINPSI)
    FY     = (BFY *COPSI - BFX *SINPSI) *STRIP
    RM     = STRIP *(BMX *COSALF *COPSI + BMY *COSALF *SINPSI + BMZ *SINALF)
    YM     = STRIP *(BMZ *COSALF - (BMX *COPSI + BMY *SINPSI) *SINALF)

    # Lift coefficient
    Clift_y   = LIFT/CHORD_strip/ES
    CL_wing   = np.add.reduceat(LIFT,span_breaks[0],axis=1)/VD.wing_areas
    CLift     = np.atleast_2d(np.sum(LIFT,axis=1)/S_ref).T

    # Drag coefficient
    results   = compute_trefftz_plane_induced_drag(conditions, VD,Clift_y, X, Y, Z, CHORD_strip,S_ref,b_ref)

    # Body-axis forces from the wind-axis (lift, drag) pair via the stability-axis rotation
    # inverted (Takahashi et al., AIAA SciTech 2025, Sec. III.F) -- a pure rotation, so no
    # division and no singularity at alpha = 0.
    CX_for   = CLift*SINALF - results.CDrag_induced*COSALF
    CZ_for   = CLift*COSALF + results.CDrag_induced*SINALF
    CY_for   = np.atleast_2d(np.sum(FY,axis=1)/S_ref).T

    # moment coefficients
    CM_mom   = np.atleast_2d(np.sum(MOMENT,axis=1)/S_ref).T/c_ref
    CL_mom   = np.atleast_2d(np.sum(RM,axis=1)/S_ref).T    /b_ref
    CN_mom   = np.atleast_2d(np.sum(YM,axis=1)/S_ref).T    /b_ref

    # ---------------------------------------------------------------------------------------
    # STEP 13: Pack outputs
    # ------------------ --------------------------------------------------------------------
    results.CLift             = CLift
    results.CX                = CX_for
    results.CY                = CY_for
    results.CZ                = -CZ_for
    results.CL                = CL_mom
    results.CM                = CM_mom
    results.CN                = -CN_mom
    results.spanwise_stations = Y
    results.CLift_wing        = CL_wing
    results.sectional_CLift   = Clift_y
    results.CP                = np.array(CP    , dtype=settings.floating_point_precision )
    results.gamma             = np.array(GAMMA , dtype=settings.floating_point_precision )
    results.V_distribution    = rhs.V_distribution
    results.V_x               = rhs.Vx_ind_total
    results.V_z               = rhs.Vz_ind_total

    i = 0
    dim_wing_lifts      = results.CLift_wing * VD.wing_areas
    dim_wing_drags      = results.CDrag_induced_wing * S_ref
    Clift_wings         = Data()
    Cdrag_wings         = Data()
    # Assign the lift and drag and non-dimensionalize
    for wing in geometry.wings.values():
        ref = wing.areas.reference
        if wing.xz_plane_symmetric:
            Clift_wings[wing.tag]      = np.atleast_2d(np.sum(dim_wing_lifts[:,i:(i+2)],axis=1)).T/ref
            Cdrag_wings[wing.tag]      = np.atleast_2d(np.sum(dim_wing_drags[:,i:(i+2)],axis=1)).T/ref
            i+=1
        else:
            Clift_wings[wing.tag]      = np.atleast_2d(dim_wing_lifts[:,i]).T/ref
            Cdrag_wings[wing.tag]      = np.atleast_2d(dim_wing_drags[:,i]).T/ref
        i+=1
    results.CLift_wings         = Clift_wings
    results.CDrag_induced_wings = Cdrag_wings
    results.VD = VD
    return results

# ----------------------------------------------------------------------
#  CLE rotation effects helper function
# ----------------------------------------------------------------------
def compute_rotation_effects(VD, settings, EW_large, GAMMA, X, CHORD, XLE, XBAR,
                             rhs, COSINP, SINALF,COSCOS, PITCH, ROLL, YAW, STB, RNMAX):
    """ This computes the effects of the freestream and aircraft rotation rate on
    CLE, the induced flow at the leading edge

    Assumptions:
    Several of the values needed in this calculation have been computed earlier and stored in VD

    Normally, VORLAX skips the calculation implemented in this function for linear
    chordwise spacing (the if statement below). However, since the trends are correct,
    albeit underestimated, this calculation is being forced here.
    """
    LE_ind   = VD.leading_edge_indices
    RNMAX    = VD.panels_per_strip
    dim_1    = len(np.sum(LE_ind, axis=1))
    dim_2    = np.sum(LE_ind, axis=1)[0]
    dim_3    = len(LE_ind[0])

    # Rotational (pitch/roll/yaw) effects on LE suction: pick each strip's LE value of EW,
    # reshape GAMMA -> gamma to match.
    EW    = EW_large[LE_ind, :].reshape(dim_1, dim_2, dim_3)
    gamma = np.array(np.split(np.repeat(GAMMA, dim_2, axis=0), dim_1))
    CLE   = (EW*gamma).sum(axis=2)

    # XGIRO/YGIRO/ZGIRO: control point position relative to the rotation center (XBAR, 0,
    # ZBAR). EFFINC below is computed exactly as ALOC is in compute_RHS_matrix(), except
    # for this XGIRO term.
    XGIRO = X - CHORD*XLE - np.repeat( XBAR, RNMAX[LE_ind].reshape(dim_1,dim_2)[0] , axis=1)
    YGIRO = rhs.YGIRO
    ZGIRO = rhs.ZGIRO

    # VX/VY/VZ: onset flow velocity at the leading edge (strip midpoint), referenced to freestream.
    VX = (COSCOS - PITCH*ZGIRO + YAW  *YGIRO)
    VY = (COSINP - YAW  *XGIRO + ROLL *ZGIRO)
    VZ = (SINALF - ROLL *YGIRO + PITCH*XGIRO)

    # CCNTL, SCNTL, SID, and COD were computed in compute_RHS_matrix()

    # EFFINC: onset-flow component along the camberline normal at the leading edge.
    EFFINC = VX *rhs.SCNTL + VY *rhs.CCNTL *rhs.SID - VZ *rhs.CCNTL *rhs.COD
    CLE = CLE - EFFINC[LE_ind].reshape(dim_1,dim_2)
    CLE = np.where(STB > 0, CLE /RNMAX[LE_ind].reshape(dim_1,dim_2) /STB, CLE)

    return CLE

# ----------------------------------------------------------------------
#  Vectorized cumsum from indices
# ----------------------------------------------------------------------
def strip_cumsum(arr, chord_breaks, strip_lengths):
    """ Uses numpy to to compute a cumsum that resets along
    the leading edge of every strip.

    Assumptions:
    chordwise_breaks always starts at 0
    """
    cumsum  = np.cumsum(arr, axis=1)
    offsets = cumsum[:,chord_breaks-1]
    offsets[:,0]  = 0
    offsets = np.repeat(offsets, strip_lengths, axis=1)
    return cumsum - offsets


def compute_trefftz_plane_induced_drag(conditions, VD, cl, x_dist, y_dist, z_dist, chord_dist, SREF, b_ref, v_inf=1):
    """Compute induced drag using a Trefftz-plane (far-field wake) analysis: circulation is
    reconstructed from the sectional lift via Kutta-Joukowski, shed vortices are formed from
    the spanwise difference in bound circulation (Helmholtz), each shed vortex's 2-D Biot-Savart
    field is evaluated at every strip centerpoint, and induced drag is the span-integral of
    (induced angle x cl x chord), following the classical vortex-lattice/lifting-line approach
    used in AVL and VORLAX.

    Assumptions:
    Incompressible, inviscid flow. Each wing is an independent lifting surface when assembling
    shed-vortex strengths (mutual induction between wings is still captured in the induced-velocity
    step, which runs over all wings at once). Symmetric (xz-plane) wings have their inboard root
    vortex strength set to zero -- the other side's image is implicit. cl is normalised by
    arc-length area (chord x DS, DS = true spanwise arc-length) rather than projected area
    (chord x dY); the DS/|dY| factor in the circulation calc corrects back to true bound
    circulation, and is exactly 1 for a flat, unswept wing.

    Source:
    1. Drela, M. and Youngren, H., "AVL 3.36 User Primer," MIT, 2017.
    2. Katz, J. and Plotkin, A., "Low-Speed Aerodynamics," 2nd ed., Cambridge University Press,
       2001, Chap. 8.
    3. Lan, C. E., "A Quasi-Vortex-Lattice Method in Thin Wing Theory," Journal of Aircraft,
       Vol. 11, No. 9, 1974, pp. 518-527.

    Inputs:
    conditions.aerodynamics.angles.alpha      [radians]
    VD.Y/.Z[k]           global node coordinates for case k                    [m]
    VD.n_sw/.n_cw[k]     spanwise strips / chordwise panels per strip per wing [Unitless]
    VD.symmetric_wings/.vertical_wing[k]      per-wing booleans                [Unitless]
    VD.wing_areas                             per-wing reference area         [m^2]
    cl               sectional lift coefficient at each strip's LE panel      [Unitless]
    chord_dist       local chord at each strip                               [m]
    SREF             vehicle reference area                                  [m^2]
    v_inf            freestream speed (default 1, non-dimensional)           [m/s]

    Outputs:
    results.
        CDrag_induced                          [Unitless]
        sectional_CDrag_induced                [Unitless]
        CDrag_induced_wing                     [Unitless]
        alpha_induced                          [radians]

    Properties Used:
    N/A
    """
    alpha   = conditions.aerodynamics.angles.alpha
    n_cases = len(alpha)
    CDi_total         = np.zeros(n_cases)
    CDi_wing          = np.zeros((n_cases, len(VD.n_sw[0])))
    Cd_i_distribution = np.zeros_like(cl)
    alpha_i           = np.zeros_like(cl)

    for k in range(n_cases):
        alpha   = conditions.aerodynamics.angles.alpha [k]
        n_wings = len(VD.n_sw[k])
        divisions = np.cumsum(VD.n_sw[k])[:-1]

        # Split into per-wing lists — no np.stack, wings may have different n_sw
        cl_split    = np.split(cl[k], divisions)
        chord_split = np.split(chord_dist[k], divisions)

        # VD.Y[k]/VD.Z[k] are flat node arrays: each wing w contributes (n_sw[w]+1)*(n_cw[w]+1)
        # corner nodes, row-major. Stride by (n_cw[w]+1) to pick one point per spanwise station
        # (using the first wing's n_cw as a global stride would be wrong when wings differ).
        node_sizes       = (VD.n_sw[k] + 1) * (VD.n_cw[k] + 1)
        node_splits      = np.cumsum(node_sizes)[:-1]
        y_nodes_per_wing = np.split(VD.Y[k], node_splits)
        z_nodes_per_wing = np.split(VD.Z[k], node_splits)
        y_control_points = [seg[::(VD.n_cw[k][w] + 1)] for w, seg in enumerate(y_nodes_per_wing)]
        z_control_points = [seg[::(VD.n_cw[k][w] + 1)] for w, seg in enumerate(z_nodes_per_wing)]

        is_symmetric = np.array(VD.symmetric_wings[0], dtype=bool)
        is_vertical  = np.array(VD.vertical_wing[0],   dtype=bool)
        symmetric_wing_flags = np.concatenate([np.repeat(is_symmetric & ~is_vertical, 2), np.zeros(np.count_nonzero(~is_symmetric), dtype=bool)])[:n_wings]

        y_centerpoints = []
        z_centerpoints = []
        shed_vortices  = []

        for w in range(n_wings):
            ycp  = y_control_points[w]
            zcp  = z_control_points[w]
            cl_w = cl_split[w]
            ch_w = chord_split[w]

            y_centerpoints.append((ycp[:-1] + ycp[1:]) / 2)
            # Evaluated in body-axis coordinates -- no alpha rotation on Z, matching how AVL's
            # TPFORC handles the Prandtl-Glauert transform (ALFAT = 0): the wake and bound-vortex
            # geometry are already in body axes, so rotating Z by alpha would double-count incidence.
            z_centerpoints.append((zcp[:-1] + zcp[1:]) / 2)

            dy_w = np.diff(ycp)
            dz_w = np.diff(zcp)
            # Floor projected span at 1e-6 m (negligible vs. any real panel) to avoid a
            # division-by-zero for winglet panels lying entirely in the XZ plane.
            dy_w_g  = np.where(dy_w == 0, 1e-6, dy_w)
            DS_w    = np.sqrt(dy_w_g**2 + dz_w**2)
            circ_w  = 0.5 * ch_w * v_inf * cl_w * (DS_w / np.abs(dy_w_g))

            sign_w      = np.sign(ycp[1] - ycp[0])
            sv_w        = np.zeros(len(ycp))
            sv_w[1:-1]  = sign_w * np.diff(circ_w)
            sv_w[-1]    = -sign_w * circ_w[-1]
            if not symmetric_wing_flags[w]:
                sv_w[0] = -sign_w * circ_w[0]
            shed_vortices.append(sv_w)

        # Induced velocity, vectorised over all wings simultaneously
        yc    = np.concatenate(y_centerpoints)
        zc    = np.concatenate(z_centerpoints)
        yp    = np.concatenate(y_control_points)
        zp    = np.concatenate(z_control_points)
        gamma = np.concatenate(shed_vortices)

        sdy = np.concatenate([np.diff(y_control_points[w]) for w in range(n_wings)])
        sdz = np.concatenate([np.diff(z_control_points[w]) for w in range(n_wings)])
        sds = np.sqrt(sdy**2 + sdz**2)

        dy_mat = yc[:, None] - yp[None, :]
        dz_mat = zc[:, None] - zp[None, :]
        dist2  = dy_mat**2 + dz_mat**2
        dist2[dist2 == 0] = np.inf

        numerator            = sdz[:, None] * dz_mat + sdy[:, None] * dy_mat
        induced_velocity_flat = (
            (gamma[None, :] * np.sign(sdy[:, None]) * numerator
             / (4.0 * np.pi * sds[:, None] * dist2))
            .sum(axis=1)
        )

        alpha_induced_flat = np.arctan(induced_velocity_flat / v_inf)
        cl_flat            = np.concatenate(cl_split)
        cd_induced_flat    = alpha_induced_flat * cl_flat

        # Per-wing CDi integration
        CDi    = 0
        offset = 0
        for w in range(n_wings):
            n_sw_w = len(y_centerpoints[w])
            ycp_w  = y_control_points[w]
            zcp_w  = z_control_points[w]
            cd_w   = cd_induced_flat[offset:offset + n_sw_w]
            ch_w   = chord_split[w]
            ld_w   = np.cumsum(np.sqrt(np.diff(ycp_w)**2 + np.diff(zcp_w)**2))
            CDi_w  = trapezoid(cd_w * ch_w, ld_w) / SREF
            CDi_wing[k][w] = CDi_w
            CDi   += CDi_w
            offset += n_sw_w

        CDi_total[k]         = CDi
        Cd_i_distribution[k] = cd_induced_flat
        alpha_i[k]           = alpha_induced_flat


    results                          = Data()
    results.CDrag_induced            = CDi_total[:,np.newaxis]
    results.sectional_CDrag_induced  = Cd_i_distribution
    results.CDrag_induced_wing       = CDi_wing
    results.alpha_induced            = alpha_i

    return results
