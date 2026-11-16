# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/VLM.py
#
# Created: Aug 2025, M. Clarke
# Modified:Apr 2026, S. Shekar, A. Molloy, M. Clarke
#          May 2026  M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# package imports
from RCAIDE.Framework.Core import Data, array_namespace
from .compute_wing_induced_velocity      import compute_wing_induced_velocity, CASE_INDEXED_VD_FIELDS
from .generate_vortex_distribution       import generate_vortex_distribution
from .postprocess_vortex_distribution     import arctangent_of_ratio, panels_of_reversed_strips
from .compute_RHS_matrix                 import compute_RHS_matrix, rotation_in_VORLAX_axes

from scipy.integrate import trapezoid
import scipy.linalg
import numpy as np
# ----------------------------------------------------------------------
#  Vortex Lattice
# ----------------------------------------------------------------------

# fields of the vortex distribution the solve reads as numbers
LATTICE_FIELDS = ('D','SLOPE','XA1','XA2','XAH','XA_TE','XB1','XB2','XBH','XB_TE','XC','XCH','Y','YA1','YA2','YAC','YAH',
                  'YA_TE','YB1','YB2','YBC','YBH','YB_TE','YC','YCH','Z','ZA1','ZA2','ZAC','ZAH','ZA_TE','ZB1','ZB2','ZBC',
                  'ZBH','ZB_TE','ZC','ZCH','chord_lengths','normals','tangent_incidence_angle','wing_areas')

def VLM(conditions,settings,geometry,VD=None):
    """Uses the vortex lattice method to compute the lift, induced drag and moment coefficients.

    The user should be forwarned that this will cause very slight differences in results for 0 deflection due to
    the slightly different discretization.

    The user has the option to use the boundary conditions and induced velocities from either RCAIDE
    or VORLAX. See build_RHS in compute_RHS_matrix.py for more details.

    The calculations are in double precision.

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
    settings.use_VORLAX_matrix_calculation     [boolean]

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
    if VD is None:
        VD = generate_vortex_distribution(conditions,settings,geometry)
    xp = array_namespace(*[VD[name] for name in LATTICE_FIELDS], x_m, z_m, S_ref, c_ref, b_ref, conditions.freestream.mach_number)

    # unpack conditions--------------------------------------------------------------
    K_SPC    = settings.leading_edge_suction_multiplier
    aoa      = conditions.aerodynamics.angles.alpha
    mach     = conditions.freestream.mach_number
    len_mach = len(mach)

    # VORLAX's angles are all degrees-to-radians via DTR; RCAIDE's are radians/Units.degrees throughout.
    PSI, ROLLQ, YAWQ = rotation_in_VORLAX_axes(conditions)
    PITCHQ    = conditions.static_stability.pitch_rate
    VINF      = conditions.freestream.velocity

    # The Mach number and the velocity can be JAX values (a compiled evaluation at any subsonic Mach number); the
    # angles and rates are NumPy.
    mach_namespace = array_namespace(mach)

    #freestream 0 velocity safeguard
    if array_namespace(VINF) is np and not conditions.freestream.velocity.all():
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

    # The layout of the lattice does not change with the flow case: the first panel of each chordwise strip, the
    # number of panels in each strip and the strip each panel belongs to
    leading_edge   = np.flatnonzero(LE_ind[0])
    strip_lengths  = np.asarray(RNMAX)[0][leading_edge].astype(int)
    strip_of_panel = np.repeat(np.arange(len(leading_edge)), strip_lengths)
    n_strips       = len(leading_edge)

    # Compute X and Z BAR ouside of generate_vortex_distribution to avoid requiring x_m and z_m as inputs
    VD.XBAR = xp.ones(( len_mach,n_strips)) * x_m
    VD.ZBAR = xp.ones(( len_mach,n_strips)) * z_m

    # ---------------------------------------------------------------------------------------
    # STEP 10: Generate A and RHS matrices from VD and geometry
    # ------------------ --------------------------------------------------------------------
    # Compute flow tangency conditions
    phi   = arctangent_of_ratio(VD.ZBC - VD.ZAC, VD.YBC - VD.YAC, panels_of_reversed_strips(VD)) # dihedral angle
    delta = arctangent_of_ratio(VD.ZC - VD.ZCH, VD.XC - VD.XCH)   # mean camber surface angle

    # Build the RHS vector
    rhs     = compute_RHS_matrix(VD,delta,phi,conditions,settings,geometry)
    RHS     = rhs.RHS*1 # this matches numpy=1.26 in terms of dimension
    ONSET   = rhs.ONSET*1

    # Build the aerodynamic influence coefficient matrix A of the flow tangency condition. With VORLAX's matrix
    # calculation it is the VORLAX velocity EW on every row; otherwise EW is kept on the leading-edge rows only.
    use_VORLAX_induced_velocity = settings.use_VORLAX_matrix_calculation
    # The matrix depends on the geometry and the Mach number alone. Cases at one Mach number share it: it is built
    # for the first case and the arrays below keep one row, which the cases broadcast against.
    # Cases given a JAX Mach number share it.
    one_mach = len_mach > 1 and (mach_namespace is not np or bool(np.all(mach == mach[0])))
    if one_mach:
        VD_matrix = Data()
        for field in CASE_INDEXED_VD_FIELDS:
            VD_matrix[field] = VD[field][:1]
        mach_matrix, delta_matrix, phi_matrix = mach[:1], delta[:1], phi[:1]
    else:
        VD_matrix, mach_matrix, delta_matrix, phi_matrix = VD, mach, delta, phi
    if use_VORLAX_induced_velocity:
        _, s, RFLAG, EW = compute_wing_induced_velocity(VD_matrix,mach_matrix,compute_EW=True)
        A               = EW
        EW              = EW[:,leading_edge]
    else:
        A, s, RFLAG, EW = compute_wing_induced_velocity(VD_matrix,mach_matrix,compute_EW=True,delta=delta_matrix,phi=phi_matrix)

    # Turn off sonic vortices when Mach>1
    RHS = RHS*RFLAG

    # To ensure compatibility for np.linalg.solve across numpy1.0 and numpy2.0
    RHS = xp.atleast_3d(RHS)

    # Compute vortex strength: the shared matrix is factored once and every case is a right-hand side of it
    if one_mach:
        if xp is np:
            factors = scipy.linalg.lu_factor(A[0].astype(np.result_type(A, RHS)))  # same precision as np.linalg.solve
            GAMMA   = scipy.linalg.lu_solve(factors, RHS[:,:,0].T).T[:,:,None]
        else:
            import jax.scipy.linalg
            factors = jax.scipy.linalg.lu_factor(A[0])
            GAMMA   = jax.scipy.linalg.lu_solve(factors, RHS[:,:,0].T).T[:,:,None]
    else:
        GAMMA  = xp.linalg.solve(A,RHS)

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
    B2     = mach_namespace.tile((mach**2 - 1),int(np.ravel(VD.n_cp)[0]))
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

    # COMPUTE EFFECT OF SIDESLIP on DCP intermediate variables. needs change if cosine chorwise spacing added
    FORAXL = COSCOS
    FORLAT = COSIN
    XA1_LE, YA1_LE, ZA1_LE = VD.XA1[:,leading_edge], VD.YA1[:,leading_edge], VD.ZA1[:,leading_edge]
    XB1_LE, YB1_LE, ZB1_LE = VD.XB1[:,leading_edge], VD.YB1[:,leading_edge], VD.ZB1[:,leading_edge]
    TAN_LEi= (XB1_LE-XA1_LE)/  xp.sqrt((ZB1_LE-ZA1_LE)**2 +  (YB1_LE-YA1_LE)**2)
    TAN_TE = (VD.XB_TE - VD.XA_TE)/ xp.sqrt((VD.ZB_TE-VD.ZA_TE)**2 + (VD.YB_TE-VD.YA_TE)**2)
    TAN_LE = TAN_LEi[:,strip_of_panel]

    TNL    = TAN_LE * 1 # VORLAX's SIGN variable not needed, as these are taken directly from geometry
    TNT    = TAN_TE * 1
    XIA    = np.broadcast_to((RK-1)/RNMAX, np.shape(B2))
    XIB    = np.broadcast_to((RK  )/RNMAX, np.shape(B2))
    TANA   = TNL *(1. - XIA) + TNT *XIA
    TANB   = TNL *(1. - XIB) + TNT *XIB

    # cumsum GANT loop if KTOP > 0 (don't actually need KTOP with vectorized arrays and np.roll)
    GFX    = 1 /CHORD
    GANT   = strip_cumsum(GFX*GAMMA, chord_breaks[0], strip_of_panel)
    GANT   = xp.roll(GANT,1,axis=1)
    GANT   = xp.where(LE_ind,0.,GANT)

    GLAT   = GANT *(TANA - TANB) - GFX *GAMMA *TANB
    cos_DL = (VD.YBH-VD.YAH)[:,leading_edge]/VD.D
    COS_DL = cos_DL[:,strip_of_panel]
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
    # Leading edge sweep. VORLAX does it panel by panel. This will be spanwise.
    TLE   = TAN_LEi
    B2_LE = B2[:,leading_edge]
    T2    = TLE*TLE
    swept_behind_mach_line = B2_LE<T2
    STB   = xp.where(swept_behind_mach_line, xp.sqrt(xp.where(swept_behind_mach_line, T2-B2_LE, 1.)), 0.)

    # COD/SID: dihedral angle (x-y plane) of each strip's horseshoe vortices, LE value only.
    COD = xp.cos(phi[:,leading_edge])
    SID = xp.sin(phi[:,leading_edge])

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
    CNC    = strip_sum(SINF       ,chord_breaks[0],strip_of_panel)
    SICPLE = strip_sum(SINF*CORMED,chord_breaks[0],strip_of_panel)

    # TX: chordwise slope at load points, interpolated between control points (incidence included).
    XX   = (RK - .75) *PION /2.0
    TX    = VD.SLOPE - ZETA
    CAXL  = -SINF*TX/(1.0+TX**2) # These are the axial forces on each panel
    BMLE  = (XLE-XX)*SINF        # These are moment on each panel

    # Sum onto the panel
    CAXL = strip_sum(CAXL,chord_breaks[0],strip_of_panel)
    BMLE = strip_sum(BMLE,chord_breaks[0],strip_of_panel)

    # /3: VORLAX's original SICPLE over-predicts dCl/dbeta vs wind tunnel (Takahashi
    # et al., AIAA SciTech 2025, Sec. III.E).
    SICPLE = SICPLE * ((-1/3) * COSIN * COD * GAF)
    DCP_LE = DCP[:,leading_edge]

    # CLE: leading-edge thrust coefficient, from the total induced flow at the leading edge.
    # Only strictly valid for cosine chordwise spacing (LAX = 0); RCAIDE only has linear
    # chordwise spacing today, so this underestimates the true magnitude. See
    # RCAIDE_VLM_Physics_and_Cleanup_Review.md section 1.2.
    CLE = compute_rotation_effects(VD, settings, EW, GAMMA, X, CHORD, XLE, VD.XBAR, rhs, COSINP, SINALF,COSCOS, PITCH, ROLL, YAW, STB, RNMAX)

    # Leading edge suction multiplier (negative integer if used). Defaults to 1.
    SPC  = K_SPC*np.ones(np.shape(B2_LE))

    # Subsonic + vortex lift enabled -> SPC becomes -1
    VL   = np.repeat(VD.vortex_lift,VD.n_sw[0], axis=1)
    m_b  = np.atleast_2d(mach[:,0]<1.) if mach_namespace is np else np.ones((1,len_mach),dtype=bool)   # a JAX Mach number is subsonic
    SPC_cond      = VL*m_b.T
    SPC[SPC_cond] = -1.
    SPC           = SPC * exposed_leading_edge_flag

    CLE  = CLE + 0.5* DCP_LE *np.sqrt(XLE[:,leading_edge])
    CSUC = 0.5*np.pi*np.abs(SPC)*(CLE**2)*STB

    # TFX/TFZ: leading-edge force vector components along the body x/z axes.
    SLE  = VD.SLOPE[:,leading_edge]
    ZETA = ZETA[:,leading_edge]
    XCOS = xp.cos(SLE-ZETA)
    XSIN = xp.sin(SLE-ZETA)

    # Negative SPC: Lan's correction (see VORLAX documentation) applies instead.
    TFX  = xp.where(SPC<0, XSIN*xp.sign(DCP_LE)        ,  1.*XCOS)
    TFZ  = xp.where(SPC<0, xp.abs(XCOS)*xp.sign(DCP_LE), -1.*XSIN)

    CAXL = CAXL - TFX*CSUC

    # Add a dimension into the suction to be chordwise
    CNC   = CNC + CSUC*xp.sqrt(1+T2)*TFZ

    # FCOS/FSIN: cosine/sine of the angle between the strip chordline and the x-axis.
    FCOS = xp.cos(ZETA)
    FSIN = xp.sin(ZETA)

    # BFX/BFY/BFZ: strip force contribution, body-axis components.
    BFX = -  CNC *FSIN + CAXL *FCOS
    BFY = - (CNC *FCOS + CAXL *FSIN) *SID
    BFZ =   (CNC *FCOS + CAXL *FSIN) *COD

    # CONVERT CNC FROM CN INTO CNC (COEFF. *CHORD).
    CHORD_strip = CHORD[:,leading_edge]
    CNC         = CNC  * CHORD_strip
    BMLE        = BMLE * CHORD_strip

    # BMX/BMY/BMZ: strip moment (about the moment reference point), body-axis components.
    X      = VD.XCH[:,leading_edge]  # These are all LE values
    Y      = VD.YCH[:,leading_edge]  # These are all LE values
    Z      = VD.ZCH[:,leading_edge]  # These are all LE values
    BMX    = BFZ * Y - BFY * (Z - VD.ZBAR)
    BMX    = BMX + SICPLE
    BMY    = BMLE * COD + BFX * (Z - VD.ZBAR) - BFZ * (X - VD.XBAR)
    BMZ    = BMLE * SID - BFX * Y + BFY * (X - VD.XBAR)
    CDC    = BFZ * SINALF +  (BFX *COPSI + BFY *SINPSI) * COSALF
    CDC    = CDC * CHORD_strip

    ES     = 2*s[:,leading_edge]
    STRIP  = ES *CHORD_strip
    LIFT   = (BFZ *COSALF - (BFX *COPSI + BFY *SINPSI) *SINALF)*STRIP
    MOMENT = STRIP * (BMY *COPSI - BMX *SINPSI)
    FY     = (BFY *COPSI - BFX *SINPSI) *STRIP
    RM     = STRIP *(BMX *COSALF *COPSI + BMY *COSALF *SINPSI + BMZ *SINALF)
    YM     = STRIP *(BMZ *COSALF - (BMX *COPSI + BMY *SINPSI) *SINALF)

    # horizontal tail strips see the tail dynamic pressure ratio, q_h/q (Raymer Eq. 16.6)
    eta_h  = np.repeat(np.where(VD.horizontal_tail[0] == 1, settings.get('horizontal_tail_dynamic_pressure_ratio', 1.0), 1.0), VD.n_sw[0])
    LIFT   = LIFT   * eta_h
    MOMENT = MOMENT * eta_h
    FY     = FY     * eta_h
    RM     = RM     * eta_h
    YM     = YM     * eta_h

    # Lift coefficient
    wing_of_strip = np.repeat(np.arange(len(VD.n_sw[0])), VD.n_sw[0])
    Clift_y   = LIFT/CHORD_strip/ES
    CL_wing   = strip_sum(LIFT,span_breaks[0],wing_of_strip)/VD.wing_areas
    CLift     = xp.atleast_2d(xp.sum(LIFT,axis=1)/S_ref).T

    # Drag coefficient
    Cside_y   = FY/CHORD_strip/ES
    results   = compute_trefftz_plane_induced_drag(VD, Clift_y, Cside_y, CHORD_strip, S_ref)

    # Body-axis forces from the wind-axis (lift, drag) pair via the stability-axis rotation
    # inverted (Takahashi et al., AIAA SciTech 2025, Sec. III.F) -- a pure rotation, so no
    # division and no singularity at alpha = 0.
    CX_for   = CLift*SINALF - results.CDrag_induced*COSALF
    CZ_for   = CLift*COSALF + results.CDrag_induced*SINALF
    CY_for   = xp.atleast_2d(xp.sum(FY,axis=1)/S_ref).T

    # moment coefficients
    CM_mom   = xp.atleast_2d(xp.sum(MOMENT,axis=1)/S_ref).T/c_ref
    CL_mom   = xp.atleast_2d(xp.sum(RM,axis=1)/S_ref).T    /b_ref
    CN_mom   = xp.atleast_2d(xp.sum(YM,axis=1)/S_ref).T    /b_ref

    # ---------------------------------------------------------------------------------------
    # STEP 13: Pack outputs
    # ------------------ --------------------------------------------------------------------
    results.CLift             = CLift
    results.CX                = CX_for
    results.CY                = CY_for
    results.CZ                = -CZ_for
    results.CL                = -CL_mom
    results.CM                = CM_mom
    results.CN                = -CN_mom
    results.spanwise_stations = Y
    results.CLift_wing        = CL_wing
    results.sectional_CLift   = Clift_y
    results.CP                = xp.array(CP)
    results.gamma             = xp.array(GAMMA)
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
            Clift_wings[wing.tag]      = xp.atleast_2d(xp.sum(dim_wing_lifts[:,i:(i+2)],axis=1)).T/ref
            Cdrag_wings[wing.tag]      = xp.atleast_2d(xp.sum(dim_wing_drags[:,i:(i+2)],axis=1)).T/ref
            i+=1
        else:
            Clift_wings[wing.tag]      = xp.atleast_2d(dim_wing_lifts[:,i]).T/ref
            Cdrag_wings[wing.tag]      = xp.atleast_2d(dim_wing_drags[:,i]).T/ref
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
    xp             = array_namespace(EW_large, GAMMA, X, CHORD, XBAR, STB)
    leading_edge   = np.flatnonzero(VD.leading_edge_indices[0])
    strip_lengths  = np.asarray(VD.panels_per_strip)[0][leading_edge].astype(int)
    strip_of_panel = np.repeat(np.arange(len(leading_edge)), strip_lengths)

    # Rotational (pitch/roll/yaw) effects on LE suction: EW_large holds each strip's LE row of EW
    CLE   = (EW_large*GAMMA[:,None,:]).sum(axis=2)

    # XGIRO/YGIRO/ZGIRO: control point position relative to the rotation center (XBAR, 0,
    # ZBAR). EFFINC below is computed exactly as ALOC is in compute_RHS_matrix(), except
    # for this XGIRO term.
    XGIRO = X - CHORD*XLE - XBAR[:,strip_of_panel]
    YGIRO = rhs.YGIRO
    ZGIRO = rhs.ZGIRO

    # VX/VY/VZ: onset flow velocity at the leading edge (strip midpoint), referenced to freestream.
    VX = (COSCOS - PITCH*ZGIRO + YAW  *YGIRO)
    VY = (COSINP - YAW  *XGIRO + ROLL *ZGIRO)
    VZ = (SINALF - ROLL *YGIRO + PITCH*XGIRO)

    # CCNTL, SCNTL, SID, and COD were computed in compute_RHS_matrix()
    # EFFINC: onset-flow component along the camberline normal at the leading edge.
    EFFINC = VX *rhs.SCNTL + VY *rhs.CCNTL *rhs.SID - VZ *rhs.CCNTL *rhs.COD
    CLE    = CLE - EFFINC[:,leading_edge]
    swept  = STB > 0
    CLE    = xp.where(swept, CLE /RNMAX[:,leading_edge] /xp.where(swept, STB, 1.), CLE)

    return CLE

# ----------------------------------------------------------------------
#  Sums along the chordwise strips
# ----------------------------------------------------------------------
def strip_cumsum(arr, chord_breaks, strip_of_panel):
    """ Cumulative sum of arr along each chordwise strip, restarting at the first panel of every strip.

    Inputs:
    arr              (cases, panels)
    chord_breaks     index of the first panel of each strip
    strip_of_panel   strip each panel belongs to

    Outputs:
    cumulative sum   (cases, panels)
    """
    xp      = array_namespace(arr)
    cumsum  = xp.cumsum(arr, axis=1)
    offsets = cumsum[:,(chord_breaks-1)[strip_of_panel]]
    offsets = xp.where(strip_of_panel == 0, 0., offsets)
    return cumsum - offsets


def strip_sum(arr, breaks, strip_of_panel):
    """ Sum of arr over each strip.

    Inputs:
    arr              (cases, panels)
    breaks           index of the first panel of each strip
    strip_of_panel   strip each panel belongs to

    Outputs:
    sums             (cases, strips)
    """
    xp = array_namespace(arr)
    if xp is np:
        return np.add.reduceat(arr,breaks,axis=1)
    return xp.zeros((arr.shape[0],len(breaks)),dtype=arr.dtype).at[:,strip_of_panel].add(arr)


def compute_trefftz_plane_induced_drag(VD, cl, cy, chord_dist, SREF, v_inf=1):
    """Induced drag from a Trefftz-plane (far-field wake) analysis, for lifting surfaces at any dihedral angle,
    vertical ones included.

    Each chordwise strip runs from its station A to its station B in the plane, with unit tangent t = (dy, dz)/ds and
    unit normal n = (-dz, dy)/ds. Its circulation is that of the force along n (Kutta-Joukowski),
    Gamma/V = c (cl t_y - cy t_z)/2. It leaves a trailing vortex Gamma at B and -Gamma at A (Helmholtz); where two
    strips meet, or a wing meets its mirror image at the root, these add. A trailing vortex G at Q induces at P the
    velocity G (t . (P - Q))/(2 pi |P - Q|^2) along n in the far wake, and half of it at the surface. The induced drag
    of a strip is its normal force times the angle by which that velocity turns the flow.

    Assumptions:
    Incompressible, inviscid flow. The wake leaves along the body x axis, so the stations are taken in body axes with
    no rotation by the angle of attack, as in AVL.

    Source:
    1. Drela, M. and Youngren, H., "AVL 3.36 User Primer," MIT, 2017.
    2. Katz, J. and Plotkin, A., "Low-Speed Aerodynamics," 2nd ed., Cambridge University Press,
       2001, Chap. 8.
    3. Lan, C. E., "A Quasi-Vortex-Lattice Method in Thin Wing Theory," Journal of Aircraft,
       Vol. 11, No. 9, 1974, pp. 518-527.

    Inputs:
    VD.Y/.Z              global node coordinates, the same for every case      [m]
    VD.n_sw/.n_cw        spanwise strips / chordwise panels per strip per wing [Unitless]
    cl, cy               lift and side force coefficient of each strip, on its chord times its width [Unitless]
    chord_dist           chord of each strip                                   [m]
    SREF                 vehicle reference area                                [m^2]
    v_inf                freestream speed (default 1, non-dimensional)         [m/s]

    Outputs:
    results.
        CDrag_induced                          [Unitless]
        sectional_CDrag_induced                [Unitless]
        CDrag_induced_wing                     [Unitless]
        alpha_induced                          [radians], positive where the flow is turned against the lift of
                                               an upright surface (downwash)

    Properties Used:
    N/A
    """
    xp        = array_namespace(cl, cy, chord_dist, VD.Y, VD.Z, SREF)
    n_sw      = VD.n_sw[0]
    n_cw      = VD.n_cw[0]

    # the stations of each wing in the plane: one node per spanwise station (a wing has (n_sw+1)*(n_cw+1) nodes)
    node_splits = np.cumsum((n_sw + 1) * (n_cw + 1))[:-1]
    y_stations  = [nodes[::(n_cw[w] + 1)] for w, nodes in enumerate(xp.split(VD.Y[0], node_splits))]
    z_stations  = [nodes[::(n_cw[w] + 1)] for w, nodes in enumerate(xp.split(VD.Z[0], node_splits))]

    # tangent, width and center of every strip
    dy       = xp.concatenate([xp.diff(y) for y in y_stations])
    dz       = xp.concatenate([xp.diff(z) for z in z_stations])
    ds       = xp.sqrt(dy**2 + dz**2)
    t_y      = dy/ds
    t_z      = dz/ds
    y_center = xp.concatenate([(y[:-1] + y[1:])/2 for y in y_stations])
    z_center = xp.concatenate([(z[:-1] + z[1:])/2 for z in z_stations])

    # force of each strip along its normal, and its circulation
    normal_force = cl * t_y - cy * t_z
    circulation  = 0.5 * chord_dist * v_inf * normal_force

    # trailing vortices at the stations of each wing: -Gamma at the first station of a strip, +Gamma at its second
    divisions = np.cumsum(n_sw)[:-1]
    trailing  = []
    for gamma in xp.split(circulation, divisions, axis=1):
        trailing.append(xp.concatenate([-gamma[:,:1], gamma[:,:-1] - gamma[:,1:], gamma[:,-1:]], axis=1))
    trailing  = xp.concatenate(trailing, axis=1)
    y_vortex  = xp.concatenate(y_stations)
    z_vortex  = xp.concatenate(z_stations)

    # velocity along the normal of each strip at its center, half of that in the far wake
    r_y     = y_center[:, None] - y_vortex[None, :]
    r_z     = z_center[:, None] - z_vortex[None, :]
    r2      = r_y**2 + r_z**2
    r2      = xp.where(r2 == 0, np.inf, r2)
    kernel  = (t_y[:, None] * r_y + t_z[:, None] * r_z) / (4.0 * np.pi * r2)
    normal_velocity = (trailing[:, None, :] * kernel).sum(axis=2)

    turning    = -xp.arctan(normal_velocity / v_inf)
    cd_induced = turning * normal_force

    # per-wing integration over the strips
    CDi_of_wings = []
    for cd, chord, width in zip(xp.split(cd_induced, divisions, axis=1), xp.split(chord_dist, divisions, axis=1), xp.split(ds, divisions)):
        CDi_of_wings.append(xp.sum(cd * chord * width, axis=1) / SREF)
    CDi_of_wings = xp.stack(CDi_of_wings, axis=1)

    # the turning is about the normal of the strip, which points down on the mirrored side of a wing; reported as downwash
    results                          = Data()
    results.CDrag_induced            = xp.sum(CDi_of_wings, axis=1)[:,np.newaxis]
    results.sectional_CDrag_induced  = cd_induced
    results.CDrag_induced_wing       = CDi_of_wings
    results.alpha_induced            = turning * xp.where(dy < 0, -1., 1.)
    return results
