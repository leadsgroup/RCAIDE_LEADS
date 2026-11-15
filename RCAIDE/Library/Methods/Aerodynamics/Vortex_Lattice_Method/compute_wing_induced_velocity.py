# compute_wing_induced_velocity.py
#
# Created:  Dec 2020, E. Botero
# Modified: May 2021, E. Botero
#           Jun 2021, E. Botero

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# package imports
from RCAIDE.Framework.Core import Data, array_namespace, set_at
import numpy as np

# Fields consumed by compute_wing_induced_velocity_chunk(), indexed by
# flight condition/mach case along axis 0.
CASE_INDEXED_VD_FIELDS = [
    "XAH","YAH","ZAH","XBH","YBH","ZBH",
    "XA1","YA1","ZA1","XB1","YB1","ZB1",
    "XA2","YA2","ZA2","XB2","YB2","ZB2",
    "XC","YC","ZC","XA_TE","XB_TE",
    "leading_edge_indices","trailing_edge_indices",
    "panels_per_strip","chord_lengths",
]

# entries of one working array of the influence calculation: 8 MB in double precision
ROW_BLOCK_ENTRIES = 1000000

def compute_wing_induced_velocity(VD,mach,compute_EW=False,delta=None,phi=None):
    """ This computes the induced velocities at each control point of the vehicle vortex lattice

    Assumptions:
    Trailing vortex legs infinity are alligned to freestream

    Outside of a call to the VLM() function itself, EW does not need to be computed, as C_mn
    provides the same information in the body-frame.

    Chunks over the mach/case dimension (see compute_wing_induced_velocity_chunk) to
    bound peak memory -- each array here is (n_mach, n_panels, n_panels[, 3]), and cases
    don't couple to each other in these formulas, so chunking is exact, not an approximation.

    Source:
    1. Miranda, Luis R., Robert D. Elliot, and William M. Baker. "A generalized vortex
    lattice method for subsonic and supersonic flow applications." (1977). (NASA CR)

    2. VORLAX Source Code

    Inputs:
    VD       - vehicle vortex distribution                    [Unitless]
    mach                                                      [Unitless]

    Outputs:
    C_mn     - total induced velocity matrix                  [Unitless]
    s        - semispan of the horshoe vortex of each panel   [m]
    t        - tangent of the horshoe vortex                  [-]
    CHORD    - chord length for a panel                       [m]
    RFLAG    - sonic vortex flag                               [boolean]
    ZETA     - tangent incidence angle of the chordwise strip [-]

    Properties Used:
    N/A
    """
    n_mach     = len(mach)
    chunk_size = 1
    if n_mach <= chunk_size:
        return compute_wing_induced_velocity_chunk(VD,mach,compute_EW,delta,phi)

    C_mn_parts, s_parts, RFLAG_parts, EW_parts = [], [], [], []
    for i0 in range(0, n_mach, chunk_size):
        i1       = min(i0+chunk_size, n_mach)
        VD_chunk = Data()
        for field in CASE_INDEXED_VD_FIELDS:
            VD_chunk[field] = VD[field][i0:i1]
        delta_chunk = None if delta is None else delta[i0:i1]
        phi_chunk   = None if phi   is None else phi[i0:i1]
        C_mn_c, s_c, RFLAG_c, EW_c = compute_wing_induced_velocity_chunk(VD_chunk,mach[i0:i1],compute_EW,delta_chunk,phi_chunk)
        C_mn_parts.append(C_mn_c)
        s_parts.append(s_c)
        RFLAG_parts.append(RFLAG_c)
        if compute_EW:
            EW_parts.append(EW_c)

    C_mn  = np.concatenate(C_mn_parts, axis=0)
    s     = np.concatenate(s_parts,    axis=0)
    RFLAG = np.concatenate(RFLAG_parts,axis=0)
    EW    = np.concatenate(EW_parts,   axis=0) if compute_EW else np.nan
    return C_mn, s, RFLAG, EW

def compute_wing_induced_velocity_chunk(VD,mach,compute_EW=False,delta=None,phi=None):
    """ Computes compute_wing_induced_velocity()'s result for one slice of the mach/case dimension, on NumPy or JAX.

    Assumptions:
    The Mach numbers are NumPy values (they select the subsonic or supersonic formulas). On JAX the lattice
    coordinates may be traced; supersonic cases are NumPy only.

    Properties Used:
    N/A
    """
    # unpack
    LE_ind       = VD.leading_edge_indices
    TE_ind       = VD.trailing_edge_indices
    n_mach       = len(mach)
    mach         = np.array(mach,dtype=float)

    # Control points from the VLM
    names = ('XAH','YAH','ZAH','XBH','YBH','ZBH','XC','YC','ZC')
    xp    = array_namespace(*[VD[name] for name in names], delta, phi)
    XAH, YAH, ZAH, XBH, YBH, ZBH, XC, YC, ZC = [xp.atleast_2d(xp.asarray(VD[name],dtype=float)) for name in names]

    # Panel Dihedral Angle, using AH and BH location
    D      = xp.sqrt((YAH-YBH)**2+(ZAH-ZBH)**2)
    COS_DL = (YBH-YAH)/D
    # a panel without dihedral has a cosine of exactly 1 or -1, where the derivative of the arccosine is infinite
    flat   = xp.abs(COS_DL) >= 1.
    DL     = xp.where(flat, xp.where(COS_DL>0, 0., np.pi), xp.arccos(xp.where(flat, 0., COS_DL)))
    DL     = xp.where(DL>np.pi/2, DL - np.pi, DL) # This flips the dihedral angle for the other side of the wing

    # -------------------------------------------------------------------------------------------
    # Compute velocity induced by horseshoe vortex segments on every control point by every panel
    # -------------------------------------------------------------------------------------------
    # If YBH is negative, flip A and B, ie negative side of the airplane. Vortex order flips
    boolean  = YAH>YBH
    XAH, XBH = xp.where(boolean,XBH,XAH), xp.where(boolean,XAH,XBH)
    ZAH, ZBH = xp.where(boolean,ZBH,ZAH), xp.where(boolean,ZAH,ZBH)
    YAH, YBH = xp.where(boolean,YBH,YAH), xp.where(boolean,YAH,YBH)

    # These vortices will use AH and BH, rather than the typical location
    xa = XAH[:,None,:]
    ya = YAH[:,None,:]
    za = ZAH[:,None,:]
    xb = XBH[:,None,:]
    yb = YBH[:,None,:]
    zb = ZBH[:,None,:]

    # This is not the control point for the panel, its the middle front of the vortex
    xc = 0.5*(xa+xb)
    yc = 0.5*(ya+yb)
    zc = 0.5*(za+zb)

    # Incline the vortex
    theta    = xp.arctan2(zb-za,yb-ya)
    costheta = xp.cos(theta)
    sintheta = xp.sin(theta)

    # rotated axes
    x1bar = (xb - xc)
    y1bar = (yb - yc)*costheta + (zb - zc)*sintheta

    # The cutoff hardcoded into vorlax
    CUTOFF = 0.8

    # The notation in this method is flipped from the paper
    B2  = np.atleast_3d(mach**2-1.)
    sub = (B2<0)[:,0,0]
    sup = (B2>=0)[:,0,0]
    if xp is not np and np.sum(sup)>0:
        raise NotImplementedError("the supersonic vortex lattice runs on NumPy only")

    # the receiving points are the control points of the lattice, or other points (a rotor disc)
    n_points = np.shape(XC)[1]
    n_panels = np.shape(XAH)[1]
    RFLAG    = np.ones((n_mach,n_panels),dtype=np.int8)

    # With the camber angle delta and the dihedral angle phi of the control points, the result is the influence
    # matrix of the flow tangency condition and the VORLAX velocity EW on the leading-edge rows only: one array of
    # panels by panels. Without them it is the velocity itself, three such arrays, and EW on every row.
    tangency = delta is not None
    if tangency:
        C_mn             = xp.empty((n_mach,n_points,n_panels),dtype=float)
        normal_x         = xp.atleast_3d(xp.sin(delta)*xp.cos(phi))
        normal_y         = xp.atleast_3d(xp.cos(delta)*xp.sin(phi))
        normal_z         = xp.atleast_3d(xp.cos(phi)*xp.cos(delta))
        leading_edge     = LE_ind[0]
        EW               = xp.empty((n_mach,int(np.sum(leading_edge)),n_panels),dtype=float) if compute_EW == True else np.nan
        leading_edge_row = 0
    else:
        C_mn = xp.empty((n_mach,n_points,n_panels,3),dtype=float)
        EW   = xp.empty((n_mach,n_points,n_panels),dtype=float) if compute_EW == True else np.nan

    # Supersonic cases: which horseshoe vortices are sonic, and the wave term of each panel on itself
    if np.sum(sup)>0:
        RFLAG_sup, wave_diagonal, sonic = sonic_vortex_corrections((x1bar/y1bar)[:,0,:],B2,VD.chord_lengths,VD.panels_per_strip,TE_ind,LE_ind)
        RFLAG[sup,:] = RFLAG_sup[sup,:]

    # The receiving (control) points are taken a block of rows at a time, so the working arrays have a fixed size
    # and only the result grows with the square of the number of panels. A JAX trace takes them in one block.
    rows_per_block = max(1, ROW_BLOCK_ENTRIES//n_panels) if xp is np else n_points
    for first_row in range(0, n_points, rows_per_block):
        rows = slice(first_row, min(first_row + rows_per_block, n_points))

        # This is the receiving point, or the control points
        xo = XC[:,rows,None]
        yo = YC[:,rows,None]
        zo = ZC[:,rows,None]

        xobar = (xo - xc)
        yobar = (yo - yc)*costheta + (zo - zc)*sintheta
        zobar =-(yo - yc)*sintheta + (zo - zc)*costheta

        # COMPUTE COORDINATES OF RECEIVING POINT WITH RESPECT TO END POINTS OF SKEWED LEG.
        shape   = np.shape(xobar)
        s       = xp.repeat(xp.abs(y1bar),shape[1],axis=1)
        t       = xp.repeat(x1bar/y1bar ,shape[1],axis=1)

        X1 = xobar + t*s # In a planar case XC-XAH
        Y1 = yobar + s   # In a planar case YC-YAH
        X2 = xobar - t*s # In a planar case XC-XBH
        Y2 = yobar - s   # In a planar case YC-YBH

        # CALCULATE AXIAL DISTANCE BETWEEN PROJECTION OF RECEIVING POINT ONTO HORSESHOE PLANE AND EXTENSION OF SKEWED LEG.
        XTY = xobar - t*yobar

        # SET VALUES OF NUMERICAL TOLERANCE CONSTANTS.
        TOL    = s /500.0
        TOLSQ  = TOL *TOL
        TOLSQ2 = 2500.0 *TOLSQ
        ZSQ    = zobar *zobar
        YSQ1   = Y1 *Y1
        YSQ2   = Y2 *Y2
        RTV1   = YSQ1 + ZSQ
        RTV2   = YSQ2 + ZSQ
        XSQ1   = X1 *X1
        XSQ2   = X2 *X2
        RO1    = B2*RTV1
        RO2    = B2*RTV2

        # ZERO-OUT PERTURBATION VELOCITY COMPONENTS
        U = xp.zeros((n_mach,shape[1],shape[2] ),dtype=float)
        V = xp.zeros((n_mach,shape[1],shape[2] ),dtype=float)
        W = xp.zeros((n_mach,shape[1],shape[2] ),dtype=float)

        if np.sum(sub)>0:
            # COMPUTATION FOR SUBSONIC HORSESHOE VORTEX
            U_sub, V_sub, W_sub = subsonic(zobar,XSQ1,RO1,XSQ2,RO2,XTY,t,B2,ZSQ,TOLSQ,X1,Y1,X2,Y2,RTV1,RTV2)
            subsonic_case       = sub[:,None,None]
            U, V, W             = xp.where(subsonic_case,U_sub,U), xp.where(subsonic_case,V_sub,V), xp.where(subsonic_case,W_sub,W)

        # COMPUTATION FOR SUPERSONIC HORSESHOE VORTEX. some values computed in a preprocessing section in VLM
        if np.sum(sup)>0:
            U_sup, V_sup, W_sup = supersonic(zobar,XSQ1,RO1,XSQ2,RO2,XTY,t,B2,ZSQ,TOLSQ,TOL,TOLSQ2,X1,Y1,X2,Y2,RTV1,RTV2,CUTOFF)
            W_sup               = apply_sonic_vortex_corrections(W_sup,rows,n_panels,wave_diagonal,sonic)
            U[sup], V[sup], W[sup] = U_sup[sup], V_sup[sup], W_sup[sup]

        # Rotate into the vehicle frame
        if tangency:
            # velocity normal to the camber surface at each control point (validated from book eqn 7.42)
            block =   xp.multiply(U                        ,normal_x[:,rows]) \
                    + xp.multiply(V*costheta - W*sintheta  ,normal_y[:,rows]) \
                    - xp.multiply(V*sintheta + W*costheta  ,normal_z[:,rows])
        else:
            block = xp.stack([U, V*costheta - W*sintheta, V*sintheta + W*costheta],axis=-1)
        C_mn = set_at(C_mn,(slice(None),rows),block)

        if compute_EW == True:
            # Calculate the W velocity in the VORLAX frame for later calcs
            # The angles are Dihedral angle of the current panel - dihedral angle of the influencing panel
            COS1   = xp.cos(DL[:,rows,None] - DL[:,None,:])
            SIN1   = xp.sin(DL[:,rows,None] - DL[:,None,:])
            if tangency:
                in_block          = np.flatnonzero(leading_edge[rows])
                EW                = set_at(EW,(slice(None),slice(leading_edge_row,leading_edge_row + len(in_block))),(W*COS1-V*SIN1)[:,in_block])
                leading_edge_row += len(in_block)
            else:
                EW = set_at(EW,(slice(None),rows),W*COS1-V*SIN1)

    # semispan of the horseshoe vortex of each panel
    s = xp.abs(y1bar)[:,0,:]

    return C_mn, s, RFLAG, EW

def subsonic(Z,XSQ1,RO1,XSQ2,RO2,XTY,T,B2,ZSQ,TOLSQ,X1,Y1,X2,Y2,RTV1,RTV2):
    """  This computes the induced velocities at each control point
    of the vehicle vortex lattice for subsonic mach numbers

    Assumptions:
    Trailing vortex legs infinity are alligned to freestream

    Source:
    1. Miranda, Luis R., Robert D. Elliot, and William M. Baker. "A generalized vortex
    lattice method for subsonic and supersonic flow applications." (1977). (NASA CR)

    2. VORLAX Source Code

    Inputs:
    Z       Z relative location of the vortices          [m]
    XSQ1    X1 squared                                   [m^2]
    RO1     coefficient                                  [-]
    XSQ2    X2 squared                                   [m^2]
    RO2     coefficient                                  [-]
    XTY     AXIAL DISTANCE BETWEEN PROJECTION OF RECEIVING POINT ONTO HORSESHOE PLANE AND EXTENSION OF SKEWED LEG [m]
    T       tangent of the horshoe vortex                [-]
    B2      mach^2-1 (-beta2)                            [-]
    ZSQ     Z squared                                    [m^2]
    TOLSQ   coefficient                                  [-]
    X1      X coordinate of the left side of the vortex  [m]
    Y1      Y coordinate of the left side of the vortex  [m]
    X2      X coordinate of the right side of the vortex [m]
    Y2      Y coordinate of the right side of the vortex [m]
    RTV1    coefficient                                  [-]
    RTV2    coefficient                                  [-]


    Outputs:
    U       X velocity       [unitless]
    V       Y velocity       [unitless]
    W       Z velocity       [unitless]

    Properties Used:
    N/A
    """

    xp   = array_namespace(Z,XSQ1,XSQ2,XTY,T,X1,Y1,X2,Y2)
    CPI  = 4 * np.pi
    RAD1 = xp.sqrt(XSQ1 - RO1)
    RAD2 = xp.sqrt(XSQ2 - RO2)

    TBZ  = (T*T-B2)*ZSQ
    DENOM = XTY * XTY + TBZ

    TOLSQ = xp.broadcast_to(TOLSQ,np.shape(DENOM))

    DENOM = xp.where(DENOM<TOLSQ, TOLSQ, DENOM)

    # a receiving point on the line of a trailing leg: that leg's term is zero, and its division is not evaluated
    on_leg_1 = RTV1<TOLSQ
    on_leg_2 = RTV2<TOLSQ

    FB1 = (T *X1 - B2 *Y1) /RAD1
    FT1 = xp.where(on_leg_1, 0., (X1 + RAD1) /xp.where(on_leg_1, 1., RAD1 *RTV1))

    FB2 = (T *X2 - B2 *Y2) /RAD2
    FT2 = xp.where(on_leg_2, 0., (X2 + RAD2) /xp.where(on_leg_2, 1., RAD2 *RTV2))

    QB = (FB1 - FB2) /DENOM
    ZETAPI = Z /CPI
    in_plane = ZSQ<TOLSQ
    U = xp.where(in_plane, 0., ZETAPI *QB)
    V = xp.where(in_plane, 0., ZETAPI * (FT1 - FT2 - QB *T))
    W = - (QB *XTY + FT1 *Y1 - FT2 *Y2) /CPI

    return U, V, W

def supersonic(Z,XSQ1,RO1,XSQ2,RO2,XTY,T,B2,ZSQ,TOLSQ,TOL,TOLSQ2,X1,Y1,X2,Y2,RTV1,RTV2,CUTOFF):
    """  This computes the induced velocities at each control point
    of the vehicle vortex lattice for supersonic mach numbers

    Assumptions:
    Trailing vortex legs infinity are alligned to freestream

    Source:
    1. Miranda, Luis R., Robert D. Elliot, and William M. Baker. "A generalized vortex
    lattice method for subsonic and supersonic flow applications." (1977). (NASA CR)

    2. VORLAX Source Code

    Inputs:
    Z            Z relative location of the vortices          [m]
    XSQ1         X1 squared                                   [m^2]
    RO1          coefficient                                  [-]
    XSQ2         X2 squared                                   [m^2]
    RO2          coefficient                                  [-]
    XTY          AXIAL DISTANCE BETWEEN PROJECTION OF RECEIVING POINT ONTO HORSESHOE PLANE AND EXTENSION OF SKEWED LEG [m]
    T            tangent of the horshoe vortex                [-]
    B2           mach^2-1 (-beta2)                            [-]
    ZSQ          Z squared                                    [m^2]
    TOLSQ        coefficient                                  [-]
    X1           X coordinate of the left side of the vortex  [m]
    Y1           Y coordinate of the left side of the vortex  [m]
    X2           X coordinate of the right side of the vortex [m]
    Y2           Y coordinate of the right side of the vortex [m]
    RTV1         coefficient                                  [-]
    RTV2         coefficient                                  [-]
    CUTOFF       coefficient                                  [-]



    Outputs:
    U       X velocity        [unitless]
    V       Y velocity        [unitless]
    W       Z velocity        [unitless], without the wave term and the sonic vortex rows (apply_sonic_vortex_corrections)

    Properties Used:
    N/A
    """

    CPI    = 2 * np.pi
    T2     = T*T
    ZETAPI = Z/CPI
    shape  = np.shape(RO1)
    RAD1   = np.sqrt(XSQ1 - RO1)
    RAD2   = np.sqrt(XSQ2 - RO2)

    RAD1[np.isnan(RAD1)] = 0.
    RAD2[np.isnan(RAD2)] = 0.

    DENOM             = XTY * XTY + (T2 - B2) *ZSQ # The last part of this is the TBZ term
    SIGN              = np.ones(shape,dtype=np.int8)
    SIGN[DENOM<0]     = -1.
    TOLSQ             = np.broadcast_to(TOLSQ,shape)
    DENOM_COND        = np.abs(DENOM)<TOLSQ
    DENOM[DENOM_COND] = SIGN[DENOM_COND]*TOLSQ[DENOM_COND]

    # Create a boolean for various conditions for F1 that goes to zero
    bool1           = np.ones(shape,dtype=bool)
    bool1[X1<TOL]   = False
    bool1[RAD1==0.] = False
    RAD1[X1<TOL]    = 0.0

    REPS = CUTOFF*XSQ1
    FRAD = RAD1

    bool1[RO1>REPS] = False
    FB1 = (T*X1-B2*Y1)/FRAD
    FT1 = X1/(FRAD*RTV1)
    FT1[RTV1<TOLSQ] = 0.

    # Use the boolean to turn things off
    FB1[np.isnan(FB1)] = 1.
    FT1[np.isnan(FT1)] = 1.
    FB1[np.isinf(FB1)] = 1.
    FT1[np.isinf(FT1)] = 1.
    FB1 = FB1*bool1
    FT1 = FT1*bool1

    # Round 2
    # Create a boolean for various conditions for F2 that goes to zero
    bool2           = np.ones(shape,dtype=bool)
    bool2[X2<TOL] = False
    bool2[RAD2==0.] = False
    RAD2[X2<TOL]  = 0.0

    REPS = CUTOFF *XSQ2
    FRAD = RAD2

    bool2[RO2>REPS] = False
    FB2 = (T *X2 - B2 *Y2)/FRAD
    FT2 = X2 /(FRAD *RTV2)
    FT2[RTV2<TOLSQ] = 0.

    # Use the boolean to turn things off
    FB2[np.isnan(FB2)] = 1.
    FT2[np.isnan(FT2)] = 1.
    FB2[np.isinf(FB2)] = 1.
    FT2[np.isinf(FT2)] = 1.
    FB2 = FB2*bool2
    FT2 = FT2*bool2

    QB = (FB1 - FB2) /DENOM
    U  = ZETAPI *QB
    V  = ZETAPI *(FT1 - FT2 - QB *T)
    W  = - (QB *XTY + FT1 *Y1 - FT2 *Y2) /CPI

    # COMPUTATION FOR SUPERSONIC HORSESHOE VORTEX WHEN RECEIVING POINT IS IN THE PLANE OF THE HORSESHOE
    in_plane = np.broadcast_to(ZSQ<TOLSQ2,shape)
    RAD1_in  = RAD1[in_plane]
    RAD2_in  = RAD2[in_plane]
    Y1_in    = Y1[ZSQ<TOLSQ2]
    Y2_in    = Y2[ZSQ<TOLSQ2]
    XTY_in   = XTY[ZSQ<TOLSQ2]
    TOL_in   = TOL[ZSQ<TOLSQ2]

    if np.sum(in_plane)>0:
        W_in = supersonic_in_plane(RAD1_in, RAD2_in, Y1_in, Y2_in, TOL_in, XTY_in, CPI)
    else:
        W_in = []

    U[in_plane] = 0.
    V[in_plane] = 0.
    W[in_plane] = W_in

    return U, V, W


def sonic_vortex_corrections(T,B2,chord_lengths,RNMAX,TE_ind,LE_ind):
    """ The parts of the supersonic horseshoe vortex that depend on a panel and its chordwise neighbors, not on
    the receiving point.

    Source:
    1. Miranda, Luis R., Robert D. Elliot, and William M. Baker. "A generalized vortex
    lattice method for subsonic and supersonic flow applications." (1977). (NASA CR)
    2. VORLAX Source Code

    Inputs:
    T               tangent of the horseshoe vortex of each panel   [-]
    B2              mach^2-1                                        [-]
    chord_lengths   chord of the strip of each panel                [m]
    RNMAX           panels per strip                                [-]
    TE_ind, LE_ind  trailing and leading edge panels                [boolean]

    Outputs:
    RFLAG           0 for a sonic vortex, 1 otherwise               [-]
    wave_diagonal   wave term of each panel on itself               [-]
    sonic           True for a sonic vortex                         [boolean]
    """
    # DETERMINE IF TRANSVERSE VORTEX LEG OF HORSESHOE ASSOCIATED TO THE
    # CONTROL POINT UNDER CONSIDERATION IS SONIC (SWEPT PARALLEL TO MACH
    # LINE)? IF SO THEN RFLAG = 0.0, OTHERWISE RFLAG = 1.0.
    n_mach, size = np.shape(T)
    T2S    = T*T
    T2F    = np.zeros((n_mach,size))
    T2A    = np.zeros((n_mach,size))

    # Setup masks
    F_mask = np.ones((n_mach,size),dtype=bool)
    A_mask = np.ones((n_mach,size),dtype=bool)
    F_mask[TE_ind] = False
    A_mask[LE_ind] = False

    # Apply the mask
    T2F[A_mask] = T2S[F_mask]
    T2A[F_mask] = T2S[A_mask]

    # Zero out terms on the LE and TE
    T2F[TE_ind] = 0.
    T2A[LE_ind] = 0.

    TRANS = (B2[:,:,0]-T2F)*(B2[:,:,0]-T2A)
    sonic = TRANS<0
    RFLAG = np.ones((n_mach,size),dtype=np.int8)
    RFLAG[sonic] = 0.

    # WWAVE: a panel's self-induced wave-drag term, only ever nonzero on the diagonal
    COX           = chord_lengths/RNMAX
    B2_panel      = np.broadcast_to(B2[:,0,0][:,None], T2S.shape)
    supersonic_LE = B2_panel > T2S
    wave_diagonal = np.zeros_like(T2S)
    wave_diagonal[supersonic_LE] = - 0.5 *np.sqrt(B2_panel[supersonic_LE] - T2S[supersonic_LE])/COX[supersonic_LE]
    return RFLAG, wave_diagonal, sonic


def apply_sonic_vortex_corrections(W,rows,size,wave_diagonal,sonic):
    """ Adds the wave term to the diagonal of a block of rows of W and sets the rows of sonic vortices.

    Inputs:
    W               normal velocity of a block of receiving points  [-]
    rows            slice of the receiving points in the block
    size            number of panels                                [-]
    wave_diagonal   wave term of each panel on itself               [-]
    sonic           True for a sonic vortex                         [boolean]

    Outputs:
    W               corrected, same block                           [-]
    """
    n_mach     = np.shape(W)[0]
    first      = rows.start
    block      = np.arange(rows.start,rows.stop)
    W[:,block - first,block] = W[:,block - first,block] + wave_diagonal[:,block]

    # A sonic horseshoe vortex's strength is set to the average of its neighbors (the
    # panels immediately before/after it): zero the row, then set self=2 and neighbors=-1.
    cases, panels = np.where(sonic[:,rows])
    W[cases,panels,:] = 0.

    # positions of a sonic panel's own entry in the whole matrix flattened; the entries before and after it are its
    # neighbors, as in the full-matrix form of this step
    cases, panels = np.where(sonic)
    own           = cases*size*size + panels*(size+1)
    for flat, value in ((own, 2.), (own - 1, -1.), (own + 1, -1.)):
        flat = np.where(flat<0, flat + n_mach*size*size, flat)
        if np.any(flat >= n_mach*size*size):
            raise IndexError("the last panel of the lattice is a sonic vortex and has no panel after it")
        case, entry = np.divmod(flat, size*size)
        row, column = np.divmod(entry, size)
        inside      = (row >= rows.start) & (row < rows.stop)
        W[case[inside],row[inside] - first,column[inside]] = value
    return W


def supersonic_in_plane(RAD1,RAD2,Y1,Y2,TOL,XTY,CPI):
    """  This computes the induced velocities at each control point
    in the special case where the vortices lie in the same plane

    Assumptions:
    Trailing vortex legs infinity are alligned to freestream
    In plane vortices only produce W velocity

    Source:
    1. Miranda, Luis R., Robert D. Elliot, and William M. Baker. "A generalized vortex
    lattice method for subsonic and supersonic flow applications." (1977). (NASA CR)

    2. VORLAX Source Code

    Inputs:
    RAD1    array of zeros                               [-]
    RAD2    array of zeros                               [-]
    Y1      Y coordinate of the left side of the vortex  [m]
    Y2      Y coordinate of the right side of the vortex [m]
    TOL     coefficient                                  [-]
    XTY     AXIAL DISTANCE BETWEEN PROJECTION OF RECEIVING POINT ONTO HORSESHOE PLANE AND EXTENSION OF SKEWED LEG [m]
    CPI     2 Pi                                         [radians]


    Outputs:
    W       Z velocity       [unitless]

    Properties Used:
    N/A
    """

    shape = np.shape(RAD2)
    F1    = np.zeros(shape)
    F2    = np.zeros(shape)

    reps  = int(shape[0]/np.size(Y1))

    Y1  = np.tile(Y1,reps)
    Y2  = np.tile(Y2,reps)
    TOL = np.tile(TOL,reps)
    XTY = np.tile(XTY,reps)

    F1[np.abs(Y1)>TOL] = RAD1[np.abs(Y1)>TOL]/Y1[np.abs(Y1)>TOL]
    F2[np.abs(Y2)>TOL] = RAD2[np.abs(Y2)>TOL]/Y2[np.abs(Y2)>TOL]

    W = np.zeros(shape)
    W[np.abs(XTY)>TOL] = (-F1[np.abs(XTY)>TOL] + F2[np.abs(XTY)>TOL])/(XTY[np.abs(XTY)>TOL]*CPI)

    return W
