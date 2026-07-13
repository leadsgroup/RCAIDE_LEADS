# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Lifting_Line_Theory/biot_savart_velocity_induction.py
# 
# Created:  Jun 2026, H. Hussien 

# package imports
import  numpy as  np 

# ----------------------------------------------------------------------------------------------------------------------
#  Biot_Savart_velocity_induction
# ----------------------------------------------------------------------------------------------------------------------
def biot_savart_velocity_induction(P, A, B, rc=1e-6, vc_correction=1, tol=1e-6):
    # Created:  Jun 2026, H. Hussien
    """
    Computes the Biot-Savart influence tensor for a set of straight vortex filaments.

    Parameters
    ----------
    P : (M, 3) array_like
        Field points expressed in the vehicle/body frame [m].
    A : (N, 3) array_like
        Filament start points expressed in the vehicle/body frame [m].
    B : (N, 3) array_like
        Filament end points expressed in the vehicle/body frame [m].
    rc : float, optional
        Rankine core radius [m]. Default is 1e-6 m.

    Returns
    -------
    K : (M, N, 3) ndarray
        Influence tensor [1/m]. The velocity induced at field point m by
        filament n carrying circulation Gamma_n is K[m, n, :] * Gamma_n.
        Contraction with Gamma is the caller's responsibility.

    Notes
    -----
    **Coordinate frame**
        All points (P, A, B) must be expressed in the same vehicle/body (fixed)
        frame. For multi-rotor configurations this is mandatory: convert
        blade-fixed or hub-fixed coordinates to the vehicle/body frame before
        calling this function.

    **Sign convention**
        Positive Gamma [m^2/s] produces velocity consistent with the
        right-hand rule about the direction A -> B.

    **Desingularization**
        A Rankine solid core of radius rc is applied by adding rc^2 to the
        squared magnitude of the cross product in the denominator. This
        ensures the induced velocity approaches zero smoothly as the field
        point approaches the filament axis. rc is a forced constant in this
        version.

    **Theory**
        The velocity induced by a finite straight vortex filament from A to B
        at field point P is given by [1]:

        .. math::

            \\mathbf{u} = \\frac{\\Gamma}{4\\pi}
            \\frac{\\mathbf{r}_1 \\times \\mathbf{r}_2}
                 {|\\mathbf{r}_1 \\times \\mathbf{r}_2|^2 + r_c^2}
            \\, \\mathbf{r}_0 \\cdot
            \\left( \\frac{\\mathbf{r}_1}{|\\mathbf{r}_1|}
                  - \\frac{\\mathbf{r}_2}{|\\mathbf{r}_2|} \\right)

        where :math:`\\mathbf{r}_0 = B - A`,
        :math:`\\mathbf{r}_1 = P - A`, :math:`\\mathbf{r}_2 = P - B`,
        and :math:`r_c` is the Rankine core radius.

    References
    ----------
    [1] J. Katz and A. Plotkin, Low-Speed Aerodynamics, 2nd ed.,
        Cambridge University Press, 2001, Section 2.12, Eq. 2.72.
    [2] W. Johnson, Rotorcraft Aeromechanics, Cambridge University Press, 2013, Section 9.9, Eq. 9.97

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line.lifting_line_performance
    """
    P = np.asarray(P, dtype=float)
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float)

    # r1: vector from A to P,  r2: vector from B to P  -- (M, N, 3)
    r1 = P[:, np.newaxis, :] - A[np.newaxis, :, :]
    r2 = P[:, np.newaxis, :] - B[np.newaxis, :, :]

    r1_norm = np.linalg.norm(r1, axis=-1, keepdims=True)  # (M, N, 1)
    r2_norm = np.linalg.norm(r2, axis=-1, keepdims=True)  # (M, N, 1)

    rc_sq = np.atleast_1d(rc)**2

    # r0 = B - A  (filament direction vector)  -- (N, 3)
    r0      = B - A

    # Cross product r1 x r2  -- (M, N, 3)
    cross = np.cross(r1, r2)
    cross_norm_sq = np.einsum('mnk,mnk->mn', cross, cross)
    
    # Unit vectors along r1 and r2  -- (M, N, 3)
    r1_hat = r1 / (r1_norm + 1e-300)
    r2_hat = r2 / (r2_norm + 1e-300)

    # Geometric dot products:  r1_hat . r0  and  r2_hat . r0  -- (M, N)
    cos1 = np.einsum('mnk,nk->mn', r1_hat, r0)
    cos2 = np.einsum('mnk,nk->mn', r2_hat, r0)
    
    # Vortex core correction
    if   vc_correction == 1: # Standard method
        denom = cross_norm_sq + rc_sq

    elif vc_correction in (2, 3, 4, 5):

        r1_dot_r2   = np.einsum('mnk,mnk->mn', r1, r2)           # (M, N)
        s           = np.sqrt(r1_norm**2 + r2_norm**2 - 2*r1_dot_r2 + 1e-300)   # (M, N)
        r2_minus_r1 = r2 - r1                               # (M, N, 3)
        s1 = np.einsum('mnk,mnk->mn', r1, r2_minus_r1) / s     # (M, N)
        s2 = np.einsum('mnk,mnk->mn', r2, r2_minus_r1) / s
        rm = (r1 * s2[:, :, np.newaxis] - r2 * s1[:, :, np.newaxis]) / s[:, :, np.newaxis]   # (M, N, 3)
        rm_sq = np.einsum('mnk,mnk->mn', rm, rm)   # (M, N)

        if vc_correction == 2: # Rankine method
            f = np.minimum(rm_sq/rc_sq[np.newaxis, :],1)
            denom = cross_norm_sq * (1 / f)

        elif vc_correction == 3: # Scully method
            f = rm_sq/(rm_sq+rc_sq[np.newaxis, :])
            denom = cross_norm_sq * (1 / f)

        elif vc_correction == 4: # Vatistas method
            rc_qd = np.atleast_1d(rc)**4
            f = rm_sq/np.sqrt(rm_sq**2+rc_qd[np.newaxis, :])
            denom = cross_norm_sq * (1 / f)
            
        elif vc_correction == 5: # Oseen method
            a = 1.25643
            f = 1 - np.exp(-a*rm_sq/rc_sq[np.newaxis, :])
            denom = cross_norm_sq * (1 / f)

    # Biot-Savart scalar factor: (cos1 - cos2) / (4*pi*denom)  -- (M, N)
    factor = (cos1 - cos2) / (4.0 * np.pi * denom)

    # Influence tensor  -- (M, N, 3)
    K = cross * factor[:, :, np.newaxis]

    # Mask: skip contribution if P is on or near A, on or near B,
    # or if P is collinear with the segment (cross product near zero)
    mask = (r1_norm[..., 0] < tol) | (r2_norm[..., 0] < tol) | (cross_norm_sq < tol)
    K[mask] = 0.0

    return K