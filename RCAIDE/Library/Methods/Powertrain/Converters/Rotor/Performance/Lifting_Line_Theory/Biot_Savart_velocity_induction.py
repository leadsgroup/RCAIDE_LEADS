# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Lifting_Line_Theory/biot_savart_velocity_induction.py
#
# Created:  Jun 2026, H. Hussien

# package imports
import  numpy as  np

# ----------------------------------------------------------------------------------------------------------------------
#  _expand_rc_for_broadcast
# ----------------------------------------------------------------------------------------------------------------------
def _expand_rc_for_broadcast(rc_arr, target_ndim):
    """Insert size-1 axes into rc_arr, right before its last (N) axis, so it broadcasts
    correctly against a target of ndim `target_ndim` whose last axis is also N and whose
    remaining leading axes are (some or all of) P/A/B's batch dims.

    rc_arr's own leading axes (if any) are assumed to align with the OUTERMOST leading
    axes of the target (e.g. rc varying by ctrl_pts while being shared/constant across an
    INNER batch axis, like an azimuth-column index J, nested inside ctrl_pts). Plain numpy
    broadcasting can't do this on its own: its automatic left-padding only ever adds
    implicit size-1 axes at the very front, which would silently misalign rc_arr's own
    batch axis against the wrong target axis whenever rc_arr has fewer dims than the
    target but more than just the bare (N,)/scalar case (e.g. rc_arr=(ctrl_pts,N) against
    a (ctrl_pts,J,M,N) target would otherwise align ctrl_pts against M, not against
    ctrl_pts). When rc_arr is already just (N,)/scalar, n_insert below reduces to exactly
    the single np.newaxis the original (pre-batching) code used, so this is a no-op change
    for every existing 2-D call site.
    """
    n_insert = target_ndim - rc_arr.ndim
    if n_insert <= 0:
        return rc_arr
    idx = (Ellipsis,) + (np.newaxis,) * n_insert + (slice(None),)
    return rc_arr[idx]

# ----------------------------------------------------------------------------------------------------------------------
#  Biot_Savart_velocity_induction
# ----------------------------------------------------------------------------------------------------------------------
def biot_savart_velocity_induction(P, A, B, rc=1e-6, vc_correction=1, tol=1e-6):
    # Created:  Jun 2026, H. Hussien
    """
    Computes the Biot-Savart influence tensor for a set of straight vortex filaments.

    Parameters
    ----------
    P : (..., M, 3) array_like
        Field points expressed in the vehicle/body frame [m]. An optional leading batch
        shape (e.g. an azimuth-column axis) is supported ahead of the (M, 3) shape --
        each batch slice only interacts with the matching slice of A/B (block-diagonal,
        not all-pairs across the batch). Plain (M, 3) input (no batch dims) is the
        original, most common case.
    A : (..., N, 3) array_like
        Filament start points expressed in the vehicle/body frame [m]. Batch shape (if
        any) must match P's.
    B : (..., N, 3) array_like
        Filament end points expressed in the vehicle/body frame [m]. Batch shape (if
        any) must match P's.
    rc : float, optional
        Rankine core radius [m]. Default is 1e-6 m. May be a plain (N,)-shaped (or
        scalar) array shared across every batch slice, OR carry its own leading batch
        dims (e.g. (ctrl_pts, N)) that align with the OUTERMOST leading batch dims of
        P/A/B -- any remaining (inner) batch dims of P/A/B that rc does not itself vary
        over (e.g. an azimuth-column axis nested inside ctrl_pts) are broadcast
        automatically. See _expand_rc_for_broadcast below for why this needs explicit
        handling instead of relying on plain numpy broadcasting.

    Returns
    -------
    K : (..., M, N, 3) ndarray
        Influence tensor [1/m]. The velocity induced at field point m by
        filament n carrying circulation Gamma_n is K[..., m, n, :] * Gamma_n.
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

    # r1: vector from A to P,  r2: vector from B to P  -- (..., M, N, 3). P/A/B may carry
    # matching leading batch dims (e.g. an azimuth-column axis) ahead of the (M,3)/(N,3) shape;
    # the ellipsis indexing below broadcasts those batch dims through unchanged, and reduces
    # to the original (M, N, 3) behavior when P/A/B are plain 2-D.
    r1 = P[..., :, np.newaxis, :] - A[..., np.newaxis, :, :]
    r2 = P[..., :, np.newaxis, :] - B[..., np.newaxis, :, :]

    # All the 3-component reductions below (norm, dot, cross) are hand-unrolled instead of
    # going through np.linalg.norm/np.einsum/np.cross -- those carry real generic-dispatch
    # overhead for what is, per element, just a handful of multiply-adds on a fixed-size-3
    # vector; profiling showed this was a significant chunk of the function's own runtime.
    # Mathematically identical to the generic-numpy version (only floating-point reassociation-
    # level differences, same as already tolerated by the project's existing bit-for-bit checks).
    r1x, r1y, r1z = r1[..., 0], r1[..., 1], r1[..., 2]
    r2x, r2y, r2z = r2[..., 0], r2[..., 1], r2[..., 2]

    r1_norm_sq = r1x*r1x + r1y*r1y + r1z*r1z          # (..., M, N)
    r2_norm_sq = r2x*r2x + r2y*r2y + r2z*r2z
    r1_norm    = np.sqrt(r1_norm_sq)
    r2_norm    = np.sqrt(r2_norm_sq)

    r1_dot_r2   = r1x*r2x + r1y*r2y + r1z*r2z                                   # (..., M, N)
    s           = np.sqrt(r1_norm_sq + r2_norm_sq - 2*r1_dot_r2 + 1e-300)       # (..., M, N)

    r2mr1x, r2mr1y, r2mr1z = r2x - r1x, r2y - r1y, r2z - r1z    # r2_minus_r1 components

    s1 = (r1x*r2mr1x + r1y*r2mr1y + r1z*r2mr1z) / s     # (..., M, N)
    s2 = (r2x*r2mr1x + r2y*r2mr1y + r2z*r2mr1z) / s

    # rm_sq = |(r1*s2 - r2*s1)/s|^2 -- only the squared magnitude is ever used, so the full
    # (..., M, N, 3) rm vector (and the division of it by s) is skipped entirely.
    wx = r1x*s2 - r2x*s1
    wy = r1y*s2 - r2y*s1
    wz = r1z*s2 - r2z*s1
    rm_sq = (wx*wx + wy*wy + wz*wz) / (s*s)              # (..., M, N)

    # Cross product r1 x r2  -- (..., M, N, 3).
    cross = np.empty_like(r1)
    cross[..., 0] = r1y*r2z - r1z*r2y
    cross[..., 1] = r1z*r2x - r1x*r2z
    cross[..., 2] = r1x*r2y - r1y*r2x

    rc_sq   = np.atleast_1d(rc)**2
    rc_sq_b = _expand_rc_for_broadcast(rc_sq, rm_sq.ndim)   # aligned against (..., M, N)

    factor = cross / (4.0 * np.pi * s[..., np.newaxis])

    bracket = (s2/r2_norm - s1/r1_norm)

    denom = rm_sq

    f = 1

    # Vortex core correction
    if   vc_correction == 1: # Standard/Scully method
        denom = rm_sq + rc_sq_b

    elif vc_correction == 2: # Rankine method
        f = np.minimum(rm_sq/rc_sq_b,1)

    elif vc_correction == 3: # Vatistas method
        rc_qd   = np.atleast_1d(rc)**4
        rc_qd_b = _expand_rc_for_broadcast(rc_qd, rm_sq.ndim)
        f = rm_sq/np.sqrt(rm_sq**2+rc_qd_b)

    elif vc_correction == 4: # Oseen method
        a = 1.25643
        f = 1 - np.exp(-a*rm_sq/rc_sq_b)

    # Influence tensor  -- (..., M, N, 3)
    scalar_coeff = f * bracket / denom          # (..., M, N) -- f, bracket, denom scalar per (...,m,n)
    K            = factor * scalar_coeff[..., np.newaxis]

    # Mask: skip contribution if P is on or near A, on or near B,
    # or if P is collinear with the segment (cross product near zero)
    mask = (r1_norm < tol) | (r2_norm < tol) | (rm_sq < tol)
    K[mask] = 0.0

    return K
