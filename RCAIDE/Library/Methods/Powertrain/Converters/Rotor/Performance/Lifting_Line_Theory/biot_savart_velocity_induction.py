# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Lifting_Line_Theory/biot_savart_velocity_induction.py
#
# Created:  Jun 2026, H. Hussien

# package imports
import  numpy as  np
import math

# Numba is an OPTIONAL accelerator, not a hard dependency -- every call site keeps working
# identically (falling back to the pure-numpy path below) if it isn't installed. See the
# fast-path dispatch at the bottom of biot_savart_velocity_induction() for how/when it's used.
try:
    from numba import njit
    _NUMBA_AVAILABLE = True
except ImportError:
    _NUMBA_AVAILABLE = False

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
#  _biot_savart_kernel_2d -- fused Numba fast path (plain 2-D P/A/B only, no batch dims)
# ----------------------------------------------------------------------------------------------------------------------
if _NUMBA_AVAILABLE:
    # error_model='numpy' -- Numba's default ('python') raises ZeroDivisionError on float
    # division by exact zero, mimicking CPython; numpy instead produces inf/nan silently.
    # The zero-length-filament degenerate case (s=|B-A|=0) hits this directly (s1=s2=.../s),
    # and the surrounding mask (rm_sq < tol, using a possibly-nan rm_sq) still zeros it out
    # correctly after the fact -- but only if the division itself doesn't raise first.
    @njit(cache=True, error_model='numpy')
    def _biot_savart_kernel_2d(P, A, B, rc_sq_arr, rc_qd_arr, vc_correction, tol):
        """Bit-for-bit transcription of _biot_savart_velocity_induction_numpy's plain 2-D
        (no batch dims) path into one fused (M,N) double loop -- same formula, same order
        of operations, same 0-guards, same mask semantics -- just without materializing
        ~25 separate (M,N)-shaped numpy temporaries for each elementwise pass (that
        function's own docstring notes it's memory-bandwidth-bound at this codebase's
        array sizes, not compute-bound, which is exactly what re-reading each temporary
        from RAM 25 times over costs).

        rc_sq_arr/rc_qd_arr are rc**2/rc**4 (already a plain (N,)-length array, scalar rc
        pre-broadcast by the caller) -- computed via plain numpy in the caller, NOT
        reimplemented here, so this kernel never has to reproduce numpy's own power
        function to stay bit-exact; only sqrt (IEEE-754 correctly-rounded on every
        platform, so guaranteed bit-identical to np.sqrt) and, for vc_correction==4 only,
        exp (math.exp vs np.exp -- verified bit-identical empirically for this codebase's
        inputs in test_biot_savart_bitexact.py, but not IEEE-754-guaranteed in general;
        the dispatcher in biot_savart_velocity_induction() below only routes to this
        kernel at all after that verification held).
        """
        M = P.shape[0]
        N = A.shape[0]
        cross = np.empty((M, N, 3))
        four_pi = 4.0 * np.pi

        for n in range(N):
            r0x = B[n, 0] - A[n, 0]
            r0y = B[n, 1] - A[n, 1]
            r0z = B[n, 2] - A[n, 2]
            r0_norm_sq = r0x*r0x + r0y*r0y + r0z*r0z
            s = math.sqrt(r0_norm_sq)
            rc_sq = rc_sq_arr[n]
            rc_qd = rc_qd_arr[n]

            for m in range(M):
                r1x = P[m, 0] - A[n, 0]
                r1y = P[m, 1] - A[n, 1]
                r1z = P[m, 2] - A[n, 2]

                r1_norm_sq = r1x*r1x + r1y*r1y + r1z*r1z
                r1_norm    = math.sqrt(r1_norm_sq)
                if r1_norm == 0.0:
                    r1_norm = 1e-300

                r1_dot_r0  = r1x*r0x + r1y*r0y + r1z*r0z
                r2_norm_sq = r1_norm_sq - 2.0*r1_dot_r0 + r0_norm_sq
                r2_norm    = math.sqrt(r2_norm_sq)
                if r2_norm == 0.0:
                    r2_norm = 1e-300

                s1 = -r1_dot_r0 / s
                s2 = (r0_norm_sq - r1_dot_r0) / s

                s2ms1 = s2 - s1
                wx = r1x*s2ms1 + r0x*s1
                wy = r1y*s2ms1 + r0y*s1
                wz = r1z*s2ms1 + r0z*s1
                rm_sq = (wx*wx + wy*wy + wz*wz) / (s*s)

                cross_x = r0y*r1z - r0z*r1y
                cross_y = r0z*r1x - r0x*r1z
                cross_z = r0x*r1y - r0y*r1x

                bracket = s2/r2_norm - s1/r1_norm

                f     = 1.0
                denom = rm_sq
                if vc_correction == 1:            # Standard/Scully
                    denom = rm_sq + rc_sq
                elif vc_correction == 2:           # Rankine
                    ratio = rm_sq / rc_sq
                    f = ratio if ratio < 1.0 else 1.0
                elif vc_correction == 3:           # Vatistas
                    f = rm_sq / math.sqrt(rm_sq*rm_sq + rc_qd)
                elif vc_correction == 4:           # Oseen
                    f = 1.0 - math.exp(-1.25643*rm_sq/rc_sq)

                scalar_coeff   = f * bracket / denom
                combined_coeff = scalar_coeff / (four_pi * s)

                if (r1_norm < tol) or (r2_norm < tol) or (rm_sq < tol):
                    combined_coeff = 0.0

                cross[m, n, 0] = cross_x * combined_coeff
                cross[m, n, 1] = cross_y * combined_coeff
                cross[m, n, 2] = cross_z * combined_coeff

        return cross

# ----------------------------------------------------------------------------------------------------------------------
#  Biot_Savart_velocity_induction
# ----------------------------------------------------------------------------------------------------------------------
def biot_savart_velocity_induction(P, A, B, rc=1e-6, vc_correction=1, tol=1e-6):
    # Created:  Jun 2026, H. Hussien
    """
    Dispatches to a fused Numba kernel (bit-identical to the pure-numpy path below, see
    _biot_savart_kernel_2d's docstring) when P/A/B are plain 2-D (no leading batch dims,
    the shape every current call site in this codebase actually uses) and Numba is
    installed. Falls back to the general, batch-capable numpy implementation
    (_biot_savart_velocity_induction_numpy, completely unmodified) for everything else --
    any batch-shaped call, or a plain install with no Numba available.
    """
    P_arr = np.asarray(P, dtype=float)
    A_arr = np.asarray(A, dtype=float)
    B_arr = np.asarray(B, dtype=float)

    if _NUMBA_AVAILABLE and P_arr.ndim == 2 and A_arr.ndim == 2 and B_arr.ndim == 2:
        N = A_arr.shape[0]
        # Broadcast rc to a plain (N,) array of rc^2/rc^4 using numpy's own pow (not
        # reimplemented in the kernel) -- this is the same O(N) computation the numpy
        # path's _expand_rc_for_broadcast machinery reduces to for the plain 2-D case
        # (see that function's docstring), just done once up front instead of being
        # re-broadcast on every (M,N) elementwise pass.
        rc_1d = np.atleast_1d(np.asarray(rc, dtype=float))
        if rc_1d.shape[0] == 1 and N > 1:
            rc_1d = np.full(N, rc_1d[0])
        rc_sq_arr = rc_1d**2
        rc_qd_arr = rc_1d**4
        return _biot_savart_kernel_2d(P_arr, A_arr, B_arr, rc_sq_arr, rc_qd_arr,
                                       int(vc_correction), float(tol))

    return _biot_savart_velocity_induction_numpy(P_arr, A_arr, B_arr, rc, vc_correction, tol)


def _biot_savart_velocity_induction_numpy(P, A, B, rc=1e-6, vc_correction=1, tol=1e-6):
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

    # r1: vector from A to P -- (..., M, N, 3). P/A/B may carry matching leading batch dims
    # (e.g. an azimuth-column axis) ahead of the (M,3)/(N,3) shape; the ellipsis indexing below
    # broadcasts those batch dims through unchanged, and reduces to the original (M, N, 3)
    # behavior when P/A/B are plain 2-D.
    #
    # r2 = P - B is NEVER materialized as an (..., M, N, 3) array: r2 = r1 - r0 where
    # r0 = B - A depends only on the source filament (n), not the field point (m) --
    # s = |B-A| (filament length), r1x2 = r1 x r2, and every other quantity below that
    # historically went through r2 is re-derived algebraically in terms of r1 and the tiny
    # (..., N, 3)-shaped r0 instead (identities verified numerically to ~1e-13 against the
    # direct r1/r2 formulas; this halves the number of full (M,N[,3])-sized temporaries this
    # function allocates, which matters because profiling showed this function is memory-
    # bandwidth-bound, not compute-bound, at the array sizes this codebase actually uses).
    r1 = P[..., :, np.newaxis, :] - A[..., np.newaxis, :, :]
    r0 = B - A                                              # (..., N, 3) -- independent of m

    r1x, r1y, r1z = r1[..., 0], r1[..., 1], r1[..., 2]
    r0x, r0y, r0z = r0[..., 0], r0[..., 1], r0[..., 2]

    # Broadcast r0's components against the (..., M, N) shape (insert the M axis) -- r0 always
    # carries exactly A/B's own batch dims, so a single newaxis before the last (N) axis is
    # always correct here (unlike rc, which may carry fewer batch dims than the target and
    # needs _expand_rc_for_broadcast's more general handling).
    r0x_b, r0y_b, r0z_b = r0x[..., np.newaxis, :], r0y[..., np.newaxis, :], r0z[..., np.newaxis, :]

    r1_norm_sq = r1x*r1x + r1y*r1y + r1z*r1z          # (..., M, N)
    r1_norm    = np.sqrt(r1_norm_sq)
    # Guard against exact 0/0 when P sits exactly on a filament endpoint -- s1/r1_norm below
    # would otherwise divide by zero before the tol-based mask gets a chance to exclude that
    # point. 1e-300 is still well below any reasonable tol, so the mask's own exclusion is
    # unaffected -- this only silences the division itself (same pattern as the Cd/(Cl+1e-300)
    # guard in evaluate_bound_vortex_circulation.py).
    r1_norm    = np.where(r1_norm == 0.0, 1e-300, r1_norm)

    r0_norm_sq   = r0x*r0x + r0y*r0y + r0z*r0z          # (..., N) -- = filament length^2
    s            = np.sqrt(r0_norm_sq)                  # (..., N)
    s_b          = s[..., np.newaxis, :]                # (..., 1, N), broadcasts against (...,M,N)
    r0_norm_sq_b = r0_norm_sq[..., np.newaxis, :]

    r1_dot_r0  = r1x*r0x_b + r1y*r0y_b + r1z*r0z_b                        # (..., M, N)
    r2_norm_sq = r1_norm_sq - 2.0*r1_dot_r0 + r0_norm_sq_b                # (..., M, N)
    r2_norm    = np.sqrt(r2_norm_sq)
    # Same guard as r1_norm above -- P exactly on the filament's B endpoint would otherwise
    # divide by zero in s2/r2_norm below, before the tol-based mask excludes that point.
    r2_norm    = np.where(r2_norm == 0.0, 1e-300, r2_norm)

    s1 = -r1_dot_r0 / s_b                             # (..., M, N)
    s2 = (r0_norm_sq_b - r1_dot_r0) / s_b

    # rm_sq = |(r1*s2 - r2*s1)/s|^2 -- only the squared magnitude is ever used, so the full
    # (..., M, N, 3) rm vector (and the division of it by s) is skipped entirely. r2 = r1 - r0,
    # so r1*s2 - r2*s1 = r1*(s2-s1) + r0*s1 -- avoids ever forming r2's components.
    s2ms1 = s2 - s1
    wx = r1x*s2ms1 + r0x_b*s1
    wy = r1y*s2ms1 + r0y_b*s1
    wz = r1z*s2ms1 + r0z_b*s1
    rm_sq = (wx*wx + wy*wy + wz*wz) / (s_b*s_b)          # (..., M, N)

    # Cross product r1 x r2 == r0 x r1 (since r2 = r1 - r0 and r1 x r1 = 0) -- (..., M, N, 3).
    cross = np.empty_like(r1)
    cross[..., 0] = r0y_b*r1z - r0z_b*r1y
    cross[..., 1] = r0z_b*r1x - r0x_b*r1z
    cross[..., 2] = r0x_b*r1y - r0y_b*r1x

    rc_sq   = np.atleast_1d(rc)**2
    rc_sq_b = _expand_rc_for_broadcast(rc_sq, rm_sq.ndim)   # aligned against (..., M, N)

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
    scalar_coeff   = f * bracket / denom          # (..., M, N) -- f, bracket, denom scalar per (...,m,n)
    combined_coeff = scalar_coeff[..., np.newaxis] / (4.0 * np.pi * s_b[..., np.newaxis])

    # Mask: skip contribution if P is on or near A, on or near B,
    # or if P is collinear with the segment (cross product near zero)
    mask = (r1_norm < tol) | (r2_norm < tol) | (rm_sq < tol)
    combined_coeff[mask] = 0.0

    cross *= combined_coeff

    return cross