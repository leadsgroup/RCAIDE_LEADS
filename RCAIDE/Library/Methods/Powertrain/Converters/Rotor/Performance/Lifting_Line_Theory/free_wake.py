# This code is used to update the wake locations using the free-wake psuedo implicit model introducted by
# Bagai and Leishman, 1995.

# The first step is to model the wake shed from all the blades as a 2-D grid.
# Each node in the grid has its own psi and wake_age, making it uniquely defined by these two variables

# The second step is to compute the velocity induction all the wake nodes.

# The third step is to use the use the psuedo implicit method to update the location of the wake nodes

import numpy as np
from concurrent.futures                import ThreadPoolExecutor
from RCAIDE.Framework.Core            import orientation_transpose
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory import biot_savart_velocity_induction, initialize_wake_geometry, initialize_lifting_line

def free_wake(rotor, wake_inputs, conditions):

    #------------------------------------
    # 0 - pre-computations
    #------------------------------------
    if 'blades' not in rotor or rotor.blades.bound.get('nodes_body_14c', None) is None:
        initialize_lifting_line(rotor, conditions)
    r_blade_old = rotor.blades.bound.nodes_body_14c

    if rotor.blades.wake.get('nodes_body', None) is None:
        initialize_wake_geometry(rotor, wake_inputs, conditions)
    r_wake_old = rotor.blades.wake.nodes_body

    Gamma_b = rotor.blades.bound.gamma
    rcb     = rotor.blades.bound.rCb
    r_b_start = r_blade_old[:, :-1, :, :]
    r_b_end   = r_blade_old[:,  1:, :, :]
    Gamma_w = rotor.blades.wake.gamma
    rcvf    = rotor.blades.wake.rCvf

    #-----------------------------------
    # 1 - creating the grids
    #-----------------------------------
    ctrl_pts = r_wake_old.shape[0]
    B        = r_wake_old.shape[2]
    N_wake   = r_wake_old.shape[1] - 1
    Nr_s     = r_b_start.shape[1]

    dzeta = rotor.blades.wake.wakeage[1] - rotor.blades.wake.wakeage[0]
    nwa   = len(rotor.blades.wake.wakeage)
    dpsi  = dzeta
    J     = int(round(2*np.pi / dpsi))

    CW   = wake_inputs.omega[:, 0] > 0     # (ctrl_pts,) -- per-control-point rotation sense,
                                            # matching initialize_lifting_line.py/initialize_wake_geometry.py
    CW_b = CW[:, np.newaxis, np.newaxis]   # (ctrl_pts, 1, 1) -- broadcast helper for the
                                            # (ctrl_pts, J, Nr_s)/(ctrl_pts, J, N_wake+1) grids below
    j_rot    = dpsi * np.arange(J)
    cos_psi, sin_psi = np.cos(j_rot), np.sin(j_rot)

    # The per-j rotation below represents "where is this point once the blade has swept
    # forward by j*dpsi" -- a pure (y,z) rotation ONLY in the frame where the rotor's own
    # spin axis is the x-axis (hub/thrust frame -- same frame initialize_lifting_line.py/
    # initialize_wake_geometry.py use for this exact purpose). Body frame can differ from
    # thrust frame by a nonzero tilt (rotor cant / commanded thrust vector angle), so
    # rotating directly in body-frame (y,z) is wrong whenever that tilt is nonzero --
    # confirmed by reproducing per-blade wake asymmetry, with otherwise perfectly uniform
    # circulation, purely from a nonzero tilt.
    #
    # Rather than transform to hub/thrust frame just for this rotation and immediately back
    # to body frame, the WHOLE relaxation below (grids, Biot-Savart, V_inf's convection term)
    # runs in hub/thrust frame -- Biot-Savart is purely geometric and doesn't care which
    # consistent frame it's given, so this is a strictly smaller number of transforms: one
    # conversion in (here), one conversion back to body out (at the final feedback), instead
    # of converting in, back out, and separately converting V_inf's frame too.
    hub_origin = np.array(rotor.origin[0]).reshape(1, 1, 1, 3)
    T_b2t      = wake_inputs.T_body2thrust                      # (ctrl_pts,3,3)
    T_t2b_geom = orientation_transpose(T_b2t)

    r_b_start_hub  = np.einsum('cij,crbj->crbi', T_b2t, r_b_start  - hub_origin)
    r_b_end_hub    = np.einsum('cij,crbj->crbi', T_b2t, r_b_end    - hub_origin)
    r_wake_old_hub = np.einsum('cij,cwbj->cwbi', T_b2t, r_wake_old - hub_origin)

    # -- bound position grid (built in hub/thrust frame) --
    Bgrid_start = np.zeros((ctrl_pts, B, J, Nr_s, 3))
    Bgrid_end   = np.zeros((ctrl_pts, B, J, Nr_s, 3))
    for b in range(B):
        xb0 = r_b_start_hub[:, :, b, 0]; xb1 = r_b_end_hub[:, :, b, 0]
        yb0 = r_b_start_hub[:, :, b, 1]; yb1 = r_b_end_hub[:, :, b, 1]
        zb0 = r_b_start_hub[:, :, b, 2]; zb1 = r_b_end_hub[:, :, b, 2]
        Bgrid_start[:, b, :, :, 0] = xb0[:, None, :]
        Bgrid_end[:,   b, :, :, 0] = xb1[:, None, :]
        Bgrid_start[:, b, :, :, 1] = np.where(CW_b,
            cos_psi[None, :, None]*yb0[:, None, :] - sin_psi[None, :, None]*zb0[:, None, :],
            cos_psi[None, :, None]*yb0[:, None, :] + sin_psi[None, :, None]*zb0[:, None, :])
        Bgrid_start[:, b, :, :, 2] = np.where(CW_b,
            sin_psi[None, :, None]*yb0[:, None, :] + cos_psi[None, :, None]*zb0[:, None, :],
           -sin_psi[None, :, None]*yb0[:, None, :] + cos_psi[None, :, None]*zb0[:, None, :])
        Bgrid_end[:,   b, :, :, 1] = np.where(CW_b,
            cos_psi[None, :, None]*yb1[:, None, :] - sin_psi[None, :, None]*zb1[:, None, :],
            cos_psi[None, :, None]*yb1[:, None, :] + sin_psi[None, :, None]*zb1[:, None, :])
        Bgrid_end[:,   b, :, :, 2] = np.where(CW_b,
            sin_psi[None, :, None]*yb1[:, None, :] + cos_psi[None, :, None]*zb1[:, None, :],
           -sin_psi[None, :, None]*yb1[:, None, :] + cos_psi[None, :, None]*zb1[:, None, :])

    # -- wake position grid (built in hub/thrust frame) --
    r_wake_grid = np.zeros((ctrl_pts, B, J, N_wake+1, 3))
    for b in range(B):
        x0 = r_wake_old_hub[:, :, b, 0]
        y0 = r_wake_old_hub[:, :, b, 1]
        z0 = r_wake_old_hub[:, :, b, 2]
        r_wake_grid[:, b, :, :, 0] = x0[:, None, :]
        r_wake_grid[:, b, :, :, 1] = np.where(CW_b,
            cos_psi[None, :, None]*y0[:, None, :] - sin_psi[None, :, None]*z0[:, None, :],
            cos_psi[None, :, None]*y0[:, None, :] + sin_psi[None, :, None]*z0[:, None, :])
        r_wake_grid[:, b, :, :, 2] = np.where(CW_b,
            sin_psi[None, :, None]*y0[:, None, :] + cos_psi[None, :, None]*z0[:, None, :],
           -sin_psi[None, :, None]*y0[:, None, :] + cos_psi[None, :, None]*z0[:, None, :])

    # Bgrid_start/Bgrid_end/r_wake_grid stay in hub/thrust frame from here on -- see the
    # relaxation-constants section below for why (V_inf), and the final feedback section
    # for the single hub->body conversion back, at the very end.

    # -- Gamma / core-radius, blade-major (position-independent, same for every J column) --
    Gamma_w_bk = np.transpose(Gamma_w, (0, 2, 1))
    rcvf_bk    = np.broadcast_to(rcvf[:, None, :], (ctrl_pts, B, N_wake))
    Gamma_b_bk = np.transpose(Gamma_b, (0, 2, 1))
    rcb_bk     = np.transpose(rcb,     (0, 2, 1))

    #-----------------------------------
    # relaxation constants
    #-----------------------------------
    vc_correction = wake_inputs.vc_correction
    Omega = np.abs(wake_inputs.omega)                                 # (ctrl_pts,1)
    V_inf = wake_inputs.V_thrust   # (ctrl_pts,3) -- already thrust/hub-frame, no conversion
                                   # needed now that the relaxation runs in that frame throughout
    eta1  = (dpsi - dzeta) / (dpsi + dzeta)   # = 0 exactly here, since dpsi = dzeta
    eta2  = (dpsi * dzeta) / (dpsi + dzeta)

    # Everything below is fixed for the whole relaxation (bound geometry/circulation never
    # change, and Gamma_w/rcvf/rcb don't vary across the J columns) -- precomputed once here
    # instead of being re-sliced and re-reshaped (forcing a copy, since these are broadcast
    # views) on every compute_V_ind call.
    Gamma_w_flat = Gamma_w_bk.reshape(ctrl_pts, B*N_wake)          # same for every j
    rcvf_flat    = rcvf_bk.reshape(ctrl_pts, B*N_wake)
    Gamma_b_flat = Gamma_b_bk.reshape(ctrl_pts, B*Nr_s)
    rcb_flat     = rcb_bk.reshape(ctrl_pts, B*Nr_s)

    # bound geometry does rotate with j (real, meaningful variation) -- precompute all J columns
    # once, up front, instead of re-slicing/reshaping the same values on every call.
    A_blade_all = Bgrid_start.transpose(0, 2, 1, 3, 4).reshape(ctrl_pts, J, B*Nr_s, 3)
    B_blade_all = Bgrid_end.transpose(  0, 2, 1, 3, 4).reshape(ctrl_pts, J, B*Nr_s, 3)

    # Each (cp,j) task below is fully independent -- no data dependency between columns/control
    # points within compute_V_ind (that coupling happens through the OUTER predictor/corrector
    # loop, not here) -- and each writes to a disjoint V_ind[cp,:,j,:,:] slice, so this is safe
    # to run across threads with no locking. numpy's C-level array ops release the GIL during
    # computation, so this gets real multi-core parallelism, not just concurrency. One
    # ThreadPoolExecutor is created here and reused for every compute_V_ind call in this
    # free_wake() invocation (shut down at the very end) -- creating a fresh one per call
    # measured meaningfully slower than reusing one (thread creation/teardown overhead, paid
    # 2x per outer iteration otherwise). Verified bit-for-bit identical to the serial version;
    # measured ~2.9x faster via a controlled wall-clock A/B (10 runs, 5 each, min-of-N) on a
    # 20-core machine -- unlike the batching/merging attempts documented below, this doesn't
    # grow any array size (so no new cache pressure), it just runs the same cache-sized
    # per-(cp,j) calls on different cores at once.
    _tasks    = [(cp, j) for cp in range(ctrl_pts) for j in range(J)]
    _executor = ThreadPoolExecutor()

    def compute_V_ind(field_grid, source_grid):
        """Same sourcing rule as update_free_wake_location.py -- sources for a query at column
        j are column j of every blade's grid, matched to that query's own instant. Runs the
        (cp,j) tasks across the thread pool created above -- see that comment for why/how this
        is safe and the measured speedup.

        Also tried and measured NOT faster: merging the wake and blade sources into a single
        biot_savart_velocity_induction call per (cp,j) (concatenating along N instead of two
        separate calls). Verified bit-for-bit correct, but a controlled wall-clock A/B (10 runs,
        5 each, min-of-N to filter system noise) showed the merged version ~10% SLOWER, not
        faster -- despite halving call count for only modestly more N at real scale, it didn't
        pay off the way the earlier hand-unroll/r0-elimination wins did. Kept as two separate
        calls.

        Deliberately NOT batched over ctrl_pts or J either (both were tried and measured to be
        SLOWER, not faster, despite doing the same total elementwise work with fewer Python
        calls): biot_savart_velocity_induction runs ~25 sequential elementwise passes over
        temporaries shaped like its (M,N) output. At real scale (ctrl_pts=8, J=24,
        M~222, N~230) those temporaries are only ~1.2 MB per (cp,j) slice -- comfortably
        cache-resident -- but batching either axis multiplies that working set by
        ctrl_pts and/or J (up to ~220 MB), blowing past L2/L3 and turning every one of
        those ~25 passes into a full RAM round-trip. Measured on the real Twin Otter case
        (scratchpad/test_free_wake_realscale.py-style benchmark): fully unbatched here is
        ~1.6x FASTER than batching both axes, and batching ctrl_pts alone is already
        slower than not batching at all. Re-benchmark before re-batching this if the
        problem size changes substantially.
        """
        V_ind = np.zeros_like(field_grid)

        def _compute_one(cp, j):
            P = field_grid[cp, :, j, :, :].reshape(B*(N_wake+1), 3)

            wake_src  = source_grid[cp, :, j, :, :]
            r_w_start = wake_src[:, :-1, :].reshape(B*N_wake, 3)
            r_w_end   = wake_src[:,  1:, :].reshape(B*N_wake, 3)
            K_wake = biot_savart_velocity_induction(
                P, r_w_start, r_w_end, rcvf_flat[cp], vc_correction)
            v_wake = np.einsum('mnk,n->mk', K_wake, Gamma_w_flat[cp])

            K_blade = biot_savart_velocity_induction(
                P, A_blade_all[cp, j], B_blade_all[cp, j], rcb_flat[cp], vc_correction)
            v_blade = np.einsum('mnk,n->mk', K_blade, Gamma_b_flat[cp])
            v_sum = (v_blade + v_wake).reshape(B, N_wake+1, 3)
            V_ind[cp, :, j, :, :] = np.where(CW[cp], -v_sum, v_sum)

        list(_executor.map(lambda t: _compute_one(*t), _tasks))
        return V_ind

    # A single lap of the j-loop can't fully resolve the periodic seam (j=0 reading j=J-1's
    # value) when the wake-age range (nwa) exceeds the number of columns (J) -- the "freshness"
    # from column 0 needs multiple laps to reach every k. n_inner_passes = ceil(nwa/J) is the
    # minimum that fully resolves it -- verified via the Appendix B constant-source-term check
    # (exact one-outer-iteration convergence, machine-precision match to the closed form), and
    # via a direct comparison against the pre-fix version on real hover/FF cases: same converged
    # answer (~1e-5 of R), fewer outer iterations needed (hover 29->9, FF 15->10 at tol=1e-6).
    # Only used by the eta1!=0 fallback loop below -- the eta1==0 fast path resolves the seam
    # exactly in closed form regardless of pass count.
    n_inner_passes = int(np.ceil(nwa / J))

    # -- Closed-form fast path (eta1==0, i.e. dpsi==dzeta, always true in this codebase) --
    # With eta1=0, r_new[j,k] = r_new[j-1,k-1] + C[j,k-1] for k=1..N_wake, with r_new[j,0] fixed
    # (the wake-root boundary condition, never updated). This is a pure diagonal recurrence:
    # unrolling it gives r_new[j,k] = anchor[j0] + sum_{i=1}^{k} C[(j0+i)%J, i-1], j0=(j-k)%J --
    # computable with two fancy-index gathers + one cumsum instead of the O(n_inner_passes*J)
    # sequential loop. Verified (scratchpad/test_diagonal_recurrence.py) to reproduce the
    # nested-loop result exactly (to floating-point reassociation, ~1e-14) once the loop is
    # given enough passes to itself converge -- and, for this repo's actual parameter regime
    # (N_wake > J, e.g. N_wake=73/J=24), the documented n_inner_passes already provides enough.
    # These index arrays depend only on J/N_wake (fixed for the whole free_wake call).
    _j0_ar = np.arange(J)[:, None]
    _m_ar  = np.arange(N_wake)[None, :]
    _idx_gather1 = (_j0_ar + _m_ar + 1) % J             # (J, N_wake) -- for building G[j0,m]
    _idx_gather2 = (np.arange(J)[:, None] - np.arange(1, N_wake+1)[None, :]) % J   # (J,N_wake)
    _m_broadcast = np.broadcast_to(_m_ar, (J, N_wake))

    def pseudoimplicit_update(V_field):
        """Same Eq. 3-form update as the reference version. The V_field-derived convection term
        (eta2*(2/Omega)*(V_inf + v_avg4)) depends only on V_field, this call's fixed input --
        precomputed once for every J column via np.roll (matching the same j-1-with-wraparound
        semantics the old per-j negative-index access relied on).

        eta1==0 always in this codebase (dpsi==dzeta, hardcoded above) -- in that case the
        (pass, j) recurrence collapses to the closed-form diagonal cumsum above, computed in one
        shot with no Python loop. The nested-loop version is kept as an exact fallback so this
        stays correct if dpsi and dzeta are ever decoupled (eta1!=0), which is never exercised
        currently.
        """
        Omega_5 = Omega.reshape(ctrl_pts, 1, 1, 1, 1)   # broadcasts against (ctrl_pts,B,J,N_wake,3)
        V_inf_5 = V_inf[:, None, None, None, :]         # (ctrl_pts,1,1,1,3)

        V_shift    = np.roll(V_field, shift=1, axis=2)    # V_shift[:,:,j] = V_field[:,:,j-1]
        v_avg4_all = 0.25*(V_shift[:, :, :, :-1, :] + V_shift[:, :, :, 1:, :]
                          + V_field[:, :, :, :-1, :] + V_field[:, :, :, 1:, :])   # (ctrl_pts,B,J,N_wake,3)
        C_conv_all = eta2*(2.0/Omega_5)*(V_inf_5 + v_avg4_all)

        if eta1 == 0.0:
            G      = C_conv_all[:, :, _idx_gather1, _m_broadcast, :]      # (ctrl_pts,B,J,N_wake,3)
            S_arr  = np.cumsum(G, axis=3)
            anchor = r_wake_grid[:, :, :, 0, :]                           # (ctrl_pts,B,J,3)
            anchor_plus_S = anchor[:, :, :, None, :] + S_arr
            r_new_body = anchor_plus_S[:, :, _idx_gather2, _m_broadcast, :]
            r_new = r_wake_grid.copy()
            r_new[:, :, :, 1:, :] = r_new_body
            return r_new

        r_new = r_wake_grid.copy()
        for _pass in range(n_inner_passes):
            for j in range(J):
                base   = r_new[:, :, j-1, :-1, :]                          # (ctrl_pts,B,nwa-1,3)
                same_j = r_new[:, :, j,   :-1, :] - r_new[:, :, j-1, 1:, :]
                r_new[:, :, j, 1:, :] = base + eta1*same_j + C_conv_all[:, :, j, :, :]
        return r_new

    #-----------------------------------
    # 2/3/4 - relax: V_ind1 -> predictor -> V_ind2 -> corrector, repeated to convergence
    #-----------------------------------
    max_iter = wake_inputs.get('free_wake_max_iter', 15)
    tol      = wake_inputs.get('free_wake_tol', 1e-4)
    relax    = wake_inputs.get('free_wake_relax', 1.0)   # see update_free_wake_location.py --
                                                           # relax=1 -> no change (default)
    R        = rotor.tip_radius

    # Control points outside the model's valid advance-ratio range (mu > mu_max) are frozen by
    # evaluate_bound_vortex_circulation.py's own valid_cp logic -- their Gamma_b/CT never update,
    # so their wake geometry has no reason to settle to anything meaningful. Excluded here from
    # the convergence check for the same reason: without this, a handful of garbage control
    # points (observed: mu up to ~157 when an outer solver drives omega toward 0) permanently
    # drag the reported residual down to a plateau/oscillation floor even after every valid
    # control point has actually converged -- "did not converge" when the model is, in the only
    # sense that matters, done. Still computed/updated for all control points (batched together),
    # just not scored.
    mu     = wake_inputs.get('mu', None)
    mu_max = wake_inputs.get('mu_max', None)
    if mu is None or mu_max is None:
        # Caller didn't provide mu/mu_max (e.g. standalone/unit-test harnesses that call
        # free_wake directly, outside the full lifting_line_performance pipeline) -- fall back
        # to scoring every control point, matching this function's original behavior exactly.
        valid_cp = np.ones(ctrl_pts, dtype=bool)
    else:
        valid_cp = mu <= mu_max
        if not np.any(valid_cp):
            valid_cp = np.ones_like(valid_cp)   # degenerate case: nothing valid, fall back to
                                                 # scoring everything rather than an empty reduction

    # try/finally so a mid-loop exception (e.g. a NaN blowing up somewhere) still shuts the
    # thread pool down instead of leaking worker threads -- this function runs many times per
    # mission (once per outer CT iteration), so a leaked pool per bad evaluation adds up.
    try:
        for n in range(max_iter):
            V_ind1 = compute_V_ind(r_wake_grid, r_wake_grid)
            r_pred = pseudoimplicit_update(V_ind1)

            V_ind2 = compute_V_ind(r_pred, r_pred)
            V_avg  = 0.5*(V_ind1 + V_ind2)
            r_wake_grid_new = pseudoimplicit_update(V_avg)
            r_wake_grid_new = relax*r_wake_grid_new + (1.0-relax)*r_wake_grid

            residual = np.sqrt(np.mean((r_wake_grid_new[valid_cp] - r_wake_grid[valid_cp])**2)) / R
            print(f"[free_wake] iteration {n+1}: rms(delta r)/R = {residual:.3e}")

            r_wake_grid = r_wake_grid_new
            if residual < tol:
                print(f"[free_wake] converged after {n+1} iterations")
                break
        else:
            print(f"[free_wake] did not converge after {max_iter} iterations, residual = {residual:.3e}")
    finally:
        _executor.shutdown()

    #-----------------------------------
    # feed back -- blade b's real filament is column j=0 of its own grid. r_wake_grid has
    # been in hub/thrust frame since it was built (see the frame comment near the top) --
    # this is the single conversion back to body frame, matching rotor.blades.wake.nodes_body's
    # established convention.
    #-----------------------------------
    r_filament = r_wake_grid[:, :, 0, :, :]   # (ctrl_pts, B, N_wake+1, 3) -- note B before
                                               # wake-age here, the internal grid convention,
                                               # NOT rotor.blades.wake.nodes_body's own
                                               # (ctrl_pts, N_wake+1, B, 3) (fixed by the
                                               # transpose on the next line).
    r_filament_body = np.einsum('cij,cbnj->cbni', T_t2b_geom, r_filament) + hub_origin
    rotor.blades.wake.nodes_body = np.transpose(r_filament_body, (0, 2, 1, 3))

    return
