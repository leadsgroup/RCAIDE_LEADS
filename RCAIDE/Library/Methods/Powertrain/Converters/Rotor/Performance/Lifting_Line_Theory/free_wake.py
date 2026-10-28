# This code is used to update the wake locations using the free-wake psuedo implicit model introducted by
# Bagai and Leishman, 1995.
#
# Multi-rotor: ported from model15's update_free_wake_location.py (the reference
# implementation's own validated multi-rotor extension -- see that file's 2-rotor
# coaxial/tandem/side-by-side result sets), not re-derived here. All rotors passed in are
# relaxed TOGETHER in one joint predictor/corrector loop; every wake node of every relaxed
# rotor sees induction from the bound vortices and wakes of EVERY rotor (self + mutual),
# refreshed every inner relaxation iteration (not frozen/Jacobi between outer CT iterations).
# A rotor with wake_inputs.free_wake=False is not itself relaxed but still acts as a fixed
# source for the others. n_rot=1 degenerates to the plain single-rotor case (M/d transforms
# are identity, the column time-alignment is exact) -- the same algorithm, not a special case.
#
# Two differences from the model15 source this was ported from:
#   1. CW/CCW uses rotor.clockwise_rotation (fixed rotor property), not omega's sign -- the
#      same bug fix already applied everywhere else in this codebase (omega is always a
#      positive magnitude here; model15's own `omega[:, 0] > 0` predates that fix).
#   2. mu/mu_max-based valid_cp masking for the convergence residual (not in model15): control
#      points outside this method's valid advance-ratio range have no reason for their wake to
#      settle to anything meaningful and shouldn't drag down the reported residual -- same
#      rationale as the single-rotor path this replaces.
#
# The first step is to model the wake shed from all the blades as a 2-D grid.
# Each node in the grid has its own psi and wake_age, making it uniquely defined by these two variables

# The second step is to compute the velocity induction all the wake nodes.

# The third step is to use the use the psuedo implicit method to update the location of the wake nodes

import numpy as np
from concurrent.futures                import ThreadPoolExecutor
from RCAIDE.Framework.Core            import Data, orientation_transpose
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory import biot_savart_velocity_induction, initialize_wake_geometry, initialize_lifting_line

def free_wake(rotor, wake_inputs, conditions):
    """Relaxes the wake of every rotor whose wake_inputs.free_wake is True (default True), with
    induction from the bound vortices and wakes of ALL rotors passed in. Rotors with
    free_wake False keep their current (prescribed) wake and act as fixed sources only.

    Parameters
    ----------
    rotor : Data, list, or dict of Data
        Rotor object(s) -- same single-or-list convention as evaluate_bound_vortex_circulation.
    wake_inputs : Data, or list of Data (one per rotor, same order as rotor)
    conditions : Data

    Multi-rotor coupling
    --------------------
    Column j of rotor i's periodic grid is the instant t = j*dpsi_i/|Omega_i|; every other
    rotor s is sampled at the same instant, i.e. its column round(|Omega_s|*t/dpsi_s). Exact
    when dpsi_s/|Omega_s| is the same for all rotors (e.g. same |Omega| and dpsi); otherwise
    the nearest column is used and a warning is printed. Source geometry of rotor s is moved
    into rotor i's hub/thrust frame before Biot-Savart; the CW/CCW sign is carried by the
    source rotor (sigma = -1 CW, +1 CCW), same convention as evaluate_bound_vortex_circulation.
    """
    rotor_list = [rotor] if hasattr(rotor, 'number_of_blades') else (list(rotor.values()) if isinstance(rotor, dict) else list(rotor))
    wake_inputs_list = [wake_inputs] if len(rotor_list) == 1 and not isinstance(wake_inputs, (list, tuple)) else list(wake_inputs)

    #------------------------------------
    # 0/1 - per-rotor pre-computations and grids
    #------------------------------------
    G = [_setup_rotor_grid(r, wi) for r, wi in zip(rotor_list, wake_inputs_list)]
    n_rot    = len(G)
    ctrl_pts = G[0].ctrl_pts
    free_idx = [i for i, g in enumerate(G) if g.relaxed]
    if not free_idx:
        return

    # Source geometry of rotor s expressed in rotor i's hub/thrust frame: p_i = M_is p_s + d_is
    M = [[None]*n_rot for _ in range(n_rot)]
    d = [[None]*n_rot for _ in range(n_rot)]
    for i, gi in enumerate(G):
        for s, gs in enumerate(G):
            if s != i:
                M[i][s] = np.einsum('cij,cjk->cik', gi.T_b2t, gs.T_t2b_geom)                          # (ctrl_pts,3,3)
                d[i][s] = np.einsum('cij,j->ci', gi.T_b2t, (gs.hub_origin - gi.hub_origin).ravel())    # (ctrl_pts,3)

    def to_frame(i, s, grid):
        """grid (..., J_s, n, 3) in rotor s hub frame, leading axis ctrl_pts -> rotor i hub frame."""
        if s == i:
            return grid
        shp = grid.shape
        flat = grid.reshape(ctrl_pts, -1, 3)
        return (np.einsum('cij,cnj->cni', M[i][s], flat) + d[i][s][:, None, :]).reshape(shp)

    # Time-aligned column of rotor s for column j of rotor i
    colmap = [[None]*n_rot for _ in range(n_rot)]
    for i, gi in enumerate(G):
        for s, gs in enumerate(G):
            t_j = np.arange(gi.J) * gi.dpsi / gi.Omega_ref
            k_f = t_j * gs.Omega_ref / gs.dpsi
            k   = np.round(k_f).astype(int) % gs.J
            if s != i and not np.allclose(k_f, np.round(k_f)):
                print(f"[free_wake] warning: {gi.tag} and {gs.tag} azimuth grids are not time-aligned "
                      f"(dpsi/|Omega| differ); using the nearest column of {gs.tag}")
            colmap[i][s] = k

    # Blade sources in every target frame (fixed during the relaxation)
    blade_src = [[(to_frame(i, s, gs.Bgrid_start), to_frame(i, s, gs.Bgrid_end)) for s, gs in enumerate(G)] for i in range(n_rot)]

    # Each (i,cp,j) task below is fully independent and writes to a disjoint V_ind slice, so
    # this is safe to run across threads with no locking. One ThreadPoolExecutor is created
    # here and reused for every compute_V_ind call in this invocation.
    _executor = ThreadPoolExecutor()

    def compute_V_ind(field_grids, source_grids):
        """Induced velocity at the wake nodes of every relaxed rotor i (its own hub frame),
        from the wakes (source_grids) and blades of every rotor, sampled at the same instant
        as column j."""
        V_ind    = {i: np.zeros_like(field_grids[i]) for i in free_idx}
        wake_src = {i: [to_frame(i, s, source_grids[s]) for s in range(n_rot)] for i in free_idx}

        def _compute_one(i, cp, j):
            gi = G[i]
            P  = field_grids[i][cp, :, j, :, :].reshape(gi.B*(gi.N_wake+1), 3)
            v_sum = np.zeros((gi.B*(gi.N_wake+1), 3))
            for s, gs in enumerate(G):
                k = colmap[i][s][j]
                wake_s    = wake_src[i][s][cp, :, k, :, :]
                r_w_start = wake_s[:, :-1, :].reshape(gs.B*gs.N_wake, 3)
                r_w_end   = wake_s[:,  1:, :].reshape(gs.B*gs.N_wake, 3)
                K_wake = biot_savart_velocity_induction(
                    P, r_w_start, r_w_end, gs.rcvf_flat[cp], gs.vc_correction)
                v_wake = np.einsum('mnk,n->mk', K_wake, gs.Gamma_w_flat[cp])

                A_s, B_s = blade_src[i][s]
                A_blade  = A_s[cp, :, k].reshape(gs.B*gs.Nr_s, 3)
                B_blade  = B_s[cp, :, k].reshape(gs.B*gs.Nr_s, 3)
                K_blade = biot_savart_velocity_induction(
                    P, A_blade, B_blade, gs.rcb_flat[cp], gs.vc_correction)
                v_blade = np.einsum('mnk,n->mk', K_blade, gs.Gamma_b_flat[cp])

                v_s = v_blade + v_wake
                v_sum += -v_s if gs.CW[cp] else v_s
            V_ind[i][cp, :, j, :, :] = v_sum.reshape(gi.B, gi.N_wake+1, 3)

        tasks = [(i, cp, j) for i in free_idx for cp in range(ctrl_pts) for j in range(G[i].J)]
        list(_executor.map(lambda t: _compute_one(*t), tasks))
        return V_ind

    #-----------------------------------
    # 2/3/4 - relax: V_ind1 -> predictor -> V_ind2 -> corrector, all rotors together, repeated to convergence
    #-----------------------------------
    max_iter = max(G[i].max_iter for i in free_idx)
    tol_default = max(G[i].tol for i in free_idx)
    grids    = [g.r_wake_grid for g in G]

    converged = False
    try:
        for n in range(max_iter):
            V_ind1 = compute_V_ind(grids, grids)
            r_pred = list(grids)
            for i in free_idx:
                r_pred[i] = _pseudoimplicit_update(G[i], grids[i], V_ind1[i])

            V_ind2 = compute_V_ind(r_pred, r_pred)
            residuals = {}
            new_grids = list(grids)
            for i in free_idx:
                gi    = G[i]
                V_avg = 0.5*(V_ind1[i] + V_ind2[i])
                r_new = _pseudoimplicit_update(gi, grids[i], V_avg)
                r_new = gi.relax*r_new + (1.0-gi.relax)*grids[i]
                residuals[i] = np.sqrt(np.mean((r_new[gi.valid_cp] - grids[i][gi.valid_cp])**2)) / gi.R if np.any(gi.valid_cp) else 0.0
                new_grids[i] = r_new
            grids = new_grids

            residual = max(residuals.values())
            if all(residuals[i] < G[i].tol for i in free_idx):
                converged = True
                break
    finally:
        _executor.shutdown()

    tags = ', '.join(G[i].tag for i in free_idx)
    if converged:
        print(f"[free_wake] {tags}: converged after {n+1} iteration(s), residual = {residual:.3e}")
    else:
        print(f"[free_wake] {tags}: did NOT converge after {max_iter} iteration(s), residual = {residual:.3e}")

    #-----------------------------------
    # feed back -- blade b's real filament is column j=0 of its own grid. r_wake_grid has
    # been in hub/thrust frame since it was built -- this is the single conversion back to
    # body frame, matching rotor.blades.wake.nodes_body's established convention.
    #-----------------------------------
    for i, g in enumerate(G):
        if not g.relaxed:
            continue
        r_filament      = grids[i][:, :, 0, :, :]      # (ctrl_pts, B, N_wake+1, 3) -- note B before
                                                        # wake-age, the internal grid convention, NOT
                                                        # rotor.blades.wake.nodes_body's own
                                                        # (ctrl_pts, N_wake+1, B, 3) (fixed by the
                                                        # transpose below).
        r_filament_body  = np.einsum('cij,cbnj->cbni', g.T_t2b_geom, r_filament) + g.hub_origin
        g.rotor.blades.wake.nodes_body = np.transpose(r_filament_body, (0, 2, 1, 3))

    return


def _setup_rotor_grid(rotor, wake_inputs):
    """Per-rotor pre-computations: current blade/wake geometry, the periodic azimuth grids (in
    this rotor's hub/thrust frame), source strengths and relaxation constants."""
    if 'blades' not in rotor or rotor.blades.bound.get('nodes_body_14c', None) is None:
        raise RuntimeError(f"{rotor.tag}: free_wake() requires initialize_lifting_line() to have "
                            f"already run (bound geometry missing).")
    r_blade_old = rotor.blades.bound.nodes_body_14c

    if rotor.blades.wake.get('nodes_body', None) is None:
        raise RuntimeError(f"{rotor.tag}: free_wake() requires initialize_wake_geometry() to have "
                            f"already run (prescribed wake geometry missing).")
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

    # rotation sense is a fixed rotor property, not a sign on omega (omega is always a positive
    # magnitude here, same convention as initialize_lifting_line.py/initialize_wake_geometry.py)
    omega = wake_inputs.omega
    CW   = np.full(omega[:, 0].shape, rotor.clockwise_rotation, dtype=bool)   # (ctrl_pts,)
    CW_b = CW[:, np.newaxis, np.newaxis]   # (ctrl_pts, 1, 1) -- broadcast helper for the
                                            # (ctrl_pts, J, Nr_s)/(ctrl_pts, J, N_wake+1) grids below
    j_rot    = dpsi * np.arange(J)
    cos_psi, sin_psi = np.cos(j_rot), np.sin(j_rot)

    # The per-j rotation represents "where is this point once the blade has swept forward by
    # j*dpsi" -- a pure (y,z) rotation, valid only in the frame where the rotor's spin axis is
    # the x-axis (hub/thrust frame). Body frame can differ from thrust frame by a nonzero tilt
    # (rotor cant / commanded thrust vector angle), so this whole relaxation (grids,
    # Biot-Savart, V_inf's convection term) runs in hub/thrust frame throughout -- one
    # conversion in here, one conversion back to body out at the final feedback.
    hub_origin = np.array(rotor.origin[0]).reshape(1, 1, 1, 3)
    T_b2t      = wake_inputs.T_body2thrust                       # (ctrl_pts,3,3)
    T_t2b_geom = orientation_transpose(T_b2t)

    r_b_start_hub  = np.einsum('cij,crbj->crbi', T_b2t, r_b_start  - hub_origin)
    r_b_end_hub    = np.einsum('cij,crbj->crbi', T_b2t, r_b_end    - hub_origin)
    r_wake_old_hub = np.einsum('cij,cwbj->cwbi', T_b2t, r_wake_old - hub_origin)

    def rotate(yz0_y, yz0_z):
        y = np.where(CW_b,
            cos_psi[None, :, None]*yz0_y[:, None, :] - sin_psi[None, :, None]*yz0_z[:, None, :],
            cos_psi[None, :, None]*yz0_y[:, None, :] + sin_psi[None, :, None]*yz0_z[:, None, :])
        z = np.where(CW_b,
            sin_psi[None, :, None]*yz0_y[:, None, :] + cos_psi[None, :, None]*yz0_z[:, None, :],
           -sin_psi[None, :, None]*yz0_y[:, None, :] + cos_psi[None, :, None]*yz0_z[:, None, :])
        return y, z

    # -- bound position grid (built in hub/thrust frame) --
    Bgrid_start = np.zeros((ctrl_pts, B, J, Nr_s, 3))
    Bgrid_end   = np.zeros((ctrl_pts, B, J, Nr_s, 3))
    for b in range(B):
        Bgrid_start[:, b, :, :, 0] = r_b_start_hub[:, None, :, b, 0]
        Bgrid_end[:,   b, :, :, 0] = r_b_end_hub[:,   None, :, b, 0]
        Bgrid_start[:, b, :, :, 1], Bgrid_start[:, b, :, :, 2] = rotate(r_b_start_hub[:, :, b, 1], r_b_start_hub[:, :, b, 2])
        Bgrid_end[:,   b, :, :, 1], Bgrid_end[:,   b, :, :, 2] = rotate(r_b_end_hub[:,   :, b, 1], r_b_end_hub[:,   :, b, 2])

    # -- wake position grid (built in hub/thrust frame) --
    r_wake_grid = np.zeros((ctrl_pts, B, J, N_wake+1, 3))
    for b in range(B):
        r_wake_grid[:, b, :, :, 0] = r_wake_old_hub[:, None, :, b, 0]
        r_wake_grid[:, b, :, :, 1], r_wake_grid[:, b, :, :, 2] = rotate(r_wake_old_hub[:, :, b, 1], r_wake_old_hub[:, :, b, 2])

    # -- Gamma / core-radius, blade-major (position-independent, same for every J column) --
    Gamma_w_bk = np.transpose(Gamma_w, (0, 2, 1))
    rcvf_bk    = np.broadcast_to(rcvf[:, None, :], (ctrl_pts, B, N_wake))
    Gamma_b_bk = np.transpose(Gamma_b, (0, 2, 1))
    rcb_bk     = np.transpose(rcb,     (0, 2, 1))

    g = Data()
    g.rotor, g.tag     = rotor, rotor.tag
    g.relaxed          = bool(wake_inputs.get('free_wake', True))
    g.ctrl_pts, g.B, g.N_wake, g.Nr_s, g.J, g.nwa = ctrl_pts, B, N_wake, Nr_s, J, nwa
    g.dpsi, g.dzeta    = dpsi, dzeta
    g.CW               = CW
    g.hub_origin       = hub_origin
    g.T_b2t, g.T_t2b_geom = T_b2t, T_t2b_geom
    g.Bgrid_start, g.Bgrid_end = Bgrid_start, Bgrid_end
    g.r_wake_grid      = r_wake_grid
    g.Gamma_w_flat     = Gamma_w_bk.reshape(ctrl_pts, B*N_wake)          # same for every j
    g.rcvf_flat        = rcvf_bk.reshape(ctrl_pts, B*N_wake)
    g.Gamma_b_flat     = Gamma_b_bk.reshape(ctrl_pts, B*Nr_s)
    g.rcb_flat         = rcb_bk.reshape(ctrl_pts, B*Nr_s)

    #-----------------------------------
    # relaxation constants
    #-----------------------------------
    g.vc_correction = wake_inputs.vc_correction
    g.Omega     = np.abs(omega)                                     # (ctrl_pts,1)
    g.Omega_ref = float(g.Omega[0, 0])                              # for time-aligning rotor grids
    g.V_inf     = wake_inputs.V_thrust     # (ctrl_pts,3) -- already thrust/hub-frame
    g.eta1      = (dpsi - dzeta) / (dpsi + dzeta)   # = 0 exactly here, since dpsi = dzeta
    g.eta2      = (dpsi * dzeta) / (dpsi + dzeta)
    g.max_iter  = wake_inputs.get('free_wake_max_iter', 15)
    g.tol       = wake_inputs.get('free_wake_tol', 1e-4)
    g.relax     = wake_inputs.get('free_wake_relax', 1.0)   # relax=1 -> no change (default)
    g.R         = rotor.tip_radius

    # Control points outside this method's valid advance-ratio range (mu > mu_max) are frozen
    # by evaluate_bound_vortex_circulation.py's own valid_cp logic -- their wake geometry has
    # no reason to settle to anything meaningful. Excluded here from the convergence check for
    # the same reason (still computed/updated for all control points, just not scored).
    mu     = wake_inputs.get('mu', None)
    mu_max = wake_inputs.get('mu_max', None)
    if mu is None or mu_max is None:
        g.valid_cp = np.ones(ctrl_pts, dtype=bool)
    else:
        g.valid_cp = mu <= mu_max
        if not np.any(g.valid_cp):
            g.valid_cp = np.ones_like(g.valid_cp)
    return g


def _pseudoimplicit_update(g, r_wake_grid, V_field):
    """Eq. 3-form update for one rotor. The V_field-derived convection term (eta2*(2/Omega)*(V_inf +
    v_avg4)) depends only on V_field, this call's fixed input -- precomputed once for
    every J column via np.roll (matching the j-1-with-wraparound semantics the per-j
    negative-index access relies on).

    eta1==0 always in this codebase, so the recurrence collapses to the closed-form
    diagonal cumsum: r_new[j,k] = r_new[j-1,k-1] + C[j,k-1] for k=1..N_wake, with r_new[j,0]
    fixed (the wake-root boundary condition). Unrolling it gives r_new[j,k] = anchor[j0] +
    sum_{i=1}^{k} C[(j0+i)%J, i-1], j0=(j-k)%J -- computed with two fancy-index gathers + one
    cumsum instead of a sequential loop. The nested-loop version is kept as an exact fallback
    for eta1!=0, which is never exercised currently."""
    ctrl_pts, J, N_wake = g.ctrl_pts, g.J, g.N_wake
    Omega_5 = g.Omega.reshape(ctrl_pts, 1, 1, 1, 1)   # broadcasts against (ctrl_pts,B,J,N_wake,3)
    V_inf_5 = g.V_inf[:, None, None, None, :]         # (ctrl_pts,1,1,1,3)

    V_shift    = np.roll(V_field, shift=1, axis=2)    # V_shift[:,:,j] = V_field[:,:,j-1]
    v_avg4_all = 0.25*(V_shift[:, :, :, :-1, :] + V_shift[:, :, :, 1:, :]
                      + V_field[:, :, :, :-1, :] + V_field[:, :, :, 1:, :])   # (ctrl_pts,B,J,N_wake,3)
    C_conv_all = g.eta2*(2.0/Omega_5)*(V_inf_5 + v_avg4_all)

    if g.eta1 == 0.0:
        _j0_ar = np.arange(J)[:, None]
        _m_ar  = np.arange(N_wake)[None, :]
        _idx_gather1 = (_j0_ar + _m_ar + 1) % J             # (J, N_wake) -- for building G[j0,m]
        _idx_gather2 = (np.arange(J)[:, None] - np.arange(1, N_wake+1)[None, :]) % J   # (J,N_wake)
        _m_broadcast = np.broadcast_to(_m_ar, (J, N_wake))

        Gather = C_conv_all[:, :, _idx_gather1, _m_broadcast, :]      # (ctrl_pts,B,J,N_wake,3)
        S_arr  = np.cumsum(Gather, axis=3)
        anchor = r_wake_grid[:, :, :, 0, :]                           # (ctrl_pts,B,J,3)
        anchor_plus_S = anchor[:, :, :, None, :] + S_arr
        r_new_body = anchor_plus_S[:, :, _idx_gather2, _m_broadcast, :]
        r_new = r_wake_grid.copy()
        r_new[:, :, :, 1:, :] = r_new_body
        return r_new

    # A single lap of the j-loop can't fully resolve the periodic seam (j=0 reading j=J-1's
    # value) when the wake-age range (nwa) exceeds J -- freshness from column 0 needs multiple
    # laps to reach every k. n_inner_passes = ceil(nwa/J) is sufficient here (N_wake is always
    # an exact multiple of J in this codebase) but not in general.
    n_inner_passes = int(np.ceil(g.nwa / J))
    r_new = r_wake_grid.copy()
    for _pass in range(n_inner_passes):
        for j in range(J):
            base   = r_new[:, :, j-1, :-1, :]                          # (ctrl_pts,B,nwa-1,3)
            same_j = r_new[:, :, j,   :-1, :] - r_new[:, :, j-1, 1:, :]
            r_new[:, :, j, 1:, :] = base + g.eta1*same_j + C_conv_all[:, :, j, :, :]
    return r_new
