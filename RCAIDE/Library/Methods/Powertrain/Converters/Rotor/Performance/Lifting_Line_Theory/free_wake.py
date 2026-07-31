# This code is used to update the wake locations using the free-wake psuedo implicit model introducted by
# Bagai and Leishman, 1995.

# The first step is to model the wake shed from all the blades as a 2-D grid.
# Each node in the grid has its own psi and wake_age, making it uniquely defined by these two variables

# The second step is to compute the velocity induction all the wake nodes.

# The third step is to use the use the psuedo implicit method to update the location of the wake nodes

import numpy as np
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

    # rc/Gamma for the wake- and blade-source contributions are both pass-invariant (fixed for
    # the whole relaxation) -- precompute their concatenation once here too, so compute_V_ind
    # only has to build the (per-call-varying) combined position arrays and can issue ONE
    # biot_savart_velocity_induction call instead of two. Biot-Savart is purely elementwise per
    # (field point, source filament) pair with no cross-coupling between sources, so summing two
    # separate calls' results is mathematically identical to concatenating the sources and
    # calling once -- this is a pure overhead reduction (halves the number of Biot-Savart
    # dispatches per compute_V_ind call), not a numerical approximation.
    rc_all_flat    = np.concatenate([rcvf_flat, rcb_flat], axis=-1)         # (ctrl_pts, B*N_wake+B*Nr_s)
    Gamma_all_flat = np.concatenate([Gamma_w_flat, Gamma_b_flat], axis=-1)

    def compute_V_ind(field_grid, source_grid):
        """Same sourcing rule as update_free_wake_location.py -- sources for a query at column
        j are column j of every blade's grid, matched to that query's own instant. Both ctrl_pts
        and J are batched into a single biot_savart_velocity_induction call, using that
        function's leading-batch-dim support: each (cp, j) batch slice only interacts with its
        own column's sources -- same block-diagonal sourcing as the old per-cp/per-j loops, just
        evaluated as one larger vectorized op. rcvf/rcb genuinely vary per control point (unlike
        Gamma/rc across j, which are constant), so this relies on
        biot_savart_velocity_induction's rc broadcasting supporting rc varying over an OUTER
        batch dim (ctrl_pts) while being shared over an INNER one (J) -- see
        _expand_rc_for_broadcast there.

        NOTE: this was tried once before and reverted -- at ctrl_pts=16, n_turns=5 (K_wake
        ~1.2 GB, several same-shaped temporaries alive at once inside
        biot_savart_velocity_induction) it was memory-bandwidth/allocation-bound rather than
        faster. Re-enabled now that the actual problem size shrank (ctrl_pts=8, n_turns=3 ->
        K_wake ~0.22 GB) -- re-check this math (or fall back to the per-cp-loop version, kept
        in git history) if ctrl_pts/n_turns/dpsi grow again and this gets slow.
        """
        # (ctrl_pts, B, J, N_wake+1, 3) -> (ctrl_pts, J, B*(N_wake+1), 3), same B-major
        # flattening per column as the old per-cp P = field_grid[cp,:,j,:,:].reshape(...) call.
        P_all = field_grid.transpose(0, 2, 1, 3, 4).reshape(ctrl_pts, J, B*(N_wake+1), 3)

        src_all   = source_grid.transpose(0, 2, 1, 3, 4)              # (ctrl_pts, J, B, N_wake+1, 3)
        r_w_start = src_all[:, :, :, :-1, :].reshape(ctrl_pts, J, B*N_wake, 3)
        r_w_end   = src_all[:, :, :,  1:, :].reshape(ctrl_pts, J, B*N_wake, 3)

        # Combined wake+blade source list -- see rc_all_flat/Gamma_all_flat's precomputation
        # above for why this is one call instead of two.
        A_all = np.concatenate([r_w_start, A_blade_all], axis=2)   # (ctrl_pts, J, B*N_wake+B*Nr_s, 3)
        B_all = np.concatenate([r_w_end,   B_blade_all], axis=2)

        K_all  = biot_savart_velocity_induction(
            P_all, A_all, B_all, rc_all_flat, vc_correction)              # (ctrl_pts, J, M, N_wake_tot+N_blade_tot, 3)
        v_flat = np.einsum('cjmnk,cn->cjmk', K_all, Gamma_all_flat)       # (ctrl_pts, J, M, 3)

        v_sum = v_flat.reshape(ctrl_pts, J, B, N_wake+1, 3).transpose(0, 2, 1, 3, 4)
        return np.where(CW[:, None, None, None, None], -v_sum, v_sum)

    # A single lap of the j-loop can't fully resolve the periodic seam (j=0 reading j=J-1's
    # value) when the wake-age range (nwa) exceeds the number of columns (J) -- the "freshness"
    # from column 0 needs multiple laps to reach every k. n_inner_passes = ceil(nwa/J) is the
    # minimum that fully resolves it -- verified via the Appendix B constant-source-term check
    # (exact one-outer-iteration convergence, machine-precision match to the closed form), and
    # via a direct comparison against the pre-fix version on real hover/FF cases: same converged
    # answer (~1e-5 of R), fewer outer iterations needed (hover 29->9, FF 15->10 at tol=1e-6).
    n_inner_passes = int(np.ceil(nwa / J))

    def pseudoimplicit_update(V_field):
        """Same Eq. 3-form update as the reference version, vectorized over b and k -- only j
        stays a Python loop, since column j depends on column j-1 (already fully computed).
        eta1's own-column term is kept (multiplying by 0 here) so this stays correct if dpsi
        and dzeta are ever decoupled. Repeats the j-sweep n_inner_passes times, reusing this
        pass's own output as the next pass's input, so the periodic seam is fully resolved
        within one call instead of leaking across multiple outer iterations.

        The V_field-derived convection term (eta2*(2/Omega)*(V_inf + v_avg4)) depends only on
        V_field, this call's fixed input -- never reassigned inside the loop -- so it is
        identical on every one of the n_inner_passes passes. It's precomputed once for every J
        column via np.roll (matching the same j-1-with-wraparound semantics the old per-j
        negative-index access relied on) instead of being recomputed on each (pass, j) pair.
        Only "base" (and eta1*same_j, when dpsi != dzeta) genuinely depends on the
        still-evolving r_new and must stay inside the sequential loop.
        """
        r_new = r_wake_grid.copy()
        Omega_5 = Omega.reshape(ctrl_pts, 1, 1, 1, 1)   # broadcasts against (ctrl_pts,B,J,N_wake,3)
        V_inf_5 = V_inf[:, None, None, None, :]         # (ctrl_pts,1,1,1,3)

        V_shift    = np.roll(V_field, shift=1, axis=2)    # V_shift[:,:,j] = V_field[:,:,j-1]
        v_avg4_all = 0.25*(V_shift[:, :, :, :-1, :] + V_shift[:, :, :, 1:, :]
                          + V_field[:, :, :, :-1, :] + V_field[:, :, :, 1:, :])   # (ctrl_pts,B,J,N_wake,3)
        C_conv_all = eta2*(2.0/Omega_5)*(V_inf_5 + v_avg4_all)

        for _pass in range(n_inner_passes):
            for j in range(J):
                base = r_new[:, :, j-1, :-1, :]                          # (ctrl_pts,B,nwa-1,3)
                if eta1 != 0.0:
                    same_j = r_new[:, :, j, :-1, :] - r_new[:, :, j-1, 1:, :]
                    r_new[:, :, j, 1:, :] = base + eta1*same_j + C_conv_all[:, :, j, :, :]
                else:
                    r_new[:, :, j, 1:, :] = base + C_conv_all[:, :, j, :, :]
        return r_new

    #-----------------------------------
    # 2/3/4 - relax: V_ind1 -> predictor -> V_ind2 -> corrector, repeated to convergence
    #-----------------------------------
    max_iter = wake_inputs.get('free_wake_max_iter', 15)
    tol      = wake_inputs.get('free_wake_tol', 1e-4)
    relax    = wake_inputs.get('free_wake_relax', 1.0)   # see update_free_wake_location.py --
                                                           # relax=1 -> no change (default)
    R        = rotor.tip_radius

    for n in range(max_iter):
        V_ind1 = compute_V_ind(r_wake_grid, r_wake_grid)
        r_pred = pseudoimplicit_update(V_ind1)

        V_ind2 = compute_V_ind(r_pred, r_pred)
        V_avg  = 0.5*(V_ind1 + V_ind2)
        r_wake_grid_new = pseudoimplicit_update(V_avg)
        r_wake_grid_new = relax*r_wake_grid_new + (1.0-relax)*r_wake_grid

        residual = np.sqrt(np.mean((r_wake_grid_new - r_wake_grid)**2)) / R
        print(f"[free_wake] iteration {n+1}: rms(delta r)/R = {residual:.3e}")

        r_wake_grid = r_wake_grid_new
        if residual < tol:
            print(f"[free_wake] converged after {n+1} iterations")
            break
    else:
        print(f"[free_wake] did not converge after {max_iter} iterations, residual = {residual:.3e}")

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
