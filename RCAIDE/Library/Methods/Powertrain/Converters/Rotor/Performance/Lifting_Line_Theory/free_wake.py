# This code is used to update the wake locations using the free-wake psuedo implicit model introducted by
# Bagai and Leishman, 1995.

# The first step is to model the wake shed from all the blades as a 2-D grid.
# Each node in the grid has its own psi and wake_age, making it uniquely defined by these two variables

# The second step is to compute the velocity induction all the wake nodes.

# The third step is to use the use the psuedo implicit method to update the location of the wake nodes

import numpy as np
from RCAIDE.Framework.Core            import orientation_product, orientation_transpose
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

    # -- bound position grid --
    Bgrid_start = np.zeros((ctrl_pts, B, J, Nr_s, 3))
    Bgrid_end   = np.zeros((ctrl_pts, B, J, Nr_s, 3))
    for b in range(B):
        xb0 = r_b_start[:, :, b, 0]; xb1 = r_b_end[:, :, b, 0]
        yb0 = r_b_start[:, :, b, 1]; yb1 = r_b_end[:, :, b, 1]
        zb0 = r_b_start[:, :, b, 2]; zb1 = r_b_end[:, :, b, 2]
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

    # -- wake position grid --
    r_wake_grid = np.zeros((ctrl_pts, B, J, N_wake+1, 3))
    for b in range(B):
        x0 = r_wake_old[:, :, b, 0]
        y0 = r_wake_old[:, :, b, 1]
        z0 = r_wake_old[:, :, b, 2]
        r_wake_grid[:, b, :, :, 0] = x0[:, None, :]
        r_wake_grid[:, b, :, :, 1] = np.where(CW_b,
            cos_psi[None, :, None]*y0[:, None, :] - sin_psi[None, :, None]*z0[:, None, :],
            cos_psi[None, :, None]*y0[:, None, :] + sin_psi[None, :, None]*z0[:, None, :])
        r_wake_grid[:, b, :, :, 2] = np.where(CW_b,
            sin_psi[None, :, None]*y0[:, None, :] + cos_psi[None, :, None]*z0[:, None, :],
           -sin_psi[None, :, None]*y0[:, None, :] + cos_psi[None, :, None]*z0[:, None, :])

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
    T_thrust2body = orientation_transpose(wake_inputs.T_body2thrust)
    V_inf = orientation_product(T_thrust2body, wake_inputs.V_thrust)   # (ctrl_pts,3)
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

    def compute_V_ind(field_grid, source_grid):
        """Same sourcing rule as update_free_wake_location.py -- sources for a query at column
        j are column j of every blade's grid, matched to that query's own instant. Only the wake
        side is re-sliced per call, since wake positions are what's actually evolving."""
        V_ind = np.zeros_like(field_grid)
        for cp in range(ctrl_pts):
            for j in range(J):
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
                V_ind[cp, :, j, :, :] = np.where(CW[cp],
                    -(v_blade + v_wake).reshape(B, N_wake+1, 3),
                     (v_blade + v_wake).reshape(B, N_wake+1, 3))
        return V_ind

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
        within one call instead of leaking across multiple outer iterations."""
        r_new = r_wake_grid.copy()
        Omega_b = Omega[:, None, None]           # actually (ctrl_pts,1,1,1) since Omega is already
                                                  # (ctrl_pts,1) -- extra trailing 1 broadcasts
                                                  # harmlessly against the xyz axis below
        V_inf_b = V_inf[:, None, None, :]         # (ctrl_pts,1,1,3)
        for _pass in range(n_inner_passes):
            for j in range(J):
                base   = r_new[:, :, j-1, :-1, :]                          # (ctrl_pts,B,nwa-1,3)
                same_j = r_new[:, :, j,   :-1, :] - r_new[:, :, j-1, 1:, :]  # eta1 term, =0 contribution
                v_avg4 = 0.25*(V_field[:, :, j-1, :-1, :] + V_field[:, :, j-1, 1:, :]
                              + V_field[:, :, j,   :-1, :] + V_field[:, :, j,   1:, :])
                r_new[:, :, j, 1:, :] = (
                    base
                    + eta1*same_j
                    + eta2*(2.0/Omega_b)*(V_inf_b + v_avg4)
                )
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
    # feed back -- blade b's real filament is column j=0 of its own grid
    #-----------------------------------
    r_filament = r_wake_grid[:, :, 0, :, :]
    rotor.blades.wake.nodes_body = np.transpose(r_filament, (0, 2, 1, 3))

    return
