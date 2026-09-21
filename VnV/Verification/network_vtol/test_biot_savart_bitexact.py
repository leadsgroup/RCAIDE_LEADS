# test_biot_savart_bitexact.py
#
# Verifies the Numba fast path in biot_savart_velocity_induction.py reproduces the
# pure-numpy path (_biot_savart_velocity_induction_numpy) BIT-FOR-BIT -- not "close to",
# not "within tolerance". Run before trusting the fast path for anything.

import os
import sys
import numpy as np

repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, repo_root)

from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.biot_savart_velocity_induction import (
    _biot_savart_velocity_induction_numpy,
    _biot_savart_kernel_2d,
    _NUMBA_AVAILABLE,
)

assert _NUMBA_AVAILABLE, "numba not installed -- nothing to verify"

rng = np.random.default_rng(0)

def run_case(name, M, N, rc, vc_correction, P=None, A=None, B=None):
    if P is None:
        P = rng.uniform(-5, 5, size=(M, 3))
    if A is None:
        A = rng.uniform(-5, 5, size=(N, 3))
    if B is None:
        B = A + rng.uniform(-1, 1, size=(N, 3))
    N = A.shape[0]

    tol = 1e-6
    expected = _biot_savart_velocity_induction_numpy(P, A, B, rc, vc_correction, tol)

    rc_1d = np.atleast_1d(np.asarray(rc, dtype=float))
    if rc_1d.shape[0] == 1 and N > 1:
        rc_1d = np.full(N, rc_1d[0])
    rc_sq_arr = rc_1d**2
    rc_qd_arr = rc_1d**4
    actual = _biot_savart_kernel_2d(P.astype(float), A.astype(float), B.astype(float),
                                     rc_sq_arr, rc_qd_arr, int(vc_correction), float(tol))

    exact       = np.array_equal(expected, actual, equal_nan=True)
    finite_mask = np.isfinite(expected) & np.isfinite(actual)
    max_abs_err = np.max(np.abs(expected[finite_mask] - actual[finite_mask])) if np.any(finite_mask) else 0.0
    n_nan_exp   = np.sum(np.isnan(expected))
    n_nan_act   = np.sum(np.isnan(actual))
    status = "BIT-EXACT" if exact else f"MISMATCH (max_abs_err={max_abs_err:.3e})"
    print(f"[{name}] vc_correction={vc_correction}  shape P={P.shape} A={A.shape}  "
          f"nan(expected)={n_nan_exp} nan(actual)={n_nan_act}  -> {status}", flush=True)
    return exact


if __name__ == '__main__':
    all_exact = True

    # Random, well-separated filaments/points, every vc_correction mode, a few sizes.
    for vc in (1, 2, 3, 4):
        for (M, N) in [(1, 1), (5, 7), (50, 40), (222, 230)]:
            ok = run_case(f"random", M, N, rc=0.02, vc_correction=vc)
            all_exact = all_exact and ok

    # Scalar rc (not pre-broadcast to length N by the caller) -- exercises the
    # rc_1d.shape[0]==1 broadcast branch in the dispatcher.
    for vc in (1, 2, 3, 4):
        ok = run_case(f"scalar_rc", 30, 25, rc=0.01, vc_correction=vc)
        all_exact = all_exact and ok

    # Per-filament varying rc (the real wake_inputs.rCvf/rCb usage -- not uniform).
    for vc in (1, 2, 3, 4):
        M, N = 40, 35
        rc_varying = rng.uniform(0.005, 0.05, size=N)
        ok = run_case(f"varying_rc", M, N, rc=rc_varying, vc_correction=vc)
        all_exact = all_exact and ok

    # Degenerate cases: P exactly on a filament endpoint, P collinear with the
    # filament, and a zero-length filament (A==B) -- these hit every 0-guard/mask path.
    A_deg = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]])
    B_deg = np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 0.0, 0.0]])  # 3rd filament: zero-length
    P_deg = np.array([
        [0.0, 0.0, 0.0],   # exactly on filament 0's start
        [1.0, 0.0, 0.0],   # exactly on filament 0's end / filament 1's start
        [0.5, 0.0, 0.0],   # collinear with filament 0, between endpoints
        [5.0, 0.0, 0.0],   # collinear with filament 0, beyond both endpoints
        [0.5, 1.0, 0.0],   # generic off-axis point
    ])
    for vc in (1, 2, 3, 4):
        ok = run_case("degenerate", None, None, rc=0.01, vc_correction=vc, P=P_deg, A=A_deg, B=B_deg)
        all_exact = all_exact and ok

    print()
    print("ALL BIT-EXACT" if all_exact else "SOME MISMATCHES -- DO NOT TRUST THE FAST PATH AS-IS")
    sys.exit(0 if all_exact else 1)
