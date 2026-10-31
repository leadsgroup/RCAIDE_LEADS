# VnV/Verification/powertrain/test_compute_cryogenic_tank_heat_leak.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import importlib
import numpy as np
import pytest
from scipy.optimize import brentq

from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_tank_heat_leak import (
    CYLINDER, CUBOID, compute_cryogenic_tank_heat_leak, compute_cryogenic_tank_heat_leak_cuboid,
    heat_balance_residual_cylinder, heat_balance_residual_cuboid, heat_balance_residual, brent_heat_leak)

# the package __init__ shadows the module name with the function, so fetch the module itself for monkeypatching
heat_leak_module = importlib.import_module('RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_tank_heat_leak')

# ----------------------------------------------------------------------------------------------------------------------
#  Inputs: LH2 tank, aluminum wall, foam insulation, sea-level air
# ----------------------------------------------------------------------------------------------------------------------
T_ENV, T_COLD = 288.15, 20.0
COMMON        = (0.05, T_ENV, T_COLD, 167.0, 0.02, 0.025, 1.5e-5, 2.1e-5, 0.71)
CYLINDER_ARGS = COMMON + (1.0, 0.99, 5.0)
CUBOID_ARGS   = COMMON + (4.0, 2.0, 1.5, 0.005)
CASES         = [(CYLINDER, CYLINDER_ARGS, heat_balance_residual_cylinder, compute_cryogenic_tank_heat_leak),
                 (CUBOID,   CUBOID_ARGS,   heat_balance_residual_cuboid,   compute_cryogenic_tank_heat_leak_cuboid)]


def python_version(kernel):
    """Plain-Python body of a numba kernel (traceable by coverage); the function itself without numba."""
    return getattr(kernel, 'py_func', kernel)


def with_temperatures(args, T_env, T_cold):
    return np.array((args[0], T_env, T_cold) + args[3:], dtype=float)


# ----------------------------------------------------------------------------------------------------------------------
#  Residual kernels
# ----------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("geometry, args, residual, solve", CASES)
def test_residual_python_matches_compiled(geometry, args, residual, solve):
    p = np.array(args, dtype=float)
    for Te in [25.0, 150.0, 280.0]:
        expected = residual(Te, p)
        assert python_version(residual)(Te, p) == pytest.approx(expected, rel=1e-12)
        assert python_version(heat_balance_residual)(geometry, Te, p) == pytest.approx(expected, rel=1e-12)


@pytest.mark.parametrize("geometry, args, residual, solve", CASES)
def test_residual_brackets_root(geometry, args, residual, solve):
    p = np.array(args, dtype=float)
    assert python_version(residual)(T_COLD, p)[0] > 0      # surface at cryogen temperature: net heat gain
    assert python_version(residual)(T_ENV,  p)[0] < 0      # surface at ambient: conduction only


# ----------------------------------------------------------------------------------------------------------------------
#  Brent kernel
# ----------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("geometry, args, residual, solve", CASES)
def test_brent_matches_scipy(geometry, args, residual, solve):
    p        = np.array(args, dtype=float)
    expected = brentq(lambda x: residual(x, p)[0], T_COLD, T_ENV, xtol=1e-9)
    for brent in [brent_heat_leak, python_version(brent_heat_leak)]:
        Te, status = brent(geometry, T_COLD, T_ENV, 1e-9, 4 * np.finfo(float).eps, 100, p)
        assert status == 0
        assert Te == pytest.approx(expected, abs=1e-8)


def test_brent_bisection_fallback_matches_scipy(monkeypatch):
    # a triple root forces the interpolation step to be rejected in favor of bisection
    cubic = lambda x: (x - 1.3)**3
    monkeypatch.setattr(heat_leak_module, 'heat_balance_residual', lambda geometry, x, p: (cubic(x), 0.))
    Te, status = python_version(brent_heat_leak)(CYLINDER, 0.0, 4.0, 1e-9, 4 * np.finfo(float).eps, 100, np.zeros(1))
    assert status == 0
    assert Te == brentq(cubic, 0.0, 4.0, xtol=1e-9)


@pytest.mark.parametrize("geometry, args, residual, solve", CASES)
def test_brent_exact_zero_at_bracket_end(geometry, args, residual, solve):
    p = with_temperatures(args, 300.0, 300.0)               # residual is exactly zero at Te = T_env = T_cold
    brent = python_version(brent_heat_leak)
    assert brent(geometry, 300.0, 250.0, 1e-9, 1e-15, 100, p) == (300.0, 0)
    assert brent(geometry, 250.0, 300.0, 1e-9, 1e-15, 100, p) == (300.0, 0)


@pytest.mark.parametrize("geometry, args, residual, solve", CASES)
def test_brent_no_sign_change(geometry, args, residual, solve):
    p = np.array(args, dtype=float)
    assert python_version(brent_heat_leak)(geometry, 200.0, 250.0, 1e-9, 1e-15, 100, p) == (0., 1)


@pytest.mark.parametrize("geometry, args, residual, solve", CASES)
def test_brent_not_converged(geometry, args, residual, solve):
    p = np.array(args, dtype=float)
    assert python_version(brent_heat_leak)(geometry, T_COLD, T_ENV, 1e-9, 1e-15, 1, p)[1] == 2


# ----------------------------------------------------------------------------------------------------------------------
#  Public solve
# ----------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("geometry, args, residual, solve", CASES)
def test_heat_balance_at_solution(geometry, args, residual, solve):
    Te, Q = solve(*args)
    residual_at_Te, Qc = residual(Te, np.array(args, dtype=float))
    assert T_COLD < Te < T_ENV
    assert Q > 0
    assert Q == pytest.approx(Qc, rel=1e-12)
    assert abs(residual_at_Te) < 1e-6 * Q


@pytest.mark.parametrize("geometry, args, residual, solve", CASES)
def test_equal_temperatures_give_zero_heat_leak(geometry, args, residual, solve):
    Te, Q = solve(*with_temperatures(args, 100.0, 100.0))
    assert Te == 100.0
    assert Q == 0.0


@pytest.mark.parametrize("geometry, args, residual, solve", CASES)
def test_non_finite_temperature_raises(geometry, args, residual, solve):
    with pytest.raises(ValueError, match="non-finite bracket"):
        solve(*with_temperatures(args, np.inf, T_COLD))


@pytest.mark.parametrize("geometry, args, residual, solve", CASES)
def test_no_sign_change_falls_back_to_find_root(geometry, args, residual, solve, monkeypatch):
    expected, _ = solve(*args)
    monkeypatch.setattr(heat_leak_module, 'brent_heat_leak', lambda *a: (0., 1))
    Te, _ = solve(*args)
    assert Te == pytest.approx(expected, abs=1e-6)


@pytest.mark.parametrize("geometry, args, residual, solve", CASES)
def test_not_converged_raises(geometry, args, residual, solve, monkeypatch):
    monkeypatch.setattr(heat_leak_module, 'brent_heat_leak', lambda *a: (0., 2))
    with pytest.raises(RuntimeError, match="failed to converge"):
        solve(*args)
