# VnV/Verification/propulsion/ehd_thruster_physics_test.py
#
# Created:  Oct 2026, RCAIDE EHD MVP
#
# Unit, reference-vector and trend tests for the wire-to-NACA 0010 EHD thruster (spec 3.3, 3.3.1, 3.3.2).
# Only the trends marked "Yes" in spec 3.3.2 are asserted; spacing and chord optima are not.

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                               import Data
from RCAIDE.Library.Methods.Powertrain                                   import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Converters.EHD_Electrode_Array    import *
from RCAIDE.Library.Methods.Powertrain.Propulsors.EHD_Thruster           import *
from RCAIDE.Library.Methods.Geometry.Airfoil                             import compute_naca_4series

import numpy as np
import pytest
import warnings

MU = 2.0e-4

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def sig3(x):
    return float('%.3g' % x)

def mit_like_thruster(N=1):
    thruster = RCAIDE.Library.Components.Powertrain.Propulsors.EHD_Thruster()
    array    = RCAIDE.Library.Components.Powertrain.Converters.EHD_Electrode_Array()
    array.number_of_units              = N
    array.unit_spacing                 = 0.10
    array.collector_chord              = 0.10
    array.collector_foam_density       = 30.
    array.collector_foil_areal_density = 0.05
    hvpc                               = RCAIDE.Library.Components.Powertrain.Modulators.High_Voltage_Converter()
    hvpc.rated_power                   = 600.
    thruster.electrode_array           = array
    thruster.high_voltage_converter    = hvpc
    return thruster

def evaluate(thruster, throttle, velocity=0.0, altitude=0.0):
    throttle = np.atleast_1d(np.asarray(throttle, dtype=float))
    V        = velocity * np.ones_like(throttle)
    state    = setup_operating_conditions(thruster, velocity_range=V, altitude=altitude)
    state.conditions.energy.propulsors[thruster.tag].throttle[:,0] = throttle
    thruster.compute_performance(state)
    c = state.conditions.energy
    return c.propulsors[thruster.tag], c.converters[thruster.electrode_array.tag], c.modulators[thruster.high_voltage_converter.tag]

def reference_unit(d, D_w, S, b, V_a, k_T=1.0, k_P=1.0):
    a     = D_w / 2
    E_i   = compute_peek_inception_field(a, 1.0, 1.0)
    V_i   = compute_inception_voltage(E_i, a, d)
    V_hat = V_a / V_i
    j_hat = compute_dimensionless_current(V_hat)[()]
    T, P, I = compute_unit_thrust_and_power(V_a, d, MU, j_hat, S, b, k_T, k_P)
    return Data(E_i=E_i, V_i=V_i, V_hat=V_hat, j_hat=j_hat, T=T, P=P, I=I)

# ----------------------------------------------------------------------------------------------------------------------
#  R21: j_hat values, limits and continuity
# ----------------------------------------------------------------------------------------------------------------------
def test_j_hat_values_and_limits():
    j = compute_dimensionless_current([2.0, 5.0, np.inf])
    assert round(j[0], 4) == 0.8080
    assert round(j[1], 4) == 1.0695
    assert round(j[2], 4) == 1.1250
    assert abs(compute_dimensionless_current(1e12)[()] - 9/8) < 1e-9

def test_j_hat_continuity_at_inception():
    eps = np.array([1e-4, 1e-6, 1e-8])
    j_above = compute_dimensionless_current(1 + eps)
    assert compute_dimensionless_current(1.0)[()] == 0.0
    assert np.all(compute_dimensionless_current(1 - eps) == 0.0)
    assert np.allclose(j_above / (2 * eps), 1.0, rtol=1e-3)    # j_hat ~ 2(V_hat - 1) near inception

def test_zero_below_inception():
    T, P, I = compute_unit_thrust_and_power(np.array([1e3, 5e3]), 0.06, MU,
                                            compute_dimensionless_current([0.3, 0.99]), 0.1, 3.0)
    assert np.all(T == 0) and np.all(P == 0) and np.all(I == 0)
    thruster = mit_like_thruster()
    p, arr, hv = evaluate(thruster, [0.0, -0.5])                # throttle 0 puts V_a at V_i
    assert np.all(arr.unit_thrust == 0) and np.all(arr.unit_power == 0)
    assert np.all(p.electrode_power == 0) and np.all(p.bus_power == 0)
    thruster.electrode_array.inception_voltage_measured = 41.0e3  # above V_max: never reaches inception
    p, arr, hv = evaluate(thruster, [0.5, 1.0])
    assert np.all(arr.unit_thrust == 0) and np.all(p.bus_power == 0)

# ----------------------------------------------------------------------------------------------------------------------
#  R2 identity: T/P = d/(mu·V_a)·(k_T/k_P)
# ----------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("k_T,k_P", [(1.0, 1.0), (0.7, 1.3)])
def test_thrust_to_power_identity(k_T, k_P):
    d   = 0.06
    V_a = np.linspace(8e3, 40e3, 9)
    j   = compute_dimensionless_current(V_a / 7.7e3)
    T, P, I = compute_unit_thrust_and_power(V_a, d, MU, j, 0.1, 3.0, k_T, k_P)
    assert np.allclose(T / P, d / (MU * V_a) * (k_T / k_P), rtol=4 * np.finfo(float).eps, atol=0)

# ----------------------------------------------------------------------------------------------------------------------
#  Reference test vectors (spec 3.3), 3 significant figures
# ----------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("inputs,expected", [
    (dict(d=0.060, D_w=0.2e-3, S=0.10,  b=3.0,  V_a=40.3e3), dict(E_i=120.3, V_i=7.70, V_hat=5.24, j_hat=1.0742, T=1.287,  P=172.9, I=4.29,  TP=7.44)),
    (dict(d=0.020, D_w=30e-6,  S=0.035, b=0.12, V_a=20.0e3), dict(E_i=263.2, V_i=2.84, V_hat=7.04, j_hat=1.0964, T=0.0408, P=8.15,  I=0.408, TP=5.00)),
], ids=['MIT-like', 'Kahol-like'])
def test_reference_vectors(inputs, expected):
    r = reference_unit(**inputs)
    assert sig3(r.E_i / 1e5)       == sig3(expected['E_i'])   # kV/cm
    assert sig3(r.V_i / 1e3)       == sig3(expected['V_i'])   # kV
    assert sig3(r.V_hat)           == sig3(expected['V_hat'])
    assert sig3(r.j_hat)           == sig3(expected['j_hat'])
    assert sig3(r.T)               == sig3(expected['T'])     # N
    assert sig3(r.P)               == sig3(expected['P'])     # W
    assert sig3(r.I * 1e3)         == sig3(expected['I'])     # mA
    assert sig3(r.T / r.P * 1e3)   == sig3(expected['TP'])    # N/kW

# ----------------------------------------------------------------------------------------------------------------------
#  Trends marked "Yes" in spec 3.3.2
# ----------------------------------------------------------------------------------------------------------------------
def test_exact_scaling_span_units_hvpc_and_densities():
    base          = mit_like_thruster(N=2)
    p0, a0, h0    = evaluate(base, 0.8, velocity=5.0)
    m0            = compute_ehd_thruster_mass(base)

    t = mit_like_thruster(N=6)
    p, a, h = evaluate(t, 0.8, velocity=5.0)
    assert np.allclose(p.net_thrust, 3 * p0.net_thrust) and np.allclose(p.bus_power, 3 * p0.bus_power)

    t = mit_like_thruster(N=2); t.electrode_array.span = 1.5
    p, a, h = evaluate(t, 0.8, velocity=5.0)
    assert np.allclose(p.net_thrust, 0.5 * p0.net_thrust) and np.allclose(p.electrode_power, 0.5 * p0.electrode_power)

    t = mit_like_thruster(N=2); t.high_voltage_converter.efficiency = 0.5
    p, a, h = evaluate(t, 0.8, velocity=5.0)
    assert np.allclose(p.bus_power, p0.electrode_power / 0.5)
    assert np.allclose(h.heat, h.inputs.power - h.outputs.power)

    t = mit_like_thruster(N=2); t.high_voltage_converter.specific_power = 2300.
    assert np.isclose(compute_ehd_thruster_mass(t).high_voltage_converter, 0.5 * m0.high_voltage_converter)

    t = mit_like_thruster(N=2)
    t.electrode_array.emitter_density *= 2; t.electrode_array.collector_foam_density *= 2
    t.electrode_array.collector_foil_areal_density *= 2
    m = compute_ehd_thruster_mass(t)
    assert np.isclose(m.wire, 2 * m0.wire) and np.isclose(m.collector, 2 * m0.collector)

def test_thrust_monotonic_with_throttle():
    p, a, h = evaluate(mit_like_thruster(), np.linspace(0, 1, 21))
    assert np.all(np.diff(a.applied_voltage[:,0]) > 0)
    assert np.all(np.diff(p.net_thrust[1:,0]) > 0)
    assert np.isclose(a.applied_voltage[-1,0], 40.3e3)

def test_gap_thrust_to_power_proportional_to_gap():
    V_a, S, b = 30e3, 0.1, 3.0
    ratios = []
    for d in (0.03, 0.06, 0.12):
        j = compute_dimensionless_current(V_a / 5e3)
        T, P, I = compute_unit_thrust_and_power(V_a, d, MU, j, S, b)
        ratios.append(T / P / d)
    assert np.allclose(ratios, ratios[0], rtol=1e-12)

def test_ion_mobility_R1():
    d, V_a = 0.06, 30e3
    for mu in (1.6e-4, 2.0e-4, 2.15e-4):
        j = compute_dimensionless_current(V_a / 7.7e3)
        T, P, I = compute_unit_thrust_and_power(V_a, d, mu, j, 0.1, 3.0)
        assert np.isclose(T, I * d / mu, rtol=1e-12)              # R1: T = I·d/mu
        assert np.isclose(T / P, d / (mu * V_a), rtol=1e-12)      # R2

def test_wire_diameter_direction_of_inception_voltage():
    D_w = np.array([30e-6, 50e-6, 0.1e-3, 0.2e-3])
    V_i = [compute_inception_voltage(compute_peek_inception_field(D / 2), D / 2, 0.06) for D in D_w]
    assert np.all(np.diff(V_i) > 0)                               # thicker wire -> higher V_i

def test_pressure_direction_of_inception_voltage():
    p, a_low, h = evaluate(mit_like_thruster(), 0.5, altitude=0.0)
    p, a_high, h = evaluate(mit_like_thruster(), 0.5, altitude=5000.0)
    assert a_high.relative_air_density[0,0] < a_low.relative_air_density[0,0]
    assert a_high.inception_voltage[0,0] < a_low.inception_voltage[0,0]

def test_flight_speed_affects_drag_only():
    t = mit_like_thruster(); t.electrode_array.collector_drag_coefficient = 0.02
    p, a, h = evaluate(t, [0.8, 0.8, 0.8], velocity=0.0)
    T_static = a.unit_thrust.copy()
    for V in (2.0, 4.0):
        p, a, h = evaluate(t, [0.8], velocity=V)
        assert np.allclose(a.unit_thrust, T_static[0])           # electrical model ignores V_inf
        if V == 2.0:
            D2 = a.unit_collector_drag[0,0] + a.unit_wire_drag[0,0]
    assert np.isclose(a.unit_collector_drag[0,0] + a.unit_wire_drag[0,0], 4 * D2)   # drag ~ V^2 at fixed c_d
    t = mit_like_thruster()                                        # R24 default: drag rises with speed
    D = [evaluate(t, [0.8], velocity=V)[1].unit_collector_drag[0,0] for V in (1.0, 3.0, 6.0)]
    assert np.all(np.diff(D) > 0)

# ----------------------------------------------------------------------------------------------------------------------
#  Parameter-role enforcement (spec 3.3.1)
# ----------------------------------------------------------------------------------------------------------------------
def test_group4_fixed_values_raise():
    a = RCAIDE.Library.Components.Powertrain.Converters.EHD_Electrode_Array()
    with pytest.raises(ValueError): a.collector_airfoil = 'NACA 0012'
    with pytest.raises(ValueError): a.polarity = 'negative'
    with pytest.raises(ValueError): a.emitters_per_collector = 2
    a.polarity = 'positive'                                       # re-setting the fixed value is allowed
    dict.__setitem__(a, 'polarity', 'negative')                   # bypass set-time check -> caught at run time
    with pytest.raises(ValueError): check_ehd_electrode_array_inputs(a)

def test_calibration_provenance_warnings():
    t = mit_like_thruster(); a = t.electrode_array
    with warnings.catch_warnings():
        warnings.simplefilter('error')
        check_ehd_electrode_array_inputs(a)                       # defaults: no warning
    a.k_T = 1.2
    with pytest.warns(UserWarning, match='provenance'):
        check_ehd_electrode_array_inputs(a)
    a.calibration_geometry = dict(gap=0.06, emitter_diameter=0.2e-3, collector_chord=0.10, unit_spacing=0.10)
    with warnings.catch_warnings():
        warnings.simplefilter('error')
        check_ehd_electrode_array_inputs(a)                       # within tolerance: no warning
    a.gap = 0.075
    with pytest.warns(UserWarning, match='extrapolated'):
        check_ehd_electrode_array_inputs(a)

def test_gap_range_warning():
    a = mit_like_thruster().electrode_array
    a.gap = 0.005
    with pytest.warns(UserWarning, match='10-300 mm'):
        check_ehd_electrode_array_inputs(a)

def test_spacing_correction_hook_disabled():
    a = mit_like_thruster().electrode_array
    assert a.spacing_correction is None
    a.spacing_correction = dict(k1=1, k2=1, k3=1, k4=1, k5=1, k6=1)
    with pytest.raises(NotImplementedError):
        check_ehd_electrode_array_inputs(a)
    a.spacing_correction = dict(k1=1)
    with pytest.raises(ValueError):
        check_ehd_electrode_array_inputs(a)

def test_spacing_and_chord_not_optimizable():
    a = RCAIDE.Library.Components.Powertrain.Converters.EHD_Electrode_Array()
    assert a.optimizer_permissions['unit_spacing'] == 'no' and a.optimizer_permissions['collector_chord'] == 'no'
    path = 'vehicle_configurations.base.networks.electric.propulsors.ehd.electrode_array.'
    for attribute in ('unit_spacing', 'collector_chord'):
        problem = Data(inputs=[['x', 0.1, 0.01, 1.0, 1.0, 1.0]], aliases=[['x', path + attribute]])
        with pytest.raises(ValueError):
            check_ehd_optimizer_inputs(problem)
    problem = Data(inputs=[['x', 0.1, 0.01, 1.0, 1.0, 1.0]], aliases=[['x', [path + 'emitter_diameter']]])
    with pytest.warns(UserWarning):
        check_ehd_optimizer_inputs(problem)
    problem = Data(inputs=[['x', 0.1, 0.01, 1.0, 1.0, 1.0]], aliases=[['x', path + 'gap']])
    check_ehd_optimizer_inputs(problem)

# ----------------------------------------------------------------------------------------------------------------------
#  Mass, geometry constants and design
# ----------------------------------------------------------------------------------------------------------------------
def test_naca_0010_constants_match_rcaide_geometry():
    g = compute_naca_4series('0010', npoints=2001)
    x = np.array(g.x_coordinates).ravel(); y = np.array(g.y_coordinates).ravel()
    area      = 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))
    perimeter = np.sum(np.hypot(np.diff(x), np.diff(y)))
    assert sig3(area) == 0.0685 and sig3(perimeter) == 2.03

def test_mass_model():
    t = mit_like_thruster(N=4); t.additional_mass = 0.1
    a = t.electrode_array
    m = compute_ehd_thruster_mass(t)
    assert np.isclose(m.wire, 4 * 8000 * np.pi * (0.1e-3)**2 * 3.0)
    assert np.isclose(m.collector, 4 * 3.0 * (30 * 0.0685 * 0.1**2 + 0.05 * 2.029 * 0.1))
    assert np.isclose(m.high_voltage_converter, 600 / 1150)
    assert np.isclose(t.mass_properties.mass, m.wire + m.collector + m.high_voltage_converter + 0.1)

def test_design_sizes_number_of_units():
    t = mit_like_thruster(N=None); t.electrode_array.collector_drag_coefficient = 0.02
    design = design_ehd_thruster(t, design_thrust=10.0, design_velocity=5.0, design_altitude=0.0)
    assert design.net_thrust >= 10.0
    assert (design.number_of_units - 1) * design.unit_net_thrust < 10.0
    t2 = mit_like_thruster(N=4); t2.electrode_array.collector_drag_coefficient = 0.02
    design = design_ehd_thruster(t2, design_thrust=10.0, design_velocity=5.0, design_altitude=0.0, sizing_variable='span')
    assert np.isclose(design.net_thrust, 10.0)

# ----------------------------------------------------------------------------------------------------------------------
#  main (VnV convention)
# ----------------------------------------------------------------------------------------------------------------------
def main():
    assert pytest.main([__file__, '-q']) == 0

if __name__ == '__main__':
    main()
