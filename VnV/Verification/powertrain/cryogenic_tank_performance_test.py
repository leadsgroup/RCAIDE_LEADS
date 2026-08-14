# VnV/Verification/powertrain/cryogenic_tank_performance_test.py
#
#
# Created: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Framework.Mission.Common import State, Conditions
from RCAIDE.Library.Methods.Utilities.Chebyshev.chebyshev_data import chebyshev_data
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_rounded_end_cylindrical_tank_volume import compute_rounded_end_cylindrical_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_cylindrical_tank_volume import compute_cryogenic_cylindrical_tank_volume
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.append_cryogenic_tank_conditions import append_cryogenic_tank_conditions
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.append_cryogenic_tank_unknown_and_residual import append_cryogenic_tank_unknown_and_residual
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_tank_performance import compute_cryogenic_tank_performance
from RCAIDE.Library.Mission.Solver.solver import _magnitude_scale

import numpy as np
from scipy.optimize import fsolve
from scipy.integrate import solve_ivp

# ----------------------------------------------------------------------------------------------------------------------
#  Test harness: builds a single Cryogenic_Tank and drives its residual function
#  directly (fsolve on the 6-state boil-off system alone) at a fixed cruise
#  altitude and fixed engine fuel demand -- isolated from the full mission
#  solver, aerodynamic surrogates, and other tank instances, so failures here
#  are unambiguously in the tank physics rather than aircraft-level coupling.
# ----------------------------------------------------------------------------------------------------------------------
def build_tank(fuel, design_inlet_temperature, diameter, length):
    tank                            = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank()
    tank.tag                        = 'cryo_tank'
    tank.fuel                       = fuel
    tank.design_inlet_temperature   = design_inlet_temperature
    tank.design_altitude            = 35000. * Units.ft
    tank.design_heat_flux           = 20.0
    tank.design_total_heat_transfer = 2000.0
    tank.ullage_volume_fraction     = 0.07
    tank.diameters.external         = diameter
    tank.lengths.external           = length
    tank.xz_plane_symmetric         = False
    tank.assigned_distributors      = [['fuel_line']]
    tank.power_split_ratio          = 1.0

    compute_rounded_end_cylindrical_tank_volume(tank)
    compute_cryogenic_cylindrical_tank_volume(tank, Data(cryo_tank=tank))
    tank.fuel.mass_properties.mass = tank.volume_properties.net_volume * tank.fuel.density

    return tank


def build_state(n_nodes, duration, chemical_power_demand, cruise_altitude):
    state       = State()
    state._size = n_nodes

    x, D, I = chebyshev_data(n_nodes, integration=True)
    state.numerics.time.differentiate = D / duration
    state.numerics.time.integrate      = I * duration
    state.numerics.time.control_points = x * duration

    state.conditions.weights                  = Conditions()
    state.conditions.weights.components       = Conditions()
    state.conditions.weights.components.mass  = Conditions()

    state.conditions.energy               = Conditions()
    state.conditions.energy.sources       = Conditions()
    state.conditions.energy.distributors  = Conditions()
    state.conditions.energy.distributors['fuel_line'] = Conditions()
    state.conditions.energy.distributors['fuel_line'].outputs = Conditions()
    state.conditions.energy.distributors['fuel_line'].outputs.power = Conditions()
    state.conditions.energy.distributors['fuel_line'].outputs.power.chemical = chemical_power_demand * state.ones_row(1)

    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(cruise_altitude)
    state.conditions.freestream = Conditions()
    state.conditions.freestream.temperature           = float(np.ravel(atmo_data.temperature)[0])           * state.ones_row(1)
    state.conditions.freestream.kinematic_viscosity    = float(np.ravel(atmo_data.kinematic_viscosity)[0])    * state.ones_row(1)
    state.conditions.freestream.prandtl_number          = float(np.ravel(atmo_data.prandtl_number)[0])          * state.ones_row(1)
    state.conditions.freestream.thermal_conductivity    = float(np.ravel(atmo_data.thermal_conductivity)[0])    * state.ones_row(1)

    return state


def run_case(fuel, design_inlet_temperature, chemical_power_demand, label, n_nodes=8,
             diameter=2.0, length=6.0, duration_hr=2.0, check_truth=False):
    duration = duration_hr * 3600.0

    tank    = build_tank(fuel, design_inlet_temperature, diameter=diameter, length=length)
    state   = build_state(n_nodes, duration, chemical_power_demand, cruise_altitude=35000. * Units.ft)

    segment       = Data()
    segment.state = state

    append_cryogenic_tank_conditions(tank, segment)
    append_cryogenic_tank_unknown_and_residual(tank, segment)

    keys = [tank.tag + suffix for suffix in
            ('_ullage_mass', '_fuel_mass', '_ullage_temperature', '_fuel_temperature', '_ullage_volume', '_fuel_volume')]

    x_flat = np.concatenate([state.unknowns.network[k][:,0] for k in keys])

    def residual_fn(x):
        for i, k in enumerate(keys):
            state.unknowns.network[k][:,0] = x[i*n_nodes:(i+1)*n_nodes]
        compute_cryogenic_tank_performance(tank, state, network=None)
        return np.concatenate([state.residuals.network[k][:,0] for k in keys])

    # Adaptive-step (stiff) ODE warm start: a flat (steady-state) initial guess is
    # degenerate for the Chebyshev differentiation matrix (D @ constant = 0), so its
    # residual at every non-pinned node collapses to a single rate, -dy/dt(y0) -- which
    # lets any uniform state be read as an ODE right-hand side via the same trick. The
    # early transient here is stiff (ullage mass starts tiny, ~0.5 kg, so 1/m_g in the
    # ullage energy balance is large and rate-of-change accelerates fast within the
    # first Chebyshev interval); a fixed-step explicit-Euler march badly overshoots it,
    # so use solve_ivp's implicit stiff integrator (Radau) evaluated at the exact
    # collocation times instead -- standard collocation warm-start practice for a
    # system whose transient shape isn't known a priori.
    t = np.ravel(state.numerics.time.control_points)
    y0 = x_flat[::n_nodes].copy()  # one value per key, at node 0

    def rate_fn(ti, y):
        r_uniform = residual_fn(np.repeat(y, n_nodes))
        return -r_uniform[1::n_nodes]

    sol = solve_ivp(rate_fn, (t[0], t[-1]), y0, method='Radau', t_eval=t, rtol=1e-6, atol=1e-9)
    x0  = sol.y.flatten()

    # Mirror the production mission solver's scaling (RCAIDE.Library.Mission.Solver.solver.converge):
    # unknowns/residuals of very different physical magnitude (masses ~100s kg, volumes ~1-10 m^3,
    # temperatures ~20K) sharing one solver tolerance otherwise leaves the small ones degenerate.
    unknown_scale  = _magnitude_scale(x0)
    residual_scale = _magnitude_scale(residual_fn(x0))

    def scaled_residual_fn(x_scaled):
        return residual_fn(x_scaled * unknown_scale) / residual_scale

    x_scaled_sol, info, ier, msg = fsolve(scaled_residual_fn, x0 / unknown_scale, full_output=True, xtol=1e-10)

    # Re-scale and restart from the first pass's result -- re-centering the
    # scale/warm-start on wherever fsolve actually got to often escapes a
    # shallow stall that the original (cruder) linear warm-start couldn't see.
    for _ in range(3):
        x_sol = x_scaled_sol * unknown_scale
        if np.max(np.abs(residual_fn(x_sol))) < 1e-6:
            break
        unknown_scale  = _magnitude_scale(x_sol)
        residual_scale = _magnitude_scale(residual_fn(x_sol))
        x_scaled_sol, info, ier, msg = fsolve(scaled_residual_fn, x_sol / unknown_scale, full_output=True, xtol=1e-10)

    x_sol = x_scaled_sol * unknown_scale

    residual_norm = np.max(np.abs(residual_fn(x_sol)))
    converged     = (ier == 1) and (residual_norm < 1e-6)

    # Truth value: an independent adaptive IVP integration (not the collocation
    # method under test), evaluated at the same collocation times. This is the same
    # cross-validation methodology the source paper describes --
    # 2026_AST_Cryogenic_BWB_Sensitivity_Analysis, Section V.C (p.12): "This method
    # was validated internally against a conventional IVP solver for a
    # representative single-tank case ... with the two solution methods showing
    # close agreement in both temperature and mass trajectories." The paper does not
    # publish the numeric trajectory itself, so this regenerates an equivalent truth
    # trajectory for this specific test case rather than citing its numbers. Only
    # run for one representative case (check_truth=True) -- each IVP step re-solves
    # the per-node heat-leak brentq roots, making this too slow to run on every case.
    if check_truth:
        y0_truth  = x_flat[::n_nodes].copy()
        sol_truth = solve_ivp(rate_fn, (t[0], t[-1]), y0_truth, method='Radau', t_eval=t, rtol=1e-6, atol=1e-8)
        assert sol_truth.success, f"{label}: IVP truth integration failed ({sol_truth.message})"
        x_truth   = sol_truth.y.flatten()

    residual_fn(x_sol)  # restore state to the collocation solution for the checks below

    print(f"\n----- {label} -----")
    print(f"  fsolve status: {msg.strip()} (ier={ier})")
    print(f"  max |residual|: {residual_norm:.3e}")

    tank_conditions = state.conditions.energy.sources[tank.tag]
    m_g = state.unknowns.network[tank.tag + '_ullage_mass'][:,0]
    m_l = state.unknowns.network[tank.tag + '_fuel_mass'][:,0]
    total_mass_0 = m_g[0] + m_l[0]
    total_mass_f = m_g[-1] + m_l[-1]

    m_dot_l_engine       = chemical_power_demand / fuel.lower_heating_value
    m_dot_total_out      = m_dot_l_engine + tank_conditions.vent_rate[:,0]
    I                    = state.numerics.time.integrate
    expected_mass_loss   = np.dot(I, m_dot_total_out)[-1]
    actual_mass_loss     = total_mass_0 - total_mass_f

    print(f"  Initial total mass (liquid+ullage): {total_mass_0:.4f} kg")
    print(f"  Final total mass (liquid+ullage):   {total_mass_f:.4f} kg")
    print(f"  Expected mass loss (engine + vent): {expected_mass_loss:.4f} kg")
    print(f"  Actual mass loss:                    {actual_mass_loss:.4f} kg")
    print(f"  Boil-off rate range: [{tank_conditions.boil_off_flow_rate[:,0].min():.3e}, {tank_conditions.boil_off_flow_rate[:,0].max():.3e}] kg/s")
    print(f"  Heater power range:  [{tank_conditions.heater_power[:,0].min():.3e}, {tank_conditions.heater_power[:,0].max():.3e}] W")
    print(f"  Pressure range:      [{tank_conditions.pressure[:,0].min():.3e}, {tank_conditions.pressure[:,0].max():.3e}] Pa")

    if check_truth:
        # Per-state agreement against the independent IVP truth trajectory (relative,
        # with a small absolute floor since ullage mass/volume start near zero)
        rel_errors = np.abs(x_sol - x_truth) / (np.abs(x_truth) + 1e-6)
        max_rel_error = rel_errors.max()
        print(f"  Max relative error vs. IVP truth trajectory: {max_rel_error:.3e}")

    assert converged, f"{label}: tank residual system failed to converge (max|R|={residual_norm:.3e})"
    mass_error = abs(actual_mass_loss - expected_mass_loss)
    assert mass_error < 1e-3, f"{label}: mass conservation violated by {mass_error:.3e} kg"
    assert np.all(m_g > 0) and np.all(m_l > 0), f"{label}: non-physical negative mass in solution"
    if check_truth:
        assert max_rel_error < 1e-2, f"{label}: collocation solution disagrees with IVP truth trajectory by {max_rel_error:.3e} (relative)"

    return tank, state


def main():
    lh2 = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    run_case(lh2, design_inlet_temperature=20.0, chemical_power_demand=2.0e5, label="LH2 cryogenic tank")

    lng = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()
    run_case(lng, design_inlet_temperature=112.0, chemical_power_demand=2.0e5, label="LNG cryogenic tank")

    # Larger tank, near-dormancy draw, 1-hour window -- roughly matching the scale
    # of the paper's own validation case (Section V.C: 14.68 m^3 liquid + 0.815 m^3
    # ullage, 20 W total heat ingress, 25K/20K initial ullage/liquid temps, 1 hour,
    # validated internally against a conventional IVP solver).
    run_case(lh2, design_inlet_temperature=20.0, chemical_power_demand=1.0e3,
              label="LH2 large tank, near-dormancy", diameter=2.6, length=4.5, duration_hr=1.0,
              check_truth=True)

    print("\nAll cryogenic tank performance tests passed.")


if __name__ == '__main__':
    main()
