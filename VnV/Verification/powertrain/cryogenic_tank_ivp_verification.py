# VnV/Verification/powertrain/cryogenic_tank_ivp_verification.py
#
#
# Created: Aug 2026, M. Clarke

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

import numpy as np
from scipy.integrate import solve_ivp

# ----------------------------------------------------------------------------------------------------------------------
#  Direct-IVP verification harness
#
#  Deliberately bypasses the pseudospectral-collocation + simultaneous-Newton
#  solve used by the real mission solver (and by cryogenic_tank_performance_test.py's
#  fsolve path). Now that the ullage regulation flow is computed explicitly (a finite-
#  gain feedback law, not an implicit 7th unknown), the tank's 6 governing equations
#  are plain explicit ODEs -- a textbook initial value problem, since the full initial
#  state (m_g, m_l, T_g, T_l, V_g, V_l) is known exactly at t=0. This harness solves
#  that IVP directly with scipy's mature adaptive stiff integrator, with zero
#  collocation/Newton machinery involved, to verify the PHYSICS is producing sane
#  trajectories independent of whether the collocation solver converges.
#
#  It reuses compute_cryogenic_tank_performance itself (not a reimplementation) via
#  the same "repeat a single state to all n_nodes, then read off the resulting rate"
#  trick already used for the warm start in cryogenic_tank_performance_test.py: at a
#  flat (constant-in-node) trajectory, the Chebyshev differentiation matrix gives
#  D @ constant = 0, so the residual R = D@y - rate(y) collapses to R = -rate(y) at
#  every non-pinned node. This guarantees the IVP is driven by the exact same physics
#  code the collocation solver uses -- no separate, potentially-drifting reimplementation.
# ----------------------------------------------------------------------------------------------------------------------
def build_tank(fuel, design_inlet_temperature, diameter, length, zero_heat_leak=False):
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

    if zero_heat_leak:
        # Geometry/insulation thickness is already sized (fixed) above; this only
        # affects the runtime heat-leak evaluation inside compute_cryogenic_tank_performance,
        # via the *same* production heat-leak function -- not a separate override.
        tank.insulation.material.thermal_conductivity = 1e-12

    return tank


def build_state(n_nodes, chemical_power_demand, cruise_altitude):
    """A minimal state scaffold -- only what compute_cryogenic_tank_performance reads.
    Time arrays are placeholders sized for the repeat-and-extract trick; the actual
    integration time comes from solve_ivp, not from this D matrix."""
    state       = State()
    state._size = n_nodes

    _, D, I = chebyshev_data(n_nodes, integration=True)
    state.numerics.time.differentiate = D  # unit time base; unused beyond D@const=0
    state.numerics.time.integrate      = I

    state.conditions.weights                 = Conditions()
    state.conditions.weights.components      = Conditions()
    state.conditions.weights.components.mass = Conditions()

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
    state.conditions.freestream.temperature          = float(np.ravel(atmo_data.temperature)[0])          * state.ones_row(1)
    state.conditions.freestream.kinematic_viscosity   = float(np.ravel(atmo_data.kinematic_viscosity)[0])   * state.ones_row(1)
    state.conditions.freestream.prandtl_number         = float(np.ravel(atmo_data.prandtl_number)[0])         * state.ones_row(1)
    state.conditions.freestream.thermal_conductivity   = float(np.ravel(atmo_data.thermal_conductivity)[0])   * state.ones_row(1)

    return state


def run_ivp_case(fuel, design_inlet_temperature, chemical_power_demand, label, n_nodes=8,
                  diameter=2.0, length=6.0, duration_hr=2.0, zero_heat_leak=False):
    duration = duration_hr * 3600.0

    tank  = build_tank(fuel, design_inlet_temperature, diameter, length, zero_heat_leak=zero_heat_leak)
    state = build_state(n_nodes, chemical_power_demand, cruise_altitude=35000. * Units.ft)

    segment       = Data()
    segment.state = state

    append_cryogenic_tank_conditions(tank, segment)
    append_cryogenic_tank_unknown_and_residual(tank, segment)

    keys = [tank.tag + suffix for suffix in
            ('_ullage_mass', '_fuel_mass', '_ullage_temperature', '_fuel_temperature', '_ullage_volume', '_fuel_volume')]
    y0 = np.array([state.unknowns.network[k][0, 0] for k in keys])

    def rate_fn(t, y):
        for k, val in zip(keys, y):
            state.unknowns.network[k][:, 0] = val  # flat: same value at every node
        compute_cryogenic_tank_performance(tank, state, network=None)
        # Non-pinned node (index 1): R = D@const - rate = -rate  =>  rate = -R
        return np.array([-state.residuals.network[k][1, 0] for k in keys])

    sol = solve_ivp(rate_fn, (0.0, duration), y0, method='Radau', rtol=1e-8, atol=1e-10, dense_output=True)

    print(f"\n----- {label} -----")
    print(f"  solve_ivp success: {sol.success}  ({sol.message})")
    print(f"  steps taken: {len(sol.t)}")

    if not sol.success:
        return tank, state, sol

    # Re-evaluate at the trajectory's own time points to report physical outputs
    t_report = np.linspace(0.0, duration, 9)
    y_report = sol.sol(t_report)
    print(f"  {'t [s]':>8}  {'m_g [kg]':>10}  {'m_l [kg]':>10}  {'T_g [K]':>8}  {'T_l [K]':>8}  {'P [bar]':>9}  {'bo [kg/s]':>11}  {'vent [kg/s]':>12}  {'heater [W]':>11}")
    for i, t in enumerate(t_report):
        y = y_report[:, i]
        for k, val in zip(keys, y):
            state.unknowns.network[k][:, 0] = val
        compute_cryogenic_tank_performance(tank, state, network=None)
        tc = state.conditions.energy.sources[tank.tag]
        m_g, m_l, T_g, T_l = y[0], y[1], y[2], y[3]
        print(f"  {t:8.1f}  {m_g:10.4f}  {m_l:10.3f}  {T_g:8.3f}  {T_l:8.3f}  {tc.pressure[1,0]/1e5:9.4f}  "
              f"{tc.boil_off_flow_rate[1,0]:11.3e}  {tc.vent_rate[1,0]:12.3e}  {tc.heater_power[1,0]:11.3e}")

    return tank, state, sol


def main():
    lh2 = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()

    run_ivp_case(lh2, design_inlet_temperature=20.0, chemical_power_demand=2.0e5,
                 label="LH2, zero environment heat leak", zero_heat_leak=True)


if __name__ == '__main__':
    main()
