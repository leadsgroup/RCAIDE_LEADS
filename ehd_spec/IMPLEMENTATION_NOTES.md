# EHD wire-to-NACA 0010 thruster: implementation notes

Spec: *RCAIDE and Electrohydrodynamic UAV Propulsion: Current State, Usable Physics, and an
Implementation Plan*, Revision 2.1 (43 pages, file dated 4 Oct 2026 15:15). Only one revision of
the PDF was present, so no Revision 1 / Revision 2 conflicts had to be resolved.

Repository: local clone of RCAIDE_LEADS, `RCAIDE/VERSION` = 1.5.0, branched from
`feature/ehd-propulsor` (HEAD 5d80bd500) to `feature-ehd_thruster`.

Python used for all runs: `C:\Users\avite\anaconda3\envs\rcaide1.5.0\python.exe` (3.13.12). That
environment has the PyPI RCAIDE 1.5.0 installed (not editable), so every command sets
`PYTHONPATH` to the clone root, which makes `import RCAIDE` resolve to this clone. The environment was
not modified.

---------------------------------------------------------------------------------------------------

## 1. Verified interfaces (read from code)

### 1.1 Layout and registration
* `RCAIDE/Framework` (Analyses, Core, Mission, Networks, Optimization) and `RCAIDE/Library`
  (Attributes, Components, Methods, Mission, Plots). Packages are registered by explicit imports in
  each `__init__.py`, for example `Library/Components/Powertrain/Propulsors/__init__.py` imports one
  class per line and `Library/Methods/Powertrain/Propulsors/__init__.py` imports one sub-package
  per line.
* Tests: `VnV/Verification/**` and `VnV/Validation/**` are scripts with a `main()`. They run through
  the hard-coded `modules` list in `VnV/test_automatic_regression.py`; CI (`.github/workflows/CI.yml`)
  runs `pytest` from the repository root. No pytest config restricts `testpaths`.
* Docstring style: NumPy style (Parameters / Returns / Notes / `**Major Assumptions**` /
  `**Theory**` / `References` / `See Also`), file header `# RCAIDE/Library/...py` + `# Created:`.

### 1.2 `Propulsor` base (`Library/Components/Powertrain/Propulsors/Propulsor.py`)
* Defaults (l.85-93): `tag, active, wing_mounted, nacelle, sealevel_static_thrust, diameter, length,
  height, working_fluid`. Defines only `compute_moments_of_inertia` (l.95-111, cylinder of
  `length`, `diameter/2`).
* Methods every propulsor must provide (called by `Framework/Networks/Network.py`):
  * `append_operating_conditions(segment, energy_conditions, noise_conditions=None)` (Network.py l.408)
  * `append_propulsor_unknowns_and_residuals(segment)` (l.420, l.440)
  * `unpack_propulsor_unknowns(segment)` (l.339, l.346)
  * `pack_propulsor_residuals(segment)` (l.375, l.380)
  * `compute_performance(state, center_of_gravity)` → `(thrust[n,3], moment[n,3], P_mech[n,1],
    P_elec[n,1], stored_results_flag, stored_propulsor_tag)` (l.154-158)
  * `reuse_stored_data(state, network, stored_propulsor_tag, center_of_gravity)` →
    `(thrust, moment, P_mech, P_elec)` (l.161)

### 1.3 Electric rotor end to end (template)
* Component `Propulsors/Electric_Rotor.py`: holds `motor`, `rotor`, `electronic_speed_controller`;
  every interface method is a one-line delegate to `Methods/Powertrain/Propulsors/Electric_Rotor/*`.
  Unknown/residual methods are skipped for `Set_Speed_Set_Altitude_No_Propulsion` segments (l.96-113).
* `append_electric_rotor_conditions.py` l.88-103: creates `energy_conditions.propulsors[tag]` with
  `throttle, commanded_thrust_vector_angle, thrust(3), power, moment(3)`, then calls
  `item.append_operating_conditions(segment, energy_conditions, noise_conditions)` on every
  sub-item that is a `Component`. `Component` itself does **not** define
  `append_operating_conditions`, so every sub-component must.
* `compute_electric_rotor_performance.py` l.106-145: reads `conditions.energy.propulsors[tag].throttle`,
  pushes it through ESC → motor → rotor, stores component results under
  `conditions.energy.modulators[esc.tag]` and `conditions.energy.converters[...]`, moment =
  `cross(origin − CG, thrust)`, returns ESC input power as `P_elec`.
* `reuse_stored_electric_rotor_data` l.147-198: deep-copies the stored component conditions and
  recomputes only the moment.
* The rotor adds one unknown (motor current) and one residual (torque balance).

### 1.4 Network evaluate (`Framework/Networks/Network.py`, `Electric.py` is a 45-line subclass)
* Bus loop l.131-170: for each `propulsor_group` in `bus.assigned_propulsors`, calls
  `propulsor.compute_performance` (or `reuse_stored_data` for identical propulsors). **Dispatch is
  generic**: no type check on propulsors. Sums thrust, moment, `P_mech`, `P_elec`, then
  `bus.power_draw += total_elec_power * power_split_ratio / bus.efficiency` and
  `current_draw = power_draw / bus_voltage` (l.169-170).
* Bus-level converters (l.207-223) **are** type-checked (DC/PMSM motor and generator only). A
  high-voltage converter placed in `bus.assigned_converters` would be ignored, so the HVPC must live
  inside the propulsor and its efficiency must be folded into the returned `P_elec`, exactly as the
  ESC is folded into the rotor's `P_elec`.
* Batteries are then stepped per control point from `bus.power_draw` (l.244-287).
* `conditions.energy.power` = sum of `P_mech` (l.308).
* Unknowns/residuals: `add_unknowns_and_residuals_to_segment` (l.383-485) calls
  `append_operating_conditions` on every propulsor and `append_propulsor_unknowns_and_residuals` on
  the first propulsor of each group.
* Conclusion: **no Framework edits are needed.**

### 1.5 Segment fields
* Freestream (`Framework/Mission/Common/Results.py` l.127-142): `conditions.freestream.velocity,
  pressure, temperature, density, dynamic_viscosity` (kinematic viscosity = dynamic_viscosity / density).
* Throttle: `Library/Mission/Common/Unpack_Unknowns/energy.py` writes
  `conditions.energy.propulsors[tag].throttle` from `segment.throttle` or the solver unknown
  `throttle_i`.
* Component results: `conditions.energy.propulsors[tag]`, `.converters[tag]`, `.modulators[tag]`.

### 1.6 Reusable pieces
* ESC (`Components/Powertrain/Modulators/Electronic_Speed_Controller.py`): only `efficiency` and
  `bus_voltage`; its method maps `V_out = throttle·V_bus` (`compute_esc_performance.py` l.57-68).
  It has no specific power, rated power or input window, and its throttle→voltage law is wrong
  for an HVPC. **Not reused**; a new `High_Voltage_Converter` modulator is created alongside it.
* NACA 4-series geometry exists: `Methods/Geometry/Airfoil/compute_naca_4series.py`. For '0010' it
  gives area 0.06851·c² and perimeter 2.0291·c, matching the spec's 0.0685·c² and 2.029·c. The
  model uses the spec constants; a test cross-checks them against this function.
* Design-point state: `Methods/Powertrain/setup_operating_conditions.py` (used by
  `design_electric_rotor`) builds a one-point state from altitude and velocity. Reused by
  `design_ehd_thruster`.
* Optimizer inputs: Nexus `problem.inputs` tags mapped to data paths by `problem.aliases`
  (`VnV/Verification/optimization/optimization_packages.py` l.161-198).

---------------------------------------------------------------------------------------------------

## 2. File locations (new files only)

| Spec item | File |
|---|---|
| `EHD_Thruster(Propulsor)` | `RCAIDE/Library/Components/Powertrain/Propulsors/EHD_Thruster.py` |
| `EHD_Electrode_Array(Converter)` | `RCAIDE/Library/Components/Powertrain/Converters/EHD_Electrode_Array.py` |
| `High_Voltage_Converter(Component)` | `RCAIDE/Library/Components/Powertrain/Modulators/High_Voltage_Converter.py` |
| R19, R20, δ | `RCAIDE/Library/Methods/Powertrain/Converters/EHD_Electrode_Array/compute_peek_inception.py` |
| R21, R22, per-unit T, P, I | `.../EHD_Electrode_Array/compute_space_charge_performance.py` |
| R24 + wire drag | `.../EHD_Electrode_Array/compute_electrode_drag.py` |
| Wire and collector mass | `.../EHD_Electrode_Array/compute_electrode_array_mass.py` |
| Group 4 errors, range and provenance warnings | `.../EHD_Electrode_Array/check_ehd_electrode_array_inputs.py` |
| Electrode-array conditions | `.../EHD_Electrode_Array/append_ehd_electrode_array_conditions.py` |
| HVPC conditions, performance, mass | `RCAIDE/Library/Methods/Powertrain/Modulators/High_Voltage_Converter/{append_hvpc_conditions,compute_hvpc_performance,compute_hvpc_mass}.py` |
| Steps 1-10, reuse path | `RCAIDE/Library/Methods/Powertrain/Propulsors/EHD_Thruster/compute_ehd_thruster_performance.py` |
| Thruster conditions, no-op unknowns/residuals | `.../Propulsors/EHD_Thruster/append_ehd_thruster_conditions.py` |
| Mass model (total) | `.../Propulsors/EHD_Thruster/compute_ehd_thruster_mass.py` |
| Design / sizing | `.../Propulsors/EHD_Thruster/design_ehd_thruster.py` |
| Non-optimizable check | `.../Propulsors/EHD_Thruster/check_ehd_optimizer_inputs.py` |
| Tests | `VnV/Verification/propulsion/ehd_thruster_physics_test.py`, `VnV/Verification/network_electric/ehd_thruster_mission_test.py` |

`__init__.py` registration lines added (one line each, no other edits) in:
`Components/Powertrain/{Propulsors,Converters,Modulators}/__init__.py` and
`Methods/Powertrain/{Propulsors,Converters,Modulators}/__init__.py`.

Why the array is a `Converter` and the HVPC a modulator: this mirrors the rotor chain exactly (rotor =
converter owned by the propulsor, ESC = modulator owned by the propulsor), so results land in
`conditions.energy.converters[array.tag]` and `conditions.energy.modulators[hvpc.tag]` the same way.

---------------------------------------------------------------------------------------------------

## 3. Spec-to-code mapping

### 3.1 Six parameter roles (spec 3.3.1)

| Group | Attribute (where) | Default | Note |
|---|---|---|---|
| 1 | `gap` (array) | 0.060 m | MIT example |
| 1 | `emitter_diameter` (array) | 0.2e-3 m | MIT example; optimizer: yes, with warning |
| 1 | `span` (array) | 3.0 m | MIT example |
| 1 | `number_of_units` (array) | None (required) | |
| 1 | `unit_spacing` (array) | None (required) | **non-optimizable** |
| 1 | `collector_chord` (array) | None (required) | **non-optimizable** |
| 1 | `maximum_voltage` (array) | 40.3e3 V | MIT example |
| 1 | `sparkover_voltage` (array) | None | non-optimizable; user/bench value |
| 1 | `emitter_density` (array) | 8000 kg/m³ | spec "stainless, typical" |
| 1 | `collector_foam_density`, `collector_foil_areal_density` (array) | None (required for mass) | |
| 1 | `additional_mass` (thruster) | 0.0 kg | spacers and wiring lumps |
| 1 | `efficiency`, `specific_power`, `rated_power`, `input_voltage_window`, `bus_voltage` (HVPC) | 0.85, 1150 W/kg, None, [160, 225] V, None | Shevgaonkar 2025 |
| 2 | `ion_mobility` | 2.0e-4 m²/(V·s) | Vaddi et al. 2020 |
| 2 | `peek_surface_factor` | 1.0 | R19 |
| 2 | `collector_drag_coefficient` | None → R24 | |
| 2 | `emitter_drag_coefficient` | 1.0 | spec "~1 [UNVERIFIED]" |
| 3 | `inception_voltage_measured` | None → R20 | |
| 3 | `k_T`, `k_P`, `k_Vi` | 1, 1, 1 | explicit inputs, never hidden |
| 3 | `calibration_geometry` | None | record of d, D_w, c, S |
| 3 | `calibration_tolerance` | 0.10 | spec's suggested 10% |
| 3 | `spacing_correction` | None (hook, disabled) | Kahol Eq. 31 coefficients; no defaults |
| 4 | `collector_airfoil` = 'NACA 0010', `polarity` = 'positive', `emitters_per_collector` = 1 | fixed | `ValueError` on any other value (set-time and run-time) |
| 5 | freestream p, T, ρ, V∞, μ_air; throttle | read only | from `state.conditions` |
| 6 | E_i, V_i, V_a, V̂, ĵ, I, T_unit, D_c, D_w, T_net, P_EHD, P_bus, heat, T/P, masses | outputs | stored in conditions / `mass_properties`, never attributes |

Renames versus the spec's suggested names: none for the six-role table. Two attributes were added that
the spec describes but does not name: `calibration_tolerance` (the "user-set tolerance") and
`emitters_per_collector` (the Group 4 item). The spec's `append_ehd_conditions` and
`compute_ehd_performance` are named `append_ehd_thruster_conditions` and
`compute_ehd_thruster_performance` to follow RCAIDE's `<verb>_<propulsor>_<noun>` pattern.

### 3.2 Per-control-point calculation (spec 3.3 steps 1-10)
All in `compute_ehd_thruster_performance`: (1) δ = (p/p₀)(T₀/T); (2) R19, R20 or measured V_i;
(3) V_a = V_i + throttle·(V_max − V_i), throttle clipped to [0, 1], sparkover flag; (4) V̂, ĵ (R21);
(5)-(6) per-unit T, P, I; (7) D_c (R24 or user c_d), D_w; (8) T_net = N·(T_unit − D_c − D_w) along
body +x; (9) P_EHD, P_bus = P_EHD/η_HV, heat; (10) storage.

Return values to the network: `thrust` = [T_net, 0, 0]; `P_mech` = P_EHD (power delivered to the
electrodes, the EHD analogue of rotor shaft power, so it shows up in `conditions.energy.power`);
`P_elec` = P_bus (what the bus and battery see).

Throttle is clipped to [0, 1] before the voltage map (spec: throttle ∈ [0, 1]). The solver's unclipped
throttle stays in `propulsors[tag].throttle`; a thrust-limited condition therefore shows up as a
segment that cannot converge rather than as V_a > V_max.

Output arrays are sized from the computed per-point arrays, not from `state.ones_row`, because the
standalone state built by `setup_operating_conditions` does not update `ones_row` when given several
velocities.

Storage layout:
* `conditions.energy.propulsors[thruster.tag]`: throttle, commanded_thrust_vector_angle, thrust,
  moment, power, net_thrust, electrode_power (P_EHD), bus_power (P_bus), thrust_to_power_ratio
  (electrical T_unit/P_unit in N/W; 0 below inception)
* `conditions.energy.converters[array.tag]`: relative_air_density, inception_field,
  inception_voltage, applied_voltage, normalized_voltage, dimensionless_current, unit_current,
  unit_thrust, unit_power, collector_reynolds_number, collector_drag_coefficient,
  unit_collector_drag, unit_wire_drag, sparkover_flag
* `conditions.energy.modulators[hvpc.tag]`: inputs.power, outputs.power, outputs.voltage,
  heat, input_voltage_flag

### 3.3 Mass (spec 3.3, Mass)
m_HVPC = P_rated / specific_power; m_wire = N·ρ_w·π·a²·b; m_collector = N·b·(ρ_f·0.0685c² +
σ_foil·2.029c); total = sum + `additional_mass`.

---------------------------------------------------------------------------------------------------

## 4. Conflicts, gaps and decisions

1. **ε₀ has no numeric value in the spec.** R22, R23 and the per-unit equations use ε₀ symbolically,
   and RCAIDE defines no permittivity constant. Used `scipy.constants.epsilon_0`
   (CODATA, 8.8541878188e-12 F/m). scipy is already a declared RCAIDE dependency (`pyproject.toml`
   l.29), so this is not a new dependency. The spec's reference vectors reproduce to 3 s.f. with it.
   This is a defined physical constant, not a fitted value; flagged here rather than treated as a
   stop condition.
2. **Peek reference state** p₀ = 76 cmHg, T₀ = 25 °C: implemented as 101325 Pa and 298.15 K (exact
   unit conversions of the spec's values). Spec marks this [VERIFY] against Peek 1929.
3. **R24 laminar vs turbulent.** The spec gives both C_f laws and no selection rule. Added the Group 2
   input `collector_boundary_layer` with default 'laminar', on the strength of the spec's own
   statement that slow EHD aircraft operate at Re_c ~10⁴. Form-factor constants are [VERIFY] in
   the spec.
4. **"V̂ near 1" warning.** The spec asks for a warning but gives no threshold. Not implemented
   (would require an invented number). Gap 10–300 mm warning is implemented.
5. **Non-optimizable marking.** RCAIDE has no attribute-level lock for Nexus. Implemented as the
   `EHD_OPTIMIZER_PERMISSIONS` table (defined in `check_ehd_electrode_array_inputs.py` to avoid a
   Components↔Methods circular import, exposed as `EHD_Electrode_Array.optimizer_permissions`) plus
   `check_ehd_optimizer_inputs(problem)`, which raises for aliases ending in `.unit_spacing` /
   `.collector_chord` / `.sparkover_voltage` and warns for `.emitter_diameter`. Nexus does not call it
   automatically; the user must call it after defining `problem.inputs` / `problem.aliases`. Making
   it automatic would need a Framework edit.
6. **Weight buildups do not see EHD mass.** The FLOPS/Raymer weight buildups type-check propulsor
   classes (e.g. `Methods/Mass_Properties/Weight_Buildups/Conventional/General_Aviation/FLOPS/compute_propulsion_system_weight.py`
   l.82-89). Adding EHD there means editing existing files, which is out of scope. The thruster
   stores its mass in `mass_properties.mass`; the mission test sets vehicle mass by hand.
7. **Thrust axis.** Follows the turbofan/turbojet convention (body +x, `Propulsors/Turbofan/compute_thurst.py`
   l.208-212). `orientation_euler_angles` is not applied, same as the jet propulsors.
8. **Freestream current correction (R3/R16)** neglected in the electrical model, as the spec directs;
   V∞ enters only the drag model.
9. **Test registration.** New tests are standalone `*_test.py` files with `test_*` functions (collected by
   root `pytest`) and a `main()`. Adding them to the `modules` list in
   `VnV/test_automatic_regression.py` would modify an existing test file, so it was not done.
10. **Bus voltage vs HVPC window.** The HVPC flags (does not raise) when bus voltage is outside its
    input window, matching the spec's "flag a constraint" wording for V_spark.
11. **`spacing_correction` hook.** Attribute exists (default None, no coefficients). If coefficients
    are supplied, `check_spacing_correction` validates that k1..k6 are present and then raises
    `NotImplementedError`. Reason: the spec's Eq. 31 text is garbled in the PDF (subscripts lost) and
    the fit is valid only at its fitted gap and voltage, so how it should replace the S-proportional
    terms at other voltages is undefined; the spec says to leave the hook disabled.
12. **HVPC rated power** sets mass only. No overload check (P_EHD > P_rated) is made, since the spec
    does not ask for one.
13. **Mission test weights.** A weights analysis is mandatory in RCAIDE missions
    (`Library/Mission/Common/Pre_Process/mass_properties.py` l.106-107). The test attaches
    `Weights.Electric_Drone` with `settings.run_weights_analysis = False`, so the hand-set mass is kept
    (see item 6).
14. **Identical thrusters.** Each thruster needs its own `electrode_array.tag` and
    `high_voltage_converter.tag`, because results are keyed by those tags (same rule as rotor/ESC tags
    in `VnV/Vehicles/Electric_Twin_Otter.py`).
15. **MIT-like aircraft test** (`VnV/Verification/network_electric/ehd_mit_aircraft_test.py`, added
    on request after the MVP). Not a validation case: chord, spacing, unit count/layout, wing and tail
    geometry, and battery are guesses in a `GUESS` block (spec 2.4.1 marks them [UNVERIFIED]); sourced
    values are in a separate `MIT` block. It asserts the spec's 4.3 N/m² frontal thrust-density check
    at 40.3 kV with S = 0.10 m (δ = 1), and steady level flight at 5 m/s with SOC closure. It does not
    assert agreement with the reported 3.2 N.
16. **OpenVSP export.** RCAIDE's `export_vsp_vehicle` skips EHD propulsors. The MIT test's
    `export_vsp_with_electrodes` adds wires (circular sections) and NACA 0010 collectors as
    display-only OpenVSP wings in the `.vsp3` file only, never in the RCAIDE vehicle (drag
    bookkeeping rule). **Upstream bug found:** in
    `Framework/External_Interfaces/OpenVSP/export_vsp_vehicle.py` l.133-136 the `write_vsp_wing` call
    is indented inside `if verbose:`, so `verbose=False` exports no wings. Not fixed here (Framework is
    out of scope); the helper passes `verbose=True`. Worth a separate upstream issue/PR.
17. **Environment for this work.** Conda env `rcaide1.5.0EHDproj` (Python 3.13) with the clone
    installed via `pip install -e .`, plus pytest and the local OpenVSP 3.47 packages from
    `C:\VSP313\python` (editable, `--no-deps`). Matches upstream's contributor setup; no PYTHONPATH
    needed.
