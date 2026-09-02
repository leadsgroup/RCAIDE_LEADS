# Consolidated backup snapshots of climb_1/climb_2's segment definitions from
# vtol_aircraft_test.py's TR_mission_setup(), taken 2026-08-31.
#
# Both fidelities need the SAME segment (speed/altitude range, throttle/thrust_vector_angle/
# blade_pitch_command bounds) but a DIFFERENT pitch_angle treatment:
#   - LL:   pitch_angle inactive, angle_of_attack fixed at 1deg. Confirmed clean for both
#           LL_prescribed and LL_free_wake (validated cheap settings: dpsi=10deg, n_turns=3.0).
#           Under BEMT this structure gives "Inequality constraints incompatible" (climb_2)
#           or a power spike on the first hp-decomposed piece (climb_1) -- the fixed 1deg AoA
#           forces a wing-unloaded trim that's feasible for LL's thrust prediction but not
#           necessarily BEMT's.
#   - BEMT: pitch_angle ACTIVE with a narrow bound instead. Confirmed clean (no spikes, no
#           discontinuities) at climb_1=[3,6]deg (guess 4deg) / climb_2=[2,5]deg (guess
#           3.5deg) -- narrower bounds got specific pieces stuck (climb_1_0/climb_2_0, the
#           lowest-speed piece of each, needed MORE AoA room, not less; wider ranges did too).
#           Under LL this SAME active-pitch_angle structure pins throttle at its 0.2 floor
#           at every bound width tried (wide [-2,15], narrow [3,6]/[2,5], [1,6]/[1,5]) --
#           the classic degenerate-DOF symptom (4 unknowns vs 2 constraints, objective=None).
#           So this structure is BEMT-only.
#
# To restore either: swap the pitch_angle block in the segment definitions below for whichever
# fidelity is currently active in Tiltrotor_EVTOL.py's prop_rotor.fidelity.


# ====================================================================================
#  Climb 1  (90kts -> 170mph)
# ====================================================================================
segment                           = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
segment.tag                       = "climb_1"
segment.analyses.extend(analyses.cruise)
segment.altitude_start            = 500.0 * Units.ft
segment.climb_rate                = 300. * Units['ft/min']
segment.air_speed_start           = 90.  * Units['knots']
segment.air_speed_end             = 170.  * Units['mph']
segment.altitude_end              = 700.0 * Units.ft
segment.true_course               = 90 * Units.degree
segment.state.numerics.mission_solver.type      = 'optimize'
segment.state.numerics.mission_solver.step_size = 1E-2
segment.state.numerics.mission_solver.tolerance = 1E-6
segment.state.numerics.mission_solver.objective = None

segment.flight_dynamics.force_x                       = True
segment.flight_dynamics.force_z                       = True

segment.assigned_control_variables.throttle.active               = True
segment.assigned_control_variables.throttle.assigned_propulsors  = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                      'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
segment.assigned_control_variables.throttle.bounds               = [[0.2, 0.8]]
segment.assigned_control_variables.throttle.initial_guess_values = [[0.3]]

# --- pitch_angle: pick ONE of the two blocks below depending on fidelity ---

# LL version:
segment.angle_of_attack = 1.0 * Units.degrees

# BEMT version (comment the LL line above, uncomment this):
#segment.assigned_control_variables.pitch_angle.active               = True
#segment.assigned_control_variables.pitch_angle.bounds               = [[3.0 * Units.degrees, 6.0 * Units.degrees]]
#segment.assigned_control_variables.pitch_angle.initial_guess_values = [[4.0 * Units.degrees]]

segment.assigned_control_variables.thrust_vector_angle.active                     = True
segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                    'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
segment.assigned_control_variables.thrust_vector_angle.bounds                     = [[15.0 * Units.degrees, 90.0 * Units.degrees]]
segment.assigned_control_variables.thrust_vector_angle.initial_guess_values       = [[70.0 * Units.degrees]]

segment.assigned_control_variables.blade_pitch_command.active                     = True
segment.assigned_control_variables.blade_pitch_command.assigned_rotors            = [['front_port_rotor','front_starboard_rotor','outboard_port_rotor',
                                                                                    'outboard_starboard_rotor','rear_port_rotor','rear_starboard_rotor']]
segment.assigned_control_variables.blade_pitch_command.bounds                     = [[5.0 * Units.degrees, 35.0 * Units.degrees]]
segment.assigned_control_variables.blade_pitch_command.initial_guess_values       = [[26.0 * Units.degrees]]

mission.append_segment(segment)


# ====================================================================================
#  Climb 2  (130mph -> 170mph)
# ====================================================================================
segment                           = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
segment.tag                       = "climb_2"
segment.analyses.extend(analyses.cruise)
segment.altitude_start            = 700.0 * Units.ft
segment.climb_rate                = 300. * Units['ft/min']
segment.air_speed_start           = 130.  * Units['mph']
segment.air_speed_end             = 170.  * Units['mph']
segment.altitude_end              = 1000.0 * Units.ft
segment.true_course               = 100 * Units.degree

segment.state.numerics.mission_solver.type      = 'optimize'
segment.state.numerics.mission_solver.step_size = 1E-2
segment.state.numerics.mission_solver.tolerance = 1E-6
segment.state.numerics.mission_solver.objective = None # 'energy', 'power', None

segment.flight_dynamics.force_x                       = True
segment.flight_dynamics.force_z                       = True

segment.assigned_control_variables.throttle.active               = True
segment.assigned_control_variables.throttle.assigned_propulsors  = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                      'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
segment.assigned_control_variables.throttle.bounds               = [[0.2, 0.8]]
segment.assigned_control_variables.throttle.initial_guess_values = [[0.35]]

# --- pitch_angle: pick ONE of the two blocks below depending on fidelity ---

# LL version:
segment.angle_of_attack = 1.0 * Units.degrees

# BEMT version (comment the LL line above, uncomment this):
#segment.assigned_control_variables.pitch_angle.active               = True
#segment.assigned_control_variables.pitch_angle.bounds               = [[2.0 * Units.degrees, 5.0 * Units.degrees]]
#segment.assigned_control_variables.pitch_angle.initial_guess_values = [[3.5 * Units.degrees]]

segment.assigned_control_variables.thrust_vector_angle.active                     = True
segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                    'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
segment.assigned_control_variables.thrust_vector_angle.bounds                     = [[40.0 * Units.degrees, 90.0 * Units.degrees]]
segment.assigned_control_variables.thrust_vector_angle.initial_guess_values       = [[70.0 * Units.degrees]]

segment.assigned_control_variables.blade_pitch_command.active                     = True
segment.assigned_control_variables.blade_pitch_command.assigned_rotors            = [['front_port_rotor','front_starboard_rotor','outboard_port_rotor',
                                                                                    'outboard_starboard_rotor','rear_port_rotor','rear_starboard_rotor']]
segment.assigned_control_variables.blade_pitch_command.bounds                     = [[5.0 * Units.degrees, 40.0 * Units.degrees]]
segment.assigned_control_variables.blade_pitch_command.initial_guess_values       = [[24.0 * Units.degrees]]

mission.append_segment(segment)
