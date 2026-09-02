# Reference snapshot of the three LL free_wake wake_inputs configurations tested on the
# Tiltrotor, taken 2026-09-01. All live in Tiltrotor_EVTOL.py's vehicle_setup(), on
# prop_rotor.wake_inputs. Keep this file as the fallback source of truth for these values --
# restore by copying the relevant block's lines back into vehicle_setup() if the live file
# ever drifts or needs rolling back.
#
# CHEAP / validated setup (~2.5hr full mission with free_wake=True; confirmed to give the
# same converged mission result as the REALISTIC setup below -- the "discretization
# independence" finding from 2026-08-31/09-01):
#
#   prop_rotor.fidelity                        = 'Lifting_Line_Theory'
#   prop_rotor.wake_inputs.dpsi                = 10 deg  (np.radians(10.0))
#   prop_rotor.wake_inputs.n_turns             = 3.0
#   prop_rotor.wake_inputs.max_iter_Gammab_0   = 1000
#   prop_rotor.wake_inputs.max_iter_CT_0       = 100
#   prop_rotor.wake_inputs.free_wake           = True
#   prop_rotor.wake_inputs.free_wake_max_iter  = 20
#   prop_rotor.wake_inputs.free_wake_tol       = 1e-4   (unchanged across both setups)
#
# REALISTIC / long setup (~1336 min / 22+hr full mission run; gave the SAME converged
# mission result as the cheap setup above, at ~9x the cost -- kept here as the "last resort"
# fallback in case the cheap setup is ever found insufficient for a future case):
#
#   prop_rotor.fidelity                        = 'Lifting_Line_Theory'
#   prop_rotor.wake_inputs.dpsi                = 15 deg  (np.radians(15.0))
#   prop_rotor.wake_inputs.n_turns             = 5.0
#   prop_rotor.wake_inputs.max_iter_Gammab_0   = 100
#   prop_rotor.wake_inputs.max_iter_CT_0       = 50
#   prop_rotor.wake_inputs.free_wake           = True
#   prop_rotor.wake_inputs.free_wake_max_iter  = 40
#   prop_rotor.wake_inputs.free_wake_tol       = 1e-4   (unchanged across both setups)
#
# HYBRID setup (~606 min full mission run; gave the SAME converged mission result as both
# setups above -- confirmed 2026-09-01. Note: dpsi=15/n_turns=3 alone was earlier misreported
# as failing (descent_2_3, iteration limit) -- that failure was actually caused by
# max_iter_Gammab_0/max_iter_CT_0 being mistakenly set to 50/20 at the time, not this dpsi/
# n_turns pairing. With Gammab_0/CT_0 correctly at 100/50, this combination converges fine.
# Slower than the cheap setup (606 vs ~150 min), so not a faster replacement -- just a third
# independently-confirmed data point for the discretization-independence result):
#
#   prop_rotor.fidelity                        = 'Lifting_Line_Theory'
#   prop_rotor.wake_inputs.dpsi                = 15 deg  (np.radians(15.0))
#   prop_rotor.wake_inputs.n_turns             = 3.0
#   prop_rotor.wake_inputs.max_iter_Gammab_0   = 100
#   prop_rotor.wake_inputs.max_iter_CT_0       = 50
#   prop_rotor.wake_inputs.free_wake           = True
#   prop_rotor.wake_inputs.free_wake_max_iter  = 20
#   prop_rotor.wake_inputs.free_wake_tol       = 1e-4   (unchanged across all setups)
#
# All three setups also require the companion fix in
# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Lifting_Line_Theory/
# lifting_line_performance.py: the max_iter_CT cap lowered from 500 to 50, and the
# warm-start block (thrust_coeff/wake_nodes_body read from conditions.energy.converters)
# re-enabled (not commented out) -- without these, free_wake is prohibitively slow
# regardless of which wake_inputs setup above is used.
