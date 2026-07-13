# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Lifting_Line_Theory/Lifting_Line_performance.py
#
# Created:  Jul 2026, H. Hussien
#
# References
# ----------
# [1] J. Katz and A. Plotkin, Low-Speed Aerodynamics, 2nd ed., Cambridge University Press, 2001.
# [2] W. Johnson, Rotorcraft Aeromechanics, Cambridge University Press, 2013.
# [3] A.J. Landgrebe, JAHS Vol. 17 No. 4, 1972.
# [4] J.D. Kocurek and J.L. Tangler, JAHS Vol. 22 No. 1, 1977.


import numpy as np
from RCAIDE.Framework.Core                           import Data, orientation_product, orientation_transpose
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.initialize_lifting_line           import initialize_lifting_line
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.initialize_wake_geometry          import initialize_wake_geometry
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.evaluate_bound_vortex_circulation import evaluate_bound_vortex_circulation
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.compute_lifting_line_loads        import compute_lifting_line_loads

def lifting_line_performance(rotor, conditions, wake_geo_inputs=None):
    """
    Computes rotor performance using the lifting-line method with prescribed tip vortex wake.
    Drop-in replacement for BEMT_Helmholtz_performance with higher-fidelity inter-blade induction.

    Parameters
    ----------
    rotor : RCAIDE Rotor object
        Must have: number_of_blades, tip_radius, hub_radius, radius_distribution,
                   chord_distribution, twist_distribution, sweep_distribution,
                   thickness_to_chord, airfoil_polar_stations, airfoils, origin,
                   body_to_prop_vel, rc
    conditions : RCAIDE Conditions object
        Must have: freestream (density, dynamic_viscosity, speed_of_sound, temperature),
                   frames (body, inertial), energy.converters[rotor.tag]
                   (omega, blade_pitch_command, commanded_thrust_vector_angle, throttle, design_flag)
    wake_geo_inputs : Data, optional
        If None, default validated hover parameters are used (Combination #22).
        Fields: wake_model, vc_correction, dpsi, n_turns, CT, lamb_oseen_rc_0,
                lamb_oseen_alpha, lamb_oseen_delta, lamb_oseen_sigma,
                lamb_oseen_core_growth_delay, r_R_shed

    Returns
    -------
    None
        Populates conditions.energy.converters[rotor.tag] with all standard RCAIDE
        rotor performance outputs (same interface as BEMT_Helmholtz_performance).
    """
    # ------------------------------------------------------------------------------------------------------------------
    #  Unpack rotor and conditions
    # ------------------------------------------------------------------------------------------------------------------
    B        = rotor.number_of_blades
    R        = rotor.tip_radius
    r_hub    = rotor.hub_radius
    r_1d     = rotor.radius_distribution
    Nr       = np.shape(r_1d)[0]
    ctrl_pts = conditions.freestream.density.shape[0]

    omega        = conditions.energy.converters[rotor.tag].omega
    omega        = np.where(omega == 0, 1e-6, omega)
    commanded_TV = conditions.energy.converters[rotor.tag].commanded_thrust_vector_angle
    pitch_c      = conditions.energy.converters[rotor.tag].blade_pitch_command
    theta_0      = float(pitch_c[0, 0])

    # Freestream velocity in thrust frame
    T_body2inertial = conditions.frames.body.transform_to_inertial
    T_inertial2body = orientation_transpose(T_body2inertial)
    Vv              = conditions.frames.inertial.velocity_vector
    V_body          = orientation_product(T_inertial2body, Vv)
    body2thrust, _  = rotor.body_to_prop_vel(commanded_TV)
    T_body2thrust   = orientation_transpose(np.ones_like(T_body2inertial[:]) * body2thrust)
    V_thrust        = orientation_product(T_body2thrust, V_body)

    V  = float(V_thrust[0, 0])
    mu = V / (float(omega[0, 0]) * R)

    # ------------------------------------------------------------------------------------------------------------------
    #  Default wake geometry inputs 
    # ------------------------------------------------------------------------------------------------------------------
    if wake_geo_inputs is None:
        wake_geo_inputs = Data()
        wake_geo_inputs.wake_model                   = 3                 # 1 simple model, 2 landgrebe, 3 landgrebe KT
        wake_geo_inputs.vc_correction                = 1                 # vortex core factor, 1 standard Rankine, 2 Rankine, 3, scully, 4 Vatistas, 5 Oseen
        wake_geo_inputs.dpsi                         = np.radians(7.4)   # filament length [rad]
        wake_geo_inputs.n_turns                      = 5.0               # Number of wake turns
        wake_geo_inputs.CT                           = 0.00654           # initial guess for CT to intialize the wake geometry
        wake_geo_inputs.lamb_oseen_rc_0              = 0.008             # initial core radius for the wake filaments [fraction of R]
        wake_geo_inputs.lamb_oseen_alpha             = 1.25643           # parameters for the core radius growth rate Lamb-Oseen model  
        wake_geo_inputs.lamb_oseen_delta             = 8.243             # ..
        wake_geo_inputs.lamb_oseen_sigma             = 1.0               # ..
        wake_geo_inputs.lamb_oseen_core_growth_delay = np.radians(30.0)  # paramter to delay the growth rate till certain wake age 
        wake_geo_inputs.r_R_shed                     = 1.0               # location as fraction of R to shed the wake filament from               

    # Populate remaining wake_geo_inputs fields from conditions
    wake_geo_inputs.V_thrust      = V_thrust
    wake_geo_inputs.T_body2thrust = T_body2thrust
    wake_geo_inputs.omega         = omega
    wake_geo_inputs.sigma         = np.mean(rotor.chord_distribution) / R / np.pi * B

    # ------------------------------------------------------------------------------------------------------------------
    #  Build velocity arrays -- (ctrl_pts, Nr, B)
    # ------------------------------------------------------------------------------------------------------------------
    omegar = np.outer(omega, r_1d)[:, :, None] * np.ones((ctrl_pts, Nr, B))
    beta   = (rotor.twist_distribution + theta_0)[None, :, None] * np.ones((ctrl_pts, Nr, B))
    Ua     = V * np.ones((ctrl_pts, Nr, B))
    Ut     = omegar.copy()

    # ------------------------------------------------------------------------------------------------------------------
    #  Build wake_inputs
    # ------------------------------------------------------------------------------------------------------------------
    wake_inputs = Data()
    wake_inputs.include_wake        = True
    wake_inputs.velocity_total      = np.sqrt(Ua**2 + Ut**2)
    wake_inputs.velocity_axial      = Ua
    wake_inputs.velocity_tangential = Ut
    wake_inputs.T_body2thrust       = T_body2thrust
    wake_inputs.ctrl_pts            = ctrl_pts
    wake_inputs.Nr                  = Nr
    wake_inputs.twist_distribution  = beta
    wake_inputs.chord_distribution  = rotor.chord_distribution[None, :, None] * np.ones((ctrl_pts, Nr, B))
    wake_inputs.radius_distribution = r_1d[None, :, None] * np.ones((ctrl_pts, Nr, B))
    wake_inputs.speed_of_sound      = conditions.freestream.speed_of_sound    * np.ones((ctrl_pts, Nr, B))
    wake_inputs.dynamic_viscosity   = conditions.freestream.dynamic_viscosity * np.ones((ctrl_pts, Nr, B))
    wake_inputs.tol                 = 1e-4
    wake_inputs.relax_0             = 0.2
    wake_inputs.relax               = wake_inputs.relax_0 / (1 + 50*mu)
    wake_inputs.max_iter_0          = 300
    wake_inputs.max_iter            = int(wake_inputs.max_iter_0 * (1 + 5*mu))
    wake_inputs.CT_iter             = True
    wake_inputs.aerofoil_aero       = 2   # 1 simplified aerofoil aero, detailed panel aerofoil aero

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 1: Blade geometry
    # ------------------------------------------------------------------------------------------------------------------
    initialize_lifting_line(rotor, conditions)

    wake_inputs.nodes_14c = rotor.blades.bound.nodes_body_14c
    wake_inputs.nodes_34c = rotor.blades.bound.nodes_body_34c

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 2: Wake geometry
    # ------------------------------------------------------------------------------------------------------------------
    initialize_wake_geometry(rotor, wake_geo_inputs, conditions)

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 3: Bound vortex circulation iteration
    # ------------------------------------------------------------------------------------------------------------------
    evaluate_bound_vortex_circulation(rotor, wake_inputs, conditions, wake_geo_inputs)

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 4: Loads
    # ------------------------------------------------------------------------------------------------------------------
    compute_lifting_line_loads(rotor, wake_inputs, conditions)

    return