# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Lifting_Line_Theory/Lifting_Line_performance.py
#
# Created:  Jul 2026, H. Hussien
#
# References
# ----------
# [1] Landgrebe, A.J., JAHS Vol. 17 No. 4, 1972.
# [2] Kocurek, J.D. and Tangler, J.L., JAHS Vol. 22 No. 1, 1977.
# [3] W. Johnson, Rotorcraft Aeromechanics, Cambridge University Press, 2013.
# [4] J. G. Leishman, Principles of Helicopters, Cambridge University Press, 2006.
# [5] Beddoes, T.S., "A Wake Model for High Resolution Airloads," 2nd International Conference
#     on Basic Rotorcraft Research, Research Triangle Park, NC, 1985.
# [6] van der Wall, B. G., "The Effect of HHC on the Vortex Convection in the Wake of a
#     Helicopter Rotor," Aerospace Science and Technology, 4 (2000), pp. 321-336.


import numpy as np
from RCAIDE.Framework.Core                           import Data, orientation_product, orientation_transpose
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.initialize_lifting_line           import initialize_lifting_line
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.initialize_wake_geometry          import initialize_wake_geometry
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.evaluate_bound_vortex_circulation import evaluate_bound_vortex_circulation
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.compute_lifting_line_loads        import compute_lifting_line_loads

def lifting_line_performance(rotor, conditions):
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
    wake_inputs : Data, optional
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
    theta_0      = pitch_c[:, 0]   # (ctrl_pts,) -- per-control-point commanded pitch

    # Freestream velocity in thrust frame
    T_body2inertial = conditions.frames.body.transform_to_inertial
    T_inertial2body = orientation_transpose(T_body2inertial)
    Vv              = conditions.frames.inertial.velocity_vector
    V_body          = orientation_product(T_inertial2body, Vv)
    body2thrust, _  = rotor.body_to_prop_vel(commanded_TV)
    T_body2thrust   = orientation_transpose(np.ones_like(T_body2inertial[:]) * body2thrust)
    V_thrust        = orientation_product(T_body2thrust, V_body)

    CW = omega[:, 0] > 0   # (ctrl_pts,) -- per-control-point rotation sense

    mu_tot = np.sqrt(V_thrust[:, 0]**2 + V_thrust[:, 1]**2
                     + V_thrust[:, 2]**2)   / (np.abs(omega[:, 0]) * R)    # (ctrl_pts,) -- per-control-point advance ratio

    mu = np.sqrt(V_thrust[:, 1]**2 + V_thrust[:, 2]**2)   / (np.abs(omega[:, 0]) * R)
    # ------------------------------------------------------------------------------------------------------------------
    #  Default wake geometry inputs if not defined in the input file
    # ------------------------------------------------------------------------------------------------------------------
    if rotor.wake_inputs is None:
        wake_inputs = Data()
        wake_inputs.include_wake                 = True
        wake_inputs.wake_model_hov               = 1                 # 1 simple model, 2 landgrebe, 3 landgrebe KT
        wake_inputs.wake_model_FF                = 5                 # 4 undisorted, 5 Beddoes distorted, 6 Modified Beddoes distorted
        wake_inputs.vc_correction                = 1                 # vortex core factor, 1 standard/Scully, 2 Rankine, 3 Vatistas, 4 Oseen
        wake_inputs.dpsi                         = np.radians(6.8)   # filament length [rad]
        wake_inputs.n_turns                      = 5.0               # Number of wake turns
        wake_inputs.thrust_coeff_initial_guess    = 0.00654           # initial guess for CT to intialize the wake geometry
        wake_inputs.lamb_oseen_rc_0              = 0.05             # initial core radius for the wake filaments [fraction of R]
        wake_inputs.lamb_oseen_alpha             = 1.25643           # parameters for the core radius growth rate Lamb-Oseen model  
        wake_inputs.lamb_oseen_delta             = 120000            # 8.243
        wake_inputs.lamb_oseen_sigma             = 1.0               # ..
        wake_inputs.lamb_oseen_core_growth_delay = np.radians(30.0)  # paramter to delay the growth rate till certain wake age 
        wake_inputs.r_R_shed                     = 1.0               # location as fraction of R to shed the wake filament from               
        wake_inputs.tol                          = 1e-4
        wake_inputs.relax_0                      = 0.2
        wake_inputs.max_iter_Gammab_0            = 1000
        wake_inputs.max_iter_CT_0                = 100
        wake_inputs.CT_iter                      = True
        wake_inputs.aerofoil_aero                = 2   # 1 simplified aerofoil aero, detailed panel aerofoil aero
        wake_inputs.mu_max                       = 1.0 # edgewise advance ratio above which a control point is treated as out of the model's valid range
    else:
        wake_inputs = rotor.wake_inputs

    # Always start CT from the known-good initial guess rather than carrying over the previous
    # call's converged value -- simpler and safer than trying to judge whether a carried-over
    # value is still trustworthy (no risk of a diverged/extreme trial point's CT contaminating
    # the next call, and no shape mismatch across calls with different ctrl_pts).
    wake_inputs.thrust_coeff_initial_guess = 0.00654

    # Populate remaining wake_inputs fields from conditions
    wake_inputs.V_thrust      = V_thrust
    wake_inputs.T_body2thrust = T_body2thrust
    wake_inputs.mu            = mu
    wake_inputs.omega         = omega
    wake_inputs.sigma         = np.mean(rotor.chord_distribution) / R / np.pi * B

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 1: Blade geometry (moved up -- psi is needed below for the Ut in-plane correction)
    # ------------------------------------------------------------------------------------------------------------------
    initialize_lifting_line(rotor, conditions)

    wake_inputs.nodes_14c = rotor.blades.bound.nodes_body_14c
    wake_inputs.nodes_34c = rotor.blades.bound.nodes_body_34c

    psi = rotor.blades.bound.psi   # (Nr, B) blade azimuth at each node

    # ------------------------------------------------------------------------------------------------------------------
    #  Build velocity arrays -- (ctrl_pts, Nr, B)
    # ------------------------------------------------------------------------------------------------------------------
    omegar = np.outer(omega, r_1d)[:, :, None] * np.ones((ctrl_pts, Nr, B))
    beta   = (rotor.twist_distribution[None, :, None] + theta_0[:, None, None]) * np.ones((ctrl_pts, Nr, B))
    Ua     = V_thrust[:, 0, None, None] * np.ones((ctrl_pts, Nr, B))

    # In-plane freestream velocity resolved onto each blade's local tangential direction.
    # Ut is the air's relative tangential velocity in the same sense as omega*r, so a
    # freestream component aligned with the blade's own rotation direction *reduces* Ut --
    # same subtraction logic as Wt = Ut - ut_ind for induced velocity.
    vy = V_thrust[:, 1]   # (ctrl_pts,) thrust-frame y freestream velocity
    vz = V_thrust[:, 2]   # (ctrl_pts,) thrust-frame z freestream velocity
    vy_term = np.where(CW[:, None, None], vy[:, None, None], -vy[:, None, None])
    Ut = np.abs(omegar) + (vy_term           * np.cos(psi)[None, :, :] +
                           vz[:, None, None] * np.sin(psi)[None, :, :])

    # ------------------------------------------------------------------------------------------------------------------
    #  Inlcuding new terms in wake_inputs
    # ------------------------------------------------------------------------------------------------------------------
    wake_inputs.velocity_total      = np.sqrt(Ua**2 + Ut**2)
    wake_inputs.velocity_axial      = Ua
    wake_inputs.velocity_tangential = Ut
    wake_inputs.T_body2thrust       = T_body2thrust
    wake_inputs.ctrl_pts            = ctrl_pts
    wake_inputs.Nr                  = Nr
    wake_inputs.twist_distribution  = beta
    wake_inputs.chord_distribution  = rotor.chord_distribution[None, :, None] * np.ones((ctrl_pts, Nr, B))
    wake_inputs.radius_distribution = r_1d[None, :, None] * np.ones((ctrl_pts, Nr, B))
    wake_inputs.speed_of_sound      = conditions.freestream.speed_of_sound[:, :, None]    * np.ones((ctrl_pts, Nr, B))
    wake_inputs.dynamic_viscosity   = conditions.freestream.dynamic_viscosity[:, :, None] * np.ones((ctrl_pts, Nr, B))
    wake_inputs.kinematic_viscosity = wake_inputs.dynamic_viscosity/conditions.freestream.density[:, :, None]
    wake_inputs.relax               = wake_inputs.relax_0 # / (1 + 50*mu_tot)[:, None, None]   # (ctrl_pts,1,1) -- broadcasts against Gamma_b (ctrl_pts, Nr-1, B)
    wake_inputs.max_iter_Gammab     = wake_inputs.max_iter_Gammab_0 # int(wake_inputs.max_iter_Gammab_0 * (1 + 5*np.max(mu_tot)))   # sized for the worst-case (highest advance ratio) control point
    wake_inputs.max_iter_CT         = wake_inputs.max_iter_CT_0 #int(wake_inputs.max_iter_CT_0    * (1 + 5*np.max(mu_tot)))   # sized for the worst-case (highest advance ratio) control point

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 2: Wake geometry
    # ------------------------------------------------------------------------------------------------------------------
    initialize_wake_geometry(rotor, wake_inputs, conditions)

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 3: Bound vortex circulation iteration
    # ------------------------------------------------------------------------------------------------------------------
    evaluate_bound_vortex_circulation(rotor, wake_inputs, conditions)

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 4: Loads
    # ------------------------------------------------------------------------------------------------------------------
    compute_lifting_line_loads(rotor, wake_inputs, conditions)

    if False: # Debug

        # Importing plotting libs
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        
        # ----------------------------------------------------------------------------------------------------------------------
        #  Plot 1: Blade and wake geometry -- 3D, rotor plane, side view
        # ----------------------------------------------------------------------------------------------------------------------
        colors = plt.cm.tab10(np.linspace(0, 1, B))

        if wake_inputs.include_wake:
            nodes_body = rotor.blades.wake.nodes_body[0]    # (N+1, B, 3)

        nodes_14c_body = rotor.blades.bound.nodes_body_14c[0]    # (Nr, B, 3)  -- add [0]
        nodes_34c_body = rotor.blades.bound.nodes_body_34c[0]    # (Nr, B, 3)  -- already correct

        fig = plt.figure(figsize=(18, 6))
        ax1 = fig.add_subplot(131, projection='3d')
        ax2 = fig.add_subplot(132)
        ax3 = fig.add_subplot(133)

        for b in range(B):
            ax1.plot(nodes_14c_body[:,b,0], nodes_14c_body[:,b,1], nodes_14c_body[:,b,2],
                    '-o', color=colors[b], markersize=2, linewidth=2, label=f'Blade {b}')
            ax1.plot(nodes_34c_body[:,b,0], nodes_34c_body[:,b,1], nodes_34c_body[:,b,2],
                    '-o', color=colors[b], markersize=2, linewidth=2, label=f'Blade {b}')
            if wake_inputs.include_wake:
                ax1.plot(nodes_body[:,b,0], nodes_body[:,b,1], nodes_body[:,b,2],
                    '-', color=colors[b], linewidth=0.8, alpha=0.7)
            
            ax2.plot(nodes_14c_body[:,b,1], nodes_14c_body[:,b,2], '-o', color=colors[b], markersize=2, linewidth=2)
            ax2.plot(nodes_34c_body[:,b,1], nodes_34c_body[:,b,2], '-o', color=colors[b], markersize=2, linewidth=2)
            if wake_inputs.include_wake:
                ax2.plot(nodes_body[:,b,1], nodes_body[:,b,2], '-', color=colors[b], linewidth=0.8, alpha=0.7)
            
            ax3.plot(nodes_14c_body[:,b,0], nodes_14c_body[:,b,2], '-o', color=colors[b], markersize=2, linewidth=2)
            ax3.plot(nodes_34c_body[:,b,0], nodes_34c_body[:,b,2], '-o', color=colors[b], markersize=2, linewidth=2)
            if wake_inputs.include_wake:
                ax3.plot(nodes_body[:,b,0], nodes_body[:,b,2], '-', color=colors[b], linewidth=1.0, label=f'Blade {b}')

        ax1.set_xlabel('x (axial) [m]'); ax1.set_ylabel('y [m]'); ax1.set_zlabel('z [m]')
        ax1.set_title(f'Wake geometry: {B} blades (body frame)'); ax1.legend(fontsize=6)
        ax2.set_xlabel('y [m]'); ax2.set_ylabel('z [m]')
        ax2.set_title('Rotor plane (y-z)'); ax2.set_aspect('equal'); ax2.invert_xaxis(); ax2.grid(True)
        ax3.set_xlabel('x (axial) [m]'); ax3.set_ylabel('z [m]')
        ax3.set_title('Side view (x-z)'); ax3.legend(fontsize=6); ax3.grid(True)

        plt.tight_layout()
        #plt.savefig('plot_wake.png', dpi=120)
        #print("Saved plot_wake.png")
        plt.show()
        '''
        # ----------------------------------------------------------------------------------------------------------------------
        #  Plot 2: Gamma and alpha distributions
        # ----------------------------------------------------------------------------------------------------------------------
        fig, axes = plt.subplots(1, 2, figsize=(14, 5))
        r_mid    = 0.5*(r_1d[:-1] + r_1d[1:])    # (ctrl_pts, Nr-1, B) 
        r_nondim = r_mid / R

        Gamma = rotor.blades.bound.gamma[0]
        alpha = rotor.blades.bound.alpha[0]

        ax = axes[0]
        for b in range(B):
            ax.plot(r_nondim, Gamma[:,b], '-o', color=colors[b], label=f'Blade {b}', markersize=3, linewidth=1.5)
        ax.set_xlabel('r/R [-]'); ax.set_ylabel('Gamma [m²/s]')
        ax.set_title('Bound circulation distribution'); ax.legend(fontsize=8); ax.grid(True)

        ax = axes[1]
        for b in range(B):
            ax.plot(r_nondim, np.degrees(alpha[:,b]), '-o', color=colors[b], label=f'Blade {b}', markersize=3, linewidth=1.5)
        ax.set_xlabel('r/R [-]'); ax.set_ylabel('Alpha [deg]')
        ax.set_title('Angle of attack distribution'); ax.legend(fontsize=8); ax.grid(True)

        plt.tight_layout()
        #plt.savefig('plot_gamma.png', dpi=120)
        #print("Saved plot_gamma.png")
        plt.show()
        '''
    return