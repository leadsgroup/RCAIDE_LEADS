# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Lifting_Line_Theory/compute_lifting_line_loads.py
# 
# Created:  Jun 2026, H. Hussien 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                              import Data , Units, orientation_product, orientation_transpose  
# package imports
import  numpy as  np 

# ---------------------------------------------------------------------------------------------------------------------- 
#  compute_lifting_line_loads
# ----------------------------------------------------------------------------------------------------------------------  
def compute_lifting_line_loads(rotor, wake_inputs, conditions):
    """
    Analyzes a general rotor given geometry and operating conditions using a
    lifting-line method with bound-vortex Biot-Savart induction.
 
    Parameters
    ----------
    rotor : Data
        Rotor component with the following attributes:
            - number_of_blades : int
                Number of blades on the rotor
            - tip_radius : float
                Tip radius of the rotor [m]
            - hub_radius : float
                Hub radius of the rotor [m]
            - twist_distribution : array_like
                Blade twist distribution [radians]
            - chord_distribution : array_like
                Blade chord distribution [m]
            - sweep_distribution : array_like
                Blade sweep distribution [m]
            - radius_distribution : array_like
                Radial station positions [m]
            - thickness_to_chord : array_like
                Thickness-to-chord ratio at each radial station
            - airfoil_polar_stations : array_like
                Indices of airfoil polars for each radial station
            - airfoils : dict
                Dictionary of airfoil objects
            - number_azimuthal_stations : int
                Number of azimuthal stations for 2D analysis
            - nonuniform_freestream : bool
                Flag for nonuniform freestream velocity
            - use_2d_analysis : bool
                Flag to use 2D (azimuthal) analysis
            - body_to_prop_vel : function
                Function to transform velocity from body to propeller frame
            - orientation_euler_angles : list
                Orientation of the rotor [rad, rad, rad]
    conditions : Data
        Flight conditions with:
            - freestream : Data
                Freestream properties
                    - density : array_like
                        Air density [kg/m³]
                    - dynamic_viscosity : array_like
                        Dynamic viscosity [kg/(m·s)]
                    - speed_of_sound : array_like
                        Speed of sound [m/s]
                    - temperature : array_like
                        Temperature [K]
            - frames : Data
                Reference frames
                    - body : Data
                        Body frame
                        - transform_to_inertial : array_like
                            Rotation matrix from body to inertial frame
                    - inertial : Data
                        Inertial frame
                        - velocity_vector : array_like
                            Velocity vector in inertial frame [m/s]
            - energy : Data
                Energy conditions
                    - converters : dict
                        Converter energy conditions indexed by tag
                        - commanded_thrust_vector_angle : array_like
                            Commanded thrust vector angle [rad]
                        - blade_pitch_command : array_like
                            Blade pitch command [rad]
                        - omega : array_like
                            Angular velocity [rad/s]
                        - throttle : array_like
                            Throttle setting [0-1]
                        - design_flag : bool
                            Flag indicating design condition
    
    Returns
    -------
    None

    Notes
    -----
    This function implements the Blade Element Momentum Theory (BEMT) with a Helmholtz
    Vortex Wake model to analyze rotor performance. It calculates detailed aerodynamic
    properties at each blade element and azimuthal position, accounting for 3D wake effects.
    
    The computation follows these steps:
        1. Extract rotor parameters and operating conditions
        2. Transform velocity from inertial to rotor frame
        3. Set up the blade geometry (radial and azimuthal distributions)
        4. Initialize induced velocities
        5. Include effects of rotor incidence and external velocity fields if specified
        6. Compute wake-induced inflow velocities using the Helmholtz wake model
        7. Calculate aerodynamic forces (lift, drag) at each blade element
        8. Compute circulation, thrust, and torque distributions
        9. Apply tip loss corrections
        10. Calculate integrated performance metrics (thrust, power, efficiency)
        11. Store results in the conditions data structure
    
    **Major Assumptions**
        * The wake is modeled using Helmholtz vortex filaments
        * Blade element theory is used to compute local aerodynamic forces
        * Tip losses are modeled using the Prandtl tip loss function
        * Compressibility effects are accounted for through Mach number corrections
        * Reynolds number effects on airfoil performance are included
    
    **Theory**
    The Helmholtz wake model represents the wake as a system of vortex filaments that
    satisfy Helmholtz's vortex theorems. The induced velocities at each blade element
    are computed by applying the Biot-Savart law to these vortex filaments.
    
    The blade forces are calculated using:
        - Lift: :math:`L = 0.5\\cdot\\rho\\cdot W^2\\cdot c\\cdot Cl`
        - Drag: :math:`D = 0.5\\cdot\\rho\\cdot W^2\\cdot c\\cdot Cd`
        - Circulation: :math:`\\Gamma = 0.5\\cdot W\\cdot c\\cdot Cl`
    
    where:
        - ρ is density
        - W is relative velocity
        - c is chord
        - Cl is lift coefficient
        - Cd is drag coefficient
        - Γ is circulation
    
    The thrust and torque are then computed by resolving these forces perpendicular
    and parallel to the rotor plane, respectively.
    
    References
    ----------
    [1] J. Katz and A. Plotkin, Low-Speed Aerodynamics, 2nd ed.,
        Cambridge University Press, 2001, Section 2.12.
    [2] W. Johnson, Rotorcraft Aeromechanics, Cambridge university press, 2013, Section 9.9.

    [3] I. Chopra and A. Datta, Helicopter Dynamics, Lecture Notes, UMD, Section 4.8.
    
    See Also
    --------
     RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.initialize_lifting_line
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.Biot_Savart_velocity_induction
    """
    # ------------------------------------------------------------------------------------------------------------------
    #  Unpack converged solution
    # ------------------------------------------------------------------------------------------------------------------
    Ua       = wake_inputs.velocity_axial
    Ut       = wake_inputs.velocity_tangential
    ctrl_pts = wake_inputs.ctrl_pts
    Nr       = wake_inputs.Nr
    Nr_s     = Nr - 1
    c_1d     = wake_inputs.chord_distribution
    psi      = rotor.blades.bound.psi   # (Nr, B)
    psi      = 0.5 * (psi[:-1, :] + psi[1:, :])              # (Nr-1, B)
    va         = rotor.blades.bound.va
    vt         = rotor.blades.bound.vt
    Gamma      = rotor.blades.bound.gamma      
    Cl         = rotor.blades.bound.cl         
    Cdval      = rotor.blades.bound.Cdval      
    alpha      = rotor.blades.bound.alpha      
    alpha_disc = rotor.blades.bound.alpha_disc 
    Ma         = rotor.blades.bound.Ma        
    Re         = rotor.blades.bound.Re        
    Re_disc    = rotor.blades.bound.Re_disc     
    Ua         = rotor.blades.bound.Ua   
    Ut         = rotor.blades.bound.Ut   
    W          = rotor.blades.bound.W     
    Wa         = rotor.blades.bound.Wa   
    Wt         = rotor.blades.bound.Wt   
    F          = rotor.blades.bound.F   

    r_1d      = rotor.radius_distribution   # (Nr,) -- use the 1D version from rotor
    c_mid = 0.5*(c_1d[:, :-1, :] + c_1d[:, 1:, :])   # (ctrl_pts, Nr-1, B)
    r_mid     = 0.5*(r_1d[:-1] + r_1d[1:])               # (Nr_s,)

    B         = rotor.number_of_blades
    R         = rotor.tip_radius
    Nr        = len(r_1d)
    ctrl_pts  = Gamma.shape[0]

    commanded_TV          = conditions.energy.converters[rotor.tag].commanded_thrust_vector_angle
    pitch_c               = conditions.energy.converters[rotor.tag].blade_pitch_command
    eta                   = conditions.energy.converters[rotor.tag].throttle 
    omega                 = conditions.energy.converters[rotor.tag].omega  
    design_flag           = conditions.energy.converters[rotor.tag].design_flag
 
    # Unpack freestream conditions
    rho     = conditions.freestream.density[:,0,None]
    T       = conditions.freestream.temperature[:,0,None]
    Vv      = conditions.frames.inertial.velocity_vector
    rho_0   = rho 
    
    T_body2inertial         = conditions.frames.body.transform_to_inertial
    T_inertial2body         = orientation_transpose(T_body2inertial)
    V_body                  = orientation_product(T_inertial2body,Vv)
    body2thrust,orientation = rotor.body_to_prop_vel(commanded_TV) 
    T_body2thrust           = orientation_transpose(np.ones_like(T_body2inertial[:])*body2thrust)
    V_thrust                = orientation_product(T_body2thrust,V_body)

    # Calculating rotational parameters
    omegar = np.outer(omega, r_mid)[:, :, None] * np.ones((ctrl_pts, Nr_s, B))   # (ctrl_pts, Nr, B)
    n        = omega/(2.*np.pi)   # Rotations per second

    # Check and correct for hover
    V         = V_thrust[:,0,None]
    V[V==0.0] = 1E-6 

    # ------------------------------------------------------------------------------------------------------------------
    #  Integration weights -- trapezoidal rule along span
    # ------------------------------------------------------------------------------------------------------------------
    diff_r    = np.diff(r_1d)                              # (Nr_s,)
    deltar_3d = diff_r[np.newaxis, :, np.newaxis] * np.ones((ctrl_pts, Nr_s, B))  # (ctrl_pts, Nr-1, B)
    r_3d      = r_mid[np.newaxis, :, np.newaxis] * np.ones((ctrl_pts, Nr_s, B))
    r_dim_2d  = np.tile(r_mid[:, None], (1, B))
    r_dim_2d  = np.repeat(r_dim_2d[None,:,:], ctrl_pts, axis=0)
    
    #---------------------------------------------------------------------------      
    # tip loss correction for velocities, since tip loss correction is only applied to loads in prior BET iteration
    va     = F*va
    vt     = F*vt
    lamdaw = r_3d*(Ua-va)/(R*(Ut-vt))

    # More Cd scaling from Mach from AA241ab notes for turbulent skin friction
    Tw_Tinf     = 1. + 1.78*(Ma*Ma)
    Tp_Tinf     = 1. + 0.035*(Ma*Ma) + 0.45*(Tw_Tinf-1.)
    Tp          = (Tp_Tinf)*T
    Rp_Rinf     = (Tp_Tinf**2.5)*(Tp+110.4)/(T+110.4)
    Cd          = ((1/Tp_Tinf)*(1/Rp_Rinf)**0.2)*Cdval

    epsilon             = Cd/Cl
    epsilon[Cl == 1e-6] = 10.

    # thrust and torque and their derivatives on the blade.
    blade_T_distribution     = rho[:, :, None]*(Gamma*(Wt-epsilon*Wa))*deltar_3d
    blade_Q_distribution     = rho[:, :, None]*(Gamma*(Wa+epsilon*Wt)*r_3d)*deltar_3d
    blade_dT_dr              = rho[:, :, None]*(Gamma*(Wt-epsilon*Wa))
    blade_dQ_dr              = rho[:, :, None]*(Gamma*(Wa+epsilon*Wt)*r_3d)

    blade_T_distribution_2d = blade_T_distribution
    blade_Q_distribution_2d = blade_Q_distribution
    blade_dT_dr_2d          = blade_dT_dr
    blade_dQ_dr_2d          = blade_dQ_dr
    blade_Gamma_2d          = Gamma

    Va_2d   = Wa
    Vt_2d   = Wt
    V_disc  = np.sqrt(Va_2d**2 + Vt_2d**2)
    M_disc  = Ma
        
    Va_avg = np.average(Wa, axis=2)      # averaged around the azimuth
    Vt_avg = np.average(Wt, axis=2)      # averaged around the azimuth

    Va_ind_2d  = va
    Vt_ind_2d  = vt
    Vt_ind_avg = np.average(vt, axis=2)
    Va_ind_avg = np.average(va, axis=2)

    # compute the hub force / rotor drag distribution along the blade
    dL_2d = 0.5*rho[:, :, None]*c_mid*Cd*omegar**2*deltar_3d
    dD_2d = 0.5*rho[:, :, None]*c_mid*Cl*omegar**2*deltar_3d

    rotor_drag_distribution = np.sum(dL_2d*np.sin(psi[None,:,:]) + dD_2d*np.cos(psi[None,:,:]), axis=2)
    
    # forces
    thrust     = np.sum(blade_T_distribution, axis=(1, 2))[:, None]   # (ctrl_pts, 1)
    torque     = np.sum(blade_Q_distribution, axis=(1, 2))[:, None]   # (ctrl_pts, 1)
    rotor_drag = np.sum(rotor_drag_distribution, axis=1)[:, None]   # (ctrl_pts, 1)
    power      = omega*torque

    c_mean    = np.mean(rotor.chord_distribution)          # scalar, mean chord (Nr,) averaged
    sigma     = B * c_mean / (np.pi * R)                   # scalar solidity

    # calculate coefficients
    A        = np.pi*(R**2)
    D         = 2*R
    Cq        = torque/(rho_0*(n*n)*(D*D*D*D*D))
    Cq_rotor  = torque / (rho_0 * A * (omega*R)**2 * R)
    Ct        = thrust/(rho_0*(n*n)*(D*D*D*D))
    Ct_rotor  = thrust / (rho_0 * A * (omega * R)**2)   # rotor convention
    Ct_sigma  = Ct_rotor / sigma                                 
    Cp        = power/(rho_0*(n*n*n)*(D*D*D*D*D))
    Cp_rotor  = power  / (rho_0 * A * (omega*R)**3)
    Crd      = rotor_drag/(rho_0*(n*n)*(D*D*D*D))
    etap     = V*thrust/power
    FoM      = thrust*np.sqrt(thrust/(2*rho_0*A))/power  

    print("FM: ", FoM)
    print("Ct_sigma: ", Ct_sigma)
    print("Ct: ", Ct_rotor)

    # prevent things from breaking
    Cq[Cq<0]                   = 0.
    Ct[Ct<0]                   = 0.
    Cp[Cp<0]                   = 0.
    thrust[omega<0.0]          = -thrust[omega<0.0]
    thrust[omega==0.0]         = 0.0
    power[omega==0.0]          = 0.0
    torque[omega==0.0]         = 0.0
    rotor_drag[omega==0.0]     = 0.0
    Ct[omega==0.0]             = 0.0
    Cp[omega==0.0]             = 0.0
    etap[omega==0.0]           = 0.
    thrust[eta[:,0]  <=0.0]    = 0.0
    power[eta[:,0]  <=0.0]     = 0.0
    torque[eta[:,0]  <=0.0]    = 0.0  
    power[eta>1.0]             = power[eta>1.0]*eta[eta>1.0]
    thrust[eta[:,0]>1.0,:]     = thrust[eta[:,0]>1.0,:]*eta[eta[:,0]>1.0,:] 

    disc_loading           = thrust/(np.pi*(R**2))
    power_loading          = thrust/(power)

    # Make the thrust a 3D vector
    thrust_prop_frame      = np.zeros((ctrl_pts,3))
    thrust_prop_frame[:,0] = thrust[:,0]
    thrust_vector          = orientation_product(orientation_transpose(T_body2thrust),thrust_prop_frame)
     
    conditions.energy.converters[rotor.tag]  = Data( 
                torque                            = torque,
                thrust                            = thrust_vector,  
                power                             = power, 
                azimuthal_distribution            = psi,  
                design_flag                       = design_flag,             
                rpm                               = omega /Units.rpm ,   
                tip_mach                          = omega * R / conditions.freestream.speed_of_sound, 
                efficiency                        = etap,         
                number_radial_stations            = Nr,
                orientation                       = orientation,  
                number_azimuthal_stations         = B,
                advance_ratio                     = V/(n*D), 
                disc_radial_distribution          = r_dim_2d,
                speed_of_sound                    = conditions.freestream.speed_of_sound,
                density                           = conditions.freestream.density,
                velocity                          = Vv,
                blade_tangential_induced_velocity = Vt_ind_avg,
                blade_axial_induced_velocity      = Va_ind_avg,
                blade_reynolds_number             = Re,
                blade_effective_angle_of_attack   = alpha,
                disc_reynolds_number              = Re_disc,
                disc_effective_angle_of_attack    = alpha_disc,
                blade_tangential_velocity         = Vt_avg,
                blade_axial_velocity              = Va_avg,
                blade_velocity                    = W,
                blade_Mach_number                 = Ma,
                disc_tangential_induced_velocity  = Vt_ind_2d,
                disc_axial_induced_velocity       = Va_ind_2d,
                disc_tangential_velocity          = Vt_2d,
                disc_axial_velocity               = Va_2d,
                disc_velocity                     = V_disc,
                disc_Mach_number                  = M_disc,
                drag_coefficient                  = Cd,
                lift_coefficient                  = Cl, 
                disc_loading                      = disc_loading, 
                power_loading                     = power_loading,      
                omega                             = omega,
                disc_circulation                  = blade_Gamma_2d,
                blade_dT_dr                       = blade_dT_dr,
                disc_dT_dr                        = blade_dT_dr_2d,
                blade_thrust_distribution         = blade_T_distribution,
                disc_thrust_distribution          = blade_T_distribution_2d, 
                thrust_per_blade                  = thrust/B,
                thrust_coefficient                = Ct,
                blade_loading                     = Ct_sigma,
                solidity                          = sigma,
                disc_azimuthal_distribution       = psi,
                blade_dQ_dr                       = blade_dQ_dr,
                disc_dQ_dr                        = blade_dQ_dr_2d,
                blade_torque_distribution         = blade_Q_distribution,
                disc_torque_distribution          = blade_Q_distribution_2d,
                torque_per_blade                  = torque/B,
                torque_coefficient                = Cq,
                power_coefficient                 = Cp, 
                converged_inflow_ratio            = lamdaw, 
                rotor_H_distribution              = rotor_drag_distribution,
                rotor_drag                        = rotor_drag,
                rotor_drag_coefficient            = Crd,
                blade_pitch_command               = pitch_c,
                commanded_thrust_vector_angle     = commanded_TV, 
                figure_of_merit                   = FoM, 
        )  

    return 

