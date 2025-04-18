# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/BEMT_Hemholtz_Vortex_Theory/BEMT_Helmholtz_performance.py
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                              import Data , Units, orientation_product, orientation_transpose  
from RCAIDE.Library.Methods.Aerodynamics.Common.Lift    import compute_airfoil_aerodynamics,compute_inflow_and_tip_loss 

# package imports
import  numpy as  np 
# ---------------------------------------------------------------------------------------------------------------------- 
#  BEMT_Helmholtz_performance
# ----------------------------------------------------------------------------------------------------------------------  
def BEMT_Helmholtz_performance(rotor, conditions):
    """
    Analyzes a general rotor given geometry and operating conditions using
    Blade Element Momentum Theory with a Helmholtz Vortex Wake Prescription.
    
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
    [1] Drela, M. "Qprop Formulation", MIT AeroAstro, June 2006 http://web.mit.edu/drela/Public/web/qprop/qprop_theory.pdf
    [2] Leishman, Gordon J. Principles of helicopter aerodynamicsCambridge university press, 2006.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Blade_Element_Momentum_Theory_Helmholtz_Wake.wake_model
    """
    commanded_TV          = conditions.energy.converters[rotor.tag].commanded_thrust_vector_angle
    pitch_c               = conditions.energy.converters[rotor.tag].blade_pitch_command
    eta                   = conditions.energy.converters[rotor.tag].throttle 
    omega                 = conditions.energy.converters[rotor.tag].omega  
    design_flag           = conditions.energy.converters[rotor.tag].design_flag
    B                     = rotor.number_of_blades
    R                     = rotor.tip_radius
    beta_0                = rotor.twist_distribution
    c                     = rotor.chord_distribution
    sweep                 = rotor.sweep_distribution     
    r_1d                  = rotor.radius_distribution
    tc                    = rotor.thickness_to_chord
    a_loc                 = rotor.airfoil_polar_stations
    airfoils              = rotor.airfoils 
    Na                    = rotor.number_azimuthal_stations
    nonuniform_freestream = rotor.nonuniform_freestream
    use_2d_analysis       = rotor.use_2d_analysis
 
    # Unpack freestream conditions
    rho     = conditions.freestream.density[:,0,None]
    mu      = conditions.freestream.dynamic_viscosity[:,0,None]
    a       = conditions.freestream.speed_of_sound[:,0,None]
    T       = conditions.freestream.temperature[:,0,None]
    Vv      = conditions.frames.inertial.velocity_vector
    nu      = mu/rho
    rho_0   = rho 

    # Number of radial stations and segment control points
    Nr       = len(c)
    ctrl_pts = len(Vv)
    
    # Helpful shorthands
    pi      = np.pi 

    # Velocity in the rotor frame
    T_body2inertial         = conditions.frames.body.transform_to_inertial
    T_inertial2body         = orientation_transpose(T_body2inertial)
    V_body                  = orientation_product(T_inertial2body,Vv)
    body2thrust,orientation = rotor.body_to_prop_vel(commanded_TV) 
    T_body2thrust           = orientation_transpose(np.ones_like(T_body2inertial[:])*body2thrust)
    V_thrust                = orientation_product(T_body2thrust,V_body)

    # Check and correct for hover
    V         = V_thrust[:,0,None]
    V[V==0.0] = 1E-6 

    # Non-dimensional radial distribution and differential radius
    chi           = r_1d/R
    diff_r        = np.diff(r_1d)
    deltar        = np.zeros(len(r_1d))
    deltar[1:-1]  = diff_r[0:-1]/2 + diff_r[1:]/2
    deltar[0]     = diff_r[0]/2
    deltar[-1]    = diff_r[-1]/2

    # Calculating rotational parameters
    omegar   = np.outer(omega,r_1d)
    n        = omega/(2.*pi)   # Rotations per second

    # 2 dimensional radial distribution non dimensionalized
    chi_2d         = np.tile(chi[:, None],(1,Na))
    chi_2d         = np.repeat(chi_2d[None,:,:], ctrl_pts, axis=0)
    r_dim_2d       = np.tile(r_1d[:, None] ,(1,Na))
    r_dim_2d       = np.repeat(r_dim_2d[None,:,:], ctrl_pts, axis=0)
    c_2d           = np.tile(c[:, None] ,(1,Na))
    c_2d           = np.repeat(c_2d[None,:,:], ctrl_pts, axis=0)

    # Azimuthal distribution of stations (in direction of rotation)
    psi            = np.linspace(0,2*pi,Na+1)[:-1]
    psi_2d         = np.tile(np.atleast_2d(psi),(Nr,1))
    psi_2d         = np.repeat(psi_2d[None, :, :], ctrl_pts, axis=0)
 
    total_blade_pitch = beta_0  +  pitch_c
    
    # apply blade sweep to azimuthal position
    if np.any(np.array([sweep])!=0):
        use_2d_analysis     = True
        sweep_2d            = np.repeat(sweep[:, None], (1,Na))
        sweep_offset_angles = np.tan(sweep_2d/r_dim_2d)
        psi_2d             += sweep_offset_angles

    # Starting with uniform freestream
    ua       = 0
    ut       = 0
    ur       = 0

    # Include velocities introduced by rotor incidence angles
    if (np.any(abs(V_thrust[:,1]) >1e-3) or np.any(abs(V_thrust[:,2]) >1e-3)) and use_2d_analysis:

        # y-component of freestream in the propeller cartesian plane
        Vy  = V_thrust[:,1,None,None]
        Vy  = np.repeat(Vy, Nr,axis=1)
        Vy  = np.repeat(Vy, Na,axis=2)

        # z-component of freestream in the propeller cartesian plane
        Vz  = V_thrust[:,2,None,None]
        Vz  = np.repeat(Vz, Nr,axis=1)
        Vz  = np.repeat(Vz, Na,axis=2)

        # compute resulting radial and tangential velocities in polar frame
        utz =  -Vz*np.sin(psi_2d)
        urz =   Vz*np.cos(psi_2d)
        uty =  -Vy*np.cos(psi_2d)
        ury =   Vy*np.sin(psi_2d)

        ut +=  (utz + uty)  # tangential velocity in direction of rotor rotation
        ur +=  (urz + ury)  # radial velocity (positive toward tip)
        ua +=  np.zeros_like(ut) 
    
    # Include external velocities introduced by user
    if nonuniform_freestream:
        use_2d_analysis   = True

        # include additional influences specified at rotor sections, shape=(ctrl_pts,Nr,Na)
        ua += rotor.axial_velocities_2d
        ut += rotor.tangential_velocities_2d
        ur += rotor.radial_velocities_2d

    if use_2d_analysis:
        # make everything 2D with shape (ctrl_pts,Nr,Na)

        # 2-D freestream velocity and omega*r
        V_2d   = V_thrust[:,0,None,None]
        V_2d   = np.repeat(V_2d, Na,axis=2)
        V_2d   = np.repeat(V_2d, Nr,axis=1)
        omegar = (np.repeat(np.outer(omega,r_1d)[:,:,None], Na, axis=2))

        # total velocities
        Ua     = V_2d + ua

        # 2-D blade pitch and radial distributions
        if np.size(pitch_c)>1:
            # control variable is the blade pitch, repeat around azimuth
            beta = np.repeat(total_blade_pitch[:,:,None], Na, axis=2)
        else:
            beta = np.tile(total_blade_pitch[:,:,None],(ctrl_pts,1,Na ))

        r    = np.tile(r_1d[None,:,None], (ctrl_pts, 1, Na))
        c    = np.tile(c[None,:,None], (ctrl_pts, 1, Na))
        deltar = np.tile(deltar[None,:,None], (ctrl_pts, 1, Na))

        # 2-D atmospheric properties
        a   = np.tile(np.atleast_2d(a),(1,Nr))
        a   = np.repeat(a[:, :, None], Na, axis=2)
        nu  = np.tile(np.atleast_2d(nu),(1,Nr))
        nu  = np.repeat(nu[:,  :, None], Na, axis=2)
        rho = np.tile(np.atleast_2d(rho),(1,Nr))
        rho = np.repeat(rho[:,  :, None], Na, axis=2)
        T   = np.tile(np.atleast_2d(T),(1,Nr))
        T   = np.repeat(T[:, :, None], Na, axis=2)

    else:
        # total velocities
        r      = r_1d
        Ua     = np.outer((V + ua),np.ones_like(r))
        beta   = total_blade_pitch

    # Total velocities
    Ut     = omegar - ut
    U      = np.sqrt(Ua*Ua + Ut*Ut + ur*ur)

    #---------------------------------------------------------------------------
    # COMPUTE WAKE-INDUCED INFLOW VELOCITIES AND RESULTING ROTOR PERFORMANCE
    #---------------------------------------------------------------------------
    # pack inputs
    wake_inputs                       = Data()
    wake_inputs.velocity_total        = U
    wake_inputs.velocity_axial        = Ua
    wake_inputs.velocity_tangential   = Ut
    wake_inputs.ctrl_pts              = ctrl_pts
    wake_inputs.Nr                    = Nr
    wake_inputs.Na                    = Na        
    wake_inputs.use_2d_analysis       = use_2d_analysis        
    wake_inputs.twist_distribution    = beta
    wake_inputs.chord_distribution    = c
    wake_inputs.radius_distribution   = r
    wake_inputs.speed_of_sounds       = a
    wake_inputs.dynamic_viscosities   = nu
     
    va, vt = RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Blade_Element_Momentum_Theory_Helmholtz_Wake.wake_model.evaluate_wake(rotor,wake_inputs,conditions)
    
    # compute new blade velocities
    Wa   = va + Ua
    Wt   = Ut - vt

    lamdaw, F, _ = compute_inflow_and_tip_loss(r,R,Wa,Wt,B)

    # Compute aerodynamic forces based on specified input airfoil or surrogate
    Cl, Cdval, alpha, alpha_disc,Ma,W,Re,Re_disc = compute_airfoil_aerodynamics(beta,c,r,R,B,Wa,Wt,a,nu,airfoils,a_loc,ctrl_pts,Nr,Na,tc,use_2d_analysis) 
    
    # compute HFW circulation at the blade
    Gamma = 0.5*W*c*Cl  

    #---------------------------------------------------------------------------      
    # tip loss correction for velocities, since tip loss correction is only applied to loads in prior BET iteration
    va     = F*va
    vt     = F*vt
    lamdaw = r*(va+Ua)/(R*(Ut-vt))

    # More Cd scaling from Mach from AA241ab notes for turbulent skin friction
    Tw_Tinf     = 1. + 1.78*(Ma*Ma)
    Tp_Tinf     = 1. + 0.035*(Ma*Ma) + 0.45*(Tw_Tinf-1.)
    Tp          = (Tp_Tinf)*T
    Rp_Rinf     = (Tp_Tinf**2.5)*(Tp+110.4)/(T+110.4)
    Cd          = ((1/Tp_Tinf)*(1/Rp_Rinf)**0.2)*Cdval

    epsilon                  = Cd/Cl
    epsilon[epsilon==np.inf] = 10.

    # thrust and torque and their derivatives on the blade.
    blade_T_distribution     = rho*(Gamma*(Wt-epsilon*Wa))*deltar
    blade_Q_distribution     = rho*(Gamma*(Wa+epsilon*Wt)*r)*deltar
    blade_dT_dr              = rho*(Gamma*(Wt-epsilon*Wa))
    blade_dQ_dr              = rho*(Gamma*(Wa+epsilon*Wt)*r)


    if use_2d_analysis:
        blade_T_distribution_2d = blade_T_distribution
        blade_Q_distribution_2d = blade_Q_distribution
        blade_dT_dr_2d          = blade_dT_dr
        blade_dQ_dr_2d          = blade_dQ_dr
        blade_Gamma_2d          = Gamma
        alpha_2d                = alpha

        Va_2d = Wa
        Vt_2d = Wt
        V_disc  = np.sqrt(Va_2d**2 + Vt_2d**2)
        M_disc  = Ma
        
        Va_avg = np.average(Wa, axis=2)      # averaged around the azimuth
        Vt_avg = np.average(Wt, axis=2)      # averaged around the azimuth

        Va_ind_2d  = va
        Vt_ind_2d  = vt
        Vt_ind_avg = np.average(vt, axis=2)
        Va_ind_avg = np.average(va, axis=2)

        # set 1d blade loadings to be the average:
        blade_T_distribution    = np.mean((blade_T_distribution_2d), axis = 2)
        blade_Q_distribution    = np.mean((blade_Q_distribution_2d), axis = 2)
        blade_dT_dr             = np.mean((blade_dT_dr_2d), axis = 2)
        blade_dQ_dr             = np.mean((blade_dQ_dr_2d), axis = 2)

        # compute the hub force / rotor drag distribution along the blade
        dL_2d    = 0.5*rho*c_2d*Cd*omegar**2*deltar
        dD_2d    = 0.5*rho*c_2d*Cl*omegar**2*deltar

        rotor_drag_distribution = np.mean(dL_2d*np.sin(psi_2d) + dD_2d*np.cos(psi_2d),axis=2)

    else:
        Va_2d   = np.repeat(Wa[ :, :, None], Na, axis=2)
        Vt_2d   = np.repeat(Wt[ :, :, None], Na, axis=2)

        blade_T_distribution_2d  = np.repeat(blade_T_distribution[:, :, None], Na, axis=2)
        blade_Q_distribution_2d  = np.repeat(blade_Q_distribution[:, :, None], Na, axis=2)
        blade_dT_dr_2d           = np.repeat(blade_dT_dr[:, :, None], Na, axis=2)
        blade_dQ_dr_2d           = np.repeat(blade_dQ_dr[:, :, None], Na, axis=2)
        blade_Gamma_2d           = np.repeat(Gamma[ :, :, None], Na, axis=2)
        alpha_2d                 = np.repeat(alpha[ :, :, None], Na, axis=2)
        V_disc                   = np.sqrt(Va_2d**2 + Vt_2d**2)
        M_disc                   = np.repeat(Ma[ :, :, None], Na, axis=2)
        
        Vt_avg                  = Wt
        Va_avg                  = Wa
        Vt_ind_avg              = vt
        Va_ind_avg              = va
        Va_ind_2d               = np.repeat(va[ :, :, None], Na, axis=2)
        Vt_ind_2d               = np.repeat(vt[ :, :, None], Na, axis=2)

        # compute the hub force / rotor drag distribution along the blade
        dL    = 0.5*rho*c*Cd*omegar**2*deltar
        dL_2d = np.repeat(dL[:, :, None], Na, axis=2)
        dD    = 0.5*rho*c*Cl*omegar**2*deltar
        dD_2d = np.repeat(dD[:, :, None], Na, axis=2)

        rotor_drag_distribution = np.mean(dL_2d*np.sin(psi_2d) + dD_2d*np.cos(psi_2d),axis=2)

    # forces
    thrust                  = np.atleast_2d((B * np.sum(blade_T_distribution, axis = 1))).T
    torque                  = np.atleast_2d((B * np.sum(blade_Q_distribution, axis = 1))).T
    rotor_drag              = np.atleast_2d((B * np.sum(rotor_drag_distribution, axis=1))).T
    power                   = omega*torque

    # calculate coefficients
    D        = 2*R
    Cq       = torque/(rho_0*(n*n)*(D*D*D*D*D))
    Ct       = thrust/(rho_0*(n*n)*(D*D*D*D))
    Cp       = power/(rho_0*(n*n*n)*(D*D*D*D*D))
    Crd      = rotor_drag/(rho_0*(n*n)*(D*D*D*D))
    etap     = V*thrust/power
    A        = np.pi*(R**2 - rotor.hub_radius**2)
    FoM      = thrust*np.sqrt(thrust/(2*rho_0*A))/power  

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
                number_azimuthal_stations         = Na,
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
                disc_azimuthal_distribution       = psi_2d,
                blade_dQ_dr                       = blade_dQ_dr,
                disc_dQ_dr                        = blade_dQ_dr_2d,
                blade_torque_distribution         = blade_Q_distribution,
                disc_torque_distribution          = blade_Q_distribution_2d,
                torque_per_blade                  = torque/B,
                torque_coefficient                = Cq,
                power_coefficient                 = Cp, 
                converged_inflow_ratio            = lamdaw, 
                blade_H_distribution              = rotor_drag_distribution,
                rotor_drag                        = rotor_drag,
                rotor_drag_coefficient            = Crd,
                blade_pitch_command               = pitch_c,
                commanded_thrust_vector_angle     = commanded_TV, 
                figure_of_merit                   = FoM, 
        )  

    return 