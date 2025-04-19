# RCAIDE/Methods/Noise/Multi_Fidelity/harmonic_noise_line.py
# 
# 
# Created:  Jun 2024, Niranjan Nanjappa

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE
from RCAIDE.Framework.Core                                 import orientation_product, orientation_transpose      
from RCAIDE.Library.Methods.Noise.Common                         import convert_to_third_octave_band 

# Python Package imports  
import numpy as np
from scipy.special import jv 
import scipy as sp

# ----------------------------------------------------------------------------------------------------------------------
# Compute Harmonic Noise 
# ---------------------------------------------------------------------------------------------------------------------- 
def harmonic_noise_line(harmonics_blade,harmonics_load,conditions,propulsor_conditions,coordinates,rotor,settings,Noise,cpt):
    '''This computes the harmonic noise (i.e. thickness and loading noise) in the frequency domain 
    of a rotor at any angle of attack with load distribution along the blade span. This is a level 1 fidelity
    approach. The thickness source is however computed using the helicoidal surface theory.

    Assumptions:
    1) Acoustic compactness of loads along blade chord.
    2) Acoustic non-compactness of loads along blade span.
    3) Acoustic compactness of loads along blade thickness.

    Source:
    1) Hanson, D. B. (1995). Sound from a propeller at angle of attack: a new theoretical viewpoint. 
    Proceedings - Royal Society of London, A, 449(1936). https://doi.org/10.1098/rspa.1995.0046
    
    2) Hanson, D. B. "Noise radiation of propeller loading sources with angular inflow" AIAA 1990-3955.
    13th Aeroacoustics Conference. October 1990. 

    3) Hubbard, Harvey H., ed. Aeroacoustics of flight vehicles: theory and practice. Vol. 1.
    NASA Office of Management, Scientific and Technical Information Program, 1991.


    Inputs: 
        harmonics_blade               - blade harmonics                                                            [Unitless]
        harmonics_load                - loading harmonics (modes within each blade harmonic mode)                  [Unitless]
        freestream                    - freestream data structure                                                  [m/s]
        angle_of_attack               - aircraft angle of attack                                                   [rad]
        position_vector               - position vector of aircraft                                                [m]
        velocity_vector               - velocity vector of aircraft                                                [m/s] 
        rotors                        - data structure of rotors                                                   [None]
        aeroacoustic_data             - data structure of acoustic data                                            [None]
        settings                      - accoustic settings                                                         [None] 
        res                           - results data structure                                                     [None] 

    Outputs 
        res.                                    *acoustic data is stored and passed in data structures*                                                                            
            SPL_prop_harmonic_bpf_spectrum       - harmonic noise in blade passing frequency spectrum              [dB]
            SPL_prop_harmonic_bpf_spectrum_dBA   - dBA-Weighted harmonic noise in blade passing frequency spectrum [dbA]                  
            SPL_prop_harmonic_1_3_spectrum       - harmonic noise in 1/3 octave spectrum                           [dB]
            SPL_prop_harmonic_1_3_spectrum_dBA   - dBA-Weighted harmonic noise in 1/3 octave spectrum              [dBA] 
            p_pref_harmonic                      - pressure ratio of harmonic noise                                [Unitless]
            p_pref_harmonic_dBA                  - pressure ratio of dBA-weighted harmonic noise                   [Unitless]


    Properties Used:
        N/A
    
    Code Convention - The number in front of a variable name indicates the number of dimensions of the variable.
                      For instance, m_6 is the 6 dimensional harmonic modes variable, m_5 is 5 dimensional harmonic modes variable
    '''

    aeroacoustic_data       = propulsor_conditions[rotor.tag] 
    angle_of_attack         = np.atleast_2d(conditions.aerodynamics.angles.alpha[cpt]) 
    velocity_vector         = np.atleast_2d(conditions.frames.inertial.velocity_vector[cpt])   
    freestream              = conditions.freestream       
    num_h_b                 = len(harmonics_blade)
    num_h_l                 = len(harmonics_load)
    num_cpt                 = len(angle_of_attack) 
    num_mic                 = len(coordinates.X_hub[cpt,:,0,0,0]) 
    phi_0                   = np.array([rotor.phase_offset_angle])  # phase angle offset  
    airfoils                = rotor.airfoils
    num_sec                 = len(rotor.radius_distribution)
    num_az                  = aeroacoustic_data.number_azimuthal_stations 
    orientation             = np.array(rotor.orientation_euler_angles) * 1 
    body2thrust             = sp.spatial.transform.Rotation.from_rotvec(orientation).as_matrix()  
    commanded_thrust_vector = np.atleast_2d(propulsor_conditions.commanded_thrust_vector_angle[cpt])
    for jj,airfoil in enumerate(airfoils):
        airfoil_points = airfoil.number_of_points
        y_u_6          = np.tile(airfoil.geometry.y_upper_surface[None,None,None,None,None,:],(num_cpt,num_mic,num_sec,num_h_b,num_h_l,1))
        y_l_6          = np.tile(airfoil.geometry.y_lower_surface[None,None,None,None,None,:],(num_cpt,num_mic,num_sec,num_h_b,num_h_l,1))
    chord_coord             = int(np.floor(airfoil_points/2))
    
    # ----------------------------------------------------------------------------------
    # Rotational Noise  Thickness and Loading Noise
    # ----------------------------------------------------------------------------------  
    # [control point, microphones, rotors, radial distribution, blade harmonics, load harmonics]  
    
    # freestream density and speed of sound
    a_3            = np.tile(freestream.speed_of_sound[cpt][:,None,None],(1,num_mic,num_h_b))
    rho_3          = np.tile(freestream.density[cpt][:,None,None],(1,num_mic,num_h_b))
    
    B              = rotor.number_of_blades
    
    # blade harmonics
    m_3            = np.tile(harmonics_blade[None,None,:],(num_cpt,num_mic,1))
    m_4            = np.tile(harmonics_blade[None,None,:,None],(num_cpt,num_mic,1,num_h_l))
    m_5            = np.tile(harmonics_blade[None,None,None,:,None],(num_cpt,num_mic,num_sec,1,num_h_l))
    m_6            = np.tile(harmonics_blade[None,None,None,:,None,None],(num_cpt,num_mic,num_sec,1,num_h_l,chord_coord))
    
    # loading harmonics
    k_4            = np.tile(harmonics_load[None,None,None,:],(num_cpt,num_mic,num_h_b,1))
    k_5            = np.tile(harmonics_load[None,None,None,None,:],(num_cpt,num_mic,num_sec,num_h_b,1))                                                                                        
    k_6            = np.tile(harmonics_load[None,None,None,None,:,None],(num_cpt,num_mic,num_sec,num_h_b,1,chord_coord))
    
    # reference atmospheric pressure
    p_ref          = 2E-5
        
    # net angle of inclination of propeller axis wrt inertial axis
    alpha_4        = np.tile((angle_of_attack + np.arccos(body2thrust[0,0]))[:,:,None,None],(1,num_mic,num_h_b,num_h_l))
    alpha_5        = np.tile((angle_of_attack + np.arccos(body2thrust[0,0]))[:,:,None,None,None],(1,num_mic,num_sec,num_h_b,num_h_l))
    alpha_6        = np.tile((angle_of_attack + np.arccos(body2thrust[0,0]))[:,:,None,None,None,None],(1,num_mic,num_sec,num_h_b,num_h_l,chord_coord))
    
    # rotor angular speed
    omega_3        = np.tile(aeroacoustic_data.omega[cpt][:,None,None],(1,num_mic,num_h_b))
    
    R              = rotor.radius_distribution
    
    # Non-dimensional radius distribution
    z_5            = np.tile((R/R[-1])[None,None,:,None,None],(num_cpt,num_mic,1,num_h_b,num_h_l))
    z_6            = np.tile((R/R[-1])[None,None,:,None,None,None],(num_cpt,num_mic,1,num_h_b,num_h_l,chord_coord))
    
    # Radial chord distribution
    c_5            = np.tile(rotor.chord_distribution[None,None,:,None,None],(num_cpt,num_mic,1,num_h_b,num_h_l))
    c_6            = np.tile(rotor.chord_distribution[None,None,:,None,None,None],(num_cpt,num_mic,1,num_h_b,num_h_l,chord_coord))
    
    # chord to diameter ratio
    R_tip          = rotor.tip_radius
    D              = 2*R[-1]
    B_D_5          = c_5/D
    B_D_6          = c_6/D
    
    # maximum thickness to chord ratio
    t_b            = rotor.thickness_to_chord
    t_b_5          = np.tile(t_b[None,None,:,None,None],(num_cpt,num_mic,1,num_h_b,num_h_l))
    
    # chordwise thickness distribution normalized wrt chord
    
    H_6            = (y_u_6 - y_l_6)/c_6
    
    # Rotorcraft speed and mach number
    V_3            = np.tile(np.linalg.norm(velocity_vector, axis=1) [:,None,None],(1,num_mic,num_h_b))
    M_3            = V_3/a_3
    M_5            = np.tile(M_3[:,:,None,:,None],(1,1,num_sec,1,num_h_l))
    M_6            = np.tile(M_3[:,:,None,:,None,None],(1,1,num_sec,1,num_h_l,chord_coord))
    
    # Rotor tip speed and mach number
    V_tip          = R_tip*omega_3                                                        
    M_t_3          = V_tip/a_3
    M_t_5          = np.tile(M_t_3[:,:,None,:,None],(1,1,num_sec,1,num_h_l))
    M_t_6          = np.tile(M_t_3[:,:,None,:,None,None],(1,1,num_sec,1,num_h_l,chord_coord))
    
    # Section relative mach number
    M_r_5          = np.sqrt(M_5**2 + (z_5**2)*(M_t_5**2))
    
    # retarded theta
    theta_r        = coordinates.theta_hub_r[cpt,:,0,0]
    theta_r_3      = np.tile(theta_r[None,:,None],(1,1,num_h_b))
    theta_r_4      = np.tile(theta_r[None,:,None,None],(1,1,num_h_b,num_h_l))
    theta_r_5      = np.tile(theta_r[None,:,None,None,None],(1,1,num_sec,num_h_b,num_h_l))
    theta_r_6      = np.tile(theta_r[None,:,None,None,None,None],(1,1,num_sec,num_h_b,num_h_l,chord_coord))
    
    # retarded distance to source
    Y              = np.sqrt(coordinates.X_hub[cpt,:,0,0,1]**2 +  coordinates.X_hub[cpt,:,0,0,2] **2)
    Y_3            = np.tile(Y[None,:,None],(1,1,num_h_b))
    r_3            = Y_3/np.sin(theta_r_3)
    
    # phase angles
    phi_0_vec      = np.tile(phi_0[:,None,None,None],(num_cpt,num_mic,num_h_b,num_h_l))
    phi_4          = np.tile(coordinates.phi_hub_r[cpt,:,0,0,None,None],(1,1,num_h_b,num_h_l)) + phi_0_vec
    phi_5          = np.tile(phi_4[:,:,None,:,:],(1,1,num_sec,1,1))
    phi_6          = np.tile(phi_4[:,:,None,:,:,None],(1,1,num_sec,1,1,chord_coord))
    
    # total angle between propeller axis and r vector
    theta_r_prime_4 = np.arccos(np.cos(theta_r_4)*np.cos(alpha_4) + np.sin(theta_r_4)*np.sin(phi_4)*np.sin(alpha_4))
    theta_r_prime_5 = np.arccos(np.cos(theta_r_5)*np.cos(alpha_5) + np.sin(theta_r_5)*np.sin(phi_5)*np.sin(alpha_5))
    theta_r_prime_6 = np.arccos(np.cos(theta_r_6)*np.cos(alpha_6) + np.sin(theta_r_6)*np.sin(phi_6)*np.sin(alpha_6))
    
    phi_prime_4    = np.arccos((np.sin(theta_r_4)*np.cos(phi_4))/np.sin(theta_r_prime_4))
    
    # Velocity in the rotor frame
    T_body2inertial = conditions.frames.body.transform_to_inertial
    T_inertial2body = orientation_transpose(T_body2inertial)
    V_body          = orientation_product(T_inertial2body,velocity_vector)
    body2thrust,_   = rotor.body_to_prop_vel(commanded_thrust_vector)
    T_body2thrust   = orientation_transpose(body2thrust)
    V_thrust        = orientation_product(T_body2thrust,V_body)
    V_thrust_perp   = np.atleast_2d(V_thrust[cpt,0,None])
    V_thrust_perp_3 = np.tile(V_thrust_perp[:,:,None],(1,num_mic,num_h_b))
    M_thrust_3      = V_thrust_perp_3/a_3
    M_thrust_5      = np.tile(M_thrust_3[:,:,None,:,None],(1,1,num_sec,1,num_h_l))
    
    # helicoid angle
    zeta_5          = np.arctan(M_thrust_5/(z_5*M_t_5))
    zeta_6          = np.tile(zeta_5[:,:,:,:,:,None],(1,1,1,1,1,chord_coord))
    
    # wavenumbers
    k_m_3          = m_3*B*omega_3/a_3
    k_m_bar        = k_m_3/(1 - M_3*np.cos(theta_r_3))
    k_x_hat_5      = 2*B_D_5*(((m_5*B-k_5)*np.cos(zeta_5))/z_5 + (m_5*B*M_t_5*np.cos(theta_r_prime_5)*np.sin(zeta_5))/(1-M_5*np.cos(theta_r_5)))
    k_x_hat_6      = 2*B_D_6*(((m_6*B-k_6)*np.cos(zeta_6))/z_6 + (m_6*B*M_t_6*np.cos(theta_r_prime_6)*np.sin(zeta_6))/(1-M_6*np.cos(theta_r_6)))
    
    Noise.f          = B*omega_3*m_3/(2*np.pi)
    
    # Frequency domain loading modes
    F_x            = (1/R_tip)*aeroacoustic_data.disc_thrust_distribution[cpt][None,:,:]
    R_temp         = np.tile(R[None,:,None],(num_cpt,1,num_az))
    F_phi          = (1/R_tip)*(1/R_temp)*aeroacoustic_data.disc_torque_distribution[cpt][None,:,:]
    F_xk           = sp.fft.rfft(F_x, axis=2)
    F_phik         = sp.fft.rfft(F_phi, axis=2)
    F_xk_5         = np.tile(F_xk[:,None,:,None,0:num_h_l],(1,num_mic,1,num_h_b,1))
    F_phik_5       = np.tile(F_phik[:,None,:,None,0:num_h_l],(1,num_mic,1,num_h_b,1))
    X_edge         = np.linspace(-0.5,0.5,chord_coord+1)
    X              = 0.5*(X_edge[0:-1] + X_edge[1:])
    X_6            = np.tile(X[None,None,None,None,None,:],(num_cpt,num_mic,num_sec,num_h_b,num_h_l,1))
    exp_term_6     = np.exp(1j*k_x_hat_6*X_6)
    
    # FREQUENCY DOMAIN PRESSURE TERM FOR LOADING
    J_mBk_5        = jv(m_5*B-k_5, (m_5*B*z_5*M_t_5*np.sin(theta_r_prime_5))/(1-M_5*np.cos(theta_r_5)))
    Term1_5        = ((m_5*B*z_5*M_t_5*np.cos(theta_r_prime_5))/(1-M_5*np.cos(theta_r_5)))*F_xk_5
    Term2_5        = -(m_5*B-k_5)*F_phik_5
    Integrand_5    = (1/z_5)*(Term1_5 + Term2_5)*J_mBk_5
    Summand_4      = np.trapz(Integrand_5, x=z_5[0,0,:,0,0], axis=2)*np.exp(1j*(m_4*B-k_4)*(phi_prime_4-(np.pi/2)))
    Summation_3    = np.sum(Summand_4, axis=3)
    P_Lm           = (1j*B*np.exp(1j*k_m_3*r_3)*Summation_3)/(4*np.pi*r_3*(1-M_3*np.cos(theta_r_3))) 
    
    # frequency domain source function for drag and lift
    psi_V_5        = np.trapz(H_6*exp_term_6, x=X, axis=5)
    
    # FREQUENCY DOMAIN PRESSURE TERM FOR THICKNESS
    V_Integrand_5  = (M_r_5**2)*(k_x_hat_5**2)*t_b_5*psi_V_5*J_mBk_5
    V_Summand_4    = np.trapz(V_Integrand_5, x=z_5[0,0,:,0,0], axis=2)*np.exp(1j*m_4*B*(phi_prime_4-(np.pi/2)))
    
    # we take a single dimension along the 4th axis because we only want the loading mode corresponding to k=0
    V_Summation_3  = V_Summand_4[:,:,:,0]
    P_Vm           = (-rho_3*(a_3**2)*B*np.exp(1j*k_m_3*r_3)*V_Summation_3)/(4*np.pi*(r_3/R_tip)*(1-M_3*np.cos(theta_r_3)))

    # SOUND PRESSURE LEVELS
    P_Lm_abs       = np.abs(P_Lm)
    P_Vm_abs       = np.abs(P_Vm)
    Noise.SPL_prop_harmonic_bpf_spectrum     = 20*np.log10((abs(P_Lm_abs + P_Vm_abs))/p_ref)  
    Noise.SPL_prop_harmonic_1_3_spectrum     = convert_to_third_octave_band(Noise.SPL_prop_harmonic_bpf_spectrum,Noise.f,settings)          
    Noise.SPL_prop_harmonic_1_3_spectrum[np.isinf(Noise.SPL_prop_harmonic_1_3_spectrum)]         = 0     
    
    return 