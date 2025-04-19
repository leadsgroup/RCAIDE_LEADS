# RCAIDE/Methods/Noise/Frequency_Domain_Buildup/Rotor/broadband_noise.py
# 
# 
# Created:  Sep 2024, Niranjan Nanjappa  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE 
from RCAIDE.Framework.Core import Units  
from RCAIDE.Library.Methods.Noise.Frequency_Domain_Buildup.Rotor.BPM_boundary_layer_properties import BPM_boundary_layer_properties
from RCAIDE.Library.Methods.Noise.Frequency_Domain_Buildup.Rotor.LBL_VS_broadband_noise        import LBL_VS_broadband_noise
from RCAIDE.Library.Methods.Noise.Frequency_Domain_Buildup.Rotor.TBL_TE_broadband_noise        import TBL_TE_broadband_noise
from RCAIDE.Library.Methods.Noise.Frequency_Domain_Buildup.Rotor.TIP_broadband_noise           import TIP_broadband_noise 
from RCAIDE.Library.Methods.Noise.Frequency_Domain_Buildup.Rotor.noise_directivities           import noise_directivities

# Python Package imports 
import numpy as np  
 
# ----------------------------------------------------------------------------------------------------------------------
# Compute Broadband Noise 
# ----------------------------------------------------------------------------------------------------------------------
def broadband_noise(conditions,propulsor_conditions,coordinates,rotor,settings,Noise,cpt):
    '''This computes the trailing edge noise compoment of broadband noise of a propeller or 
    lift-rotor in the frequency domain. Boundary layer properties are computed using RCAIDE's 
    panel method.
    
    Assumptions:
        Boundary layer thickness (delta) appear to be an order of magnitude off at the trailing edge and 
        correction factor of 0.1 is used. See lines 255 and 256 
        
    Source: 
        Li, Sicheng Kevin, and Seongkyu Lee. "Prediction of Urban Air Mobility Multirotor VTOL Broadband Noise
        Using UCD-QuietFly." Journal of the American Helicopter Society (2021).
    
    Inputs:  
        freestream                                   - freestream data structure                                                          [m/s]
        angle_of_attack                              - aircraft angle of attack                                                           [rad]
        bspv                                         - rotor blade section trailing position edge vectors                                 [m]
        velocity_vector                              - velocity vector of aircraft                                                        [m/s]
        rotors                                       - data structure of rotors                                                           [None] 
        aeroacoustic_data                            - data structure of acoustic data                                                    [None] 
        settings                                     - accoustic settings                                                                 [None] 
        res                                          - results data structure                                                             [None] 
    
    Outputs 
       res.                                           *acoustic data is stored and passed in data structures*                                          
           SPL_prop_broadband_spectrum               - broadband noise in blade passing frequency spectrum                                [dB]
           SPL_prop_broadband_spectrum_dBA           - dBA-Weighted broadband noise in blade passing frequency spectrum                   [dbA]     
           SPL_prop_broadband_1_3_spectrum           - broadband noise in 1/3 octave spectrum                                             [dB]
           SPL_prop_broadband_1_3_spectrum_dBA       - dBA-Weighted broadband noise in 1/3 octave spectrum                                [dBA]                               
           p_pref_azimuthal_broadband                - azimuthal varying pressure ratio of broadband noise                                [Unitless]       
           p_pref_azimuthal_broadband_dBA            - azimuthal varying pressure ratio of dBA-weighted broadband noise                   [Unitless]     
           SPL_prop_azimuthal_broadband_spectrum     - azimuthal varying broadband noise in blade passing frequency spectrum              [dB]      
           SPL_prop_azimuthal_broadband_spectrum_dBA - azimuthal varying dBA-Weighted broadband noise in blade passing frequency spectrum [dbA]   
        
    Properties Used:
        N/A   
    '''     
    aeroacoustic_data       = propulsor_conditions[rotor.tag]
    num_cpt                 = 1
    num_mic                 = len(coordinates.X[0,:,0,0,0])  
    num_blades              = len(coordinates.X[0,0,:,0,0])
    num_sec                 = len(coordinates.X[0,0,0,:,0]) 
    frequency               = settings.center_frequencies
    num_cf                  = len(frequency)
    num_az                  = aeroacoustic_data.number_azimuthal_stations
    
    # ----------------------------------------------------------------------------------
    # Trailing Edge Noise
    # ---------------------------------------------------------------------------------- 
    freestream         = conditions.freestream
    speed_of_sound     = freestream.speed_of_sound          # speed of sound
    density            = freestream.density                 # air density 
    dyna_visc          = freestream.dynamic_viscosity  
    alpha              = aeroacoustic_data.disc_effective_angle_of_attack/Units.degrees 
    alpha_tip          = aeroacoustic_data.disc_effective_angle_of_attack[:,-1]/Units.degrees  
    disc_Re            = aeroacoustic_data.disc_reynolds_number
    disc_speed         = aeroacoustic_data.disc_velocity
    disc_Ma            = aeroacoustic_data.disc_Mach_number
    Vt                 = aeroacoustic_data.disc_tangential_velocity  
    Va                 = aeroacoustic_data.disc_axial_velocity                
    blade_chords       = rotor.chord_distribution           # blade chord    
    r                  = rotor.radius_distribution          # radial location   
    Omega              = aeroacoustic_data.omega            # angular velocity    
    L                  = np.zeros_like(r)
    del_r              = r[1:] - r[:-1]
    L[0]               = del_r[0]
    L[-1]              = del_r[-1]
    L[1:-1]            = (del_r[:-1]+ del_r[1:])/2

    if np.all(Omega == 0):
        Noise.p_pref_broadband                          = np.zeros((num_cpt,num_mic,num_cf)) 
        Noise.SPL_prop_broadband_spectrum               = np.zeros_like(Noise.p_pref_broadband)
        Noise.SPL_prop_broadband_spectrum_dBA           = np.zeros_like(Noise.p_pref_broadband)
        Noise.SPL_prop_broadband_1_3_spectrum           = np.zeros((num_cpt,num_mic,num_cf))
        Noise.SPL_prop_broadband_1_3_spectrum_dBA       = np.zeros((num_cpt,num_mic,num_cf))
        Noise.p_pref_azimuthal_broadband                = np.zeros((num_cpt,num_mic,num_cf))
        Noise.p_pref_azimuthal_broadband_dBA            = np.zeros_like(Noise.p_pref_azimuthal_broadband)
        Noise.SPL_prop_azimuthal_broadband_spectrum     = np.zeros_like(Noise.p_pref_azimuthal_broadband)
        Noise.SPL_prop_azimuthal_broadband_spectrum_dBA = np.zeros_like(Noise.p_pref_azimuthal_broadband)
    else: 
        # dimension of matrices [control pt, microphone, # blades, # blade sections, # center frequencies, # azimuthal stations] 
        c                 = np.tile(blade_chords[None,None,None,:,None,None],(num_cpt,num_mic,num_blades,1,num_cf,num_az))
        L                 = np.tile(L[None,None,None,:,None,None],(num_cpt,num_mic,num_blades,1,num_cf,num_az))
        f                 = np.tile(frequency[None,None,None,None,:,None],(num_cpt,num_mic,num_blades,num_sec,1,num_az)) 
        
        alpha_disk        = np.tile(alpha[cpt,None,None,:,None,:],(1,num_mic,num_blades,1,num_cf,1))
        V                 = np.zeros((num_cpt,num_mic,num_blades,num_sec,num_cf,num_az,3))
        V[:,:,:,:,:,:,0]  = -np.tile(Vt[cpt,None,None,:,None,:],(1,num_mic,num_blades,1,num_cf,1)) 
        V[:,:,:,:,:,:,2]  = np.tile(Va[cpt,None,None,:,None,:],(1,num_mic,num_blades,1,num_cf,1))  
        V_tot             = np.linalg.norm(V, axis=6)
        alpha_tip         = np.tile(alpha_tip[cpt,None,None,None,None,:],(1,num_mic,num_blades,num_sec,num_cf,1))  
        c_0               = np.tile(speed_of_sound[cpt,:,None,None,None,None],(1,num_mic,num_blades,num_sec,num_cf,num_az))
        rho               = np.tile(density[cpt,:,None,None,None,None],(1,num_mic,num_blades,num_sec,num_cf,num_az)) 
        mu                = np.tile(dyna_visc[cpt,:,None,None,None,None],(1,num_mic,num_blades,num_sec,num_cf,num_az)) 
        R_c               = np.tile(disc_Re[cpt,None,None,:,None,:],(1,num_mic,num_blades,1,num_cf,1))
        U                 = np.tile(disc_speed[cpt,None,None,:,None,:],(1,num_mic,num_blades,1,num_cf,1))
        M                 = np.tile(disc_Ma[cpt,None,None,:,None,:],(1,num_mic,num_blades,1,num_cf,1)) # U/c_0 
        M_tot             = V_tot/c_0   
          
        X_prime_r         = np.tile(coordinates.X_prime_r[cpt,:,:,:,None,None,:],(1,1,1,1,num_cf,num_az,1))
        cos_zeta_r        = np.sum(X_prime_r*V, axis = 6)/(np.linalg.norm(X_prime_r, axis = 6)*V_tot) 
        r_er              = np.tile(np.linalg.norm(coordinates.X_e_r[cpt], axis = 3)[None,:,:,:,None,None],(1,1,1,1,num_cf,num_az))           
        Phi_er            = np.tile(coordinates.phi_e_r[cpt,:,:,:,None,None],(1,1,1,1,num_cf,num_az))
        Theta_er          = np.tile(coordinates.theta_e_r[cpt,:,:,:,None,None],(1,1,1,1,num_cf,num_az))    
        
        # flatten matrices 
        R_c        = flatten_matrix(R_c,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        c          = flatten_matrix(c,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        alpha_star = flatten_matrix(alpha_disk,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        alpha_tip  = flatten_matrix(alpha_tip,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        U          = flatten_matrix(U,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        cos_zeta_r = flatten_matrix(cos_zeta_r,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        M_tot      = flatten_matrix(M_tot,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        f          = flatten_matrix(f,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        c_0        = flatten_matrix(c_0,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        rho        = flatten_matrix(rho,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        r_er       = flatten_matrix(r_er,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        mu         = flatten_matrix(mu,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        L          = flatten_matrix(L,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        M          = flatten_matrix(M,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        Phi_er     = flatten_matrix(Phi_er,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        Theta_er   = flatten_matrix(Theta_er,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        
        
        # calculation of boundary layer properties, eqns 2 - 16   
        # boundary layer properies of tripped and untripped at 0 angle of attack   
        boundary_layer_data  = BPM_boundary_layer_properties(R_c,c,alpha_star)  
    
        # define simulation variables/constants   
        Re_delta_star_p_untripped = boundary_layer_data.delta_star_p_untripped*U*rho/mu
        Re_delta_star_p_tripped   = boundary_layer_data.delta_star_p_tripped*U*rho/mu  
    
        # calculation of directivitiy terms , eqns 24 - 50 
        Dbar_h, Dbar_l = noise_directivities(Theta_er,Phi_er,cos_zeta_r,M_tot) 
          
        # calculation of turbulent boundary layer - trailing edge noise,  eqns 24 - 50 
        SPL_TBL_TE_tripped   = TBL_TE_broadband_noise(f,r_er,L,U,M,R_c,Dbar_h,Dbar_l,Re_delta_star_p_tripped,
                                                  boundary_layer_data.delta_star_p_tripped,
                                                  boundary_layer_data.delta_star_s_tripped,
                                                  alpha_star) 
        
        SPL_TBL_TE_untripped = TBL_TE_broadband_noise(f,r_er,L,U,M,R_c,Dbar_h,Dbar_l,Re_delta_star_p_untripped,
                                                  boundary_layer_data.delta_star_p_untripped,
                                                  boundary_layer_data.delta_star_s_untripped,
                                                  alpha_star)  
      
        # calculation of laminar boundary layer - vortex shedding, eqns 53 - 60 
        SPL_LBL_VS = LBL_VS_broadband_noise(R_c,alpha_star,boundary_layer_data.delta_star_p_untripped,r_er,L,M,Dbar_h,f,U)
       
        # calculation of tip vortex noise, eqns 61 - 67 
        alpha_TIP = abs(alpha_tip)
        SPL_TIP   = TIP_broadband_noise(alpha_TIP,M,c,c_0,f,Dbar_h,r_er)  
         
        # TO DO : Compute BWI  
        
        
        # TO DO : Compute BVI -  isnt this actually tonal ?
    
        # Unflatten Matices 
        SPL_TBL_TE_tripped   = unflatten_matrix(SPL_TBL_TE_tripped,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az) 
        SPL_TBL_TE_untripped = unflatten_matrix(SPL_TBL_TE_untripped,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        SPL_LBL_VS           = unflatten_matrix(SPL_LBL_VS,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        SPL_TIP              = unflatten_matrix(SPL_TIP,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        
        # Pressure from each Broadband source
        P_BWI                 = np.zeros((num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)) # this will be replaced soon
        P_TBL_TE_tripped      = 10**(SPL_TBL_TE_tripped/10)
        P_TBL_TE_untripped    = 10**(SPL_TBL_TE_untripped/10)
        P_LBL_VS              = 10**(SPL_LBL_VS/10)
        P_TIP                 = 10**(SPL_TIP/10)
        P_TIP[:,:,:,:-1,:,:]  = 0
        
        # Sum broadband compoments along blade sections and blades to get self noise per rotor 
        P_b_7                     = np.zeros((4,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az))
        P_b_7[0,:,:,:,:,:,:]      = P_BWI
        P_b_7[1,:,:,:,:,:,:]      = P_TBL_TE_tripped 
        P_b_7[2,:,:,:,:,:,:]      = P_LBL_VS
        P_b_7[3,:,:,:,:,:,:]      = P_TIP
        
        # Sum all components of broadband noise pressures
        P_b_6            = np.sum(P_b_7, axis=0)
        
        # Take product of total broadband noise with Doppler shift factor and weighting factor
        M_tot            = unflatten_matrix(M_tot,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        cos_zeta_r       = unflatten_matrix(cos_zeta_r,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az)
        Doppler_shift    = 1/(1-M_tot*cos_zeta_r)
        P_b_6_shifted    = (1/num_az)*(1/Doppler_shift)*P_b_6
        
        # Sum broadband noise along blade sections
        P_b_5            = np.sum(P_b_6_shifted, axis=3)
        
        # Sum broadband noise for all blades
        P_b_4            = np.sum(P_b_5, axis=2)
        
        # Sum broadband noise along all azimuthal stations
        P_b_3            = np.sum(P_b_4, axis=3)
        
        SPL_broadband    = 10*np.log10(P_b_3)
        
        # store results 
        Noise.SPL_prop_broadband_spectrum                   = SPL_broadband
        Noise.SPL_prop_broadband_1_3_spectrum               = Noise.SPL_prop_broadband_spectrum     # already in 1/3 octave spectrum
        
    return
 
def flatten_matrix(x,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az):
    return np.reshape(x,(num_cpt*num_mic*num_blades*num_sec*num_cf*num_az))
 
def unflatten_matrix(x,num_cpt,num_mic,num_blades,num_sec,num_cf,num_az):
    return np.reshape(x,(num_cpt,num_mic,num_blades,num_sec,num_cf,num_az))