# RCAIDE/Methods/Noise/Frequency_Domain_Buildup/Rotor/compute_rotor_noise.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE Imports 
from RCAIDE.Framework.Core import  Data   
from RCAIDE.Library.Methods.Noise.Common.decibel_arithmetic                                import SPL_arithmetic  
from RCAIDE.Library.Methods.Noise.Common.compute_noise_source_coordinates                  import compute_rotor_point_source_coordinates  
from RCAIDE.Library.Methods.Noise.Frequency_Domain_Buildup.Rotor.harmonic_noise_point      import harmonic_noise_point
from RCAIDE.Library.Methods.Noise.Frequency_Domain_Buildup.Rotor.harmonic_noise_line       import harmonic_noise_line
from RCAIDE.Library.Methods.Noise.Frequency_Domain_Buildup.Rotor.harmonic_noise_plane      import harmonic_noise_plane 
from RCAIDE.Library.Methods.Noise.Frequency_Domain_Buildup.Rotor.broadband_noise           import broadband_noise
from RCAIDE.Library.Methods.Noise.Common                                                   import atmospheric_attenuation
from RCAIDE.Library.Methods.Noise.Metrics.A_weighting_metric                               import A_weighting_metric  
from RCAIDE.Library.Methods.Geometry.Airfoil.import_airfoil_geometry                       import import_airfoil_geometry
from RCAIDE.Library.Methods.Aerodynamics.Airfoil_Panel_Method.airfoil_analysis             import airfoil_analysis

# Python package imports   
import numpy as np    
from RCAIDE.Framework.Core import interp2d 

# ----------------------------------------------------------------------------------------------------------------------    
#  Rotor Noise 
# ----------------------------------------------------------------------------------------------------------------------    
def compute_rotor_noise(microphone_locations,propulsor,rotor,segment,settings, rotor_index = 0, previous_rotor_tag = None, identical_propulsors=True):
    ''' This is a collection medium-fidelity frequency domain methods for rotor acoustic noise prediction which 
    computes the acoustic signature (sound pressure level, weighted sound pressure levels,
    and frequency spectrums of a system of rotating blades           
        
    Assumptions:
    None

    Source:
    None
    
    Inputs:
        rotors                  - data structure of rotors                            [None]
        segment                 - flight segment data structure                       [None] 
        results                 - data structure containing of acoustic data          [None]
        settings                - accoustic settings                                  [None]
                               
    Outputs:
        Results.    
            blade_passing_frequencies      - blade passing frequencies                           [Hz]
            SPL                            - total SPL                                           [dB]
            SPL_dBA                        - dbA-Weighted SPL                                    [dBA]
            SPL_1_3_spectrum               - 1/3 octave band spectrum of SPL                     [dB]
            SPL_1_3_spectrum_dBA           - 1/3 octave band spectrum of A-weighted SPL          [dBA]
            SPL_broadband_1_3_spectrum     - 1/3 octave band broadband contribution to total SPL [dB] 
            SPL_harmonic_1_3_spectrum      - 1/3 octave band harmonic contribution to total SPL  [dB]
            SPL_harmonic_bpf_spectrum_dBA  - A-weighted blade passing freqency spectrum of 
                                             harmonic compoment of SPL                           [dB]
            SPL_harmonic_bpf_spectrum      - blade passing freqency spectrum of harmonic
                                             compoment of SPL                                    [dB] 
     
    Properties Used:
        N/A   
    '''
 
    # unpack 
    conditions           = segment.state.conditions
    propulsor_conditions = conditions.energy[propulsor.tag]
    harmonics_blade      = settings.harmonics
    harmonics_load       = np.linspace(0,5,6).astype(int)  
    num_mic              = len(microphone_locations[:,0]) 
    num_cpt              = conditions._size
    num_f                = len(settings.center_frequencies)
      
    # create data structures for computation
    Noise   = Data()  
    Results = Data()

    Results.SPL                                           = np.zeros((num_cpt,num_mic))
    Results.SPL_dBA                                       = np.zeros_like(Results.SPL)
    Results.SPL_harmonic                                  = np.zeros_like(Results.SPL)
    Results.SPL_broadband                                 = np.zeros_like(Results.SPL)
    Results.blade_passing_frequencies                     = np.zeros(num_f)
    Results.SPL_1_3_spectrum                              = np.zeros((num_cpt,num_mic,num_f)) 
    Results.SPL_harmonic_bpf_spectrum                     = np.zeros_like(Results.SPL_1_3_spectrum)
    Results.SPL_harmonic_bpf_spectrum_dBA                 = np.zeros_like(Results.SPL_1_3_spectrum)
    Results.one_third_frequency_spectrum                  = settings.center_frequencies 
    Results.SPL_1_3_spectrum                              = np.zeros_like(Results.SPL_1_3_spectrum)
    Results.SPL_1_3_spectrum_dBA                          = np.zeros_like(Results.SPL_1_3_spectrum)
    Results.SPL_harmonic_1_3_spectrum                     = np.zeros_like(Results.SPL_1_3_spectrum)
    Results.SPL_harmonic_1_3_spectrum_dBA                 = np.zeros_like(Results.SPL_1_3_spectrum)
    Results.SPL_broadband_1_3_spectrum                    = np.zeros_like(Results.SPL_1_3_spectrum)
    Results.SPL_broadband_1_3_spectrum_dBA                = np.zeros_like(Results.SPL_1_3_spectrum)

    # compute position vector from point source (or should it be origin) at rotor hub to microphones 
    coordinates   = compute_rotor_point_source_coordinates(propulsor,rotor,conditions,microphone_locations,settings)        

    for cpt in range(num_cpt): 
        # ----------------------------------------------------------------------------------
        # Harmonic Noise
        # ---------------------------------------------------------------------------------- 
        # harmonic noise with planar load distribution
        if settings.fidelity == 'plane_source': 
            aeroacoustic_data = propulsor_conditions[rotor.tag]       
            Re                = aeroacoustic_data.disc_reynolds_number
            AOA_sec           = aeroacoustic_data.disc_effective_angle_of_attack  
            a_loc             = rotor.airfoil_polar_stations
            num_az            = aeroacoustic_data.number_azimuthal_stations     
            airfoils          = rotor.airfoils         
            for jj,airfoil in enumerate(airfoils):
                airfoil_points      = airfoil.number_of_points 
            chord_coord             = int(np.floor(airfoil_points/2))       
                
            if (identical_propulsors == False) and rotor_index !=0: 
                prev_aeroacoustic_data                   = propulsor_conditions[previous_rotor_tag]                 
                prev_aeroacoustic_data                   = propulsor_conditions[rotor.tag]  
                aeroacoustic_data.disc_lift_distribution = prev_aeroacoustic_data.disc_lift_distribution
                aeroacoustic_data.disc_drag_distribution = prev_aeroacoustic_data.disc_lift_distribution
                aeroacoustic_data.disc_lift_coefficient  = prev_aeroacoustic_data.disc_lift_coefficient 
                aeroacoustic_data.disc_drag_coefficient  = prev_aeroacoustic_data.disc_drag_coefficient  
                aeroacoustic_data.blade_upper_surface    = prev_aeroacoustic_data.blade_upper_surface
                aeroacoustic_data.blade_lower_surface    = prev_aeroacoustic_data.blade_lower_surface
            else: 
                # Lift and Drag - coefficients and distributions 
                fL      = np.tile(np.zeros_like(Re)[:,:,:,None],(1,1,1,chord_coord))
                fD      = np.zeros_like(fL)
                CL      = np.zeros_like(Re)
                CD      = np.zeros_like(Re) 
                y_up    = np.zeros_like(fL)
                y_low   = np.zeros_like(fL)
                                  
                for jj,airfoil in enumerate(airfoils):    
                    locs                  = np.where(np.array(a_loc) == jj ) 
                    alpha_azi             = np.atleast_2d(AOA_sec[cpt,locs,:].flatten())
                    Re_azi                = np.atleast_2d(Re[cpt,locs,:].flatten())      
                    pd                    = airfoil.polars 
                    if settings.use_plane_loading_surrogate: 
                        fL[cpt,locs,:,:]      = pd.lift_distribution_func((alpha_azi,Re_azi)).reshape(1,len(a_loc), num_az,chord_coord) 
                        fD[cpt,locs,:,:]      = pd.drag_distribution_func((alpha_azi,Re_azi)).reshape(1,len(a_loc), num_az,chord_coord)  
                        cl_invisc             = interp2d(Re_azi,alpha_azi,pd.reynolds_numbers, pd.angle_of_attacks, pd.lift_coefficients)
                        cd_visc               = interp2d(Re_azi,alpha_azi,pd.reynolds_numbers, pd.angle_of_attacks, pd.drag_coefficients)   
                        CL[cpt,locs,:]        = cl_invisc.reshape(1, len(a_loc), num_az) 
                        CD[cpt,locs,:]        = cd_visc.reshape(1, len(a_loc), num_az)                             
                    
                    else : 
                        airfoil_geometry      = import_airfoil_geometry(airfoil.coordinate_file,airfoil_points)
                        airfoil_properties    = airfoil_analysis(airfoil_geometry,alpha_azi,Re_azi)
                        fL[cpt,locs,:,:]      = airfoil_properties.fL.reshape(chord_coord, len(a_loc), num_az,1).swapaxes(0, 3)
                        fD[cpt,locs,:,:]      = airfoil_properties.fD.reshape(chord_coord, len(a_loc), num_az,1).swapaxes(0, 3)
                        CL[cpt,locs,:]        = airfoil_properties.cl_invisc.reshape(1, len(a_loc), num_az) 
                        CD[cpt,locs,:]        = airfoil_properties.cd_visc.reshape(1, len(a_loc), num_az) 
                        
                    y_up[cpt,locs,:,:]    = airfoil.geometry.y_upper_surface
                    y_low[cpt,locs,:,:]   = airfoil.geometry.y_lower_surface
                        
                aeroacoustic_data.disc_lift_distribution = fL
                aeroacoustic_data.disc_drag_distribution = fD
                aeroacoustic_data.disc_lift_coefficient  = CL
                aeroacoustic_data.disc_drag_coefficient  = CD 
                aeroacoustic_data.blade_upper_surface    = y_up
                aeroacoustic_data.blade_lower_surface    = y_low                        
                        
            harmonic_noise_plane(harmonics_blade,harmonics_load,conditions,propulsor_conditions,coordinates,rotor,settings,Noise,cpt)
        elif settings.fidelity == 'line_source': 
            harmonic_noise_line(harmonics_blade,harmonics_load,conditions,propulsor_conditions,coordinates,rotor,settings,Noise,cpt)
        else:
            harmonic_noise_point(harmonics_blade,harmonics_load,conditions,propulsor_conditions,coordinates,rotor,settings,Noise,cpt) 
    
        # ----------------------------------------------------------------------------------    
        # Broadband Noise
        # ---------------------------------------------------------------------------------- 
        broadband_noise(conditions,propulsor_conditions,coordinates,rotor,settings,Noise,cpt)  
    
        # ----------------------------------------------------------------------------------    
        # Atmospheric attenuation 
        # ----------------------------------------------------------------------------------
        delta_atmo = atmospheric_attenuation(np.linalg.norm(coordinates.X_r[:,0,0,0,:],axis=1),settings.center_frequencies)
    
        # ----------------------------------------------------------------------------------    
        # Combine Harmonic (periodic/tonal) and Broadband Noise
        # ----------------------------------------------------------------------------------
        num_mic      = len(coordinates.X_hub[0,:,0,0])
        SPL_total_1_3_spectrum      = 10*np.log10( 10**(Noise.SPL_prop_harmonic_1_3_spectrum/10) + 10**(Noise.SPL_prop_broadband_1_3_spectrum/10)) - np.tile(delta_atmo[cpt,None,:],(1,num_mic,1))  
        SPL_total_1_3_spectrum[np.isnan(SPL_total_1_3_spectrum)] = 0 
    
        # ----------------------------------------------------------------------------------
        # Summation of spectra from propellers into one SPL and store results
        # ----------------------------------------------------------------------------------
        Results.SPL[cpt,:]                                 = SPL_arithmetic(SPL_total_1_3_spectrum[0], sum_axis=1) 
        Results.SPL_dBA[cpt,:]                             = SPL_arithmetic(A_weighting_metric(SPL_total_1_3_spectrum[0],settings.center_frequencies), sum_axis=1) 
        Results.SPL_harmonic[cpt,:]                        = SPL_arithmetic(Noise.SPL_prop_harmonic_1_3_spectrum[0], sum_axis=1)
        Results.SPL_broadband[cpt,:]                       = SPL_arithmetic(Noise.SPL_prop_broadband_1_3_spectrum[0], sum_axis=1) 
          
        # blade passing frequency   
        Results.blade_passing_frequencies                  = Noise.f          
        Results.SPL_harmonic_bpf_spectrum[cpt,:,:]         = Noise.SPL_prop_harmonic_bpf_spectrum 
        Results.SPL_harmonic_bpf_spectrum_dBA[cpt,:,:]     = A_weighting_metric(Results.SPL_harmonic_bpf_spectrum[cpt,:,:],Noise.f) 
          
        # 1/3 octave band   
        Results.SPL_1_3_spectrum[cpt,:,:]                  = SPL_total_1_3_spectrum 
        Results.SPL_1_3_spectrum_dBA[cpt,:,:]              = A_weighting_metric(Results.SPL_1_3_spectrum[cpt,:,:],settings.center_frequencies)      
        Results.SPL_harmonic_1_3_spectrum[cpt,:,:]         = Noise.SPL_prop_harmonic_1_3_spectrum 
        Results.SPL_harmonic_1_3_spectrum_dBA[cpt,:,:]     = A_weighting_metric(Results.SPL_harmonic_1_3_spectrum[cpt,:,:],settings.center_frequencies) 
        Results.SPL_broadband_1_3_spectrum[cpt,:,:]        = Noise.SPL_prop_broadband_1_3_spectrum 
        Results.SPL_broadband_1_3_spectrum_dBA[cpt,:,:]    = A_weighting_metric(Results.SPL_broadband_1_3_spectrum[cpt,:,:],settings.center_frequencies) 
    
    # A-weighted
    conditions.noise[propulsor.tag][rotor.tag] = Results 
    return rotor.tag 
