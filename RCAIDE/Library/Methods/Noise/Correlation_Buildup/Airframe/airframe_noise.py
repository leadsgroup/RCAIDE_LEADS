# RCAIDE/Methods/Noise/Correlation_Buildup/Airframe/airframe_noise.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE Imports
import  RCAIDE
from RCAIDE.Framework.Core                      import Data , Units 
from .clean_wing_noise             import clean_wing_noise
from .landing_gear_noise           import landing_gear_noise
from .leading_edge_slat_noise      import leading_edge_slat_noise
from .trailing_edge_flap_noise     import trailing_edge_flap_noise 
from RCAIDE.Library.Methods.Noise.Metrics  import PNL_noise_metric
from RCAIDE.Library.Methods.Noise.Metrics  import EPNL_noise_metric 
from RCAIDE.Library.Methods.Noise.Metrics  import A_weighting_metric 
from RCAIDE.Library.Methods.Noise.Common   import noise_tone_correction
from RCAIDE.Library.Methods.Noise.Common   import atmospheric_attenuation
from RCAIDE.Library.Methods.Noise.Common   import SPL_arithmetic 

# python imports 
import numpy as np

# ----------------------------------------------------------------------
#  Airframe Noise 
# ----------------------------------------------------------------------
def airframe_noise(microphone_locations,segment,config,settings):
    """ This computes the noise from different sources of the airframe for a given vehicle for a constant altitude flight. 

    Assumptions:
        Correlation based 
 
    Source:
        Fink, Martin R. Airframe noise prediction method. No. UTRC/R77-912607-11. UNITED 
        TECHNOLOGIES RESEARCH CENTER EAST HARTFORD CT, 1977.  
               
    Inputs:
        vehicle	 - RCAIDE type vehicle

        includes these fields:
            S                          - Wing Area
            bw                         - Wing Span
            Sht                        - Horizontal tail area
            bht                        - Horizontal tail span
            Svt                        - Vertical tail area
            bvt                        - Vertical tail span
            deltaf                     - Flap deflection
            Sf                         - Flap area
            cf                         - Flap chord
            slots                      - Number of slots (Flap type)
            Dp                         - Main landing gear tyre diameter
            Hp                         - Main lading gear strut length
            Dn                         - Nose landing gear tyre diameter
            Hn                         - Nose landing gear strut length
            wheels                     - Number of wheels 
            
        noise segment - flight path data, containing:
            distance_vector             - distance from the source location to observer
            angle                       - polar angle from the source to the observer
            phi                         - azimuthal angle from the source to the observer


    Outputs: One Third Octave Band SPL [dB]
        SPL_wing                        - Sound Pressure Level of the clean wing
        SPLht                           - Sound Pressure Level of the horizontal tail
        SPLvt                           - Sound Pressure Level of the vertical tail
        SPL_flap                        - Sound Pressure Level of the flaps trailing edge
        SPL_slat                        - Sound Pressure Level of the slat leading edge
        SPL_main_landing_gear           - Sound Pressure Level og the main landing gear
        SPL_nose_landing_gear           - Sound Pressure Level of the nose landing gear

    Properties Used:
        N/A      
        
    """
    # Unpack conditions 
    velocity     = segment.conditions.freestream.velocity                  # aircraft velocity  
    noise_time   = segment.conditions.frames.inertial.time[:,0]            # time discretization

    # Generate array with the One Third Octave Band Center Frequencies
    frequency = settings.center_frequencies[5:]  
    num_f     = len(frequency) 
    n_cpts    = len(noise_time)  
    n_mic     = len(microphone_locations)
    
    # Unpack Geometry  
    slots      = 0
    deltas     = 0
    for wing in config.wings:
        if type(wing) == RCAIDE.Library.Components.Wings.Main_Wing:
            taper = wing.taper 
            Sw    = wing.areas.reference  / (Units.ft)**2              # wing area, sq.ft
            bw    = wing.spans.projected / Units.ft                    # wing span, ft
            for cs in  wing.control_surfaces: 
                if type(cs) == RCAIDE.Library.Components.Wings.Control_Surfaces.Slat:
                    deltas                  = cs.deflection  
                if type(cs) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap: 
                    deltaf                  = cs.deflection                                             
                    chord_root              = 2*Sw/bw/(1+taper) 
                    chord_tip               = taper * chord_root
                    delta_chord             = chord_tip - chord_root 
                    wing_chord_flap_start   = chord_root + delta_chord * cs.span_fraction_start 
                    wing_chord_flap_end     = chord_root + delta_chord * cs.span_fraction_end
                    wing_mac_flap           = 2./3.*( wing_chord_flap_start+wing_chord_flap_end - wing_chord_flap_start*wing_chord_flap_end/   (wing_chord_flap_start+wing_chord_flap_end) ) 
                    flap_chord_start        = wing_chord_flap_start * cs.chord_fraction
                    flap_chord_end          = wing_chord_flap_end * cs.chord_fraction
                    Sf                      = (flap_chord_start + flap_chord_end) * (cs.span_fraction_end- cs.span_fraction_start)*bw / 2.  
                    cf                      = wing_mac_flap * cs.chord_fraction 
                
                    # determining flap slot number
                    if cs.configuration_type   == 'single_slotted':
                        slots = 1
                    elif cs.configuration_type == 'double_slotted':
                        slots = 2
                    elif cs.configuration_type == 'triple_slotted':
                        slots = 3  
        elif type(wing) == RCAIDE.Library.Components.Wings.Horizontal_Tail: 
            Sht                     = wing.areas.reference / (Units.ft)**2   # horizontal tail area, sq.ft
            bht                     = wing.spans.projected / Units.ft        # horizontal tail span, ft
        elif type(wing) == RCAIDE.Library.Components.Wings.Vertical_Tail:  
            Svt                     = wing.areas.reference / (Units.ft)**2     # vertical tail area, sq.ft
            bvt                     = wing.spans.projected  / Units.ft         # vertical tail span, ft
    
     
    Dp                 = 0
    Dn                 = 0
    main_wheels        = 0
    main_units         = 0
    Hp                 = 0
    Hn                 = 0 
    nose_wheels        = 0
    main_gear_extended = False
    nose_gear_extended = False
    
    for landing_gear in  config.landing_gears:
        if isinstance(landing_gear,RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear):
            Dp                 = landing_gear.tire_diameter  / Units.ft           # MLG tyre diameter, ft
            Dn                 = landing_gear.strut_length   / Units.ft           # NLG tyre diameter, ft
            main_wheels        = landing_gear.wheels                          # Number of wheels   
            main_gear_extended = landing_gear.gear_extended                   # Gear up or gear down 
            main_units         = landing_gear.units                           # Number of main units
        elif isinstance(landing_gear,RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear):
            Hp                 = landing_gear.tire_diameter  / Units.ft           # MLG strut length, ft
            Hn                 = landing_gear.strut_length   / Units.ft           # NLG strut length, ft
            nose_gear_extended = landing_gear.gear_extended                   # Gear up or gear down 
            nose_wheels        = landing_gear.wheels                          # Number of wheels   
      
    # Geometric information from the source to observer position  
    distance_vector     = np.linalg.norm(microphone_locations,axis = 1)
    thetas              = np.tile(np.arctan2(microphone_locations[:, 1], microphone_locations[:, 0])[None,:],(n_cpts,1)) 
    phis                = np.tile(np.arctan2(np.sqrt(np.square(microphone_locations[:, 0]) + np.square(microphone_locations[:, 1])),  microphone_locations[:, 2])[None,:],(n_cpts,1))  
    viscosity           = segment.conditions.freestream.dynamic_viscosity[:,0]*Units.ft*Units.ft  
    M                   = segment.conditions.freestream.mach_number
     
    SPL_wing_history              = np.zeros((n_cpts,n_mic,num_f))
    SPLht_history                 = np.zeros((n_cpts,n_mic,num_f))
    SPLvt_history                 = np.zeros((n_cpts,n_mic,num_f))
    SPL_flap_history              = np.zeros((n_cpts,n_mic,num_f))
    SPL_slat_history              = np.zeros((n_cpts,n_mic,num_f))
    SPL_main_landing_gear_history = np.zeros((n_cpts,n_mic,num_f))
    SPL_nose_landing_gear_history = np.zeros((n_cpts,n_mic,num_f))
    SPL_total_history             = np.zeros((n_cpts,n_mic,num_f)) 
    SPLt_dBA_history              = np.zeros((n_cpts,n_mic,num_f))  
    SPLt_dBA_max                  = np.zeros((n_cpts,n_mic))    
    
    # START LOOP FOR EACH POSITION OF AIRCRAFT   
    for i in range(n_cpts):
        for j in range(n_mic): 
            
            # Emission angle
            theta = thetas[i,j]
            phi   = phis[i,j]
            
            # Distance from airplane to observer, evaluated at retarded time
            distance = distance_vector[j]    
           
            # Atmospheric attenuation
            delta_atmo=atmospheric_attenuation(np.array([distance]),frequency)[:,0]
    
            # Call each noise source model
            SPL_wing = clean_wing_noise(Sw,bw,0,1,velocity[i,0],viscosity[i],M[i],phi,theta,distance,frequency) - delta_atmo    #Wing Noise
            SPLht    = clean_wing_noise(Sht,bht,0,1,velocity[i,0],viscosity[i],M[i],phi,theta,distance,frequency)  -delta_atmo    #Horizontal Tail Noise
            SPLvt    = clean_wing_noise(Svt,bvt,0,0,velocity[i,0],viscosity[i],M[i],phi,theta,distance,frequency)  -delta_atmo    #Vertical Tail Noise
     
            # Slat noise  
            if deltas == 0: 
                SPL_slat = np.zeros(num_f)
            else:
                SPL_slat = leading_edge_slat_noise(SPL_wing,Sw,bw,velocity[i,0],viscosity[i],M[i],phi,theta,distance,frequency) -delta_atmo     
            
            # Flap noise 
            if deltaf==0:
                SPL_flap = np.zeros(num_f)
            else:
                SPL_flap = trailing_edge_flap_noise(Sf,cf,deltaf,slots,velocity[i,0],M[i],phi,theta,distance,frequency) - delta_atmo  
    
            # Main landing gear noise     
            if main_gear_extended == False:  
                SPL_main_landing_gear = np.zeros(num_f)
            else:
                SPL_main_landing_gear = landing_gear_noise(Dp,Hp,main_wheels,M[i],velocity[i,0],phi,theta,distance,frequency)  - delta_atmo
                if main_units>1: # Incoherent summation of each main landing gear unit
                    SPL_main_landing_gear = SPL_main_landing_gear+3*(main_units-1)      
                
            # Nose landing gear noise
            if nose_gear_extended == False:                  
                SPL_nose_landing_gear = np.zeros(num_f)
            else:
                SPL_nose_landing_gear = landing_gear_noise(Dn,Hn,nose_wheels,M[i],velocity[i,0],phi,theta,distance,frequency)  - delta_atmo  
               
            # Total Airframe Noise
            SPL_total = 10.*np.log10(10.0**(0.1*SPL_wing)+10.0**(0.1*SPLht)+10**(0.1*SPL_flap)+ \
                 10.0**(0.1*SPL_slat)+10.0**(0.1*SPL_main_landing_gear)+10.0**(0.1*SPL_nose_landing_gear))
                
            SPL_total_history[i,j,:]             = SPL_total 
            SPL_wing_history[i,j,:]              = SPL_wing 
            SPLvt_history[i,j,:]                 = SPLvt 
            SPLht_history[i,j,:]                 = SPLht 
            SPL_flap_history[i,j,:]              = SPL_flap 
            SPL_slat_history[i,j,:]              = SPL_slat 
            SPL_nose_landing_gear_history[i,j,:] = SPL_nose_landing_gear 
            SPL_main_landing_gear_history[i,j,:] = SPL_main_landing_gear  
            
            # Calculation of dBA based on the sound pressure time history 
            SPLt_dBA_history[i,j,:] = A_weighting_metric(SPL_total,frequency)
            SPLt_dBA_max[i,j]       = np.max(A_weighting_metric(SPL_total,frequency))    
              
    # Calculation of the Perceived Noise Level EPNL based on the sound time history 
    PNL_total             = PNL_noise_metric(SPL_total_history)
    PNL_wing              = PNL_noise_metric(SPL_wing_history)
    PNL_ht                = PNL_noise_metric(SPLht_history)
    PNL_vt                = PNL_noise_metric(SPLvt_history)
    PNL_nose_landing_gear = PNL_noise_metric(SPL_nose_landing_gear_history)
    PNL_main_landing_gear = PNL_noise_metric(SPL_main_landing_gear_history)
    PNL_slat              = PNL_noise_metric(SPL_slat_history)
    PNL_flap              = PNL_noise_metric(SPL_flap_history)
     
    # Calculation of the tones corrections on the SPL for each component and total
    tone_correction_total             = noise_tone_correction(SPL_total_history) 
    tone_correction_wing              = noise_tone_correction(SPL_wing_history)
    tone_correction_ht                = noise_tone_correction(SPLht_history)
    tone_correction_vt                = noise_tone_correction(SPLvt_history)
    tone_correction_flap              = noise_tone_correction(SPL_flap_history)
    tone_correction_slat              = noise_tone_correction(SPL_slat_history)
    tone_correction_nose_landing_gear = noise_tone_correction(SPL_nose_landing_gear_history)
    tone_correction_main_landing_gear = noise_tone_correction(SPL_main_landing_gear_history)
    
    # Calculation of the PLNT for each component and total
    PNLT_total             = PNL_total+tone_correction_total
    PNLT_wing              = PNL_wing+tone_correction_wing
    PNLT_ht                = PNL_ht+tone_correction_ht
    PNLT_vt                = PNL_vt+tone_correction_vt
    PNLT_nose_landing_gear = PNL_nose_landing_gear+tone_correction_nose_landing_gear
    PNLT_main_landing_gear = PNL_main_landing_gear+tone_correction_main_landing_gear
    PNLT_slat              = PNL_slat+tone_correction_slat
    PNLT_flap              = PNL_flap+tone_correction_flap
    
    #Calculation of the EPNL for each component and total
    EPNL_total             = EPNL_noise_metric(PNLT_total)
    EPNL_wing              = EPNL_noise_metric(PNLT_wing)
    EPNL_ht                = EPNL_noise_metric(PNLT_ht)
    EPNL_vt                = EPNL_noise_metric(PNLT_vt)    
    EPNL_nose_landing_gear = EPNL_noise_metric(PNLT_nose_landing_gear)
    EPNL_main_landing_gear = EPNL_noise_metric(PNLT_main_landing_gear)
    EPNL_slat              = EPNL_noise_metric(PNLT_slat)
    EPNL_flap              = EPNL_noise_metric(PNLT_flap) 
    
    # Pack Airframe Noise 
    airframe_noise                        = Data()
    airframe_noise.EPNL_total             = EPNL_total
    airframe_noise.EPNL_wing              = EPNL_wing              
    airframe_noise.EPNL_ht                = EPNL_ht                
    airframe_noise.EPNL_vt                = EPNL_vt                
    airframe_noise.EPNL_nose_landing_gear = EPNL_nose_landing_gear 
    airframe_noise.EPNL_main_landing_gear = EPNL_main_landing_gear 
    airframe_noise.EPNL_slat              = EPNL_slat              
    airframe_noise.EPNL_flap              = EPNL_flap    
    airframe_noise.SPL                    = SPL_arithmetic(SPL_total_history, sum_axis= 2)
    airframe_noise.SPL_1_3_spectrum       = SPL_total_history
    airframe_noise.SPL_dBA                = SPL_arithmetic(np.atleast_2d(SPLt_dBA_history), sum_axis= 2) 
    airframe_noise.noise_time             = noise_time 
    return airframe_noise
