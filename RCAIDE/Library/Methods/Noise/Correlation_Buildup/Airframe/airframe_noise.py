# RCAIDE/Methods/Noise/Correlation_Buildup/Airframe/airframe_noise.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE Imports
import  RCAIDE
from RCAIDE.Framework.Core                 import Data  
from .clean_wing_noise                     import clean_wing_noise
from .landing_gear_noise                   import landing_gear_noise 
from .trailing_edge_flap_noise             import trailing_edge_flap_noise 
from RCAIDE.Library.Methods.Noise.Metrics  import A_weighting_metric  
from RCAIDE.Library.Methods.Noise.Common   import SPL_arithmetic 

# python imports 
import numpy as np

# ----------------------------------------------------------------------
#  Airframe Noise 
# ----------------------------------------------------------------------
def airframe_noise(microphone_locations, segment, config, settings):
    """
    This computes the noise from different sources of the airframe for a given vehicle for a constant altitude flight.

    Parameters
    ----------
    microphone_locations : array_like
        Coordinates of the microphones used to capture noise data.
    segment : RCAIDE type segment
        Contains flight path data and conditions.
            - conditions : object
                Contains freestream velocity, kinematic viscosity, and mach number.
            - frames : object
                Contains inertial time data.
    config : RCAIDE type config
        Configuration of the vehicle including wings and landing gears.
            - wings : list
                List of wing objects with attributes like taper, areas, spans, and control surfaces.
            - landing_gears : list
                List of landing gear objects with attributes like tire diameter, strut length, and gear status.
    settings : object
        Contains settings such as center frequencies for noise calculations.

    Returns
    -------
    airframe_noise : Data
        Contains the computed noise data.
            - SPL : float
                Sound Pressure Level.
            - SPL_1_3_spectrum : array_like
                One Third Octave Band SPL spectrum.
            - SPL_dBA : float
                A-weighted Sound Pressure Level.
            - noise_time : array_like
                Time discretization of the noise data.

    Notes
    -----
    The function assumes a correlation-based noise computation method. It uses the noise component method as described
    by Fink (1979) to calculate the noise from various airframe components.

    **Major Assumptions**
        * Constant altitude flight
        * Correlation-based noise computation

    **Theory**

    The noise is computed using the noise component method, which involves calculating the noise from individual
    components like wings, tails, flaps, and landing gears, and then summing them incoherently.

    **Definitions**

    'SPL'
        Sound Pressure Level, a measure of the sound intensity.

    References
    ----------
    [1] Fink, Martin R. "Noise component method for airframe noise." Journal of aircraft 16.10 (1979): 659-665.

    See Also
    --------
    RCAIDE.Library.Methods.Noise.Metrics.A_weighting_metric
    RCAIDE.Library.Methods.Noise.Common.SPL_arithmetic
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
    for wing in config.wings:
        if (type(wing) == RCAIDE.Library.Components.Wings.Main_Wing) or  (type(wing) == RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            taper = wing.taper 
            Sw    = wing.areas.reference                
            bw    = wing.spans.projected               
            for cs in  wing.control_surfaces:  
                if type(cs) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap: 
                    deltaf                  = cs.deflection
                    flap_span               = (cs.span_fraction_end - cs.span_fraction_start) * bw
                    chord_root              = 2*Sw/bw/(1+taper) 
                    chord_tip               = taper * chord_root
                    delta_chord             = chord_tip - chord_root 
                    wing_chord_flap_start   = chord_root + delta_chord * cs.span_fraction_start 
                    wing_chord_flap_end     = chord_root + delta_chord * cs.span_fraction_end
                    flap_chord_start        = wing_chord_flap_start* cs.chord_fraction 
                    flap_chord_end          = wing_chord_flap_end* cs.chord_fraction   
                    cf                      = (flap_chord_start +flap_chord_end) /2  
                    Sf                      = flap_span * cf
                
                    # determining flap slot number
                    if cs.configuration_type   == 'single_slotted':
                        slots = 1
                    elif cs.configuration_type == 'double_slotted':
                        slots = 2
                    elif cs.configuration_type == 'triple_slotted':
                        slots = 3  
        elif type(wing) == RCAIDE.Library.Components.Wings.Horizontal_Tail: 
            Sht                     = wing.areas.reference    # horizontal tail area, sq.ft
            bht                     = wing.spans.projected    # horizontal tail span, ft
        elif type(wing) == RCAIDE.Library.Components.Wings.Vertical_Tail:  
            Svt                     = wing.areas.reference     # vertical tail area, sq.ft
            bvt                     = wing.spans.projected     # vertical tail span, ft
    
     
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
            Dp                 = landing_gear.tire_diameter           # MLG tyre diameter 
            Dn                 = landing_gear.strut_length            # NLG tyre diameter 
            main_wheels        = landing_gear.wheels                          # Number of wheels   
            main_gear_extended = landing_gear.gear_extended                   # Gear up or gear down 
            main_units         = landing_gear.units                           # Number of main units
        elif isinstance(landing_gear,RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear):
            Hp                 = landing_gear.tire_diameter                 # MLG strut length 
            Hn                 = landing_gear.strut_length                  # NLG strut length 
            nose_gear_extended = landing_gear.gear_extended                   # Gear up or gear down 
            nose_wheels        = landing_gear.wheels                          # Number of wheels   
      
    
    
    viscosity           = segment.conditions.freestream.kinematic_viscosity[:,0] 
    M                   = segment.conditions.freestream.mach_number 
    SPL_total_history   = np.zeros((n_cpts,n_mic,num_f)) 
    SPLt_dBA_history    = np.zeros((n_cpts,n_mic,num_f))   

    # Distance vector from the aircraft position in relation to the microphone coordinates [meters]
    distance          = np.linalg.norm(microphone_locations,axis = 1)
    
    altitude          = abs(microphone_locations[:,2])
    sideline_distance =  microphone_locations[:,1] 
    
    # Polar angle emission vector relatively to the aircraft to the microphone coordinates, [rad] 

    theta     =  np.zeros(n_mic)
    bool_1    = (microphone_locations[:,1] > 0) &  (microphone_locations[:,0] > 0)
    bool_2    = (microphone_locations[:,1] > 0) &  (microphone_locations[:,0] < 0)
    bool_3    = (microphone_locations[:,1] < 0) &  (microphone_locations[:,0] < 0)
    bool_4    = (microphone_locations[:,1] < 0) &  (microphone_locations[:,0] > 0)
    
    theta[bool_1] =  np.pi - np.arctan(microphone_locations[:,1]/microphone_locations[:,0])[bool_1]
    theta[bool_2] =  np.arctan(microphone_locations[:,1]/ abs(microphone_locations[:,0]))[bool_2]
    theta[bool_3] =  np.arctan(abs(microphone_locations[:,1])/ abs(microphone_locations[:,0]))[bool_3]
    theta[bool_4] =  np.pi - np.arctan(abs(microphone_locations[:,1])/ microphone_locations[:,0])[bool_4]
    
     # Azimuthal (sideline) angle emission vector relatively to the aircraft to the microphone coordinates, [rad] 
    phi   = np.arctan(sideline_distance/altitude)
    
    
    # START LOOP FOR EACH POSITION OF AIRCRAFT   
    for i in range(n_cpts):
        for j in range(n_mic):  
             
            SPL_wing = clean_wing_noise(Sw,bw,0,1, velocity[i,0],viscosity[i],M[i],phi[j],theta[j],distance[j],frequency)      # Wing Noise
            SPLht    = clean_wing_noise(Sht,bht,0,1, velocity[i,0],viscosity[i],M[i],phi[j],theta[j],distance[j],frequency)     # Horizontal Tail Noise
            SPLvt    = clean_wing_noise(Svt,bvt,0,0,velocity[i,0],viscosity[i],M[i],phi[j],theta[j],distance[j],frequency)     # Vertical Tail Noise
      
            # Flap noise 
            if deltaf==0:
                SPL_flap = np.zeros(num_f)
            else:
                SPL_flap = trailing_edge_flap_noise(Sf,cf,deltaf,slots,velocity[i,0],M[i],phi[j],theta[j],distance[j],frequency)  
    
            # Main landing gear noise     
            if main_gear_extended == False:  
                SPL_main_landing_gear = np.zeros(num_f)
            else:
                SPL_main_landing_gear = landing_gear_noise(Dp,Hp,main_wheels,M[i],velocity[i,0],phi[j],theta[j],distance[j],frequency)   
                if main_units>1: # Incoherent summation of each main landing gear unit
                    SPL_main_landing_gear = SPL_main_landing_gear+3*(main_units-1)      
                
            # Nose landing gear noise
            if nose_gear_extended == False:                  
                SPL_nose_landing_gear = np.zeros(num_f)
            else:
                SPL_nose_landing_gear = landing_gear_noise(Dn,Hn,nose_wheels,M[i],velocity[i,0],phi[j],theta[j],distance[j],frequency)   
               
            # Total Airframe Noise
            SPL_total = 10.*np.log10( 10.0**(0.1*SPL_wing)+ 10.0**(0.1*SPLht) + 10.0**(0.1*SPLvt) + 10.0**(0.1*SPL_flap) + 10.0**(0.1*SPL_main_landing_gear)+ 10.0**(0.1*SPL_nose_landing_gear))      
                
            SPL_total_history[i,j,:]             = SPL_total  
            
            # Calculation of dBA based on the sound pressure time history 
            SPLt_dBA_history[i,j,:] = A_weighting_metric(SPL_total,frequency) 
    
    # Pack Airframe Noise 
    airframe_noise                        = Data()  
    airframe_noise.SPL                    = SPL_arithmetic(SPL_total_history, sum_axis= 2)
    airframe_noise.SPL_1_3_spectrum       = SPL_total_history
    airframe_noise.SPL_dBA                = SPL_arithmetic(np.atleast_2d(SPLt_dBA_history), sum_axis= 2) 
    airframe_noise.noise_time             = noise_time 
    return airframe_noise
