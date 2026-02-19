# RCAIDE/Methods/Aeroacoustics/Semi_Empirical/Turbofan/turbofan_engine_noise.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                        import Units , Data  
from .angle_of_attack_effect                      import angle_of_attack_effect
from .external_plug_effect                        import external_plug_effect
from .ground_proximity_effect                     import ground_proximity_effect
from .jet_installation_effect                     import jet_installation_effect
from .mixed_noise_component                       import mixed_noise_component 
from .primary_noise_component                     import primary_noise_component
from .secondary_noise_component                   import secondary_noise_component 
from RCAIDE.Library.Methods.Aeroacoustics.Common  import SPL_arithmetic 
from RCAIDE.Library.Methods.Aeroacoustics.Metrics import A_weighting_metric  

# Python package imports   
import numpy as np   
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------     
#  turbofan engine noise 
# ----------------------------------------------------------------------------------------------------------------------         
def turbofan_engine_noise(microphone_locations, turbofan, aeroacoustic_data, segment, settings):
    """
    This method predicts the free-field 1/3 Octave Band SPL of coaxial subsonic jets for turbofan engines under various conditions.

    Parameters
    ----------
    microphone_locations : array_like
        Coordinates of the microphones used to capture noise data.
    turbofan : RCAIDE type turbofan
        Contains turbofan engine specifications.
            - core_nozzle : object
                Contains attributes like exit_velocity and diameter.
            - fan_nozzle : object
                Contains attributes like exit_velocity and diameter.
            - height : float
                Engine centerline height above the ground plane.
            - length : float
                Length of the turbofan.
            - diameter : float
                Diameter of the turbofan.
            - plug_diameter : float
                Diameter of the engine external plug.
            - geometry_xe, geometry_ye, geometry_Ce : float
                Geometric parameters for jet installation effects.
    aeroacoustic_data : object
        Contains aeroacoustic data for the turbofan.
            - core_nozzle : object
                Contains attributes like exit_stagnation_temperature and exit_stagnation_pressure.
            - fan_nozzle : object
                Contains attributes like exit_stagnation_temperature and exit_stagnation_pressure.
            - low_pressure_spool : object
                Contains angular_velocity.
    segment : RCAIDE type segment
        Contains flight path data and conditions.
            - conditions : object
                Contains freestream velocity, mach number, and speed of sound.
            - frames : object
                Contains inertial time data.
    settings : object
        Contains settings such as center frequencies for noise calculations.

    Returns
    -------
    engine_noise : Data
        Contains the computed noise data.
            - SPL_1_3_spectrum : array_like
                One Third Octave Band SPL spectrum.
            - SPL : float
                Sound Pressure Level.
            - SPL_dBA : float
                A-weighted Sound Pressure Level.

    Notes
    -----
    The function uses semi-empirical methods for coaxial jet noise prediction. It accounts for various conditions such as flyover, static, and in-flight scenarios.

    **Major Assumptions**
        * Coaxial subsonic jets
        * Free-field conditions

    **Theory**

    The noise prediction is based on the combination of primary, secondary, and mixed jet components, with adjustments for installation and environmental effects.

    **Definitions**

    'SPL'
        Sound Pressure Level, a measure of the sound intensity.

    References
    ----------
    [1] SAE ARP876D: Gas Turbine Jet Exhaust Noise Prediction (original)
    [2] de Almeida, Odenir. "Semi-empirical methods for coaxial jet noise prediction." (2008). (adapted)

    See Also
    --------
    RCAIDE.Library.Methods.Aeroacoustics.Metrics.A_weighting_metric
    RCAIDE.Library.Methods.Aeroacoustics.Common.SPL_arithmetic
    """
    # unpack   
    N1                     = aeroacoustic_data.fan.angular_velocity / Units.rpm
    Velocity_secondary     = aeroacoustic_data.fan_nozzle.exit_velocity   
    Temperature_secondary  = aeroacoustic_data.fan_nozzle.exit_stagnation_temperature 
    Pressure_secondary     = aeroacoustic_data.fan_nozzle.exit_stagnation_pressure 
    Velocity_primary       = aeroacoustic_data.core_nozzle.exit_velocity  
    Temperature_primary    = aeroacoustic_data.core_nozzle.exit_stagnation_temperature 
    Pressure_primary       = aeroacoustic_data.core_nozzle.exit_stagnation_pressure      
    Velocity_aircraft      = segment.conditions.freestream.velocity
    Mach_aircraft          = segment.conditions.freestream.mach_number 
    AOA                    = segment.conditions.aerodynamics.angles.alpha / Units.deg 
    noise_time             = segment.conditions.frames.inertial.time  
    distance_microphone    = np.linalg.norm(microphone_locations,axis = 1)    
    Diameter_primary       = turbofan.core_nozzle.diameter
    Diameter_secondary     = turbofan.fan_nozzle.diameter
    engine_height          = turbofan.origin[0][2] # This needs to be updated in a future PR
    EXA                    = turbofan.length /  turbofan.diameter 
    Plug_diameter          = turbofan.plug_diameter 
    Xe                     = turbofan.geometry_xe
    Ye                     = turbofan.geometry_ye
    Ce                     = turbofan.geometry_Ce 

    frequency              = settings.center_frequencies[5:]        
    n_cpts                 = len(noise_time)     
    n_freq                  = len(frequency) 
    n_mic                  = len(microphone_locations)
  
    # ============================================================================= 
    # Step 1: Computing atmospheric conditions
    # ============================================================================= 
    sound_ambient       = segment.conditions.freestream.speed_of_sound
    density_ambient     = segment.conditions.freestream.density  
    pressure_amb        = segment.conditions.freestream.pressure 
    pressure_isa        = 101325 # [Pa]
    R_gas               = 287.1  # [J/kg K]
    gamma_primary       = 1.37  # Corretion for the primary jet
    gamma               = 1.4 

    # ============================================================================= 
    # Step 2: Compute operating conditions and properties of jet     
    # ============================================================================= 
    # Calculation of nozzle areas
    Area_primary   =  np.pi*(Diameter_primary/2)**2 
    Area_secondary =  np.pi*(Diameter_secondary/2)**2   

    # Defining each array before the main loop 
    theta     =  np.zeros(n_mic)
    bool_1    = (microphone_locations[:,1] > 0) &  (microphone_locations[:,0] > 0)
    bool_2    = (microphone_locations[:,1] > 0) &  (microphone_locations[:,0] < 0)
    bool_3    = (microphone_locations[:,1] < 0) &  (microphone_locations[:,0] < 0)
    bool_4    = (microphone_locations[:,1] < 0) &  (microphone_locations[:,0] > 0)
    
    theta[bool_1] =  np.pi - np.arctan(microphone_locations[:,1]/microphone_locations[:,0])[bool_1]
    theta[bool_2] =  np.arctan(microphone_locations[:,1]/ abs(microphone_locations[:,0]))[bool_2]
    theta[bool_3] =  np.arctan(abs(microphone_locations[:,1])/ abs(microphone_locations[:,0]))[bool_3]
    theta[bool_4] =  np.pi - np.arctan(abs(microphone_locations[:,1])/ microphone_locations[:,0])[bool_4] 

    theta_P                = np.tile(theta[None,:],(n_cpts,1))  
    theta_S                = deepcopy(theta_P)
    theta_M                = deepcopy(theta_P)
    EX_p                   = np.zeros((n_cpts,n_mic,n_freq)) 
    EX_s                   = np.zeros((n_cpts,n_mic,n_freq)) 
    EX_m                   = np.zeros((n_cpts,n_mic,n_freq))  
    SPL_p                  = np.zeros((n_cpts,n_mic,n_freq)) 
    SPL_s                  = np.zeros((n_cpts,n_mic,n_freq)) 
    SPL_m                  = np.zeros((n_cpts,n_mic,n_freq)) 
    SPL                    = np.zeros((n_cpts,n_mic))
    SPL_dBA                = np.zeros((n_cpts,n_mic))
    SPL_1_3_spectrum       = np.zeros((n_cpts,n_mic,n_freq)) 
    SPL_1_3_spectrum_dBA   = np.zeros((n_cpts,n_mic,n_freq)) 

    # Primary and Secondary jets
    Cpp = R_gas/(1-1/gamma_primary)
    Cp  = R_gas/(1-1/gamma)
    
    # densitys 
    density_primary   = Pressure_primary/(R_gas*Temperature_primary-(0.5*R_gas*Velocity_primary**2/Cpp)) 
    density_secondary = Pressure_secondary/(R_gas*Temperature_secondary-(0.5*R_gas*Velocity_secondary**2/Cp))
    
    # mas sflow 
    mass_flow_primary   = Area_primary*Velocity_primary*density_primary 
    mass_flow_secondary = Area_secondary*Velocity_secondary*density_secondary

    # Mach number of the external flow - based on the aircraft velocity
    Mach_aircraft = Velocity_aircraft/sound_ambient

    # Calculation Procedure for the Mixed Jet Flow Parameters 
    Velocity_mixed    = (mass_flow_primary*Velocity_primary+mass_flow_secondary*Velocity_secondary)/  (mass_flow_primary+mass_flow_secondary)
    Temperature_mixed = (mass_flow_primary*Temperature_primary+mass_flow_secondary*Temperature_secondary)/   (mass_flow_primary+mass_flow_secondary)
    density_mixed     = pressure_amb/(R_gas*Temperature_mixed-(0.5*R_gas*Velocity_mixed**2/Cp))
    Area_mixed        = Area_primary*density_primary*Velocity_primary*(1+(mass_flow_secondary/mass_flow_primary))/   (density_mixed*Velocity_mixed)
    Diameter_mixed    = (4*Area_mixed/np.pi)**0.5

    XBPR = mass_flow_secondary/mass_flow_primary - 5.5
    XBPR[XBPR<0] = 0
    XBPR[XBPR>4] = 4

    # Auxiliary parameter defined as DVPS
    DVPS = np.abs((Velocity_primary - (Velocity_secondary*Area_secondary+Velocity_aircraft*Area_primary)/(Area_secondary+Area_primary)))
    DVPS[DVPS<0.3] =0.3
    
    # ============================================================================= 
    # Step 3: Update dimension of jet for spectral calculations  
    # =============================================================================
   
    frequency          = np.tile(np.atleast_2d(frequency),(n_cpts,1))  
    Diameter_primary   = np.tile(np.array([[Diameter_primary]]),(n_cpts,n_freq))  
    DVPS               = np.tile(DVPS,(1,n_freq))  
    Diameter_secondary = np.tile(np.array([[Diameter_secondary]]),(n_cpts,n_freq))  
    Velocity_secondary = np.tile(Velocity_secondary,(1,n_freq))
    Velocity_primary   = np.tile(Velocity_primary,(1,n_freq))
    Velocity_aircraft  = np.tile(Velocity_aircraft,(1,n_freq))   
    Diameter_mixed     = np.tile(Diameter_mixed,(1,n_freq))  
    Velocity_mixed     = np.tile(Velocity_mixed,(1,n_freq))
    sound_ambient      = np.tile(sound_ambient,(1,n_freq))

    # =============================================================================     
    # Step 4: Comptue noise at each microphone
    # =============================================================================
    for j in range(n_mic):
        theta_p = np.tile(np.atleast_2d(abs(theta_P[:,j])).T,(1,n_freq))
        theta_s = np.tile(np.atleast_2d(abs(theta_S[:,j])).T,(1,n_freq))
        theta_m = np.tile(np.atleast_2d(abs(theta_M[:,j])).T,(1,n_freq))
   
        # Calculation of the Strouhal number for each jet component (p-primary, s-secondary, m-mixed)
        Str_p = frequency*Diameter_primary/(DVPS)  # Primary jet
        Str_s = frequency*Diameter_secondary/(Velocity_secondary-Velocity_aircraft) # Secondary jet
        Str_m = frequency*Diameter_mixed/(Velocity_mixed-Velocity_aircraft) # Mixed jet

        # Calculation of the Excitation adjustment parameter 
        excitation_Strouhal = (N1/60)*(Diameter_mixed/Velocity_mixed)

        SX = 50*(excitation_Strouhal-0.25)*(excitation_Strouhal-0.5)
        SX[excitation_Strouhal > 0.25] = 0.0 
        SX[excitation_Strouhal < 0.5]  = 0.0 

        # Effectiveness
        exps = np.exp(-SX)

        #Spectral Shape Factor
        exs = 5*exps*np.exp(-(np.log10(Str_m/(2*excitation_Strouhal+0.00001)))**2)

        #Fan Duct Lenght Factor
        exd = np.exp(0.6-(EXA)**0.5)

        #Excitation source location factor (zk)
        zk = 1-0.4*(exd)*(exps)     

        # Loop for the frequency array range 
        exc = sound_ambient/Velocity_mixed 
        exc[theta_m>1.4] = (sound_ambient[theta_m>1.4]/Velocity_mixed[theta_m>1.4])*(1-(1.8/np.pi)*(theta_m[theta_m>1.4]-1.4))

        #Acoustic excitation adjustment (EX)
        EX_m = exd*exs*exc   # mixed component - dependant of the frequency

        EX_p = +5*exd*exps   #primary component - no frequency dependance
        EX_s = 2*sound_ambient/(Velocity_secondary*(zk)) #secondary component - no frequency dependance    

        distance_primary   = np.tile(np.array([[distance_microphone[j]]]),(n_cpts,n_freq))
        distance_secondary = np.tile(np.array([[distance_microphone[j]]]),(n_cpts,n_freq))
        distance_mixed     = np.tile(np.array([[distance_microphone[j]]]),(n_cpts,n_freq))

        # Noise attenuation due to Ambient Pressure
        dspl_ambient_pressure = 20*np.log10(pressure_amb/pressure_isa)

        # Noise attenuation due to Density Gradientes
        dspl_density_p = 20*np.log10((density_primary+density_secondary)/(2*density_ambient))
        dspl_density_s = 20*np.log10((density_secondary+density_ambient)/(2*density_ambient))
        dspl_density_m = 20*np.log10((density_mixed+density_ambient)/(2*density_ambient))

        # Noise attenuation due to Spherical divergence
        dspl_spherical_p = 20*np.log10(Diameter_primary/distance_primary)
        dspl_spherical_s = 20*np.log10(Diameter_mixed/distance_secondary)
        dspl_spherical_m = 20*np.log10(Diameter_mixed/distance_mixed) 

        # Calculation of the total noise attenuation (p-primary, s-secondary, m-mixed components)
        DSPL_p = dspl_ambient_pressure+dspl_density_p+dspl_spherical_p 
        DSPL_s = dspl_ambient_pressure+dspl_density_s+dspl_spherical_s 
        DSPL_m = dspl_ambient_pressure+dspl_density_m+dspl_spherical_m  

        # Calculation of interference effects on jet noise
        ATK_m   = angle_of_attack_effect(AOA,Mach_aircraft,theta_m)
        INST_s  = jet_installation_effect(Xe,Ye,Ce,theta_s,Diameter_mixed)
        Plug    = external_plug_effect(Velocity_primary,Velocity_secondary, Velocity_mixed, Diameter_primary,Diameter_secondary,
                                       Diameter_mixed, Plug_diameter, sound_ambient, theta_p,theta_s,theta_m)

        GPROX_m = ground_proximity_effect(Velocity_mixed,sound_ambient,theta_m,engine_height,Diameter_mixed,frequency)

        # Calculation of the sound pressure level for each jet component
        SPL_p = primary_noise_component(Velocity_primary,Temperature_primary,R_gas,theta_p,DVPS,sound_ambient, Velocity_secondary,Velocity_aircraft,Area_primary,Area_secondary,DSPL_p,EX_p,Str_p) + Plug.PG_p
        SPL_p[np.isnan(SPL_p)] = 1E-6
        
        SPL_s = secondary_noise_component(Velocity_primary,theta_s,sound_ambient,Velocity_secondary, Velocity_aircraft,Area_primary,Area_secondary,DSPL_s,EX_s,Str_s) + Plug.PG_s + INST_s
        SPL_s[np.isnan(SPL_s)] = 1E-6
        
        SPL_m = mixed_noise_component(Velocity_primary,theta_m,sound_ambient,Velocity_secondary,  Velocity_aircraft,Area_primary,Area_secondary,DSPL_m,EX_m,Str_m,Velocity_mixed,XBPR) + Plug.PG_m + ATK_m + GPROX_m
        SPL_m[np.isnan(SPL_m)] = 1E-6
        
        # Sum of the Total Noise
        SPL_total = 10 * np.log10(10**(0.1*SPL_p)+10**(0.1*SPL_s)+10**(0.1*SPL_m))

        # Store SPL history      
        SPL_1_3_spectrum[:,j,:]       = SPL_total 
        SPL[:,j]                      = SPL_arithmetic(SPL_total,sum_axis=1 )
        SPL_1_3_spectrum_dBA[:,j,:]   = A_weighting_metric(SPL_total,frequency)
        SPL_dBA[:,j]                  = SPL_arithmetic(np.atleast_2d(A_weighting_metric(SPL_total,frequency)),sum_axis=1)

    engine_noise                   = Data()   
    engine_noise.SPL_1_3_spectrum  = SPL_1_3_spectrum_dBA
    engine_noise.SPL               = SPL
    engine_noise.SPL_dBA           = SPL_dBA

    return engine_noise