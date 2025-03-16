# RCAIDE/Methods/Noise/Correlation_Buildup/Turbofan/turbofan_engine_noise.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

 # RCAIDE imports 
from RCAIDE.Framework.Core                import Units , Data  
from .angle_of_attack_effect              import angle_of_attack_effect
from .external_plug_effect                import external_plug_effect
from .ground_proximity_effect             import ground_proximity_effect
from .jet_installation_effect             import jet_installation_effect
from .mixed_noise_component               import mixed_noise_component 
from .primary_noise_component             import primary_noise_component
from .secondary_noise_component           import secondary_noise_component

from RCAIDE.Library.Methods.Noise.Common  import noise_tone_correction
from RCAIDE.Library.Methods.Noise.Common  import atmospheric_attenuation 
from RCAIDE.Library.Methods.Noise.Common  import SPL_arithmetic
from RCAIDE.Library.Methods.Noise.Metrics import PNL_noise_metric
from RCAIDE.Library.Methods.Noise.Metrics import EPNL_noise_metric
from RCAIDE.Library.Methods.Noise.Metrics import A_weighting_metric  

# Python package imports   
import numpy as np   
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------     
#  turbofan engine noise 
# ----------------------------------------------------------------------------------------------------------------------         
def turbofan_engine_noise(microphone_locations,turbofan,aeroacoustic_data,segment,settings):  
    """This method predicts the free-field 1/3 Octave Band SPL of coaxial subsonic
       jets for turbofan engines under the following conditions:
       a) Flyover (observer on ground)
       b) Static (observer on ground)
       c) In-flight or in-flow (observer on airplane or in a wind tunnel)

    Assumptions:
        SAE ARP876D: Gas Turbine Jet Exhaust Noise Prediction

    Inputs:
        vehicle	 - RCAIDE type vehicle 
        includes these fields:
            Velocity_primary           - Primary jet flow velocity                           [m/s]
            Temperature_primary        - Primary jet flow temperature                        [m/s]
            Pressure_primary           - Primary jet flow pressure                           [Pa]
            Area_primary               - Area of the primary nozzle                          [m^2]
            Velocity_secondary         - Secondary jet flow velocity                         [m/s]
            Temperature_secondary      - Secondary jet flow temperature                      [m/s]
            Pressure_secondary         - Secondary jet flow pressure                         [Pa]
            Area_secondary             - Area of the secondary nozzle                        [m^2]
            AOA                        - Angle of attack                                     [rad]
            Velocity_aircraft          - Aircraft velocity                                   [m/s]
            Altitude                   - Altitude                                            [m]
            N1                         - Fan rotational speed                                [rpm]
            EXA                        - Distance from fan face to fan exit/ fan diameter    [m]
            Plug_diameter              - Diameter of the engine external plug                [m]
            height              - Engine centerline height above the ground plane     [m]
            distance_microphone        - Distance from the nozzle exhaust to the microphones [m]
            angles                     - Array containing the desired polar angles           [rad] 

    Outputs: One Third Octave Band SPL [dB]
        SPL_p                           - Sound Pressure Level of the primary jet            [dB]
        SPL_s                           - Sound Pressure Level of the secondary jet          [dB]
        SPL_m                           - Sound Pressure Level of the mixed jet              [dB]
        SPL_total                       - Sound Pressure Level of the total jet noise        [dB]

    """ 
    # unpack   
    Velocity_primary       = aeroacoustic_data.core_nozzle.exit_velocity * 0.92*(turbofan.design_thrust/52700.)   
    Temperature_primary    = aeroacoustic_data.core_nozzle.exit_stagnation_temperature[:,0] 
    Pressure_primary       = aeroacoustic_data.core_nozzle.exit_stagnation_pressure[:,0]  
    Velocity_secondary     = aeroacoustic_data.fan_nozzle.exit_velocity  * (turbofan.design_thrust/52700.) 
    Temperature_secondary  = aeroacoustic_data.fan_nozzle.exit_stagnation_temperature[:,0] 
    Pressure_secondary     = aeroacoustic_data.fan_nozzle.exit_stagnation_pressure[:,0]  
    Velocity_aircraft      = segment.conditions.freestream.velocity[:,0]
    Mach_aircraft          = segment.conditions.freestream.mach_number 
    AOA                    = np.mean(segment.conditions.aerodynamics.angles.alpha / Units.deg) 
    noise_time             = segment.conditions.frames.inertial.time[:,0]  
    distance_microphone    = np.linalg.norm(microphone_locations,axis = 1)     
    
    N1                     = turbofan.fan.design_angular_velocity * 0.92*(turbofan.design_thrust/52700.)
    Diameter_primary       = turbofan.core_nozzle.diameter
    Diameter_secondary     = turbofan.fan_nozzle.diameter
    height                 = turbofan.height
    EXA                    = turbofan.exa
    Plug_diameter          = turbofan.plug_diameter 
    Xe                     = turbofan.geometry_xe
    Ye                     = turbofan.geometry_ye
    Ce                     = turbofan.geometry_Ce 
    
    frequency              = settings.center_frequencies[5:]        
    n_cpts                 = len(noise_time)     
    num_f                  = len(frequency) 
    n_mic                  = len(microphone_locations)

    if type(Velocity_primary) == float:
        Velocity_primary    = np.ones(n_cpts)*Velocity_primary

    if type(Velocity_secondary) == float:
        Velocity_secondary  = np.ones(n_cpts)*Velocity_secondary

    # ==============================================
    # Computing atmospheric conditions
    # ==============================================  
    sound_ambient       = segment.conditions.freestream.speed_of_sound[:,0]
    density_ambient     = segment.conditions.freestream.density[:,0]  
    pressure_amb        = segment.conditions.freestream.pressure[:,0] 
    pressure_isa        = 101325 # [Pa]
    R_gas               = 287.1  # [J/kg K]
    gamma_primary       = 1.37  # Corretion for the primary jet
    gamma               = 1.4

    # Calculation of nozzle areas
    Area_primary   = np.pi*(Diameter_primary/2)**2 
    Area_secondary =  np.pi*(Diameter_secondary/2)**2   
   
    # Defining each array before the main loop  
    theta_P                = np.tile(np.arctan2(microphone_locations[:, 1], microphone_locations[:, 0])[None,:],(n_cpts,1))  
    theta_S                = deepcopy(theta_P)
    theta_M                = deepcopy(theta_P)
    EX_p                   = np.zeros((n_cpts,n_mic,num_f)) 
    EX_s                   = np.zeros((n_cpts,n_mic,num_f)) 
    EX_m                   = np.zeros((n_cpts,n_mic,num_f))  
    SPL_p                  = np.zeros((n_cpts,n_mic,num_f)) 
    SPL_s                  = np.zeros((n_cpts,n_mic,num_f)) 
    SPL_m                  = np.zeros((n_cpts,n_mic,num_f)) 
    SPL                    = np.zeros((n_cpts,n_mic))
    SPL_dBA                = np.zeros((n_cpts,n_mic))
    SPL_1_3_spectrum       = np.zeros((n_cpts,n_mic,num_f))
    SPL_primary_spectrum   = np.zeros((n_cpts,n_mic,num_f))
    SPL_secondary_spectrum = np.zeros((n_cpts,n_mic,num_f))
    SPL_mixed_specturm     = np.zeros((n_cpts,n_mic,num_f)) 
    SPL_1_3_spectrum_dBA   = np.zeros((n_cpts,n_mic,num_f)) 

    # Start loop for each position of aircraft 
    for i in range(n_cpts):
        for j in range(n_mic): 
        
            EX_p    = np.zeros(num_f)
            EX_s    = np.zeros(num_f)
            EX_m    = np.zeros(num_f) 
            theta_p = abs(theta_P[i,j])
            theta_s = abs(theta_S[i,j])
            theta_m = abs(theta_M[i,j])
 
            # Primary and Secondary jets
            Cpp = R_gas/(1-1/gamma_primary)
            Cp  = R_gas/(1-1/gamma)
    
            density_primary   = Pressure_primary[i]/(R_gas*Temperature_primary[i]-(0.5*R_gas*Velocity_primary[i]**2/Cpp))
            density_secondary = Pressure_secondary[i]/(R_gas*Temperature_secondary[i]-(0.5*R_gas*Velocity_secondary[i]**2/Cp))
    
            mass_flow_primary   = Area_primary*Velocity_primary[i]*density_primary
            mass_flow_secondary = Area_secondary*Velocity_secondary[i]*density_secondary
    
            #Mach number of the external flow - based on the aircraft velocity
            Mach_aircraft[i] = Velocity_aircraft[i]/sound_ambient[i]
    
            #Calculation Procedure for the Mixed Jet Flow Parameters
            Velocity_mixed = (mass_flow_primary*Velocity_primary[i]+mass_flow_secondary*Velocity_secondary[i])/ \
                (mass_flow_primary+mass_flow_secondary)
            Temperature_mixed =(mass_flow_primary*Temperature_primary[i]+mass_flow_secondary*Temperature_secondary[i])/ \
                (mass_flow_primary+mass_flow_secondary)
            density_mixed = pressure_amb[i]/(R_gas*Temperature_mixed-(0.5*R_gas*Velocity_mixed**2/Cp))
            Area_mixed = Area_primary*density_primary*Velocity_primary[i]*(1+(mass_flow_secondary/mass_flow_primary))/ \
                (density_mixed*Velocity_mixed)
            Diameter_mixed = (4*Area_mixed/np.pi)**0.5
     
    
            XBPR = mass_flow_secondary/mass_flow_primary - 5.5
            if XBPR<0:
                XBPR=0
            elif XBPR>4:
                XBPR=4
    
            #Auxiliary parameter defined as DVPS
            DVPS = np.abs((Velocity_primary[i] - (Velocity_secondary[i]*Area_secondary+Velocity_aircraft[i]*Area_primary)/\
                           (Area_secondary+Area_primary)))
            if DVPS<0.3:
                DVPS=0.3
    
            # Calculation of the Strouhal number for each jet component (p-primary, s-secondary, m-mixed)
            Str_p = frequency*Diameter_primary/(DVPS)  #Primary jet
            Str_s = frequency*Diameter_mixed/(Velocity_secondary[i]-Velocity_aircraft[i]) #Secondary jet
            Str_m = frequency*Diameter_mixed/(Velocity_mixed-Velocity_aircraft[i]) #Mixed jet
    
            #Calculation of the Excitation adjustment parameter 
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
            exc = sound_ambient[i]/Velocity_mixed 
            if theta_m>1.4:
                exc  = (sound_ambient[i]/Velocity_mixed)*(1-(1.8/np.pi)*(theta_m-1.4))

            #Acoustic excitation adjustment (EX)
            EX_m = exd*exs*exc   # mixed component - dependant of the frequency
    
            EX_p = +5*exd*exps   #primary component - no frequency dependance
            EX_s = 2*sound_ambient[i]/(Velocity_secondary[i]*(zk)) #secondary component - no frequency dependance    
    
            distance_primary   = distance_microphone[j] 
            distance_secondary = distance_microphone[j] 
            distance_mixed     = distance_microphone[j]
    
            #Noise attenuation due to Ambient Pressure
            dspl_ambient_pressure = 20*np.log10(pressure_amb[i]/pressure_isa)
    
            #Noise attenuation due to Density Gradientes
            dspl_density_p = 20*np.log10((density_primary+density_secondary)/(2*density_ambient[i]))
            dspl_density_s = 20*np.log10((density_secondary+density_ambient[i])/(2*density_ambient[i]))
            dspl_density_m = 20*np.log10((density_mixed+density_ambient[i])/(2*density_ambient[i]))
    
            #Noise attenuation due to Spherical divergence
            dspl_spherical_p = 20*np.log10(Diameter_primary/distance_primary)
            dspl_spherical_s = 20*np.log10(Diameter_mixed/distance_secondary)
            dspl_spherical_m = 20*np.log10(Diameter_mixed/distance_mixed)
    
            # Noise attenuation due to Geometric Near-Field 
            dspl_geometric_p = 0.0
            dspl_geometric_s = 0.0
            dspl_geometric_m = 0.0 
            
    
            # Noise attenuation due to Acoustic Near-Field 
            dspl_acoustic_p = 0.0 
            dspl_acoustic_s = 0.0 
            dspl_acoustic_m = 0.0  
     
            # Atmospheric attenuation
            delta_atmo = atmospheric_attenuation(np.array([distance_primary]),frequency)[0,:]
    
            dspl_attenuation_p = -delta_atmo 
            dspl_attenuation_s = -delta_atmo 
            dspl_attenuation_m = -delta_atmo  
    
            # Calculation of the total noise attenuation (p-primary, s-secondary, m-mixed components)
            DSPL_p = dspl_ambient_pressure+dspl_density_p+dspl_geometric_p+dspl_acoustic_p+dspl_attenuation_p+dspl_spherical_p
            DSPL_s = dspl_ambient_pressure+dspl_density_s+dspl_geometric_s+dspl_acoustic_s+dspl_attenuation_s+dspl_spherical_s
            DSPL_m = dspl_ambient_pressure+dspl_density_m+dspl_geometric_m+dspl_acoustic_m+dspl_attenuation_m+dspl_spherical_m
    
    
            # Calculation of interference effects on jet noise
            ATK_m   = angle_of_attack_effect(AOA,Mach_aircraft[i],theta_m)
            INST_s  = jet_installation_effect(Xe,Ye,Ce,theta_s,Diameter_mixed)
            Plug    = external_plug_effect(Velocity_primary[i],Velocity_secondary[i], Velocity_mixed, Diameter_primary,Diameter_secondary,
                                           Diameter_mixed, Plug_diameter, sound_ambient[i], theta_p,theta_s,theta_m)
    
            GPROX_m = ground_proximity_effect(Velocity_mixed,sound_ambient[i],theta_m,height,Diameter_mixed,frequency)
    
            # Calculation of the sound pressure level for each jet component
            SPL_p = primary_noise_component(Velocity_primary[i],Temperature_primary[i],R_gas,theta_p,DVPS,sound_ambient[i],
                                            Velocity_secondary[i],Velocity_aircraft[i],Area_primary,Area_secondary,DSPL_p,EX_p,Str_p) + Plug.PG_p
    
            SPL_s = secondary_noise_component(Velocity_primary[i],theta_s,sound_ambient[i],Velocity_secondary[i],
                                              Velocity_aircraft[i],Area_primary,Area_secondary,DSPL_s,EX_s,Str_s) + Plug.PG_s + INST_s
    
            SPL_m = mixed_noise_component(Velocity_primary[i],theta_m,sound_ambient[i],Velocity_secondary[i],
                                          Velocity_aircraft[i],Area_primary,Area_secondary,DSPL_m,EX_m,Str_m,Velocity_mixed,XBPR) + Plug.PG_m + ATK_m + GPROX_m
    
            # Sum of the Total Noise
            SPL_total = 10 * np.log10(10**(0.1*SPL_p)+10**(0.1*SPL_s)+10**(0.1*SPL_m))
    
            # Store SPL history      
            SPL_1_3_spectrum[i,j,:]       = SPL_total 
            SPL[i,j]                      = SPL_arithmetic(np.atleast_2d(SPL_total),sum_axis=1 )
            SPL_primary_spectrum[i,j,:]   = SPL_p 
            SPL_secondary_spectrum[i,j,:] = SPL_s 
            SPL_mixed_specturm[i,j,:]     = SPL_m  
            SPL_1_3_spectrum_dBA[i,j,:]   = A_weighting_metric(SPL_total,frequency)
            SPL_dBA[i,j]                  = SPL_arithmetic(np.atleast_2d(A_weighting_metric(SPL_total,frequency)),sum_axis=1) 

    # Calculation of the Perceived Noise Level EPNL based on the sound time history
    PNL_total               =  PNL_noise_metric(SPL_1_3_spectrum)    
    PNL_primary             =  PNL_noise_metric(SPL_primary_spectrum)  
    PNL_secondary           =  PNL_noise_metric(SPL_secondary_spectrum)  
    PNL_mixed               =  PNL_noise_metric(SPL_mixed_specturm)  

    # Calculation of the tones corrections on the SPL for each component and total
    tone_correction_total     = noise_tone_correction(SPL_1_3_spectrum) 
    tone_correction_primary   = noise_tone_correction(SPL_primary_spectrum) 
    tone_correction_secondary = noise_tone_correction(SPL_secondary_spectrum) 
    tone_correction_mixed     = noise_tone_correction(SPL_mixed_specturm) 

    # Calculation of the PLNT for each component and total
    PNLT_total     = PNL_total+tone_correction_total
    PNLT_primary   = PNL_primary+tone_correction_primary
    PNLT_secondary = PNL_secondary+tone_correction_secondary
    PNLT_mixed     = PNL_mixed+tone_correction_mixed

    # Calculation of the EPNL for each component and total
    EPNL_total     = EPNL_noise_metric(PNLT_total)
    EPNL_primary   = EPNL_noise_metric(PNLT_primary)
    EPNL_secondary = EPNL_noise_metric(PNLT_secondary)
    EPNL_mixed     = EPNL_noise_metric(PNLT_mixed) 

    # Open output file to print the result     
    
    engine_noise                   = Data() 
    engine_noise.EPNL_primary      = EPNL_primary  
    engine_noise.EPNL_secondary    = EPNL_secondary
    engine_noise.EPNL_mixed        = EPNL_mixed    
    engine_noise.EPNL_total        = EPNL_total  
    engine_noise.SPL_1_3_spectrum  = SPL_1_3_spectrum_dBA
    engine_noise.SPL               = SPL
    engine_noise.SPL_dBA           = SPL_dBA

    return engine_noise