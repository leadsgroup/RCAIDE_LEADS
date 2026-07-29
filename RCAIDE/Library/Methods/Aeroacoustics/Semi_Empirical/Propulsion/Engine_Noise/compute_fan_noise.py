# RCAIDE/Methods/Aeroacoustics/Semi_Empirical/Turbofan/jet_noise.py
# 
# 
# Created:  Jul 2026, P. Siripun, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.interpolate_fan_noise import get_spl_fan, create_interpolator_fan
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan          import design_turbofan
from RCAIDE.Library.Methods.Aeroacoustics.Common  import SPL_arithmetic 
from RCAIDE.Library.Methods.Aeroacoustics.Metrics import A_weighting_metric
from RCAIDE.Framework.Core                        import Units , Data

# Python package imports  
import math
import numpy as np

#initialize interpolators
interpolator_fan = create_interpolator_fan()

# ----------------------------------------------------------------------------------------------------------------------     
#  turbofan fan noise 
# ----------------------------------------------------------------------------------------------------------------------  

def compute_fan_noise(R_val, theta_engine, turbofan, m, aeroacoustic_data, segment, frequencies):
    #unpack

    conditions = segment.conditions
    N1                     = aeroacoustic_data.propulsors[turbofan.tag].fan.angular_velocity
    Velocity_secondary     = aeroacoustic_data.propulsors[turbofan.tag].fan.exit_velocity   
    Temperature_secondary  = aeroacoustic_data.propulsors[turbofan.tag].fan.exit_stagnation_temperature 
    Temperature_static_output = aeroacoustic_data.propulsors[turbofan.tag].fan.static_temperature_output
    Temperature_static_input  = aeroacoustic_data.propulsors[turbofan.tag].fan.static_temperature_input
    Pressure_secondary     = aeroacoustic_data.propulsors[turbofan.tag].fan.exit_stagnation_pressure 
    Velocity_aircraft      = segment.state.conditions.freestream.velocity
    noise_time             = segment.state.conditions.frames.inertial.time  
    distance_microphone    = R_val#ft #np.linalg.norm(microphone_locations,axis = 1)    #passed in m, used m
    Diameter_secondary     = aeroacoustic_data.propulsors[turbofan.tag].fan.diameter
    Num_blades             = aeroacoustic_data.propulsors[turbofan.tag].fan.number_of_blades

  

    frequency              = frequencies        
    n_cpts                 = len(noise_time)     
    n_freq                 = len(frequency) 
    n_mic                  = 1
  
    # ============================================================================= 
    # Step 1: Computing atmospheric conditions
    # ============================================================================= 
    sound_ambient       = segment.state.conditions.freestream.speed_of_sound
    pressure_isa        = 101325 # [Pa]
    R_gas               = 287.1  # [J/kg K]
    gamma_primary       = 1.37  # Corretion for the primary jet
    gamma               = 1.4 

    # ============================================================================= 
    # Step 2: Compute operating conditions and properties of jet     
    # ============================================================================= 
    # Calculation of nozzle areas
    Area_secondary =  np.pi*(Diameter_secondary/2)**2   

    # Defining each array before the main loop 
    theta     =  np.zeros(n_mic)


    SPL                    = np.zeros((n_cpts,n_mic))
    SPL_dBA                = np.zeros((n_cpts,n_mic))
    SPL_1_3_spectrum       = np.zeros((n_cpts,n_mic,n_freq)) 
    SPL_1_3_spectrum_dBA   = np.zeros((n_cpts,n_mic,n_freq))

    frequency          = np.tile(np.atleast_2d(frequency),(n_cpts,1))  

    # Secondary jet
    Cp  = R_gas/(1-1/gamma)
    
    # densities
    density_secondary = Pressure_secondary / (R_gas * (Temperature_secondary - (0.5 * Velocity_secondary**2 / Cp)))
    delt_T =Temperature_static_output-Temperature_static_input    #find temp difference of moving air
    M_TR = ((Velocity_aircraft**2 + ((np.pi*Diameter_secondary*N1)/60)**2)**0.5) / sound_ambient #compute the Tip relative Mach number

    if m == None:
        m = (Area_secondary*Velocity_secondary*density_secondary) / Units.lbs

    
    fan_inputs = Data(
    m = m - 600 , # Mass flow rate (kg/s -> lb/sec) Adjust for landing amount
    delta_T = delt_T*1.8,                                                  # Total temperature rise across the fan (K -> deg R)
    M_TR = M_TR,                                                                   # Tip relative Mach number
    RSS = 150.0,                                                                   # Rotor stator spacing / fan blade chord (%) Tune this
    f_b = (N1*Num_blades*Diameter_secondary) / Units.minute,                                          # Blade passage frequency (Hz)
    M_Tip = (np.pi*Diameter_secondary*N1*(1/60))/(sound_ambient),                   # Fan tip Mach number
    V_Number = 54,                                                     # Number of stator vanes
    B_Number = Num_blades,                                             # Number of rotor blades
    inlet_distortion = False                                           # Boolean
    )
    def calc_base_level(inputs):
    #Calculates the mass flow and temperature rise base terms.
        m_0 = 1.0     # 1 lb/sec reference
        delta_T_0 = 1.0 # 1 °R reference
        return 10 * math.log10(inputs.m / m_0) + 40 * math.log10(inputs.delta_T / delta_T_0)

    def calc_F2(inputs, multiplier):
        """Calculates the F2 spacing factor."""
        if not inputs.get("inlet_distortion", False):
            return multiplier * math.log10(inputs.RSS / 300)
        else:
            if inputs.RSS < 100:
                return multiplier * math.log10(inputs.RSS / 300)
            else:
                return multiplier * math.log10(100 / 300)

    def calc_tone_F4(inputs, k):
        """Calculates F4 for inlet and aft tones (cut-on / cut-off logic)."""
        if k == 1:
            cutoff_ratio = inputs.M_Tip / (1 - inputs.V_Number / inputs.B_Number)
            if cutoff_ratio > 1.05 or inputs.M_Tip > 1.05:
                return 0     # Cut-on
            else:
                return -8    # Cut-off
        else:
            return 3 - 3 * k # k > 1

    # Component Model Functions

    def calc_inlet_broadband(inputs, f, theta):
        F1 = 34 if inputs.M_TR < 0.72 else 34 - 43 * (inputs.M_TR - 0.72)
        F2 = calc_F2(inputs, -5.0)
        F3 = get_spl_fan(interpolator_fan,"Inlet Broadband",theta)
        F5 = -0.5 * (math.log(f / (4 * inputs.f_b)) / math.log(2.2))**2
        F4 = 10 * math.log10(math.exp(F5))
        
        return calc_base_level(inputs) + F1 + F2 + F3 + F4

    def calc_inlet_tones(inputs, theta, k):
        F1 = 42 - 20 * inputs.M_TR
        F2 = calc_F2(inputs, -10.0)
        F3 = get_spl_fan(interpolator_fan,"Inlet Tones",theta)
        F4 = calc_tone_F4(inputs, k)
        
        return calc_base_level(inputs) + F1 + F2 + F3 + F4

    def calc_aft_broadband(inputs, f, theta):
        F1 = 34 - 17 * (inputs.M_TR - 0.65)
        F2 = calc_F2(inputs, -5.0)
        F3 = get_spl_fan(interpolator_fan,"Aft Broadband",theta)
        F5 = -0.5 * (math.log(f / (2.5 * inputs.f_b)) / math.log(2.2))**2
        F4 = 10 * math.log10(math.exp(F5))
        
        return calc_base_level(inputs) + F1 + F2 + F3 + F4

    def calc_aft_tones(inputs, theta, k):
        F1 = 46 - 20 * inputs.M_TR
        F2 = calc_F2(inputs, -10.0)
        F3 = get_spl_fan(interpolator_fan,"Aft Tones",theta)
        F4 = calc_tone_F4(inputs, k)
        
        return calc_base_level(inputs) + F1 + F2 + F3 + F4

    def calc_combination_tones(inputs, f, theta):
            """Only valid for supersonic tip speeds."""
            F3 = get_spl_fan(interpolator_fan, "Combination Tones", theta)
            F2 = 0
            M_TR = inputs.M_TR
            f_b = inputs.f_b
            base_spl = calc_base_level(inputs)
            
            # 1/2 BPF Peak
            F1_half = -18 + 46.5*(M_TR - 1)/0.146 if M_TR < 1.146 else 28.5 - 12*(M_TR - 1.146)/0.854
            F4_half = np.where(f < 0.5 * f_b, 
                            20 * np.log10(f / (0.5 * f_b)), 
                            -20 * np.log10(f / (0.5 * f_b)))
            spl_half = base_spl + F1_half + F2 + F3 + F4_half

            # 1/4 BPF Peak
            F1_quarter = -15 + 47.5*(M_TR - 1)/0.322 if M_TR < 1.322 else 32.5 - 9*(M_TR - 1.322)/0.678
            F4_quarter = np.where(f < 0.25 * f_b, 
                                30 * np.log10(f / (0.25 * f_b)), 
                                -30 * np.log10(f / (0.25 * f_b)))
            spl_quarter = base_spl + F1_quarter + F2 + F3 + F4_quarter

            # 1/8 BPF Peak
            F1_eighth = -12 + 41.2*(M_TR - 1)/0.61 if M_TR < 1.61 else 29.2 - 4.7*(M_TR - 1.61)/0.39
            F4_eighth = np.where(f < 0.125 * f_b, 
                                30 * np.log10(f / (0.125 * f_b)), 
                                -20 * np.log10(f / (0.125 * f_b)))
            spl_eighth = base_spl + F1_eighth + F2 + F3 + F4_eighth

            # log combination of sub peaks
            total_spl = 10 * np.log10(10**(spl_half/10) + 10**(spl_quarter/10) + 10**(spl_eighth/10))
            return total_spl

    def add_spl(*levels):
        """Logarithmically adds multiple SPL values."""
        # Convert dB to linear energy, sum them (ignoring very small values)
        energy_sum = sum(10**(lvl/10) for lvl in levels if lvl > -50) 
        
        if energy_sum == 0:
            return -50
            
        return 10 * math.log10(energy_sum)
    
    fan_noise= Data()
    theta_s     =  np.degrees(theta_engine)

    for i in range(n_mic):
        spl = calc_combination_tones(fan_inputs, frequency, theta=theta_s)
        distance_attenuated_spl = spl + 20*np.log10(0.25/distance_microphone[i]) # paper sideline = 93 in
        

        SPL_1_3_spectrum[:,i,:]       = distance_attenuated_spl 
        SPL[:,i]                      = SPL_arithmetic(distance_attenuated_spl,sum_axis=1 )
        SPL_1_3_spectrum_dBA[:,i,:]   = A_weighting_metric(distance_attenuated_spl,frequency)
        SPL_dBA[:,i]                  = SPL_arithmetic(np.atleast_2d(A_weighting_metric(distance_attenuated_spl,frequency)),sum_axis=1)


    

    fan_noise.SPL_1_3_spectrum  = SPL_1_3_spectrum
    fan_noise.SPL               = SPL
    fan_noise.SPL_dBA           = SPL_dBA
    return fan_noise

