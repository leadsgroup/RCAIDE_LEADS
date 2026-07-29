# RCAIDE/Methods/Aeroacoustics/Semi_Empirical/Turbofan/core_noise.py
# 
# 
# Created:  Jun 2026, M. Clarke , P. Siripun

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.interpolate_core_noise import get_spl, create_interpolator
from RCAIDE.Library.Methods.Aeroacoustics.Common  import SPL_arithmetic 
from RCAIDE.Library.Methods.Aeroacoustics.Metrics import A_weighting_metric

# Python package imports 
import numpy as np
import math

# ----------------------------------------------------------------------------------------------------------------------     
#  turbofan engine core noise
# ---------------------------------------------------------------------------------------------------------------------- 

def compute_core_noise(R_val, theta_engine, turbofan, pr, aeroacoustic_data, segment, frequencies):
    n_mic = 1
    conditions = segment.conditions
    N1                     = aeroacoustic_data.propulsors[turbofan.tag].fan.angular_velocity
    Velocity_secondary     = aeroacoustic_data.propulsors[turbofan.tag].fan_nozzle.exit_velocity   
    Temperature_secondary  = aeroacoustic_data.propulsors[turbofan.tag].fan_nozzle.exit_stagnation_temperature 
    Pressure_secondary     = aeroacoustic_data.propulsors[turbofan.tag].fan_nozzle.exit_stagnation_pressure 
    Velocity_primary       = aeroacoustic_data.propulsors[turbofan.tag].core_nozzle.exit_velocity  
    Temperature_primary    = aeroacoustic_data.propulsors[turbofan.tag].core_nozzle.exit_stagnation_temperature 
    Pressure_primary       = aeroacoustic_data.propulsors[turbofan.tag].core_nozzle.exit_stagnation_pressure      


    Velocity_aircraft      = segment.state.conditions.freestream.velocity
    Mach_aircraft          = segment.state.conditions.freestream.mach_number 
    AOA                    = segment.state.conditions.aerodynamics.angles.alpha / Units.deg 
    noise_time             = segment.state.conditions.frames.inertial.time  
    distance_microphone    = R_val 
    Diameter_primary       = turbofan.core_nozzle.diameter
    Diameter_secondary     = turbofan.fan_nozzle.diameter
    Num_nozzle             = turbofan.combustor.number_of_fuel_nozzle

    ram                       = turbofan.ram
    inlet_nozzle              = turbofan.inlet_nozzle
    fan                       = turbofan.fan
    low_pressure_compressor   = turbofan.low_pressure_compressor
    high_pressure_compressor  = turbofan.high_pressure_compressor
    combustor                 = turbofan.combustor
    high_pressure_turbine     = turbofan.high_pressure_turbine
    low_pressure_turbine      = turbofan.low_pressure_turbine
    core_nozzle               = turbofan.core_nozzle
    fan_nozzle                = turbofan.fan_nozzle 
    bypass_ratio              = turbofan.bypass_ratio 
    
    # unpack component conditions 



    frequency              = frequencies    
    n_cpts                 = len(noise_time)     
    n_freq                 = len(frequency) 
  
    # ============================================================================= 
    # Step 1: Computing atmospheric conditions
    # ============================================================================= 
    sound_ambient       = segment.state.conditions.freestream.speed_of_sound
    density_ambient     = segment.state.conditions.freestream.density  
    pressure_amb        = segment.state.conditions.freestream.pressure 
    temp_amb            = segment.state.conditions.freestream.temperature
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

    frequency          = np.tile(np.atleast_2d(frequency),(n_cpts,1))  

    # Primary and Secondary jets
    Cpp = R_gas/(1-1/gamma_primary)
    Cp  = R_gas/(1-1/gamma)
    
    # densitys 
    density_primary   = Pressure_primary/(R_gas*Temperature_primary-(0.5*R_gas*Velocity_primary**2/Cpp)) 
    density_secondary = Pressure_secondary/(R_gas*Temperature_secondary-(0.5*R_gas*Velocity_secondary**2/Cp))

    standard_freqs          = np.tile(np.atleast_2d(frequency),(n_cpts,1))  
    
    
    #convert code theta and distance to observe ft and distance for the input
    #also convert the injected frequency list to code input

    core_noise= Data()


    # Load interpolators ONCE
    interp_C1 = create_interpolator("table_c1")
    interp_C2 = create_interpolator("table_c2")
    interp_C3 = create_interpolator("table_c3")
    
    for i in range(1): #to vectorize next
    
        model_inputs = Data(
        W1 = ((Area_primary*Velocity_primary*density_primary) / Units.lbm),  # Total core mass flow rate (lbm/sec)
        T_C_o = segment.state.conditions.energy.converters['combustor'].outputs.static_temperature * 1.8,             # Combustor outlet total temperature (deg R)
        T_C_i = segment.state.conditions.energy.converters['combustor'].inputs.static_temperature * 1.8,              # Combustor inlet total temperature (deg R)
        P_amb = pressure_amb / Units.psi,                                               # Ambient pressure (pa -> psia)
        T_amb = (temp_amb*1.8)[0][0],                                                           # Ambient temperature (deg R)
        n_f =  Num_nozzle,                                                              # Number of fuel nozzles
        R = R_val[0][0]/Units.feet, #distance_microphone[i] / Units.feet, # Microphone distance (ft), passed in m, used ft
        D_h_1 = core_nozzle.diameter / Units.feet,                                      # core nozzle hydraulic diameter
        c_amb = (sound_ambient / Units.feet)[0][0],                                             # Ambient sonic velocity (ft/sec)
        D_C = combustor.diameter / Units.feet,                                          # Combustor diameter (ft)
        c_C_o = (331.3*(1+((segment.state.conditions.energy.converters['combustor'].outputs.static_temperature-273)/273))**0.5) / Units.feet,      # Combustor exit sonic velocity (ft/sec)
        f = frequencies,                                                                                     # Frequency (Hz) -> injected list
        theta_c = np.degrees(theta_engine[0][0]),     #degrees, because interpolator is in degrees                                                                                   # theta (radians)
        pressure_ratio = pr  # Pressure ratio
        )

        # Calculate Base Parameters
        core_param_log = calc_core_param(
        model_inputs.W1, model_inputs.T_C_o, model_inputs.T_C_i, 
        model_inputs.pressure_ratio, model_inputs.T_amb
    )

        uol_c1 = calc_uol_c1(model_inputs.R, model_inputs.n_f, core_param_log)
        uol_c2 = calc_uol_c2(model_inputs.R, model_inputs.n_f, core_param_log)
        uol_c3 = calc_uol_c3(model_inputs.R, core_param_log)
        
        #Initialize output arrays and tables
        spl_c1_list, spl_c2_list, spl_c3_list = [], [], []
        theta_c = model_inputs.theta_c

        for f in model_inputs.f:
            s_c1 = calc_strouhal_c1(f, model_inputs.D_h_1, model_inputs.c_amb)
            s_c2_c3 = calc_strouhal_c2_c3(f, model_inputs.D_C, model_inputs.c_C_o) #plot these strouhals (in future to correlate)
            
            norm_spl_c1 = get_normalized_spl(interp_C1, s_c1, theta_c) 
            norm_spl_c2 = get_normalized_spl(interp_C2, s_c2_c3, theta_c)
            norm_spl_c3 = get_normalized_spl(interp_C3, s_c2_c3, theta_c)
            
            spl_c1_list.append(norm_spl_c1 + uol_c1 ) 
            spl_c2_list.append(norm_spl_c2 + uol_c2)
            spl_c3_list.append(norm_spl_c3 + uol_c3)
        
        c1_arr, c2_arr, c3_arr = np.array(spl_c1_list), np.array(spl_c2_list), np.array(spl_c3_list)
        SPL_total = 10 * np.log10(10**(c1_arr/10) + 10**(c2_arr/10) + 10**(c3_arr/10))
        # calculate log_S before get_spl
        # Store SPL history      
        SPL_1_3_spectrum[:,i,:]       = SPL_total 
        SPL[:,i]                      = SPL_arithmetic(np.atleast_2d(SPL_total), sum_axis=1)
        SPL_1_3_spectrum_dBA[:,i,:]   = A_weighting_metric(SPL_total,frequency)
        SPL_dBA[:,i]                  = SPL_arithmetic(np.atleast_2d(A_weighting_metric(SPL_total,frequency)),sum_axis=1)

    core_noise                   = Data()   
    core_noise.SPL_1_3_spectrum  = SPL_1_3_spectrum
    core_noise.SPL               = SPL
    core_noise.SPL_dBA           = SPL_dBA

    return core_noise 
# Standard 1/3-octave-band center frequencies (Hz)

def get_normalized_spl(interpolator_obj, strouhal_num, theta_c):
    if (np.array(strouhal_num) <= 0).any():
        return 0
    log_S = np.log10(strouhal_num)
    return get_spl(interpolator_obj, theta_c, log_S)

def calc_core_param(W1, T_C_o, T_C_i, pressure_ratio, T_amb):
    temp_ratio_diff = T_C_o - T_C_i
    temp_ratio_amb = T_amb / T_C_i
    inner_term = W1 * ((temp_ratio_diff * pressure_ratio * temp_ratio_amb) ** 2)
    return np.log10(inner_term)

def calc_uol_c1(R, n_f, core_param_log):
    C_C1, N_C1, F_C1 = 78.0, 7.0, 14.0
    return C_C1 - (20.0 * math.log10(R)) + (N_C1 * core_param_log) - (F_C1 * math.log10(n_f))

def calc_uol_c2(R, n_f, core_param_log):
    C_C2, N_C2, F_C2 = 60.3, 10.0, 18.0
    return C_C2 - (20.0 * math.log10(R)) + (N_C2 * core_param_log) - (F_C2 * math.log10(n_f))

def calc_uol_c3(R, core_param_log):
    C_C3, N_C3 = 42.5, 9.0
    return C_C3 - (20.0 * math.log10(R)) + (N_C3 * core_param_log)

def calc_strouhal_c1(f, D_h_1, c_amb):
    return (f * D_h_1) / c_amb

def calc_strouhal_c2_c3(f, D_C, c_C_o):
    return (f * D_C) / c_C_o