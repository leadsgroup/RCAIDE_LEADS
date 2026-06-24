import math
import numpy as np
import math
import matplotlib.pyplot as plt
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.interpolate_fan_noise import get_spl_fan, create_interpolator_fan
from RCAIDE.Library.Methods.Aeroacoustics.Common  import SPL_arithmetic 
from RCAIDE.Library.Methods.Aeroacoustics.Metrics import A_weighting_metric

from RCAIDE.Framework.Core                        import Units , Data


#initialize interpolators
interpolator_fan = create_interpolator_fan()
#help functions

def compute_fan_noise(microphone_locations, turbofan, aeroacoustic_data, segment, settings):
    #unpack

    conditions = segment.conditions
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
    Num_blades             = turbofan.number_of_blades
    engine_height          = turbofan.origin[0][2] # This needs to be updated in a future PR
    EXA                    = turbofan.length /  turbofan.diameter 
    Plug_diameter          = turbofan.plug_diameter 
    Xe                     = turbofan.geometry_xe
    Ye                     = turbofan.geometry_ye
    Ce                     = turbofan.geometry_Ce 

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
    ram_conditions          = conditions.energy.converters[ram.tag]    
    inlet_nozzle_conditions = conditions.energy.converters[inlet_nozzle.tag]
    fan_conditions          = conditions.energy.converters[fan.tag]    
    lpc_conditions          = conditions.energy.converters[low_pressure_compressor.tag]
    hpc_conditions          = conditions.energy.converters[high_pressure_compressor.tag]
    combustor_conditions    = conditions.energy.converters[combustor.tag]     
    lpt_conditions          = conditions.energy.converters[low_pressure_turbine.tag]
    hpt_conditions          = conditions.energy.converters[high_pressure_turbine.tag]
    core_nozzle_conditions  = conditions.energy.converters[core_nozzle.tag]
    fan_nozzle_conditions   = conditions.energy.converters[fan_nozzle.tag]    


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

    delt_T =fan_conditions.outputs.static_temperature-fan_conditions.inputs.static_temperature   #find temp difference of moving air
    M_TR = (Velocity_aircraft**2 + ((np.pi*Num_blades*Diameter_secondary)/60)**2) / sound_ambient#compute the Tip relative Mach number
    fan_inputs = {
    "m": (Area_secondary*Velocity_secondary*density_secondary)*2.20462 ,# Mass flow rate (kg/s -> lb/sec)
    "delta_T": delt_T*180,                                              # Total temperature rise across the fan (K -> deg R)
    "M_TR": M_TR,                                                       # Tip relative Mach number
    "RSS": 150.0,                                                       # Rotor stator spacing / fan blade chord (%) Tune this
    "f_b": (N1*Num_blades)/60,                                          # Blade passage frequency (Hz)
    "M_Tip": (np.pi*Diameter_secondary*Num_blades)/(60*sound_ambient),  # Fan tip Mach number
    "V_Number": 54,                                                     # Number of stator vanes
    "B_Number": 22,                                                     # Number of rotor blades
    "inlet_distortion": False                                           # Boolean
    }

    def calc_base_level(inputs):
    #Calculates the mass flow and temperature rise base terms.
        m_0 = 1.0     # 1 lb/sec reference
        delta_T_0 = 1.0 # 1 °R reference
        return 10 * math.log10(inputs["m"] / m_0) + 40 * math.log10(inputs["delta_T"] / delta_T_0)

    def calc_F2(inputs, multiplier):
        """Calculates the F2 spacing factor."""
        if not inputs.get("inlet_distortion", False):
            return multiplier * math.log10(inputs["RSS"] / 300.0)
        else:
            if inputs["RSS"] < 100:
                return multiplier * math.log10(inputs["RSS"] / 300.0)
            else:
                return multiplier * math.log10(100.0 / 300.0)

    def calc_tone_F4(inputs, k):
        """Calculates F4 for inlet and aft tones (cut-on / cut-off logic)."""
        if k == 1:
            cutoff_ratio = inputs["M_Tip"] / (1 - inputs["V_Number"] / inputs["B_Number"])
            if cutoff_ratio > 1.05 or inputs["M_Tip"] > 1.05:
                return 0     # Cut-on
            else:
                return -8    # Cut-off
        else:
            return 3 - 3 * k # k > 1

    # Component Model Functions

    def calc_inlet_broadband(inputs, f, theta):
        F1 = 34 if inputs["M_TR"] < 0.72 else 34 - 43 * (inputs["M_TR"] - 0.72)
        F2 = calc_F2(inputs, -5.0)
        F3 = float(get_spl_fan(interpolator_fan,"Inlet Broadband",theta))
        F5 = -0.5 * (math.log(f / (4 * inputs["f_b"])) / math.log(2.2))**2
        F4 = 10 * math.log10(math.exp(F5))
        
        return calc_base_level(inputs) + F1 + F2 + F3 + F4

    def calc_inlet_tones(inputs, theta, k):
        F1 = 42 - 20 * inputs["M_TR"]
        F2 = calc_F2(inputs, -10.0)
        F3 = float(get_spl_fan(interpolator_fan,"Inlet Tones",theta))
        F4 = calc_tone_F4(inputs, k)
        
        return calc_base_level(inputs) + F1 + F2 + F3 + F4

    def calc_aft_broadband(inputs, f, theta):
        F1 = 34 - 17 * (inputs["M_TR"] - 0.65)
        F2 = calc_F2(inputs, -5.0)
        F3 = float(get_spl_fan(interpolator_fan,"Aft Broadband",theta))
        F5 = -0.5 * (math.log(f / (2.5 * inputs["f_b"])) / math.log(2.2))**2
        F4 = 10 * math.log10(math.exp(F5))
        
        return calc_base_level(inputs) + F1 + F2 + F3 + F4

    def calc_aft_tones(inputs, theta, k):
        F1 = 46 - 20 * inputs["M_TR"]
        F2 = calc_F2(inputs, -10.0)
        F3 = float(get_spl_fan(interpolator_fan,"Aft Tones",theta))
        F4 = calc_tone_F4(inputs, k)
        
        return calc_base_level(inputs) + F1 + F2 + F3 + F4

    def calc_combination_tones(inputs, f, theta):
        """Only valid for supersonic tip speeds."""
        F3 = float(get_spl_fan(interpolator_fan,"Combination Tones",theta))
        F2 = 0
        M_TR = inputs["M_TR"]
        f_b = inputs["f_b"]
        base_spl = calc_base_level(inputs)
        
        # 1/2 BPF Peak
        F1_half = -18 + 46.5*(M_TR - 1)/0.146 if M_TR < 1.146 else 28.5 - 12*(M_TR - 1.146)/0.854
        F4_half = 20 * math.log10(f / (0.5 * f_b)) if f < 0.5 * f_b else -20 * math.log10(f / (0.5 * f_b))
        spl_half = base_spl + F1_half + F2 + F3 + F4_half

        # 1/4 BPF Peak
        F1_quarter = -15 + 47.5*(M_TR - 1)/0.322 if M_TR < 1.322 else 32.5 - 9*(M_TR - 1.322)/0.678
        F4_quarter = 30 * math.log10(f / (0.25 * f_b)) if f < 0.25 * f_b else -30 * math.log10(f / (0.25 * f_b))
        spl_quarter = base_spl + F1_quarter + F2 + F3 + F4_quarter

        # 1/8 BPF Peak
        F1_eighth = -12 + 41.2*(M_TR - 1)/0.61 if M_TR < 1.61 else 29.2 - 4.7*(M_TR - 1.61)/0.39
        F4_eighth = 30 * math.log10(f / (0.125 * f_b)) if f < 0.125 * f_b else -20 * math.log10(f / (0.125 * f_b))
        spl_eighth = base_spl + F1_eighth + F2 + F3 + F4_eighth

        # Combine the sub-spectra logarithmically
        total_spl = 10 * math.log10(10**(spl_half/10) + 10**(spl_quarter/10) + 10**(spl_eighth/10))
        return total_spl

    def add_spl(*levels):
        """Logarithmically adds multiple SPL values."""
        # Convert dB to linear energy, sum them (ignoring very small values)
        energy_sum = sum(10**(lvl/10) for lvl in levels if lvl > -50) 
        
        if energy_sum == 0:
            return -50
            
        return 10 * math.log10(energy_sum)
    
    fan_noise= Data()

    for i in range(n_mic):
        spl_values = []
        theta     =  np.zeros(n_mic)
        bool_1    = (microphone_locations[:,1] > 0) &  (microphone_locations[:,0] > 0)
        bool_2    = (microphone_locations[:,1] > 0) &  (microphone_locations[:,0] < 0)
        bool_3    = (microphone_locations[:,1] < 0) &  (microphone_locations[:,0] < 0)
        bool_4    = (microphone_locations[:,1] < 0) &  (microphone_locations[:,0] > 0)
        
        theta[bool_1] =  np.pi - np.arctan(microphone_locations[:,1]/microphone_locations[:,0])[bool_1]
        theta[bool_2] =  np.arctan(microphone_locations[:,1]/ abs(microphone_locations[:,0]))[bool_2]
        theta[bool_3] =  np.arctan(abs(microphone_locations[:,1])/ abs(microphone_locations[:,0]))[bool_3]
        theta[bool_4] =  np.pi - np.arctan(abs(microphone_locations[:,1])/ microphone_locations[:,0])[bool_4] 

        theta_S                = np.tile(theta[None,:],(n_cpts,1))  
        theta_s = np.tile(np.atleast_2d(abs(theta_S[:,j])).T,(1,n_freq))
        spl = calc_combination_tones(fan_inputs, frequency, theta=theta_s)
        spl_values.append(spl)



        SPL_1_3_spectrum[:,i,:]       = spl 
        SPL[:,i]                      = SPL_arithmetic(spl,sum_axis=1 )
        SPL_1_3_spectrum_dBA[:,i,:]   = A_weighting_metric(spl,frequency)
        SPL_dBA[:,i]                  = SPL_arithmetic(np.atleast_2d(A_weighting_metric(spl,frequency)),sum_axis=1)


    

    fan_noise.SPL_1_3_spectrum  = SPL_1_3_spectrum_dBA
    fan_noise.SPL               = SPL
    fan_noise.SPL_dBA           = SPL_dBA
    return fan_noise

