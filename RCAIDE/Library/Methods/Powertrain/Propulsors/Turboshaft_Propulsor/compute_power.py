# RCAIDE/Library/Methods/Energy/Propulsors/Turboshaft_Propulsor/compute_power.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Jun 2024, M. Guidotti & D.J. Lee

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 
# Python package imports
import numpy                               as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_power
# ----------------------------------------------------------------------------------------------------------------------

def compute_power(turboshaft,turboshaft_conditions,conditions):
    """Computes power and other properties as below.

    Assumptions:
    Perfect gas
    Turboshaft engine with free power turbine

    Sources:
    [1] https://soaneemrana.org/onewebmedia/ELEMENTS%20OF%20GAS%20TURBINE%20PROPULTION2.pdf - Page 332 - 336
    [2] https://www.colorado.edu/faculty/kantha/sites/default/files/attached-files/70652-116619_-_luke_stuyvenberg_-_dec_17_2015_1258_pm_-_stuyvenberg_helicopterturboshafts.pdf
    
    Inputs:
    conditions.freestream.
      isentropic_expansion_factor              [-] (gamma)
      specific_heat_at_constant_pressure       [J/(kg K)]
      velocity                                 [m/s]
      speed_of_sound                           [m/s]
      mach_number                              [-]
      pressure                                 [Pa]
      gravity                                  [m/s^2]
    conditions.throttle                        [-] (.1 is 10%)
    turboshaft.inputs.                         
      fuel_to_air_ratio                        [-]
      total_temperature_reference              [K]
      total_pressure_reference                 [Pa]
      core_nozzle.                             
        velocity                               [m/s]
        static_pressure                        [Pa]
        area_ratio                             [-]
      fan_nozzle.                              
        velocity                               [m/s]
        static_pressure                        [Pa]
        area_ratio                             [-]
      number_of_engines                        [-]
      bypass_ratio                             [-]
      flow_through_core                        [-] percentage of total flow (.1 is 10%)
      flow_through_fan                         [-] percentage of total flow (.1 is 10
    combustor.outputs.stagnation_temperature   [K]
    compressor.pressure_ratio                  [-]
    turboshaft.conversion_efficiency           [-]
                                               
    Outputs:                                   
    turboshaft.outputs.                        
      thrust                                   [N]
      non_dimensional_thrust                   [-]
      core_mass_flow_rate                      [kg/s]
      fuel_flow_rate                           [kg/s]
      power                                    [W]
      power_specific_fuel_consumption          [kg/(W*s)]
      Specific Impulse                         [s]
                                               
    Properties Used:                           
    turboshaft.                                
      reference_temperature                    [K]
      reference_pressure                       [Pa]
      compressor_nondimensional_massflow       [-]
      SFC_adjustment                           [-]
    """           
    #unpack the values
    fuel_type                                  = turboshaft.fuel_type
    LHV                                        = fuel_type.lower_heating_value                                                                        
    gamma                                      = conditions.freestream.isentropic_expansion_factor                                                      
    a0                                         = conditions.freestream.speed_of_sound                                                                   
    M0                                         = conditions.freestream.mach_number                                                                      
    Cp                                         = conditions.freestream.Cp                                                                               # Source [2]
    total_temperature_reference                = turboshaft_conditions.total_temperature_reference                                                          
    total_pressure_reference                   = turboshaft_conditions.total_pressure_reference                                                             # Source [1]
    eta_c                                      = turboshaft.conversion_efficiency                                                                       # Source [2]
                                                                                                                                                        
    #unpacking from turboshaft                                                                                                                          
    Tref                                       = turboshaft.reference_temperature                                                                       # Source [1]
    Pref                                       = turboshaft.reference_pressure                                                                          # Source [1]
    Tt4                                        = turboshaft_conditions.combustor_stagnation_temperature                                                    
    pi_c                                       = turboshaft.compressor.pressure_ratio                                                                   
    m_dot_compressor                           = turboshaft.compressor.mass_flow_rate                                                                   # Source [2]
                                                                                                                                                        
    tau_lambda                                 = Tt4/total_temperature_reference                                                                        
    tau_r                                      = 1 + ((gamma - 1)/2)*M0**2                                                                              
    tau_c                                      = pi_c**((gamma - 1)/gamma)                                                                              
    tau_t                                      = (1/(tau_r*tau_c)) + ((gamma - 1)*M0**2)/(2*tau_lambda*eta_c**2)                                        # Source [2]
    #tau_t                                      = x/(tau_r*tau_c)                                                                                      # Source [1]
    tau_tH                                     = 1 - (tau_r/tau_lambda)*(tau_c - 1)                                                                    # Source [2]
    tau_tL                                     = tau_t/tau_tH                                                                                          # Source [2]
    #x                                          = 1.02                                                                                                  # Source [1] Page 335
    x                                          = tau_t*tau_r*tau_c                                                                                     # Source [1] 
    #C_shaft                                    = tau_lambda*(1 - x/(tau_r*tau_c)) - tau_r*(tau_c - 1)                                                  # Source [1]
    C_shaft                                    = tau_lambda*(1 - tau_t) - tau_r*(tau_c - 1)                                                             # Source [1]    

    #Computing Specifc Thrust
    Tsp                                        = a0*(((2/(gamma - 1))*(tau_lambda/(tau_r*tau_c))*(tau_r*tau_c*tau_t - 1))**eta_c - M0)                  # Source [2]
    
    #computing the core mass flow              
    m_dot_air                                  = m_dot_compressor*np.sqrt(Tref/total_temperature_reference)*(total_pressure_reference/Pref)             # Source [1]
    
    #Computing Specifc Power
    #Psp                                        = Cp*total_temperature_reference*C_shaft                                                                 # Source [1] 
    Psp                                        =  Cp*total_temperature_reference*tau_lambda*tau_tH*(1 - tau_tL)*eta_c                                   # Source [2]    
    
    #Computing Power 
    Power                                      = Psp*m_dot_air                                                                                          
    #Power                                      = m_dot_air*Cp*total_temperature_reference*(tau_lambda*(1 - tau_t) - tau_r*(tau_c - 1))                 # Source [2]

    #fuel to air ratio
    f                                          = (Cp*total_temperature_reference/LHV)*(tau_lambda - tau_r*tau_c)                                        # Source [2]    
                                                                                                                                               
    #fuel flow rate                             
    #fuel_flow_rate                             = Power*PSFC*1./Units.hour                                                                              # Source [1]
    fuel_flow_rate                             = f*m_dot_air
    
    #Computing the PSFC                        
    PSFC                                       = f/Psp                                                                                                  
    #PSFC                                       = (tau_lambda/(C_shaft*LHV))                                                                            # Source [1]  
    
    #Computing the thermal efficiency                       
    eta_T                                      = 1 - (tau_r*(tau_c - 1))/(tau_lambda*(1 - x/(tau_r*tau_c)))                                             # Source [1]    

    #pack outputs
    turboshaft_conditions.power_specific_fuel_consumption   = PSFC
    turboshaft_conditions.fuel_flow_rate                    = fuel_flow_rate                                                                              
    turboshaft_conditions.power                             = Power
    turboshaft_conditions.non_dimensional_power             = Psp
    turboshaft_conditions.non_dimensional_thrust            = Tsp
    turboshaft_conditions.thermal_efficiency                = eta_T

    return 
