# RCAIDE/Library/Methods/Energy/Converters/Turboshaft/compute_power.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Jun 2024, M. Guidotti  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 
# Python package imports
import numpy                               as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_power
# ----------------------------------------------------------------------------------------------------------------------
def compute_power(turboshaft,conditions):
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
    working_fluid                              = turboshaft.working_fluid                                                                
    Tref                                       = turboshaft.reference_temperature                                                                   
    Pref                                       = turboshaft.reference_pressure                                  
    eta_c                                      = turboshaft.conversion_efficiency 
    SFC_adjustment                             = turboshaft.specific_fuel_consumption_reduction_factor                                                                                      
    Tt4                                        = turboshaft_conditions.combustor_stagnation_temperature                                                    
    pi_c                                       = turboshaft.compressor.pressure_ratio                                                                   
    m_dot_compressor                           = turboshaft.compressor.mass_flow_rate  
    LHV                                        = turboshaft.fuel_type.lower_heating_value                                                                        
    gamma                                      = conditions.freestream.isentropic_expansion_factor                                                      
    a0                                         = conditions.freestream.speed_of_sound                                                                   
    M0                                         = conditions.freestream.mach_number
    turboshaft_conditions                      = conditions.energy.converters[turboshaft.tag]  
    total_temperature_reference                = turboshaft_conditions.total_temperature_reference                                                          
    total_pressure_reference                   = turboshaft_conditions.total_pressure_reference                                  
    Cp                                         = working_fluid.compute_cp(total_temperature_reference,total_pressure_reference)                                                           
    Power                                      = turboshaft_conditions.power                            
                                                                                                                                                        
    #unpacking from turboshaft                                                                                                                         
                                                                                                                                                        
    tau_lambda                                 = Tt4/total_temperature_reference                                                                        
    tau_r                                      = 1 + ((gamma - 1)/2)*M0**2                                                                              
    tau_c                                      = pi_c**((gamma - 1)/gamma)                                                                              
    tau_t                                      = (1/(tau_r*tau_c)) + ((gamma - 1)*M0**2)/(2*tau_lambda*eta_c**2)                                      
    tau_tH                                     = 1 - (tau_r/tau_lambda)*(tau_c - 1)                                                                   
    tau_tL                                     = tau_t/tau_tH                                                                                         
    x                                          = tau_t*tau_r*tau_c                                                                                    

    # Computing Specifc Thrust and Power 
    Tsp                                        = a0*(((2/(gamma - 1))*(tau_lambda/(tau_r*tau_c))*(tau_r*tau_c*tau_t - 1))**eta_c - M0)                
    Psp                                        =  Cp*total_temperature_reference*tau_lambda*tau_tH*(1 - tau_tL)*eta_c     
        
    if turboshaft.inverse_calculation == False: 
        m_dot_air   = m_dot_compressor*turboshaft_conditions.throttle*np.sqrt(Tref/total_temperature_reference)*(total_pressure_reference/Pref)     
        Power       = Psp*m_dot_air
    else:
        m_dot_air = Power / Psp
        turboshaft_conditions.throttle =  m_dot_air / (m_dot_compressor*np.sqrt(Tref/total_temperature_reference)*(total_pressure_reference/Pref) )
         
    #fuel to air ratio
    f                                          = (Cp*total_temperature_reference/LHV)*(tau_lambda - tau_r*tau_c)                                                                              
    fuel_flow_rate                             = (1 - SFC_adjustment) *f*m_dot_air
    
    #Computing the PSFC                        
    PSFC                                       = f/Psp                                                                                                
    
    #Computing the thermal efficiency                       
    eta_T                                      = 1 - (tau_r*(tau_c - 1))/(tau_lambda*(1 - x/(tau_r*tau_c)))                               

    #pack outputs
    turboshaft_conditions.power_specific_fuel_consumption   = PSFC
    turboshaft_conditions.fuel_flow_rate                    = fuel_flow_rate                                                                              
    turboshaft_conditions.power                             = Power
    turboshaft_conditions.non_dimensional_power             = Psp
    turboshaft_conditions.non_dimensional_thrust            = Tsp
    turboshaft_conditions.thermal_efficiency                = eta_T        

    return 
