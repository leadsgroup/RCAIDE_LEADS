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
    """
    Computes the power output and performance characteristics of a turboshaft engine.

    Parameters
    ----------
    turboshaft : Turboshaft
        Turboshaft engine object containing component definitions
            - fuel_type : Fuel
                Fuel object with properties including lower_heating_value
            - conversion_efficiency : float
                Engine power conversion efficiency
            - reference_temperature : float
                Reference temperature [K]
            - reference_pressure : float
                Reference pressure [Pa]
            - compressor : Compressor
                Compressor object with pressure_ratio and mass_flow_rate
    turboshaft_conditions : Conditions
        Turboshaft operating conditions data structure
            - total_temperature_reference : float
                Reference total temperature [K]
            - total_pressure_reference : float
                Reference total pressure [Pa]
            - combustor_stagnation_temperature : float
                Combustor outlet stagnation temperature [K]
    conditions : Conditions
        Freestream conditions data structure
            - freestream.isentropic_expansion_factor : float
                Ratio of specific heats (gamma)
            - freestream.speed_of_sound : float
                Speed of sound [m/s]
            - freestream.mach_number : float
                Mach number
            - freestream.Cp : float
                Specific heat at constant pressure [J/kg-K]

    Returns
    -------
    None
        Results are stored in turboshaft_conditions:
            - power_specific_fuel_consumption : float
                Power specific fuel consumption [kg/W-s]
            - fuel_flow_rate : float
                Fuel mass flow rate [kg/s]
            - power : float
                Power output [W]
            - non_dimensional_power : float
                Non-dimensional power output [-]
            - non_dimensional_thrust : float
                Non-dimensional thrust [-]
            - thermal_efficiency : float
                Engine thermal efficiency [-]

    Notes
    -----
    The function computes turboshaft performance using gas turbine cycle analysis
    principles for a free power turbine configuration.

    **Major Assumptions**
        * Perfect gas behavior
        * Turboshaft engine with free power turbine
        * Constant component efficiencies
        * No bleed air extraction

    **Theory**

    The analysis uses non-dimensional parameters and temperature ratios:
    
    .. math::
        \\tau_\\lambda = \\frac{T_{t4}}{T_{t_{ref}}}
    
    .. math::
        \\tau_r = 1 + \\frac{(\\gamma - 1)}{2}M_0^2
    
    .. math::
        \\tau_c = \\pi_c^{\\frac{(\\gamma - 1)}{\\gamma}}

    References
    ----------
    [1] https://soaneemrana.org/onewebmedia/ELEMENTS%20OF%20GAS%20TURBINE%20PROPULTION2.pdf
    [2] https://www.colorado.edu/faculty/kantha/sites/default/files/attached-files/70652-116619_-_luke_stuyvenberg_-_dec_17_2015_1258_pm_-_stuyvenberg_helicopterturboshafts.pdf

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboshaft_Propulsor.design_turboshaft 
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboshaft_Propulsor.size_core 
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
