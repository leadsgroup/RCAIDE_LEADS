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
def compute_power(turboshaft, conditions):
    """
    Computes power and other performance properties for a turboshaft engine.

    Parameters
    ----------
    turboshaft : RCAIDE.Library.Components.Converters.Turboshaft
        Turboshaft engine component with the following attributes:
            - tag : str
                Identifier for the turboshaft
            - fuel_type : Data
                Fuel properties object
                - lower_heating_value : float
                    Fuel lower heating value [J/kg]
            - working_fluid : Data
                Working fluid properties object
            - reference_temperature : float
                Reference temperature for mass flow scaling [K]
            - reference_pressure : float
                Reference pressure for mass flow scaling [Pa]
            - compressor : Data
                Compressor component
                - pressure_ratio : float
                    Compressor pressure ratio
                - mass_flow_rate : float
                    Design mass flow rate [kg/s]
            - conversion_efficiency : float
                Efficiency of converting thermal energy to shaft power
            - inverse_calculation : bool
                Flag for inverse calculation mode (power to throttle)
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - freestream : Data
                Freestream properties
                - isentropic_expansion_factor : numpy.ndarray
                    Ratio of specific heats (gamma)
                - speed_of_sound : numpy.ndarray
                    Speed of sound [m/s]
                - mach_number : numpy.ndarray
                    Flight Mach number
                - gravity : numpy.ndarray
                    Gravitational acceleration [m/s²]
            - energy.converters[turboshaft.tag] : Data
                Turboshaft operating conditions
                - throttle : numpy.ndarray
                    Throttle setting [0-1]
                - total_temperature_reference : numpy.ndarray
                    Reference total temperature [K]
                - total_pressure_reference : numpy.ndarray
                    Reference total pressure [Pa]
                - combustor_stagnation_temperature : numpy.ndarray
                    Combustor exit stagnation temperature [K]
                - power : numpy.ndarray
                    Required power output (for inverse calculation) [W]

    Returns
    -------
    None
        Results are stored in conditions.energy.converters[turboshaft.tag]:
        - power_specific_fuel_consumption : numpy.ndarray
            Power specific fuel consumption [kg/(W*s)]
        - fuel_flow_rate : numpy.ndarray
            Fuel flow rate [kg/s]
        - power : numpy.ndarray
            Shaft power output [W]
        - non_dimensional_power : numpy.ndarray
            Non-dimensional power
        - non_dimensional_thrust : numpy.ndarray
            Non-dimensional thrust
        - thermal_efficiency : numpy.ndarray
            Thermal efficiency of the engine

    Notes
    -----
    This function implements a thermodynamic model for a turboshaft engine with a free power turbine.
    It can operate in two modes: direct (throttle to power) or inverse (power to throttle).
    
    **Major Assumptions**
        * Perfect gas behavior
        * Turboshaft engine with free power turbine
        * Constant component efficiencies
    
    **Theory**
    The turboshaft performance is calculated using gas turbine cycle analysis. The power output
    is determined by the temperature rise in the combustor, the compressor pressure ratio, and
    the component efficiencies.
    
    References
    ----------
    [1] Mattingly, J.D., "Elements of Gas Turbine Propulsion", 2nd Edition, AIAA Education Series, 2005.
        https://soaneemrana.org/onewebmedia/ELEMENTS%20OF%20GAS%20TURBINE%20PROPULTION2.pdf
    [2] Stuyvenberg, L., "Helicopter Turboshafts", University of Colorado, 2015.
        https://www.colorado.edu/faculty/kantha/sites/default/files/attached-files/70652-116619_-_luke_stuyvenberg_-_dec_17_2015_1258_pm_-_stuyvenberg_helicopterturboshafts.pdf
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Turbofan.compute_thrust
    RCAIDE.Library.Methods.Powertrain.Converters.Turbojet.compute_thrust
    """
    #unpack the values
    fuel_type                                  = turboshaft.fuel_type
    working_fluid                              = turboshaft.working_fluid
    LHV                                        = fuel_type.lower_heating_value                                                                        
    gamma                                      = conditions.freestream.isentropic_expansion_factor                                                      
    a0                                         = conditions.freestream.speed_of_sound                                                                   
    M0                                         = conditions.freestream.mach_number
    turboshaft_conditions                      = conditions.energy.converters[turboshaft.tag]  
    total_temperature_reference                = turboshaft_conditions.total_temperature_reference                                                          
    total_pressure_reference                   = turboshaft_conditions.total_pressure_reference                                                            
    Power                                      = turboshaft_conditions.power                                                           
    eta_c                                      = turboshaft.conversion_efficiency 
                                                                                                                                                        
    #unpacking from turboshaft                                                                                                                          
    Tref                                       = turboshaft.reference_temperature                                                                   
    Pref                                       = turboshaft.reference_pressure                                                            
    Cp                                         = working_fluid.compute_cp(total_temperature_reference,total_pressure_reference)                                                             
    Tt4                                        = turboshaft_conditions.combustor_stagnation_temperature                                                    
    pi_c                                       = turboshaft.compressor.pressure_ratio                                                                   
    m_dot_compressor                           = turboshaft.compressor.mass_flow_rate                                                                 
                                                                                                                                                        
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
    fuel_flow_rate                             = f*m_dot_air
    
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
