# RCAIDE/Methods/Energy/Propulsors/Turbofan/compute_thrust.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports  
from RCAIDE.Framework.Core      import Units 

# Python package imports
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
#  compute_thrust
# ----------------------------------------------------------------------------------------------------------------------
def compute_thrust(turbofan,conditions):
    """
    Computes thrust and other performance metrics for a turbofan engine.
    
    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Propulsors.Turbofan
        Turbofan engine component with the following attributes:
            - tag : str
                Identifier for the turbofan
            - reference_temperature : float
                Reference temperature for mass flow scaling [K]
            - reference_pressure : float
                Reference pressure for mass flow scaling [Pa]
            - compressor_nondimensional_massflow : float
                Non-dimensional mass flow parameter [kg·√K/(s·Pa)]
            - SFC_adjustment : float
                Adjustment factor for specific fuel consumption
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - freestream : Data
                Freestream properties
                    - isentropic_expansion_factor : numpy.ndarray
                        Ratio of specific heats (gamma)
                    - velocity : numpy.ndarray
                        Freestream velocity [m/s]
                    - speed_of_sound : numpy.ndarray
                        Speed of sound [m/s]
                    - mach_number : numpy.ndarray
                        Freestream Mach number
                    - pressure : numpy.ndarray
                        Freestream pressure [Pa]
                    - gravity : numpy.ndarray
                        Gravitational acceleration [m/s²]
                    - specific_heat_at_constant_pressure : numpy.ndarray
                        Specific heat at constant pressure [J/(kg·K)]
            - energy.propulsors[turbofan.tag] : Data
                Turbofan-specific conditions
                    - fuel_to_air_ratio : numpy.ndarray
                        Fuel-to-air ratio
                    - total_temperature_reference : numpy.ndarray
                        Reference total temperature [K]
                    - total_pressure_reference : numpy.ndarray
                        Reference total pressure [Pa]
                    - fan_nozzle_exit_velocity : numpy.ndarray
                        Fan nozzle exit velocity [m/s]
                    - fan_nozzle_static_pressure : numpy.ndarray
                        Fan nozzle static pressure [Pa]
                    - fan_nozzle_area_ratio : numpy.ndarray
                        Fan nozzle area ratio
                    - core_nozzle_exit_velocity : numpy.ndarray
                        Core nozzle exit velocity [m/s]
                    - core_nozzle_static_pressure : numpy.ndarray
                        Core nozzle static pressure [Pa]
                    - core_nozzle_area_ratio : numpy.ndarray
                        Core nozzle area ratio
                    - flow_through_core : numpy.ndarray
                        Fraction of flow through core
                    - flow_through_fan : numpy.ndarray
                        Fraction of flow through fan
                    - bypass_ratio : numpy.ndarray
                        Bypass ratio
                    - throttle : numpy.ndarray
                        Throttle setting [0-1]
    
    Returns
    -------
    None
        Results are stored in conditions.energy.propulsors[turbofan.tag]:
            - thrust : numpy.ndarray
                Total thrust force [N]
            - fan_thrust : numpy.ndarray
                Thrust from fan stream [N]
            - core_thrust : numpy.ndarray
                Thrust from core stream [N]
            - thrust_specific_fuel_consumption : numpy.ndarray
                Thrust specific fuel consumption [kg/(N·hr)]
            - non_dimensional_thrust : numpy.ndarray
                Non-dimensional thrust
            - core_mass_flow_rate : numpy.ndarray
                Core mass flow rate [kg/s]
            - fuel_flow_rate : numpy.ndarray
                Fuel flow rate [kg/s]
            - power : numpy.ndarray
                Power output [W]
            - specific_impulse : numpy.ndarray
                Specific impulse [s]
    
    Notes
    -----
    This function implements a thermodynamic model for a turbofan engine to calculate
    thrust, fuel consumption, and other performance metrics. It computes thrust from
    both the core and fan streams, accounting for momentum and pressure forces.
    
    **Major Assumptions**
        * Perfect gas behavior
        * Thrust is calculated from momentum and pressure forces at the nozzle exits
    
    **Theory**
    The non-dimensional thrust components are calculated as:
    
    .. math::
        F_{nd,fan} = \\phi_{fan} \\cdot (\\gamma \\cdot M_0^2 \\cdot (V_{fan}/V_0 - 1) + A_{fan} \\cdot (P_{fan}/P_0 - 1))
        
        F_{nd,core} = \\phi_{core} \\cdot (\\gamma \\cdot M_0^2 \\cdot (V_{core}/V_0 - 1) + A_{core} \\cdot (P_{core}/P_0 - 1))
    
    where:
        * :math:`\\phi_{fan}` is the flow through fan fraction
        * :math:`\\phi_{core}` is the flow through core fraction
        * :math:`\\gamma` is the ratio of specific heats
        * :math:`M_0` is the freestream Mach number
        * :math:`V_{fan}` is the fan nozzle exit velocity
        * :math:`V_{core}` is the core nozzle exit velocity
        * :math:`V_0` is the freestream velocity
        * :math:`A_{fan}` is the fan nozzle area ratio
        * :math:`A_{core}` is the core nozzle area ratio
        * :math:`P_{fan}` is the fan nozzle static pressure
        * :math:`P_{core}` is the core nozzle static pressure
        * :math:`P_0` is the freestream pressure
    
    The specific thrust is then:
    
    .. math::
        F_{sp} = \\frac{F_{nd,fan} + F_{nd,core}}{\\gamma \\cdot M_0}
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University. https://web.stanford.edu/~cantwell/AA283_Course_Material/
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.compute_turbofan_performance
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.size_core
    """      
    # Unpack flight conditions 
    gamma                       = conditions.freestream.isentropic_expansion_factor 
    u0                          = conditions.freestream.velocity
    a0                          = conditions.freestream.speed_of_sound
    M0                          = conditions.freestream.mach_number
    p0                          = conditions.freestream.pressure  
    g                           = conditions.freestream.gravity        

    # Unpack turbofan operating conditions and properties 
    Tref                        = turbofan.reference_temperature
    Pref                        = turbofan.reference_pressure
    mdhc                        = turbofan.compressor_nondimensional_massflow
    SFC_adjustment              = turbofan.specific_fuel_consumption_reduction_factor 
    turbofan_conditions         = conditions.energy.propulsors[turbofan.tag]
    f                           = turbofan_conditions.fuel_to_air_ratio
    total_temperature_reference = turbofan_conditions.total_temperature_reference
    total_pressure_reference    = turbofan_conditions.total_pressure_reference 
    flow_through_core           = turbofan_conditions.flow_through_core 
    flow_through_fan            = turbofan_conditions.flow_through_fan  
    V_fan_nozzle                = turbofan_conditions.fan_nozzle_exit_velocity
    fan_area_ratio              = turbofan_conditions.fan_nozzle_area_ratio
    P_fan_nozzle                = turbofan_conditions.fan_nozzle_static_pressure
    P_core_nozzle               = turbofan_conditions.core_nozzle_static_pressure
    V_core_nozzle               = turbofan_conditions.core_nozzle_exit_velocity
    core_area_ratio             = turbofan_conditions.core_nozzle_area_ratio                   
    bypass_ratio                = turbofan_conditions.bypass_ratio  

    # Compute  non dimensional thrust
    fan_thrust_nondim   = flow_through_fan*(gamma*M0*M0*(V_fan_nozzle/u0-1.) + fan_area_ratio*(P_fan_nozzle/p0-1.))
    core_thrust_nondim  = flow_through_core*(gamma*M0*M0*(V_core_nozzle/u0-1.) + core_area_ratio*(P_core_nozzle/p0-1.))

    thrust_nondim       = core_thrust_nondim + fan_thrust_nondim

    # Computing Specifc Thrust
    Fsp   = 1./(gamma*M0)*thrust_nondim
    Fsp_c = 1./(gamma*M0)*core_thrust_nondim
    Fsp_f = 1./(gamma*M0)*fan_thrust_nondim

    # Compute specific impulse
    Isp   = Fsp*a0*(1.+bypass_ratio)/(f*g)

    # Compute TSFC
    TSFC  = f*g/(Fsp*a0*(1.+bypass_ratio))*(1.-SFC_adjustment) * Units.hour # 1/s is converted to 1/hr here
 
    # Compute core mass flow
    mdot_core  = mdhc*np.sqrt(Tref/total_temperature_reference)*(total_pressure_reference/Pref)

    # Compute dimensional thrust
    FD2   = Fsp*a0*(1.+bypass_ratio)*mdot_core*turbofan_conditions.throttle
    FD2_f = Fsp_f*a0*(1.+bypass_ratio)*mdot_core*turbofan_conditions.throttle
    FD2_c = Fsp_c*a0*(1.+bypass_ratio)*mdot_core*turbofan_conditions.throttle

    # Compute power 
    power   = FD2*u0    

    # Compute fuel flow rate 
    fuel_flow_rate   = np.fmax(FD2*TSFC/g,np.array([0.]))*1./Units.hour

    # Pack turbofan outouts  
    turbofan_conditions.thrust                            = FD2 
    turbofan_conditions.fan_thrust                        = FD2_f 
    turbofan_conditions.core_thrust                       = FD2_c 
    turbofan_conditions.thrust_specific_fuel_consumption  = TSFC
    turbofan_conditions.non_dimensional_thrust            = Fsp  
    turbofan_conditions.power                             = power   
    turbofan_conditions.specific_impulse                  = Isp
    turbofan_conditions.core_mass_flow_rate               = mdot_core
    turbofan_conditions.fuel_flow_rate                    = fuel_flow_rate   
    
    return  