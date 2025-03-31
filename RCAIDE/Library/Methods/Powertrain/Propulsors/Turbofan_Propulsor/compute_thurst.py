# RCAIDE/Methods/Energy/Propulsors/Turbofan_Propulsor/compute_thrust.py
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
def compute_thrust(turbofan,turbofan_conditions,conditions):
    """
    Computes thrust and related performance metrics for a turbofan engine based on operating conditions and engine parameters.

    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Propulsors.Turbofan
        Turbofan engine instance containing reference conditions and geometry
    turbofan_conditions : Conditions
        Container for turbofan operating conditions and outputs
            - fuel_to_air_ratio : float
                Ratio of fuel mass to air mass
            - total_temperature_reference : float
                Reference total temperature [K]
            - total_pressure_reference : float
                Reference total pressure [Pa]
            - flow_through_core : float
                Core mass flow rate coefficient
            - flow_through_fan : float
                Fan mass flow rate coefficient
            - fan/core_nozzle properties : float
                Various nozzle exit conditions
    conditions : Conditions
        Flight conditions container
            - freestream : Conditions
                Atmospheric and flight conditions
                    - velocity : array
                        Flight velocity [m/s]
                    - pressure : array
                        Ambient pressure [Pa]
                    - other properties : various
                        Additional atmospheric properties

    Returns
    -------
    None
        Results are stored in turbofan_conditions:
            - thrust : array
                Total engine thrust [N]
            - thrust_specific_fuel_consumption : array
                TSFC [N/N-s]
            - non_dimensional_thrust : array
                Non-dimensional thrust coefficient
            - core_mass_flow_rate : array
                Core mass flow rate [kg/s]
            - fuel_flow_rate : array
                Fuel consumption rate [kg/s]
            - power : array
                Engine power output [W]

    Notes
    -----
    The function implements standard turbofan performance calculations including:
        * Non-dimensional thrust computation for core and fan
        * Specific thrust and specific impulse
        * TSFC and fuel flow calculations
        * Core mass flow rate determination

    **Major Assumptions**
        * Perfect gas behavior
        * Quasi-steady flow
        * No installation effects
        * Ideal nozzle expansion

    **Theory**
    The thrust calculation follows standard gas turbine theory:

    .. math::
        F_{sp} = \\frac{1}{\\gamma M_0}[\\dot{m}_f(\\frac{V_f}{V_0}-1) + A_f(\\frac{P_f}{P_0}-1) + \\dot{m}_c(\\frac{V_c}{V_0}-1) + A_c(\\frac{P_c}{P_0}-1)]

    Where subscripts f and c denote fan and core properties respectively.

    References
    ----------
    [1] Cantwell, B., "AA283 Aircraft and Rocket Propulsion", Stanford University Course Notes, 
        https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/

    See Also
    --------
    RCAIDE.Library.Methods.Propulsors.Turbofan_Propulsor.compute_turbofan_performance
    RCAIDE.Library.Methods.Propulsors.Turbofan_Propulsor.size_core
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
    SFC_adjustment              = turbofan.SFC_adjustment 
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