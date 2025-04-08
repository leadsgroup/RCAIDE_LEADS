# RCAIDE/Methods/Energy/Propulsors/Turbojet/compute_thrust.py
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
def compute_thrust(turbojet,conditions):
    """
    Computes thrust and other performance metrics for a turbojet engine.
    
    Parameters
    ----------
    turbojet : RCAIDE.Library.Components.Propulsors.Turbojet
        Turbojet engine component with the following attributes:
            - tag : str
                Identifier for the turbojet
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
            - energy.propulsors[turbojet.tag] : Data
                Turbojet-specific conditions
                - fuel_to_air_ratio : numpy.ndarray
                    Fuel-to-air ratio
                - total_temperature_reference : numpy.ndarray
                    Reference total temperature [K]
                - total_pressure_reference : numpy.ndarray
                    Reference total pressure [Pa]
                - core_nozzle_exit_velocity : numpy.ndarray
                    Core nozzle exit velocity [m/s]
                - core_nozzle_static_pressure : numpy.ndarray
                    Core nozzle static pressure [Pa]
                - core_nozzle_area_ratio : numpy.ndarray
                    Core nozzle area ratio
                - flow_through_core : numpy.ndarray
                    Fraction of flow through core
                - throttle : numpy.ndarray
                    Throttle setting [0-1]
    
    Returns
    -------
    None
        Results are stored in conditions.energy.propulsors[turbojet.tag]:
          - thrust : numpy.ndarray
              Thrust force [N]
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
    This function implements a thermodynamic model for a turbojet engine to calculate
    thrust, fuel consumption, and other performance metrics. It uses the outputs from
    the core nozzle to determine the overall engine performance.
    
    **Major Assumptions**
        * Perfect gas behavior
        * Thrust is calculated from momentum and pressure forces at the nozzle exit
    
    **Theory**
    The non-dimensional thrust is calculated as:
    
    .. math::
        F_{nd} = \\phi_{core} \\cdot (\\gamma \\cdot M_0^2 \\cdot (V_{core}/V_0 - 1) + A_{core} \\cdot (P_{core}/P_0 - 1))
    
    where:
      - φ_core is the flow through core fraction
      - γ is the ratio of specific heats
      - M_0 is the freestream Mach number
      - V_core is the core nozzle exit velocity
      - V_0 is the freestream velocity
      - A_core is the core nozzle area ratio
      - P_core is the core nozzle static pressure
      - P_0 is the freestream pressure
    
    The specific thrust is then:
    
    .. math::
        F_{sp} = \\frac{F_{nd}}{\\gamma \\cdot M_0}
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University. https://web.stanford.edu/~cantwell/AA283_Course_Material/
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.compute_turbojet_performance
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.size_core
    """            
    # Unpacking from conditions
    gamma                       = conditions.freestream.isentropic_expansion_factor 
    u0                          = conditions.freestream.velocity
    a0                          = conditions.freestream.speed_of_sound
    M0                          = conditions.freestream.mach_number
    p0                          = conditions.freestream.pressure  
    g                           = conditions.freestream.gravity        

    # Unpacking from inputs
    Tref                        = turbojet.reference_temperature
    Pref                        = turbojet.reference_pressure
    mdhc                        = turbojet.compressor_nondimensional_massflow
    SFC_adjustment              = turbojet.specific_fuel_consumption_reduction_factor 
    turbojet_conditions         = conditions.energy.propulsors[turbojet.tag]
    f                           = turbojet_conditions.fuel_to_air_ratio
    total_temperature_reference = turbojet_conditions.total_temperature_reference
    total_pressure_reference    = turbojet_conditions.total_pressure_reference   
    core_area_ratio             = turbojet_conditions.core_nozzle_area_ratio  
    V_core_nozzle               = turbojet_conditions.core_nozzle_exit_velocity
    P_core_nozzle               = turbojet_conditions.core_nozzle_static_pressure     
    flow_through_core           = turbojet_conditions.flow_through_core  
 
    # Computing the non dimensional thrust
    core_thrust_nondimensional  = flow_through_core*(gamma*M0*M0*(V_core_nozzle/u0-1.) + core_area_ratio*( P_core_nozzle/p0-1.)) 

    Thrust_nd                   = core_thrust_nondimensional  

    # Computing Specifc Thrust
    Fsp              = 1./(gamma*M0)*Thrust_nd

    # Computing the specific impulse
    Isp              = Fsp*a0/(f*g)

    # Computing the TSFC
    TSFC             = f*g/(Fsp*a0)*(1.-SFC_adjustment) * Units.hour # 1/s is converted to 1/hr here

    # Computing the core mass flow
    mdot_core        = mdhc*np.sqrt(Tref/total_temperature_reference)*(total_pressure_reference/Pref)

    # Computing the dimensional thrust
    FD2              = Fsp*a0*mdot_core* turbojet_conditions.throttle

    # Fuel flow rate
    a = np.array([0.])        
    fuel_flow_rate   = np.fmax(FD2*TSFC/g,a)*1./Units.hour

    # Computing the power 
    power            = FD2*u0

    # pack outputs 
    turbojet_conditions.thrust                            = FD2 
    turbojet_conditions.thrust_specific_fuel_consumption  = TSFC
    turbojet_conditions.non_dimensional_thrust            = Fsp 
    turbojet_conditions.core_mass_flow_rate               = mdot_core
    turbojet_conditions.fuel_flow_rate                    = fuel_flow_rate    
    turbojet_conditions.power                             = power   
    turbojet_conditions.specific_impulse                  = Isp

    return 