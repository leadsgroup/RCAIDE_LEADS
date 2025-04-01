# RCAIDE/Methods/Energy/Propulsors/Turbojet_Propulsor/compute_thrust.py
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
def compute_thrust(turbojet,turbojet_conditions,conditions):
  """
    Computes thrust and related performance parameters for a turbojet engine.
    
    Parameters
    ----------
    turbojet : Turbojet
        Turbojet engine object containing design parameters
            - reference_temperature : float
                Reference temperature [K]
            - reference_pressure : float
                Reference pressure [Pa]
            - compressor_nondimensional_massflow : float
                Non-dimensional mass flow parameter [-]
            - SFC_adjustment : float
                Thrust specific fuel consumption adjustment factor [-]
    turbojet_conditions : Data
        Operating conditions for the turbojet
            - fuel_to_air_ratio : float
                Fuel-to-air ratio [-]
            - total_temperature_reference : float
                Reference total temperature [K]
            - total_pressure_reference : float
                Reference total pressure [Pa]
            - core_nozzle_area_ratio : float
                Core nozzle area ratio [-]
            - core_nozzle_exit_velocity : float
                Core nozzle exit velocity [m/s]
            - core_nozzle_static_pressure : float
                Core nozzle static pressure [Pa]
            - flow_through_core : float
                Core mass flow fraction [-]
            - throttle : float
                Throttle setting [-]
    conditions : Data
        Flight conditions
            - freestream.isentropic_expansion_factor : float
                Ratio of specific heats (gamma) [-]
            - freestream.velocity : float
                Freestream velocity [m/s]
            - freestream.speed_of_sound : float
                Freestream speed of sound [m/s]
            - freestream.mach_number : float
                Freestream Mach number [-]
            - freestream.pressure : float
                Freestream pressure [Pa]
            - freestream.gravity : float
                Gravitational acceleration [m/s^2]
    
    Returns
    -------
    None
        Updates turbojet_conditions with:
            - thrust : float
                Engine thrust [N]
            - thrust_specific_fuel_consumption : float
                TSFC [1/hr]
            - non_dimensional_thrust : float
                Non-dimensional thrust [-]
            - core_mass_flow_rate : float
                Core mass flow rate [kg/s]
            - fuel_flow_rate : float
                Fuel mass flow rate [kg/s]
            - power : float
                Engine power [W]
            - specific_impulse : float
                Specific impulse [s]
    
    Notes
    -----
    This function computes thrust and performance parameters using one-dimensional gas dynamics
    and perfect gas assumptions.
    
    **Major Assumptions**
        * Perfect gas behavior
        * One-dimensional flow
        * Steady state operation
        * Adiabatic nozzle flow
    
    **Theory**
    The non-dimensional thrust is computed as:
    
    .. math::
        F_{nd} = \\phi_c(\\gamma M_0^2(\\frac{V_e}{V_0}-1) + A_r(\\frac{P_e}{P_0}-1))
    
    where:
        - :math:`\\phi_c` is the core mass flow fraction
        - :math:`\\gamma` is the ratio of specific heats
        - :math:`M_0` is the freestream Mach number
        - :math:`V_e/V_0` is the velocity ratio
        - :math:`A_r` is the nozzle area ratio
        - :math:`P_e/P_0` is the pressure ratio
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Material: Aircraft and Rocket Propulsion", Stanford University, https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet_Propulsor.size_core
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet_Propulsor.compute_turbojet_performance
    """           
  #unpack the values

  #unpacking from conditions
  gamma                       = conditions.freestream.isentropic_expansion_factor 
  u0                          = conditions.freestream.velocity
  a0                          = conditions.freestream.speed_of_sound
  M0                          = conditions.freestream.mach_number
  p0                          = conditions.freestream.pressure  
  g                           = conditions.freestream.gravity        

  #unpacking from inputs
  Tref                        = turbojet.reference_temperature
  Pref                        = turbojet.reference_pressure
  mdhc                        = turbojet.compressor_nondimensional_massflow
  SFC_adjustment              = turbojet.SFC_adjustment 
  f                           = turbojet_conditions.fuel_to_air_ratio
  total_temperature_reference = turbojet_conditions.total_temperature_reference
  total_pressure_reference    = turbojet_conditions.total_pressure_reference   
  core_area_ratio             = turbojet_conditions.core_nozzle_area_ratio  
  V_core_nozzle               = turbojet_conditions.core_nozzle_exit_velocity
  P_core_nozzle               = turbojet_conditions.core_nozzle_static_pressure     
  flow_through_core           = turbojet_conditions.flow_through_core  

  #computing the non dimensional thrust
  core_thrust_nondimensional  = flow_through_core*(gamma*M0*M0*(V_core_nozzle/u0-1.) + core_area_ratio*( P_core_nozzle/p0-1.)) 

  Thrust_nd                   = core_thrust_nondimensional  

  #Computing Specifc Thrust
  Fsp              = 1./(gamma*M0)*Thrust_nd

  #Computing the specific impulse
  Isp              = Fsp*a0/(f*g)

  #Computing the TSFC
  TSFC             = f*g/(Fsp*a0)*(1.-SFC_adjustment) * Units.hour # 1/s is converted to 1/hr here

  #computing the core mass flow
  mdot_core        = mdhc*np.sqrt(Tref/total_temperature_reference)*(total_pressure_reference/Pref)

  #computing the dimensional thrust
  FD2              = Fsp*a0*mdot_core* turbojet_conditions.throttle

  #fuel flow rate
  a = np.array([0.])        
  fuel_flow_rate   = np.fmax(FD2*TSFC/g,a)*1./Units.hour

  #computing the power 
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