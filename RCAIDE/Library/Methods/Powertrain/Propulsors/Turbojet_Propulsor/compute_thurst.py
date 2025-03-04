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
    """Computes thrust and other properties as below.

    Assumptions:
    Perfect gas

    Source:
    https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/

    Inputs:
    conditions.freestream.
      isentropic_expansion_factor        [-] (gamma)
      specific_heat_at_constant_pressure [J/(kg K)]
      velocity                           [m/s]
      speed_of_sound                     [m/s]
      mach_number                        [-]
      pressure                           [Pa]
      gravity                            [m/s^2]
    conditions.throttle                  [-] (.1 is 10%)
    turbojet_conditions.
      fuel_to_air_ratio                  [-]
      total_temperature_reference        [K]
      total_pressure_reference           [Pa]
      core_nozzle.
        velocity                         [m/s]
        static_pressure                  [Pa]
        area_ratio                       [-]
      fan_nozzle.
        velocity                         [m/s]
        static_pressure                  [Pa]
        area_ratio                       [-]
      number_of_engines                  [-]
      bypass_ratio                       [-]
      flow_through_core                  [-] percentage of total flow (.1 is 10%)
      flow_through_fan                   [-] percentage of total flow (.1 is 10%)

    Outputs:
    turbojet_conditions.
      thrust                             [N]
      thrust_specific_fuel_consumption   [N/N-s]
      non_dimensional_thrust             [-]
      core_mass_flow_rate                [kg/s]
      fuel_flow_rate                     [kg/s]
      power                              [W]
      Specific Impulse                   [s]

    Properties Used:
    turbojet.
      reference_temperature              [K]
      reference_pressure                 [Pa]
      compressor_nondimensional_massflow [-]
      SFC_adjustment                     [-]
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