# RCAIDE/Library/Methods/Powertrain/Converters/Ducted_Fan/compute_ducted_fan_performance.py

# 
# Created:  Jan 2025, M. Clarke
# Modified: Jan 2025, M. Clarke, M. Guidotti    

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports
from RCAIDE.Framework.Core    import Data , Units, orientation_product, orientation_transpose  

# package imports
import  numpy as  np 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Generalized Rotor Class
# ---------------------------------------------------------------------------------------------------------------------- 
def BEMT_performance(ducted_fan,ducted_fan_conditions,conditions):

    """
    Computes ducted fan performance characteristics Blade Element Momentum Theory (BEMT)  

    Parameters
    ----------
    propulsor : Converter
        Ducted fan propulsor component containing the ducted fan
    state : Conditions
        Mission segment state conditions

    Returns
    -------
    None
        Updates state.conditions.energy.converters[ducted_fan.tag] with computed performance data:
            - thrust : array(N,3)
                Thrust vector [N]
            - power : array(N,1)
                Power required [W]
            - torque : array(N,1)
                Shaft torque [N-m]
            - moment : array(N,3)
                Moment vector [N-m]
            - efficiency : array(N,1)
                Propulsive efficiency [-]
            - tip_mach : array(N,1)
                Blade tip Mach number [-]
            - thrust_coefficient : array(N,1)
                Non-dimensional thrust coefficient [-]
            - power_coefficient : array(N,1)
                Non-dimensional power coefficient [-]
            - figure_of_merit : array(N,1)
                Hovering figure of merit [-] (BEMT only)

    Notes
    -----  
    **Major Assumptions**
        * Steady state operation
        * Incompressible flow for Rankine-Froude theory
        * Rigid blades
        * No blade-wake interaction
        * No ground effect

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Ducted_Fan
    RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.design_ducted_fan
    """
  

    a              = conditions.freestream.speed_of_sound 
    rho            = conditions.freestream.density 
    commanded_TV   = ducted_fan_conditions.commanded_thrust_vector_angle
    omega          = ducted_fan_conditions.omega   
    alt            = conditions.freestream.altitude 
     
    altitude       = alt/ 1000  
    n              = omega/(2.*np.pi)   # Rotations per second
    D              = ducted_fan.tip_radius * 2
    A              = 0.25 * np.pi * (D ** 2)
    
    # Unpack freestream conditions 
    Vv             = conditions.frames.inertial.velocity_vector 

    # Number of radial stations and segment control point
    B              = ducted_fan.number_of_rotor_blades
    Nr             = ducted_fan.number_of_radial_stations
    ctrl_pts       = len(Vv)
     
    # Velocity in the rotor frame
    T_body2inertial         = conditions.frames.body.transform_to_inertial
    T_inertial2body         = orientation_transpose(T_body2inertial)
    V_body                  = orientation_product(T_inertial2body,Vv)
    body2thrust,orientation = ducted_fan.body_to_prop_vel(commanded_TV) 
    T_body2thrust           = orientation_transpose(np.ones_like(T_body2inertial[:])*body2thrust)
    V_thrust                = orientation_product(T_body2thrust,V_body)

    # Check and correct for hover
    V         = V_thrust[:,0,None]
    V[V==0.0] = 1E-6
     
    tip_mach = (omega * ducted_fan.tip_radius) / a
    mach     =  V/ a
    # create tuple for querying surrogate 
    pts      = (mach,tip_mach,altitude) 
    
    thrust         = ducted_fan.performance_surrogates.thrust(pts)            
    power          = ducted_fan.performance_surrogates.power(pts)                 
    efficiency     = ducted_fan.performance_surrogates.efficiency(pts)            
    torque         = ducted_fan.performance_surrogates.torque(pts)                
    Ct             = ducted_fan.performance_surrogates.thrust_coefficient(pts)    
    Cp             = ducted_fan.performance_surrogates.power_coefficient(pts) 
    Cq             = torque/(rho*(n*n)*(D*D*D*D*D))
    FoM            = thrust*np.sqrt(thrust/(2*rho*A))/power  
    
    # calculate coefficients    
    thrust_prop_frame      = np.zeros((ctrl_pts,3))
    thrust_prop_frame[:,0] = thrust[:,0]
    thrust_vector          = orientation_product(orientation_transpose(T_body2thrust),thrust_prop_frame)
    
    outputs                                   = Data(
            torque                            = torque,
            thrust                            = thrust_vector,  
            power                             = power, 
            rpm                               = omega /Units.rpm ,   
            tip_mach                          = tip_mach, 
            efficiency                        = efficiency,         
            number_radial_stations            = Nr, 
            orientation                       = orientation, 
            speed_of_sound                    = conditions.freestream.speed_of_sound,
            density                           = conditions.freestream.density,
            velocity                          = Vv,     
            omega                             = omega,  
            thrust_per_blade                  = thrust/B,
            thrust_coefficient                = Ct, 
            torque_per_blade                  = torque/B,
            figure_of_merit                   = FoM, 
            torque_coefficient                = Cq,
            power_coefficient                 = Cp)
    
    return outputs