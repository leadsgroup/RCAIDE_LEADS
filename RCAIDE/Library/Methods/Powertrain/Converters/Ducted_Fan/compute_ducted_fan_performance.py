# RCAIDE/Library/Methods/Powertrain/Converters/Ducted_Fan/compute_ducted_fan_performance.py

# 
# Created:  Jan 2025, M. Clarke
# Modified: Jan 2025, M. Clarke, M. Guidotti    

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports
import  RCAIDE 
from RCAIDE.Framework.Core                              import Data , Units, orientation_product, orientation_transpose  

# package imports
import  numpy as  np 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Generalized Rotor Class
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_ducted_fan_performance(propulsor,state,center_of_gravity= [[0.0, 0.0,0.0]]):
    """
    Computes ducted fan performance characteristics using either Blade Element Momentum Theory (BEMT) 
    or Rankine-Froude Momentum Theory.

    Parameters
    ----------
    propulsor : Converter
        Ducted fan propulsor component containing the ducted fan
    state : Conditions
        Mission segment state conditions
    center_of_gravity : array_like, optional
        Aircraft center of gravity coordinates [[x, y, z]], defaults to [[0.0, 0.0, 0.0]]

    Returns
    -------
    None
        Updates state.conditions.energy[propulsor.tag][ducted_fan.tag] with computed performance data:
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
    Two fidelity levels are available:

    1. Blade Element Momentum Theory (BEMT):
        - Uses surrogate models trained on DFDC results
        - Accounts for 3D effects and ducted fan geometry
        - Computes detailed blade loading
        - Includes tip losses and wake effects

    2. Rankine-Froude Momentum Theory:
        - Uses polynomial fits for thrust and power coefficients
        - Simple momentum theory assumptions
        - Suitable for preliminary design

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

    # Unpack ducted_fan blade parameters and operating conditions 
    conditions            = state.conditions
    ducted_fan            = propulsor.ducted_fan
    propulsor_conditions  = conditions.energy[propulsor.tag]
    commanded_TV          = propulsor_conditions.commanded_thrust_vector_angle
    ducted_fan_conditions = propulsor_conditions[ducted_fan.tag]
    
    if ducted_fan.fidelity == 'Blade_Element_Momentum_Theory': 

        outputs = BEMT_performance(ducted_fan,ducted_fan_conditions,conditions, center_of_gravity, commanded_TV)
                      
    elif ducted_fan.fidelity == 'Rankine_Froude_Momentum_Theory': 

        outputs = RFMT_performance(ducted_fan,ducted_fan_conditions,conditions, center_of_gravity)
    
    conditions.energy[propulsor.tag][ducted_fan.tag] = outputs   
    
    return  

def compute_ducted_fan_efficiency(ducted_fan, V, omega):
    """
    Calculate propeller efficiency based on propeller type and velocity.
    
    Parameters
    ----------
    propeller_type : str
        Type of propeller ('constant_speed' or 'fixed_pitch')
    u0 : float
        Current velocity
        
    Returns
    -------
    float
        Calculated propeller efficiency
    """

    n = omega/(2*np.pi)
    D = 2*ducted_fan.tip_radius
    J = V/(n*D)

    a_Cp = ducted_fan.Cp_polynomial_coefficients[0]  
    b_Cp = ducted_fan.Cp_polynomial_coefficients[1]  
    c_Cp = ducted_fan.Cp_polynomial_coefficients[2] 

    Cp = a_Cp + b_Cp*J + c_Cp*(J**2)

    a_Ct = ducted_fan.Ct_polynomial_coefficients[0]  
    b_Ct = ducted_fan.Ct_polynomial_coefficients[1]  
    c_Ct = ducted_fan.Ct_polynomial_coefficients[2] 

    Ct = a_Ct + b_Ct*J + c_Ct*(J**2)

    a_etap = ducted_fan.etap_polynomial_coefficients[0]  
    b_etap = ducted_fan.etap_polynomial_coefficients[1]  
    c_etap = ducted_fan.etap_polynomial_coefficients[2] 

    eta_p = a_etap + b_etap*J + c_etap*(J**2) 

    return n, D, J, Cp, Ct, eta_p

def BEMT_performance(ducted_fan,ducted_fan_conditions,conditions, center_of_gravity, commanded_TV):

    a              = conditions.freestream.speed_of_sound 
    rho            = conditions.freestream.density 
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
    
    # Compute moment 
    moment_vector           = np.zeros((ctrl_pts,3))
    moment_vector[:,0]      = ducted_fan.origin[0][0]  -  center_of_gravity[0][0] 
    moment_vector[:,1]      = ducted_fan.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2]      = ducted_fan.origin[0][2]  -  center_of_gravity[0][2]
    moment                  =  np.cross(moment_vector, thrust_vector)
    

    outputs                                   = Data(
            torque                            = torque,
            thrust                            = thrust_vector,  
            power                             = power,
            moment                            = moment, 
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

def RFMT_performance(ducted_fan,ducted_fan_conditions,conditions, center_of_gravity):

    a              = conditions.freestream.speed_of_sound 
    rho            = conditions.freestream.density 
    omega          = ducted_fan_conditions.omega   
    alt            = conditions.freestream.altitude  
    
    # Unpack ducted_fan blade parameters and operating conditions  
    V              = conditions.freestream.velocity  
    n, D, J, Cp, Ct, eta_p  = compute_ducted_fan_efficiency(ducted_fan, V, omega)
    ctrl_pts       = len(V)
       
    thrust              = Ct * rho * (n**2)*(D**4) 
    power               = Cp * rho * (n**3)*(D**5)           
    thrust_vector       = np.zeros((ctrl_pts,3))
    thrust_vector[:,0]  = thrust[:,0]            
    torque              = power/omega
     
    # Compute moment 
    moment_vector           = np.zeros((ctrl_pts,3))
    moment_vector[:,0]      = ducted_fan.origin[0][0]  -  center_of_gravity[0][0] 
    moment_vector[:,1]      = ducted_fan.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2]      = ducted_fan.origin[0][2]  -  center_of_gravity[0][2]
    moment                  = np.cross(moment_vector, thrust_vector)
       
    outputs                                   = Data( 
            thrust                            = thrust_vector,  
            power                             = power,
            power_coefficient                 = Cp, 
            thrust_coefficient                = Ct,
            efficiency                        = eta_p, 
            moment                            = moment, 
            torque                            = torque)
    
    return outputs
    
