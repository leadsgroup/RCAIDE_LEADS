# RCAIDE/Library/Methods/Powertrain/Converters/Ducted_Fan/Performance/RFMT_performance.py

# 
# Created:  Jan 2025, M. Clarke
# Modified: Jan 2025, M. Clarke, M. Guidotti    

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports 
from RCAIDE.Framework.Core   import Data , orientation_product, orientation_transpose  

# package imports
import  numpy as  np 

# ---------------------------------------------------------------------------------------------------------------------- 
#  RFMT_performance
# ----------------------------------------------------------------------------------------------------------------------
def RFMT_performance(ducted_fan,ducted_fan_conditions,conditions):
    """
    Computes ducted fan performance characteristics using Rankine-Froude Momentum Theory.

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
    
 
    rho            = conditions.freestream.density 
    commanded_TV   = ducted_fan_conditions.commanded_thrust_vector_angle
    omega          = ducted_fan_conditions.omega    
    Vv             = conditions.frames.inertial.velocity_vector 
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

    n, D, J, Cp, Ct, eta_p  = compute_ducted_fan_efficiency(ducted_fan, V, omega)
    
    thrust              = Ct * rho * (n**2)*(D**4) 
    power               = Cp * rho * (n**3)*(D**5)           
    thrust_prop_frame       = np.zeros((ctrl_pts,3))
    thrust_prop_frame[:,0]  = thrust[:,0]       
    thrust_vector          = orientation_product(orientation_transpose(T_body2thrust),thrust_prop_frame)     
    torque              = power/omega
      
    outputs                                   = Data( 
            thrust                            = thrust_vector,  
            power                             = power,
            power_coefficient                 = Cp, 
            thrust_coefficient                = Ct,
            efficiency                        = eta_p,  
            torque                            = torque)
    
    return outputs

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
    
