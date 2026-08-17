# RCAIDE/Library/Methods/Powertrain/Converters/Ducted_Fan/Performance/Rankine_Froude_Momentum_Theory/RFMT_performance.py

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
def RFMT_performance(ducted_fan,conditions):
    """
    Computes ducted fan performance characteristics using Rankine-Froude Momentum Theory.

    Parameters
    ----------
    ducted_fan : RCAIDE.Library.Components.Powertrain.Converters.Ducted_Fan
        Ducted fan component to evaluate
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Mission segment conditions

    Returns
    -------
    None
        Updates conditions.energy.converters[ducted_fan.tag] with computed performance data:
            - thrust : array(N,3)
                Thrust vector [N]
            - power : array(N,1)
                Power required [W]
            - torque : array(N,1)
                Shaft torque [N-m]
            - efficiency : array(N,1)
                Propulsive efficiency [-]
            - thrust_coefficient : array(N,1)
                Non-dimensional thrust coefficient [-]
            - power_coefficient : array(N,1)
                Non-dimensional power coefficient [-]

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
    
 
    rho                   = conditions.freestream.density 
    commanded_TV          = conditions.energy.converters[ducted_fan.tag].commanded_thrust_vector_angle
    omega                 = conditions.energy.converters[ducted_fan.tag].omega    
    Vv                    = conditions.frames.inertial.velocity_vector 
    ctrl_pts              = len(Vv)

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
    
    thrust                  = Ct * rho * (n**2)*(D**4) 
    power                   = Cp * rho * (n**3)*(D**5)           
    thrust_prop_frame       = np.zeros((ctrl_pts,3))
    thrust_prop_frame[:,0]  = thrust[:,0]       
    thrust_vector           = orientation_product(orientation_transpose(T_body2thrust),thrust_prop_frame)     
    torque                  = power/omega
      
    conditions.energy.converters[ducted_fan.tag] = Data( 
            thrust                            = thrust_vector,  
            power                             = power,
            power_coefficient                 = Cp, 
            thrust_coefficient                = Ct,
            efficiency                        = eta_p,  
            torque                            = torque)
    
    return  

def compute_ducted_fan_efficiency(ducted_fan, V, omega):
    """
    Evaluates the ducted fan's Rankine-Froude performance polynomials at the given
    operating point.

    Parameters
    ----------
    ducted_fan : RCAIDE.Library.Components.Powertrain.Converters.Ducted_Fan
        Ducted fan component with the following attributes:
            - tip_radius : float
                Rotor tip radius [m]
            - Cp_polynomial_coefficients : list
                Power coefficient polynomial coefficients [-, -, -]
            - Ct_polynomial_coefficients : list
                Thrust coefficient polynomial coefficients [-, -, -]
            - etap_polynomial_coefficients : list
                Propulsive efficiency polynomial coefficients [-, -, -]
    V : numpy.ndarray
        Freestream velocity in the rotor frame [m/s]
    omega : numpy.ndarray
        Rotor angular velocity [rad/s]

    Returns
    -------
    n : numpy.ndarray
        Rotor rotational speed [rev/s]
    D : float
        Rotor diameter [m]
    J : numpy.ndarray
        Advance ratio [-]
    Cp : numpy.ndarray
        Power coefficient [-]
    Ct : numpy.ndarray
        Thrust coefficient [-]
    eta_p : numpy.ndarray
        Propulsive efficiency [-]
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
    
