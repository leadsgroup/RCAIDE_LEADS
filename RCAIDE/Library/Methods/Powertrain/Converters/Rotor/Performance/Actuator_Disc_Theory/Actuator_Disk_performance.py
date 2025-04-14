# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Actuator_Disc_Theory/Actuator_Disk_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports 
from RCAIDE.Framework.Core  import Data , Units, orientation_product, orientation_transpose  

# package imports
import  numpy as  np   

# ---------------------------------------------------------------------------------------------------------------------- 
# Actuator_Disk_performance
# ----------------------------------------------------------------------------------------------------------------------  
def Actuator_Disk_performance(rotor, conditions):
    """
    Analyzes a general rotor given geometry and operating conditions using
    Actuator Disc Theory.
    
    Parameters
    ----------
    rotor : Data
        Rotor component with the following attributes:
            - number_of_blades : int
                Number of blades on the rotor
            - tip_radius : float
                Tip radius of the rotor [m]
            - hub_radius : float
                Hub radius of the rotor [m]
            - cruise : Data
                Cruise conditions
                    - design_efficiency : float
                        Design efficiency at cruise
                    - design_torque_coefficient : float
                        Design torque coefficient at cruise
            - body_to_prop_vel : function
                Function to transform velocity from body to propeller frame
            - orientation_euler_angles : list
                Orientation of the rotor [rad, rad, rad]
    conditions : Data
        Flight conditions with:
            - freestream : Data
                Freestream properties
                    - density : numpy.ndarray
                        Air density [kg/m³]
                    - speed_of_sound : numpy.ndarray
                        Speed of sound [m/s]
            - frames : Data
                Reference frames
                - body : Data
                    Body frame
                        - transform_to_inertial : numpy.ndarray
                            Rotation matrix from body to inertial frame
                - inertial : Data
                    Inertial frame
                        - velocity_vector : numpy.ndarray
                            Velocity vector in inertial frame [m/s]
            - energy : Data
                Energy conditions
                    - converters : dict
                        Converter energy conditions indexed by tag
                        - commanded_thrust_vector_angle : numpy.ndarray
                            Commanded thrust vector angle [rad]
                        - blade_pitch_command : numpy.ndarray
                            Blade pitch command [rad]
                        - omega : numpy.ndarray
                            Angular velocity [rad/s]
    
    Returns
    -------
    None
        Results are stored in conditions.energy.converters[rotor.tag]:
            - thrust : numpy.ndarray
                Thrust vector [N]
            - power : numpy.ndarray
                Power [W]
            - rpm : numpy.ndarray
                Rotational speed [RPM]
            - omega : numpy.ndarray
                Angular velocity [rad/s]
            - power_coefficient : numpy.ndarray
                Power coefficient
            - thrust_coefficient : numpy.ndarray
                Thrust coefficient
            - torque_coefficient : numpy.ndarray
                Torque coefficient
            - speed_of_sound : numpy.ndarray
                Speed of sound [m/s]
            - density : numpy.ndarray
                Air density [kg/m³]
            - tip_mach : numpy.ndarray
                Tip Mach number
            - efficiency : numpy.ndarray
                Efficiency
            - torque : numpy.ndarray
                Torque [N·m]
            - orientation : numpy.ndarray
                Orientation matrix
            - advance_ratio : numpy.ndarray
                Advance ratio
            - velocity : numpy.ndarray
                Velocity vector [m/s]
            - disc_loading : numpy.ndarray
                Disc loading [N/m²]
            - power_loading : numpy.ndarray
                Power loading [N/W]
            - thrust_per_blade : numpy.ndarray
                Thrust per blade [N]
            - torque_per_blade : numpy.ndarray
                Torque per blade [N·m]
            - blade_pitch_command : numpy.ndarray
                Blade pitch command [rad]
            - commanded_thrust_vector_angle : numpy.ndarray
                Commanded thrust vector angle [rad]
            - figure_of_merit : numpy.ndarray
                Figure of merit
    
    Notes
    -----
    This function implements the Actuator Disc Theory to analyze rotor performance.
    It calculates thrust, torque, power, and efficiency based on the rotor geometry
    and operating conditions.
    
    The computation follows these steps:
        1. Extract rotor parameters and operating conditions
        2. Transform velocity from inertial to rotor frame
        3. Calculate rotational speed and diameter
        4. Compute torque using the design torque coefficient
        5. Calculate power and thrust
        6. Compute performance metrics (thrust coefficient, power coefficient, etc.)
        7. Store results in the conditions data structure
    
    **Major Assumptions**
        * Actuator disc theory assumes a uniform pressure jump across the rotor disc
        * The rotor is modeled as an infinitely thin disc
        * The flow is steady, incompressible, and inviscid
        * The rotor efficiency is constant and equal to the design value
    
    **Theory**
    Actuator Disc Theory models the rotor as an infinitely thin disc that creates
    a pressure jump in the flow. The theory relates thrust, power, and induced velocity
    through momentum and energy conservation principles.
    
    Key relationships include:
        - Thrust: :math:`T = \\eta\\cdot P/V`
        - Torque coefficient: :math:`Cq = Q/(ρ·n²·D⁵)`
        - Thrust coefficient: :math:`Ct = T/(ρ·n²·D⁴)`
        - Power coefficient: :math:`Cp = P/(ρ·n³·D⁵)`
        - Figure of Merit: :math:`FM = T·√(T/(2·ρ·A))/P`
    
    where:
        - T is thrust
        - P is power
        - V is velocity
        - η is efficiency
        - Q is torque
        - ρ is density
        - n is rotational speed
        - D is diameter
        - A is disc area
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Blade_Element_Momentum_Theory_Helmholtz_Wake
    """
    rho                   = conditions.freestream.density     
    commanded_TV          = conditions.energy.converters[rotor.tag].commanded_thrust_vector_angle   
    pitch_c               = conditions.energy.converters[rotor.tag].blade_pitch_command
    omega                 = conditions.energy.converters[rotor.tag].omega
    B                     = rotor.number_of_blades   
    R                     = rotor.tip_radius 
    eta_p                 = rotor.cruise.design_efficiency
    Cq                    = rotor.cruise.design_torque_coefficient
    
    # Unpack ducted_fan blade parameters and operating conditions  
    Vv                    = conditions.frames.inertial.velocity_vector 

    # Velocity in the rotor frame
    T_body2inertial         = conditions.frames.body.transform_to_inertial
    T_inertial2body         = orientation_transpose(T_body2inertial)
    V_body                  = orientation_product(T_inertial2body,Vv)
    body2thrust,orientation = rotor.body_to_prop_vel(commanded_TV) 
    T_body2thrust           = orientation_transpose(np.ones_like(T_body2inertial[:])*body2thrust)
    V_thrust                = orientation_product(T_body2thrust,V_body)

    # Check and correct for hover
    V         = V_thrust[:,0,None]
    V[V==0.0] = 1E-6

    n      = omega/(2.*np.pi)      
    D      = 2*R 
    torque = Cq * (rho*(n*n)*(D*D*D*D*D)) 
    eta    = eta_p * np.ones_like(V) 
    power  = torque * omega
    thrust = eta*power/V 
    Ct     = thrust/(rho*(n*n)*(D*D*D*D)) 
    Cp     = power / (rho*(n*n*n)*(D*D*D*D*D)) 
    
    ctrl_pts              = len(V) 
    thrust_vector         = np.zeros((ctrl_pts,3))
    thrust_vector[:,0]    = thrust[:,0]         
    disc_loading          = thrust/(np.pi*(R**2))
    power_loading         = thrust/(power)    
    A                     = np.pi*(R**2 - rotor.hub_radius**2)
    FoM                   = thrust*np.sqrt(thrust/(2*rho*A))/power  
      
    conditions.energy.converters[rotor.tag]   = Data( 
            thrust                            = thrust_vector,  
            power                             = power,
            rpm                               = omega/Units.rpm,
            omega                             = omega,
            power_coefficient                 = Cp, 
            thrust_coefficient                = Ct,
            torque_coefficient                = Cq,  
            speed_of_sound                    = conditions.freestream.speed_of_sound,
            density                           = conditions.freestream.density,
            tip_mach                          = omega * R / conditions.freestream.speed_of_sound, 
            efficiency                        = eta,  
            torque                            = torque,       
            orientation                       = orientation, 
            advance_ratio                     = V/(n*D),    
            velocity                          = Vv, 
            disc_loading                      = disc_loading, 
            power_loading                     = power_loading,  
            thrust_per_blade                  = thrust/B, 
            torque_per_blade                  = torque/B,
            blade_pitch_command               = pitch_c,
            commanded_thrust_vector_angle     = commanded_TV,  
            figure_of_merit                   = FoM,) 

    return  