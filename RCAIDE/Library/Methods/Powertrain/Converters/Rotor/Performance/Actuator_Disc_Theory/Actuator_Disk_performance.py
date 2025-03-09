# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Actuator_Disc_Theory/Actuator_Disk_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports 
from RCAIDE.Framework.Core  import Data , Units  
from RCAIDE.Framework.Core                              import Data , Units, orientation_product, orientation_transpose  

# package imports
import  numpy as  np  
from scipy.interpolate import interp1d

# ---------------------------------------------------------------------------------------------------------------------- 
# Actuator_Disk_performance
# ----------------------------------------------------------------------------------------------------------------------  
def Actuator_Disk_performance(rotor, conditions, propulsor, center_of_gravity):
    '''Actuator Disc Theory    
    '''

    rho                   = conditions.freestream.density   
    propulsor_conditions  = conditions.energy[propulsor.tag]
    commanded_TV          = propulsor_conditions.commanded_thrust_vector_angle 
    rotor_conditions      = propulsor_conditions[rotor.tag]  
    pitch_c               = rotor_conditions.blade_pitch_command
    omega                 = rotor_conditions.omega
    torque                = rotor_conditions.motor_torque 
    B                     = rotor.number_of_blades   
    R                     = rotor.tip_radius 
    eta_p                 = rotor.propulsive_efficiency    
    
    # Unpack ducted_fan blade parameters and operating conditions  
    Vv      = conditions.frames.inertial.velocity_vector 

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
    
    eta    = eta_p * np.ones_like(V) 
    power  = torque*omega
    n      = omega/(2.*np.pi) 
    D      = 2*R 
    thrust = eta*power/V 
    Cp     = power/(rho*(n*n*n)*(D*D*D*D*D)) 
    Cq     = torque/(rho*(n*n)*(D*D*D*D*D))
    Ct     = thrust/(rho*(n*n)*(D*D*D*D))
    Cp     = power/(rho*(n*n*n)*(D*D*D*D*D)) 
    
    ctrl_pts              = len(V) 
    thrust_vector         = np.zeros((ctrl_pts,3))
    thrust_vector[:,0]    = thrust[:,0]         
    disc_loading          = thrust/(np.pi*(R**2))
    power_loading         = thrust/(power)    
    A                     = np.pi*(R**2 - rotor.hub_radius**2)
    FoM                   = thrust*np.sqrt(thrust/(2*rho*A))/power  
     
    # Compute moment 
    moment_vector         = np.zeros((ctrl_pts,3))
    moment_vector[:,0]    = rotor.origin[0][0]  -  center_of_gravity[0][0] 
    moment_vector[:,1]    = rotor.origin[0][1]  -  center_of_gravity[0][1] 
    moment_vector[:,2]    = rotor.origin[0][2]  -  center_of_gravity[0][2]
    moment                = np.cross(moment_vector, thrust_vector)
       
    outputs                                   = Data( 
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
            moment                            = moment, 
            torque                            = torque,       
            orientation                       = orientation, 
            advance_ratio                     = V/(n*D),    
            velocity                          = Vv, 
            disc_loading                      = disc_loading, 
            power_loading                     = power_loading,  
            thrust_per_blade                  = thrust/B, 
            torque_per_blade                  = torque/B,
            blade_pitch_command               = pitch_c, 
            figure_of_merit                   = FoM,) 

    return outputs