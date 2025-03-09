# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/compute_rotor_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports
import RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Actuator_Disc_Theory.Actuator_Disk_performance as Actuator_Disk_performance
import RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Blade_Element_Momentum_Theory_Helmholtz_Wake.BEMT_Helmholtz_performance as BEMT_Helmholtz_performance

# package imports
import  numpy as  np 
from scipy.interpolate import interp1d

# ---------------------------------------------------------------------------------------------------------------------- 
#  Generalized Rotor Class
# ----------------------------------------------------------------------------------------------------------------------  
def compute_rotor_performance(propulsor,state,center_of_gravity= [[0.0, 0.0,0.0]]):
    """Analyzes a general rotor given geometry and operating conditions.

    Assumptions:
    per source

    Source:
    Drela, M. "Qprop Formulation", MIT AeroAstro, June 2006
    http://web.mit.edu/drela/Public/web/qprop/qprop_theory.pdf

    Leishman, Gordon J. Principles of helicopter aerodynamics
    Cambridge university press, 2006.

    Inputs:
    rotor.inputs.omega                    [radian/s]
    conditions.freestream.
      density                            [kg/m^3]
      dynamic_viscosity                  [kg/(m-s)]
      speed_of_sound                     [m/s]
      temperature                        [K]
    conditions.frames.
      body.transform_to_inertial         (rotation matrix)
      inertial.velocity_vector           [m/s]
    conditions.energy.
      throttle                           [-]

    Outputs:
    conditions.energy.outputs.
       number_radial_stations            [-]
       number_azimuthal_stations         [-]
       disc_radial_distribution          [m]
       speed_of_sound                    [m/s]
       density                           [kg/m-3]
       velocity                          [m/s]
       disc_tangential_induced_velocity  [m/s]
       disc_axial_induced_velocity       [m/s]
       disc_tangential_velocity          [m/s]
       disc_axial_velocity               [m/s]
       drag_coefficient                  [-]
       lift_coefficient                  [-]
       omega                             [rad/s]
       disc_circulation                  [-]
       blade_dQ_dR                       [N/m]
       blade_dT_dr                       [N]
       blade_thrust_distribution         [N]
       disc_thrust_distribution          [N]
       thrust_per_blade                  [N]
       thrust_coefficient                [-]
       azimuthal_distribution            [rad]
       disc_azimuthal_distribution       [rad]
       blade_dQ_dR                       [N]
       blade_dQ_dr                       [Nm]
       blade_torque_distribution         [Nm]
       disc_torque_distribution          [Nm]
       torque_per_blade                  [Nm]
       torque_coefficient                [-]
       power                             [W]
       power_coefficient                 [-]

    Properties Used:
    rotor.
      number_of_blades                   [-]
      tip_radius                         [m]
      twist_distribution                 [radians]
      chord_distribution                 [m]
      orientation_euler_angles           [rad, rad, rad]
    """

    # Unpack rotor blade parameters and operating conditions 
    conditions            = state.conditions 
    if 'rotor' in  propulsor:
        rotor =  propulsor.rotor
    elif 'propeller' in  propulsor:
        rotor =  propulsor.propeller

    if rotor.fidelity == 'Blade_Element_Momentum_Theory_Helmholtz_Wake': 

        outputs = BEMT_Helmholtz_performance(rotor, conditions, propulsor, center_of_gravity)
                      
    elif rotor.fidelity == 'Actuator_Disk_Theory': 

        outputs = Actuator_Disk_performance(rotor, conditions, propulsor, center_of_gravity)
    
    conditions.energy[propulsor.tag][rotor.tag] = outputs    
      
    return