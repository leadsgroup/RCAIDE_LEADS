# Vehicles.py
# 
# Created: Dec 2021, E. Botero
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import numpy as np

import RCAIDE
from RCAIDE.Core import Units
from RCAIDE.Input_Output.OpenVSP import write
from RCAIDE.Methods.Power.Battery.Sizing                                   import initialize_from_mass, initialize_from_energy_and_power
from RCAIDE.Methods.Propulsion.electric_motor_sizing                       import size_from_mass , size_optimal_motor
from RCAIDE.Methods.Geometry.Two_Dimensional.Planform import segment_properties
from RCAIDE.Input_Output.OpenVSP.vsp_read import vsp_read
from RCAIDE.Components.Energy.Networks import Battery_Electric_Rotor
from RCAIDE.Plots.Geometry.Three_Dimensional import plot_3d_vehicle
from RCAIDE.Framework.Analyses.Propulsion.Momentum_Theory_Wake import Momentum_Theory_Wake

import matplotlib.pyplot as plt

from RCAIDE.Components.Nacelles import Rotor_Boom

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def setup():
    
    base_vehicle = base_setup()
    configs = configs_setup(base_vehicle)
    
    return configs

def base_setup():

    # -----------------------------------s-------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    net = Battery_Electric_Rotor()
    net.voltage = 400.
    net.identical_propellers = False
    

    vehicle = vsp_read('Maker.vsp3',units_type = 'Imperial',specified_network =net)
    
    #plot_3d_vehicle(vehicle,plot_control_points = False)
    
    #plt.show()
    
    # Setting up other characteristics not automatically imported: https://archer.com/maker
    vehicle.mass_properties.takeoff           = 1508. * Units.kg
    vehicle.mass_properties.operating_empty   = 1508. * Units.kg  - 400 * Units.lb
    vehicle.mass_properties.max_takeoff       = 1508. * Units.kg  
    vehicle.mass_properties.max_payload       = 400.  * Units.lb    
    vehicle.number_of_passengers                        = 2

    # basic parameters
    vehicle.flight_envelope.ultimate_load = 5.7
    vehicle.flight_envelope.positive_limit_load    = 3.    
    
    vehicle.number_of_passengers = 2
    
    S = 0
    for wing in vehicle.wings:
        if len(wing.Segments.keys())>0:
            wing = segment_properties(wing,update_wet_areas=False)
        
        if wing.areas.reference>S:
            S = wing.areas.reference
            
    vehicle.reference_area = S
    
    vehicle.fuselages.fuselage_1.tag = 'fuselage'
    vehicle.append_component(vehicle.fuselages.pop('fuselage_1'))    

    fk  = list(vehicle.fuselages.keys())

    for fuselage in fk:
        if fuselage == 'fuselage':
            continue
        else:
            boom = Rotor_Boom(vehicle.fuselages.pop(fuselage))
            boom.diameter = boom.effective_diameter
            boom.length   = boom.lengths.total
            boom.number_of_rotors = 2
            vehicle.append_component(boom)


    #------------------------------------------------------------------
    # Design Battery
    #------------------------------------------------------------------
    #
    bat                      = RCAIDE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.max_voltage          = net.voltage   
    bat.mass_properties.mass = 400. * Units.kg
    energy                   = 75 * Units['kWh']
    bat.specific_energy      = energy/bat.mass_properties.mass
    power                    = 0.
    initialize_from_energy_and_power(bat, energy, power)
    net.battery              = bat     
    
    
    #------------------------------------------------------------------
    # Design Systems
    #------------------------------------------------------------------
    systems                      = RCAIDE.Library.Components.Powertrain.Systems.Systems()
    systems.power_draw           = 0.
    systems.mass_properties.mass = 400 * Units.lb
    net.payload                  = systems

    #------------------------------------------------------------------
    # Design Avionics
    #------------------------------------------------------------------
    avionics            = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw = 500. * Units.watts
    net.avionics        = avionics
    
    #------------------------------------------------------------------
    # Design Electronic Speed Controller
    #------------------------------------------------------------------

    propeller_esc            = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    propeller_esc.efficiency = 0.97
    net.esc                  = propeller_esc
    
    net.identical_flag = False
    
    
    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------

    
    for rotor in vehicle.networks.battery_electric_rotor.rotors:

        # run the rotor
        conditions = RCAIDE.Framework.Analyses.Mission.Segments.Conditions.Aerodynamics()
        atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        h          = 1000. * Units.feet
        temperature_deviation = 0.
        
        atmo_data = atmosphere.compute_values(h,temperature_deviation)
        
        conditions.freestream.density                = atmo_data.density
        conditions.freestream.dynamic_viscosity      = atmo_data.dynamic_viscosity
        conditions.freestream.speed_of_sound         = atmo_data.speed_of_sound
        conditions.freestream.temperature            = atmo_data.temperature
        conditions.frames.inertial.velocity_vector   = np.array([[0.,0.,1]])
        conditions.propulsion.throttle               = np.array([[1]])
        conditions.frames.body.transform_to_inertial = np.array([np.eye(3)])
        
        # Set the RPM
        rotor.inputs.omega     = np.array([[1700 * Units.rpm]])
        rotor.angular_velocity = 1700 * Units.rpm
        
        rotor.airfoil_cl_surrogates = None
        rotor.airfoil_cd_surrogates = None
        
        rotor.Wake                  = Momentum_Theory_Wake()        
        
        thrust_vector, torque, power, Cp, outputs , etap = rotor.spin(conditions)
        
        rotor.design_torque = torque[0,0]
        
        rotor.design_power_coefficient = Cp
        
        # Rotor (Lift) Motor
        lift_rotor_motor                         = RCAIDE.Library.Components.Powertrain.Converters.Motor()
        lift_rotor_motor.tag                     = rotor.tag + '_motor'
        lift_rotor_motor.efficiency              = 0.95
        lift_rotor_motor.nominal_voltage         = bat.max_voltage*3/4
        lift_rotor_motor.mass_properties.mass    = 3. * Units.kg
        lift_rotor_motor.origin                  = rotor.origin
        lift_rotor_motor.propeller_radius        = rotor.tip_radius
        lift_rotor_motor.gearbox_efficiency      = 1.0
        lift_rotor_motor.no_load_current         = 4.0
        lift_rotor_motor                         = size_optimal_motor(lift_rotor_motor,rotor)      
        net.motors.append(lift_rotor_motor)


    # Calculate excressence drag
    strut_length     = 6. * Units.inches
    strut_width      = 3.  * Units.inches
    
    propeller_width  = 4.  * Units.inches
    propeller_height = 2.  * Units.inches
    
    nose_tire_height = 6.  * Units.inches
    nose_tire_width  = 3.  * Units.inches
    
    main_tire_height = 6.  * Units.inches
    main_tire_width  = 3.  * Units.inches
    
    total_excrescence_area_spin = 2*main_tire_height*main_tire_width + nose_tire_height*nose_tire_width +strut_width*strut_length
    
    total_excrescence_area_no_spin = total_excrescence_area_spin + 6*propeller_height*propeller_width 
    
    vehicle.excrescence_area_no_spin = total_excrescence_area_no_spin 
    vehicle.excrescence_area_spin    = total_excrescence_area_spin     

    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = RCAIDE.Library.Components.Configs.Config.Container()
    
    rear_rotors = ['l_inner_rotor','r_inner_rotor','l_mid_rotor','r_mid_rotor','l_outer_rotor','r_outer_rotor']

    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    base_config.excressence_drag_area = vehicle.excrescence_area_spin
    
    # The front rotors are variable pitch
    for rotor in base_config.networks.battery_electric_rotor.rotors:
        if rotor.tag not in rear_rotors:
            rotor.variable_pitch = True
    
    configs.append(base_config)
    
    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------

    cruise_config = RCAIDE.Library.Components.Configs.Config(base_config)
    cruise_config.tag = 'cruise'
    
    # Delete the rear rotors
    
    for rotor in rear_rotors:
        cruise_config.networks.battery_electric_rotor.rotors.pop(rotor)
        cruise_config.networks.battery_electric_rotor.motors.pop(rotor+'_motor')
        
    cruise_config.networks.battery_electric_rotor.number_of_propeller_engines = 6
    cruise_config.networks.battery_electric_rotor.identical_propellers = True
        
    # rotate the front rotors down and adjust pitch    
    for prop in cruise_config.networks.battery_electric_rotor.rotors:
        prop.inputs.blade_pitch_command        = 25. * Units.degrees # Changing the variable pitch
        prop.orientation_euler_angles    = [0.,0.,0.]
        
    cruise_config.excressence_drag_area = vehicle.excrescence_area_no_spin 

    configs.append(cruise_config)
    
    #plot_vehicle(cruise_config,plot_control_points = False)
    
    #plt.show()    
    
    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'hover_oei'

    # Remove two rotors and motors on opposite ends: propora and proporf1
    K1 = 'l_outer_prop'
    K2 = 'r_outer_rotor'
    
    config.networks.battery_electric_rotor.rotors.pop(K1)
    config.networks.battery_electric_rotor.rotors.pop(K2)
    config.networks.battery_electric_rotor.motors.pop(K1+'_motor')
    config.networks.battery_electric_rotor.motors.pop(K2+'_motor')   
    config.networks.battery_electric_rotor.number_of_propeller_engines -=2 

    configs.append(config)

    return configs
