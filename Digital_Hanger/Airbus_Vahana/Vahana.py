''' 
# Tiltwing_EVTOL.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

'''
#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units  
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor  import design_electric_rotor
from RCAIDE.Library.Plots                                         import * 
from RCAIDE.Framework.External_Interfaces.OpenVSP                 import export_vsp_vehicle 
from RCAIDE.load    import load as load_propulsor
from RCAIDE.save    import save as save_propulsor 

# python imports 
import os
import numpy as np 
from copy import deepcopy
import pickle
import  pandas as pd
import matplotlib.pyplot as plt  
 
# ----------------------------------------------------------------------------------------------------------------------
#  Main 
# ----------------------------------------------------------------------------------------------------------------------  
def main():           
         
    # vehicle data
    new_geometry    = False
    redesign_rotors = False
    plot_vehicle    = False
    if new_geometry :
        vehicle  = vehicle_setup(redesign_rotors)
        save_aircraft_geometry(vehicle , 'Vahana')
    else: 
        vehicle = load_aircraft_geometry('Vahana')
        
    #export_vsp_vehicle(vehicle, 'Vahana')
        
    # Set up configs
    configs  = configs_setup(vehicle)

    # vehicle analyses
    analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(analyses)
    missions = missions_setup(mission) 
     
    results = missions.base_mission.evaluate() 
     
    # plot the results 
    plot_results(results)
    
    if plot_vehicle: 
        plot_3d_vehicle(vehicle)
    return

def vehicle_setup(redesign_rotors=True): 

    ospath      = os.path.abspath(__file__)
    separator   = os.path.sep 
    local_path  = os.path.dirname(ospath) + separator +  '..' + separator   
    
    #------------------------------------------------------------------------------------------------------------------------------------
    # ################################################# Vehicle-level Properties ########################################################  
    #------------------------------------------------------------------------------------------------------------------------------------
    vehicle                                     = RCAIDE.Vehicle()
    vehicle.tag                                 = 'Vahana'
    vehicle.configuration                       = 'eVTOL'
         
    # mass properties
    vehicle.mass_properties.takeoff             = 735. 
    vehicle.mass_properties.operating_empty     = 735.
    vehicle.mass_properties.max_takeoff         = 735.
    vehicle.mass_properties.center_of_gravity   = [[ 2.0144,   0.  ,  0.]] 
    vehicle.number_of_passengers                = 0
    vehicle.flight_envelope.ultimate_load       = 5.7
    vehicle.flight_envelope.positive_limit_load = 3.
    vehicle.number_of_passengers                = 1

    #------------------------------------------------------------------------------------------------------------------------------------
    # ##################################################### Landing Gear ################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    main_gear                                = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter                  = 6  *  Units.inches 
    main_gear.rim_diameter                   = 3  *  Units.inches 
    main_gear.tire_width                     = 6  *  Units.inches 
    main_gear.strut_length                   = 12  * Units.ft 
    main_gear.wheels                         = 1   
    main_gear.number_of_gear_types_in_tandem = 1
    main_gear.number_of_wheels_in_gear_type  = 1
    main_gear.origin                         = [[4.0,0, 0]]
    main_gear.fairing                        = True
    main_gear.xz_plane_symmetric             = True
    main_gear.gear_extended                  = True
    vehicle.append_component(main_gear)  

    nose_gear                                = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                  =  5 *  Units.inches   
    nose_gear.rim_diameter                   =  3 *  Units.inches 
    nose_gear.tire_width                     =  5 *  Units.inches 
    nose_gear.strut_length                   =  6.* Units.ft 
    nose_gear.wheels                         = 1
    nose_gear.origin                         = [[0.5,0, 0]]
    nose_gear.fairing                        = True 
    nose_gear.gear_extended                  = True
    nose_gear.number_of_gear_types_in_tandem = 1
    nose_gear.number_of_wheels_in_gear_type  = 1    
    vehicle.append_component(nose_gear)    

    #------------------------------------------------------------------------------------------------------------------------------------
    # ######################################################## Wings ####################################################################  
    #------------------------------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
    wing                                        = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                                    = 'canard_wing'  
    wing.aspect_ratio                           = 11.37706641  
    wing.sweeps.quarter_chord                   = 0.0
    wing.thickness_to_chord                     = 0.18  
    wing.taper                                  = 1.  
    wing.spans.projected                        = 6.65 
    wing.chords.root                            = 0.95 
    wing.total_length                           = 0.95   
    wing.chords.tip                             = 0.95 
    wing.chords.mean_aerodynamic                = 0.95   
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = wing.chords.root*wing.spans.projected 
    wing.areas.wetted                           = 2*wing.chords.root*wing.spans.projected*0.95  
    wing.areas.exposed                          = 2*wing.chords.root*wing.spans.projected*0.95 
    wing.twists.root                            = 0.  
    wing.twists.tip                             = 0.  
    wing.origin                                 = [[0.1,  0.0 , 0.0]]  
    wing.aerodynamic_center                     = [0., 0., 0.]     
    wing.winglet_fraction                       = 0.0 
    wing.xz_plane_symmetric                     = True
    
    ospath                                      = os.path.abspath(__file__) 
    separator                                   = os.path.sep 
    airfoil                                     = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file                     = local_path + 'Airfoils' + separator + 'NACA_63_412.txt'
    
    wing.append_airfoil(airfoil)
                                                
    # add to vehicle                                          
    vehicle.append_component(wing)                            
                                                
    wing                                        = RCAIDE.Library.Components.Wings.Wing()
    wing.tag                                    = 'main_wing'  
    wing.aspect_ratio                           = 11.37706641  
    wing.sweeps.quarter_chord                   = 0.0
    wing.thickness_to_chord                     = 0.18  
    wing.taper                                  = 1.  
    wing.spans.projected                        = 6.65 
    wing.chords.root                            = 0.95 
    wing.total_length                           = 0.95   
    wing.chords.tip                             = 0.95 
    wing.chords.mean_aerodynamic                = 0.95   
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = wing.chords.root*wing.spans.projected 
    wing.areas.wetted                           = 2*wing.chords.root*wing.spans.projected*0.95  
    wing.areas.exposed                          = 2*wing.chords.root*wing.spans.projected*0.95 
    wing.twists.root                            = 0.  
    wing.twists.tip                             = 0.  
    wing.origin                                 = [[ 5.138, 0.0  ,  1.323 ]]  # for images 1.54
    wing.aerodynamic_center                     = [0., 0., 0.]     
    wing.winglet_fraction                       = 0.0  
    wing.xz_plane_symmetric                     = True  
    vehicle.reference_area                      = 2*wing.areas.reference 
    wing.append_airfoil(airfoil)

    # add to vehicle 
    vehicle.append_component(wing)   


    #------------------------------------------------------------------------------------------------------------------------------------
    # ##########################################################  Fuselage ############################################################## 
    #------------------------------------------------------------------------------------------------------------------------------------
    
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 

    # define cabin    
    cabin                                       = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.origin                                = [[1, 0, 0]] 
    economy_class                               = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest        = 1
    economy_class.number_of_rows                = 1 
    economy_class.seat_arm_rest_width           = 2 *  Units.inches 
    economy_class.seat_width                    = 15 *  Units.inches
    economy_class.aisle_width                   = 0  *  Units.inches   
    cabin.append_cabin_class(economy_class)
    fuselage.append_cabin(cabin)

    fuselage.fineness.nose                      = 1.5 
    fuselage.fineness.tail                      = 4.0 
    fuselage.lengths.nose                       = 1.7   
    fuselage.lengths.tail                       = 2.7 
    fuselage.lengths.cabin                      = 1.7  
    fuselage.lengths.total                      = 6.1  
    fuselage.width                              = 1.15  
    fuselage.heights.maximum                    = 1.7 
    fuselage.heights.at_quarter_length          = 1.2  
    fuselage.heights.at_wing_root_quarter_chord = 1.7  
    fuselage.heights.at_three_quarters_length   = 0.75 
    fuselage.areas.wetted                       = 12.97989862  
    fuselage.areas.front_projected              = 1.365211404  
    fuselage.effective_diameter                 = 1.318423736  
    fuselage.differential_pressure              = 0.  

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'   
    segment.percent_x_location                  = 0.  
    segment.percent_z_location                  = 0.  
    segment.height                              = 0.09  
    segment.width                               = 0.23473  
    segment.length                              = 0.  
    segment.effective_diameter                  = 0. 
    fuselage.segments.append(segment)             

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 0.97675/6.1 
    segment.percent_z_location                  = 0.21977/6.1
    segment.height                              = 0.9027  
    segment.width                               = 1.01709  
    fuselage.segments.append(segment)             


    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'    
    segment.percent_x_location                  = 1.93556/6.1 
    segment.percent_z_location                  = 0.39371/6.1
    segment.height                              = 1.30558   
    segment.width                               = 1.38871  
    fuselage.segments.append(segment)             


    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'    
    segment.percent_x_location                  = 3.44137/6.1 
    segment.percent_z_location                  = 0.57143/6.1
    segment.height                              = 1.52588 
    segment.width                               = 1.47074 
    fuselage.segments.append(segment)             

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 4.61031/6.1
    segment.percent_z_location                  = 0.10893
    segment.height                              = 1.3906
    segment.width                               = 1.11463  
    fuselage.segments.append(segment)              

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.9827
    segment.percent_z_location                  = 0.180
    segment.height                              = 0.6145
    segment.width                               = 0.3838
    fuselage.segments.append(segment)            
    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 1. 
    segment.percent_z_location                  = 0.18
    segment.height                              = 0.4
    segment.width                               = 0.25
    fuselage.segments.append(segment)        

    # add to vehicle
    vehicle.append_component(fuselage)     

    #------------------------------------------------------------------------------------------------------------------------------------
    # ########################################################  Energy Network  ######################################################### 
    #------------------------------------------------------------------------------------------------------------------------------------
    # define network
    network                                                = RCAIDE.Framework.Networks.Electric() 
    network.charging_power                                 = 1000
    #==================================================================================================================================== 
    # Lift Bus 
    #====================================================================================================================================          
    bus                                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    bus.tag                                                = 'bus' 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus Battery
    #------------------------------------------------------------------------------------------------------------------------------------ 
    bat                                                    = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC() 
    bat.tag                                                = 'bus_battery'
    bat.electrical_configuration.series                    = 8 
    bat.electrical_configuration.parallel                  = 60 
    bat.geometrtic_configuration.normal_count              = 20
    bat.geometrtic_configuration.parallel_count            = 24  
    
    for _ in range(10):
        bus.battery_modules.append(deepcopy(bat))   
    bus.initialize_bus_properties()
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Lift Propulsors 
    #------------------------------------------------------------------------------------------------------------------------------------    
     
    # Define Lift Propulsor Container 
    prop_rotor_propulsor                                = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
    prop_rotor_propulsor.tag                            = 'prop_rotor_propulsor'      
    prop_rotor_propulsor.wing_mounted                   = True 
              
    # Electronic Speed Controller           
    prop_rotor_esc                                = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    prop_rotor_esc.efficiency                     = 0.95    
    prop_rotor_esc.tag                            = 'esc_1'  
    prop_rotor_esc.bus_voltage                    = bus.voltage   
    prop_rotor_propulsor.electronic_speed_controller = prop_rotor_esc  
    
    # Lift Rotor Design
    g                                             = 9.81                                    # gravitational acceleration   
    Hover_Load                                    = vehicle.mass_properties.takeoff*g *1.1  # hover load   

    prop_rotor                                    = RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor()   
    prop_rotor.tag                                = 'prop_rotor'   
    prop_rotor.tip_radius                         = 0.8875
    prop_rotor.hub_radius                         = 0.15 * prop_rotor.tip_radius
    prop_rotor.number_of_blades                   = 3
    prop_rotor.hover.design_altitude              = 40 * Units.feet   
    prop_rotor.hover.design_thrust                = Hover_Load/8 
    prop_rotor.hover.design_freestream_velocity   = np.sqrt(prop_rotor.hover.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))  
    prop_rotor.oei.design_altitude                = 40 * Units.feet  
    prop_rotor.oei.design_thrust                  = Hover_Load/7  
    prop_rotor.oei.design_freestream_velocity     = np.sqrt(prop_rotor.oei.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))   
    prop_rotor.cruise.design_altitude             = 1500 * Units.feet
    prop_rotor.cruise.design_thrust               = 500   
    prop_rotor.cruise.design_freestream_velocity  = 130.* Units['mph'] 
    
    
    airfoil                                       = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file                       =  local_path + 'Airfoils' + separator + 'NACA_4412.txt'
    airfoil.polar_files                           = [local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_50000.txt' ,
                                                     local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_100000.txt' ,
                                                     local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_200000.txt' ,
                                                     local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_500000.txt' ,
                                                     local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_1000000.txt',
                                                     local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_3500000.txt',
                                                     local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_5000000.txt',
                                                     local_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_7500000.txt' ]
    prop_rotor.append_airfoil(airfoil)                
    prop_rotor.airfoil_polar_stations             = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    prop_rotor_propulsor.rotor                    =  prop_rotor

   
    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Motor  
    #------------------------------------------------------------------------------------------------------------------------------------    
    prop_rotor_motor                         = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    prop_rotor_motor.efficiency              = 0.95
    prop_rotor_motor.nominal_voltage         = bus.voltage *0.75
    prop_rotor_motor.tag                     = 'motor_1'
    prop_rotor_motor.no_load_current         = 0.1    
    prop_rotor_propulsor.motor               = prop_rotor_motor

    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Nacelle
    #------------------------------------------------------------------------------------------------------------------------------------     
    nacelle                           = RCAIDE.Library.Components.Nacelles.Nacelle() 
    nacelle.length                    = 0.9
    nacelle.diameter                  = 0.3 
    nacelle.flow_through              = False    
    prop_rotor_propulsor.nacelle      = nacelle
    
    if redesign_rotors:
        design_electric_rotor(prop_rotor_propulsor, print_iterations=True)
        save_propulsor(prop_rotor_propulsor, os.path.join(local_path, 'vahana_tilt_rotor_propulsor.res'))
    else:
        regression_prop_rotor_propulsor = deepcopy(prop_rotor_propulsor)        
        design_electric_rotor(regression_prop_rotor_propulsor, iterations=2)
        loaded_propulsor = load_propulsor(os.path.join(local_path, 'vahana_tilt_rotor_propulsor.res'))  
        for key,item in prop_rotor_propulsor.rotor.items():
            prop_rotor_propulsor.rotor[key] = loaded_propulsor.rotor[key] 
        for key,item in prop_rotor_propulsor.motor.items():
            prop_rotor_propulsor.motor[key] = loaded_propulsor.motor[key] 
         
    # Front Rotors Locations 
    nacelle_origins = [[-0.2, 1.347, 0.0], [-0.2, 3.2969999999999997, 0.0], [-0.2, -1.347, 0.0], [-0.2, -3.2969999999999997, 0.0],\
               [4.938, 1.347, 1.4], [4.938, 3.2969999999999997, 1.4],[4.938, -1.347, 1.5], [4.938, -3.2969999999999997, 1.4]] 
    rotor_origins = [[0., 1.347, 0.0], [0., 3.2969999999999997, 0.0], [0., -1.347, 0.0], [0., -3.2969999999999997, 0.0],\
               [5.0, 1.347, 1.4], [5.0, 3.2969999999999997, 1.4],[5.0, -1.347, 1.5], [5.0, -3.2969999999999997, 1.4]] 
    motor_origins = [[0.5, 1.347, 0.0], [0.5, 3.2969999999999997, 0.0], [0.5, -1.347, 0.0], [0.5, -3.2969999999999997, 0.0],\
               [5.7, 1.347, 1.4], [5.7, 3.2969999999999997, 1.4],[5.7, -1.347, 1.5], [5.7, -3.2969999999999997, 1.4]] 
    assigned_propulsor_list =  []
    for i in range(8): 
        prop_rotor_propulsor_i                                       = deepcopy(prop_rotor_propulsor)
        prop_rotor_propulsor_i.tag                                   = 'prop_rotor_propulsor_' + str(i + 1)
        prop_rotor_propulsor_i.rotor.tag                             = 'rotor_' + str(i + 1) 
        prop_rotor_propulsor_i.rotor.origin                          = [rotor_origins[i]]  
        prop_rotor_propulsor_i.motor.tag                             = 'motor_' + str(i + 1)  
        if i < 4: 
            prop_rotor_propulsor_i.motor.wing_tag                    = 'canard_wing'
        else:
            prop_rotor_propulsor_i.motor.wing_tag                    = 'main_wing'
        prop_rotor_propulsor_i.motor.origin                          = [motor_origins[i]]  
        prop_rotor_propulsor_i.electronic_speed_controller.tag       = 'esc_' + str(i + 1)  
        prop_rotor_propulsor_i.electronic_speed_controller.origin    = [motor_origins[i]]  
        prop_rotor_propulsor_i.nacelle.tag                           = 'nacelle_' + str(i + 1)  
        prop_rotor_propulsor_i.nacelle.origin                        = [nacelle_origins[i]]
        assigned_propulsor_list.append(prop_rotor_propulsor_i.tag)
        network.propulsors.append(prop_rotor_propulsor_i)  
    bus.assigned_propulsors = [assigned_propulsor_list]       
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Additional Bus Loads
    #------------------------------------------------------------------------------------------------------------------------------------            
    # Payload   
    systems                         = RCAIDE.Library.Components.Powertrain.Systems.Systems()
    systems.power_draw              = 10. # Watts 
    systems.mass_properties.mass    = 1.0 * Units.kg
    bus.systems                     = systems 
                             
    # Avionics                            
    avionics                        = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw             = 10. # Watts  
    avionics.mass_properties.mass   = 1.0 * Units.kg
    bus.avionics                    = avionics    
    
  
    network.busses.append(bus) 
        
    # append energy network 
    vehicle.append_energy_network(network)  

    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    '''
    The configration set up below the scheduling of the nacelle angle and vehicle speed.
    Since one prop_rotor operates at varying flight conditions, one must perscribe  the 
    pitch command of the prop_rotor which us used in the variable pitch model in the analyses
    Note: low pitch at take off & low speeds, high pitch at cruise
    '''
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    configs = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config                                                       = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag                                                   = 'base'     
    configs.append(base_config) 
 
    # ------------------------------------------------------------------
    #   Hover Climb Configuration
    # ------------------------------------------------------------------
    config                                                 = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag                                             = 'vertical_climb'
    vector_angle                                           = 90.0 * Units.degrees
    config.wings.main_wing.twists.root                     = vector_angle
    config.wings.main_wing.twists.tip                      = vector_angle
    config.wings.canard_wing.twists.root                   = vector_angle
    config.wings.canard_wing.twists.tip                    = vector_angle    
    for network in  config.networks:  
        for propulsor in  network.propulsors:
            propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
    configs.append(config)

    # ------------------------------------------------------------------
    #    
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    vector_angle                                      = 30.0  * Units.degrees 
    config.tag                                        = 'vertical_transition'
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle
    for network in  config.networks:  
        for propulsor in  network.propulsors:
            propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
            propulsor.rotor.blade_pitch_command   = propulsor.rotor.hover.design_blade_pitch_command * 0.5 
    configs.append(config) 

    # ------------------------------------------------------------------
    #   Hover-to-Cruise Configuration
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag                                        = 'climb_transition'
    vector_angle                                      = 5.0  * Units.degrees  
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle 
    for network in  config.networks:  
        for propulsor in  network.propulsors:
            propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
            propulsor.rotor.blade_pitch_command     = propulsor.rotor.cruise.design_blade_pitch_command  
    configs.append(config) 

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag                                        = 'cruise'   
    vector_angle                                      = 0.0 * Units.degrees 
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle  
    for network in  config.networks:  
        for propulsor in  network.propulsors:
            propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
            propulsor.rotor.blade_pitch_command      = propulsor.rotor.cruise.design_blade_pitch_command  
    configs.append(config)     
    
    # ------------------------------------------------------------------
    #   
    # ------------------------------------------------------------------ 
    config                                                 = RCAIDE.Library.Components.Configs.Config(vehicle)
    vector_angle                                           = 75.0  * Units.degrees   
    config.tag                                             = 'descent_transition'   
    config.wings.main_wing.twists.root                     = vector_angle
    config.wings.main_wing.twists.tip                      = vector_angle
    config.wings.canard_wing.twists.root                   = vector_angle
    config.wings.canard_wing.twists.tip                    = vector_angle
    for network in  config.networks:  
        for propulsor in  network.propulsors:
            propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
            propulsor.rotor.blade_pitch_command      = propulsor.rotor.cruise.design_blade_pitch_command * 0.5
    configs.append(config)  

    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag                                        = 'vertical_descent'
    vector_angle                                      = 90.0  * Units.degrees   
    config.wings.main_wing.twists.root                = vector_angle
    config.wings.main_wing.twists.tip                 = vector_angle
    config.wings.canard_wing.twists.root              = vector_angle
    config.wings.canard_wing.twists.tip               = vector_angle     
    for network in  config.networks:  
        for propulsor in  network.propulsors:
            propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
    configs.append(config)

    return configs 

    
 
def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        if config.networks.electric.propulsors['prop_rotor_propulsor_1'].rotor.orientation_euler_angles[1] > 45*Units.degrees: 
            analysis.aerodynamics.settings.drag_coefficient_increment =  0.10
        elif config.networks.electric.propulsors['prop_rotor_propulsor_1'].rotor.orientation_euler_angles[1] > 15*Units.degrees: 
            analysis.aerodynamics.settings.drag_coefficient_increment =  0.05
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle 

    # ------------------------------------------------------------------
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.vehicle                               = vehicle 
    geometry.settings.update_center_of_gravity     = True 
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Weights
    weights         = RCAIDE.Framework.Analyses.Weights.Electric_VTOL()
    weights.aircraft_type = "VTOL"
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics         = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.maximum_lift_coefficient   =  1.5 
    aerodynamics.settings.drag_coefficient_increment =  0.01  
    analyses.append(aerodynamics)
     
    ## ------------------------------------------------------------------
    ##  Stability Analysis
    #stability         = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method() 
    #stability.vehicle = vehicle 
    #analyses.append(stability)    

    # ------------------------------------------------------------------
    # #  Noise Analysis
    # noise = RCAIDE.Framework.Analyses.Noise.Frequency_Domain_Buildup()
    # noise.vehicle = vehicle 
    # analyses.append(noise)    
    
    # ------------------------------------------------------------------
    #  Energy 
    energy          = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    # done!
    return analyses    
 
def mission_setup(analyses ):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment()
  
    # ------------------------------------------------------------------
    # Vertical Climb 
    # ------------------------------------------------------------------ 
    segment                                                          = Segments.Vertical_Flight.Climb(base_segment)
    segment.tag                                                      = "Vertical_Climb"   
    segment.analyses.extend(analyses.vertical_climb)                
    segment.altitude_start                                           = 0  * Units.ft  
    segment.altitude_end                                             = 100.  * Units.ft   
    segment.climb_rate                                               = 300. * Units['ft/min']  
    segment.initial_battery_state_of_charge                          = 1.0 

    # define flight dynamics to model  
    segment.flight_dynamics.force_z                                  = True 

    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4',
                                                                         'prop_rotor_propulsor_5','prop_rotor_propulsor_6','prop_rotor_propulsor_7','prop_rotor_propulsor_8']]
    
    mission.append_segment(segment)     


    # ------------------------------------------------------------------
    #   Hover 
    # ------------------------------------------------------------------ 
    segment                                                          = Segments.Vertical_Flight.Hover(base_segment)
    segment.tag                                                      = "Hover"   
    segment.analyses.extend(analyses.vertical_climb)

    segment.state.numerics.solver.type                               = "root_finder"    
    segment.altitude                                                 = 100.0  * Units.ft   
                        
    # define flight dynamics to model              
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4',
                                                                         'prop_rotor_propulsor_5','prop_rotor_propulsor_6','prop_rotor_propulsor_7','prop_rotor_propulsor_8']]
      
    mission.append_segment(segment)

    '''
    # ------------------------------------------------------------------
    #  Second Vertical Climb
    # ------------------------------------------------------------------ 
    segment                                                          = Segments.Vertical_Flight.Climb(base_segment)
    segment.tag                                                      = "Vertical_Climb_1"   
    segment.analyses.extend(analyses.vertical_climb)                   
    segment.altitude_end                                             = 60.  * Units.ft   
    segment.climb_rate                                               = 500. * Units['ft/min']  
    segment.state.numerics.solver.type                              = "root_finder"
          
    # define flight dynamics to model            
    segment.flight_dynamics.force_z                                  = True 

    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True  
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.6]]
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4',
                                                            'prop_rotor_propulsor_5','prop_rotor_propulsor_6','prop_rotor_propulsor_7','prop_rotor_propulsor_8']]
    
    mission.append_segment(segment)   
    

    # ------------------------------------------------------------------
    #  First Transition Segment
    # ------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                                           = "Low_Altitude_Climb"  
    segment.analyses.extend( analyses.vertical_transition)   
    segment.altitude_end                                  = 500. * Units.ft 
    segment.air_speed_start                               = 10 * Units.kts
    segment.air_speed_end                                 = 90 * Units.kts
    segment.climb_rate                                    = 628.0 * Units['ft/min'] 

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4',
                                                                             'prop_rotor_propulsor_5','prop_rotor_propulsor_6','prop_rotor_propulsor_7','prop_rotor_propulsor_8']]
    segment.assigned_control_variables.body_angle.active             = True 
    
    mission.append_segment(segment)
 
      
    # ------------------------------------------------------------------
    #  Second Transition Segment
    # ------------------------------------------------------------------ 
    segment                           = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                       = "high_speed_climb_transition"  
    segment.analyses.extend( analyses.climb_transition)   
    segment.air_speed_end             = 150.  * Units['mph']  
    segment.acceleration              = 9.81/5  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4',
                                                                             'prop_rotor_propulsor_5','prop_rotor_propulsor_6','prop_rotor_propulsor_7','prop_rotor_propulsor_8']]
    segment.assigned_control_variables.body_angle.active             = True
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                           = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                       = "Climb"  
    segment.analyses.extend(analyses.cruise) 
    segment.climb_rate                = 500. * Units['ft/min']
    segment.air_speed_start           = 125.   * Units['mph']
    segment.air_speed_end             = 130.  * Units['mph']  
    segment.altitude_start            = 500.0 * Units.ft   
    segment.altitude_end              = 1000.0 * Units.ft  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4',
                                                                             'prop_rotor_propulsor_5','prop_rotor_propulsor_6','prop_rotor_propulsor_7','prop_rotor_propulsor_8']]  
        
    segment.assigned_control_variables.body_angle.active             = True
    
    mission.append_segment(segment)
    
    

    # ------------------------------------------------------------------
    #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                      = "Cruise"  
    segment.analyses.extend(analyses.cruise) 
    segment.altitude                 = 1000.0 * Units.ft
    segment.air_speed                = 150.  * Units['mph']   
    segment.distance                 = 30*Units.nmi 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4',
                                                                             'prop_rotor_propulsor_5','prop_rotor_propulsor_6','prop_rotor_propulsor_7','prop_rotor_propulsor_8']]
    segment.assigned_control_variables.body_angle.active             = True
    mission.append_segment(segment)     
    
    # ------------------------------------------------------------------
    #    Descent Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "Descent"  
    segment.analyses.extend(analyses.cruise)
    segment.climb_rate               = -300. * Units['ft/min']
    segment.air_speed_start          = 130.  * Units['mph'] 
    segment.air_speed_end            = 90 * Units.kts 
    segment.altitude_start           = 2500.0 * Units.ft
    segment.altitude_end             = 500.0 * Units.ft
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4',
                                                                             'prop_rotor_propulsor_5','prop_rotor_propulsor_6','prop_rotor_propulsor_7','prop_rotor_propulsor_8']]
    segment.assigned_control_variables.body_angle.active             = True
    
        
    mission.append_segment(segment)     
  
    # ------------------------------------------------------------------
    #    Descent Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "Cruise descent"  
    segment.analyses.extend(analyses.cruise)
    segment.climb_rate               = -500. * Units['ft/min']
    segment.air_speed_start          = 130.  * Units['mph'] 
    segment.air_speed_end            = 100.   * Units['mph'] 
    segment.altitude_start           = 1000.0 * Units.ft
    segment.altitude_end             = 500.0 * Units.ft 

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4',
                                                                             'prop_rotor_propulsor_5','prop_rotor_propulsor_6','prop_rotor_propulsor_7','prop_rotor_propulsor_8']]
    segment.assigned_control_variables.body_angle.active             = True
    
        
    mission.append_segment(segment)    
 
    # ------------------------------------------------------------------
    #  High-Speed Descending Transition Segment
    # ------------------------------------------------------------------ 
    segment                          = Segments.Climb.Constant_Acceleration_Constant_Pitchrate_Constant_Angle(base_segment)
    segment.tag                      = "Approach_Transition"   
    segment.analyses.extend(analyses.approach_transition)  
    segment.altitude_start                                = 500.0 * Units.ft   
    segment.altitude_end                                  = 50.0 * Units.ft   
    segment.climb_angle                                   = 7.125 * Units.degrees
    segment.acceleration                                  = -0.9574 * Units['m/s/s']    
    segment.pitch_initial                                 = 4.3  * Units.degrees  
    segment.pitch_final                                   = 7. * Units.degrees      

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4',
                                                                             'prop_rotor_propulsor_5','prop_rotor_propulsor_6','prop_rotor_propulsor_7','prop_rotor_propulsor_8']]
    segment.assigned_control_variables.body_angle.active             = True
    
        
    mission.append_segment(segment)
    '''
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Vertical Descent 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                                         = Segments.Vertical_Flight.Descent(base_segment)
    segment.tag                                                     = "Vertical_Descent" 
    segment.analyses.extend( analyses.vertical_descent)               
    segment.altitude_start                                          = 100.0 * Units.ft   
    segment.altitude_end                                            = 0.   * Units.ft  
    segment.descent_rate                                            = 300. * Units['ft/min']   
                  
    # define flight dynamics to model              
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4',
                                                                             'prop_rotor_propulsor_5','prop_rotor_propulsor_6','prop_rotor_propulsor_7','prop_rotor_propulsor_8']]  
            
    mission.append_segment(segment)      
 
    return mission


def missions_setup(mission): 
 
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions  

# ----------------------------------------------------------------------
#   Plot Results
# ----------------------------------------------------------------------

def plot_results(results):
    # Plots fligh conditions 
    plot_flight_conditions(results) 
    
    # Plot arcraft trajectory
    plot_flight_trajectory(results)
    
    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results)  
     
    # Plot Aircraft Stability
    plot_longitudinal_stability(results) 
    
    # Plot Aircraft Electronics 
    plot_battery_temperature(results)
    plot_battery_cell_conditions(results) 
    plot_battery_degradation(results) 
    plot_electric_propulsor_efficiencies(results) 
    
    # Plot Propeller Conditions 
    plot_rotor_conditions(results)  
     
    # Plot Battery Degradation  
    plot_battery_degradation(results)   
      
    return  

def save_aircraft_geometry(geometry,filename): 
    pickle_file  = filename + '.pkl'
    with open(pickle_file, 'wb') as file:
        pickle.dump(geometry, file) 
    return 


def load_aircraft_geometry(filename):  
    load_file = filename + '.pkl' 
    with open(load_file, 'rb') as file:
        results = pickle.load(file) 
    return results

if __name__ == '__main__': 
    main()    
    plt.show()
