
#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units 
from RCAIDE.Library.Methods.Geometry.Planform                     import segment_properties,wing_segmented_planform   
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor  import design_electric_rotor
from RCAIDE.Library.Plots                                         import * 
from RCAIDE import  load 
from RCAIDE import  save  

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
    new_geometry = True
    redesign_rotors =  False 
    if new_geometry :
        vehicle = vehicle_setup(redesign_rotors)
        save_aircraft_geometry(vehicle , 'Hexacopter')
    else: 
        vehicle = load_aircraft_geometry('Hexacopter') 

    # Set up configs
    configs = configs_setup(vehicle)

    # vehicle analyses
    analyses = analyses_setup(configs)

    # mission analyses
    mission = mission_setup(analyses)
    missions = missions_setup(mission) 
     
    results = missions.base_mission.evaluate() 
     
    # plot the results 
    plot_results(results) 

    ## plot vehicle 
  
    return 
# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup(redesign_rotors) : 

    ospath                                      = os.path.abspath(__file__)
    separator                                   = os.path.sep
    airfoil_path                                = os.path.dirname(ospath) + separator  + '..' + separator  
    local_path                                  = os.path.dirname(ospath) + separator       
     
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    vehicle                                     = RCAIDE.Vehicle()
    vehicle.tag                                 = 'Hexacopter'
    vehicle.configuration                       = 'eVTOL'
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties 
    vehicle.mass_properties.max_takeoff         = 3300 
    vehicle.mass_properties.takeoff             = vehicle.mass_properties.max_takeoff
    vehicle.mass_properties.operating_empty     = vehicle.mass_properties.max_takeoff 
    vehicle.mass_properties.center_of_gravity   = [[2.6, 0., 0. ] ] 
                                                
    # This needs updating                       
    vehicle.number_of_passengers                = 6
    vehicle.reference_area                      = 73  * Units.feet**2 
    vehicle.flight_envelope.ultimate_load       = 5.7   
    vehicle.flight_envelope.positive_limit_load = 3.  
                                                
    wing                                        = RCAIDE.Library.Components.Wings.Main_Wing()   
    wing.tag                                    = 'main_wing'   
    wing.aspect_ratio                           = 0.5 
    wing.sweeps.quarter_chord                   = 0.  
    wing.thickness_to_chord                     = 0.01   
    wing.spans.projected                        = 0.01  
    wing.chords.root                            = 0.01
    wing.total_length                           = 0.01
    wing.chords.tip                             = 0.01
    wing.chords.mean_aerodynamic                = 0.01
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = 0.0001 
    wing.areas.wetted                           = 0.01
    wing.areas.exposed                          = 0.01  
    wing.symbolic                               = True 
    wing.xz_plane_symmetric                     = True 
    
    vehicle.append_component(wing)
    
    # ------------------------------------------------------    
    # FUSELAGE    
    # ------------------------------------------------------    
    # FUSELAGE PROPERTIES
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 2.  
    fuselage.seat_pitch                         = 3.  
    fuselage.fineness.nose                      = 0.88   
    fuselage.fineness.tail                      = 1.13   
    fuselage.lengths.nose                       = 0.5 
    fuselage.lengths.tail                       = 0.5
    fuselage.lengths.cabin                      = 4.
    fuselage.lengths.total                      = 5.
    fuselage.width                              = 1.8
    fuselage.heights.maximum                    = 1.8
    fuselage.heights.at_quarter_length          = 1.8
    fuselage.heights.at_wing_root_quarter_chord = 1.8
    fuselage.heights.at_three_quarters_length   = 1.8
    fuselage.areas.wetted                       = 19.829265
    fuselage.areas.front_projected              = 1.4294246 
    fuselage.effective_diameter                 = 1.4
    fuselage.differential_pressure              = 1. 
    
    # Segment  
    segment                          = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                      = 'segment_0'   
    segment.percent_x_location       = 0.  
    segment.percent_z_location       = 0.0 
    segment.height                   = 0.1   
    segment.width                    = 0.1   
    fuselage.append_segment(segment)            
                                                
    # Segment                                   
    segment                         = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                     = 'segment_1'   
    segment.percent_x_location      = 0.200/4.
    segment.percent_z_location      = 0.1713/4.
    segment.height                  = 0.737
    segment.width                   = 1.2   
    fuselage.append_segment(segment)            
                                                
    # Segment                                   
    segment                         = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                     = 'segment_2'   
    segment.percent_x_location      = 0.8251/4.
    segment.percent_z_location      = 0.2840/4.
    segment.height                  = 1.40 
    segment.width                   = 1.8 
    fuselage.append_segment(segment)            
                                                
    # Segment                                  
    segment                         = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                     = 'segment_3'   
    segment.percent_x_location      = 3.342/4.
    segment.percent_z_location      = 0.356/4.
    segment.height                  = 1.40
    segment.width                   = 1.8 
    fuselage.append_segment(segment)  
                                                
    # Segment                                   
    segment                         = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                     = 'segment_4'   
    segment.percent_x_location      = 3.70004/4.
    segment.percent_z_location      = 0.4636/4.
    segment.height                  = 0.9444
    segment.width                   = 1.2 
    fuselage.append_segment(segment)             
    
    # Segment                                   
    segment                         = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                     = 'segment_5'   
    segment.percent_x_location      = 1.
    segment.percent_z_location      = 0.6320/4.
    segment.height                  = 0.1    
    segment.width                   = 0.1    
    fuselage.append_segment(segment)             
    
                                                 
    # add to vehicle
    vehicle.append_component(fuselage)   
    

    #------------------------------------------------------------------------------------------------------------------------------------
    # ########################################################  Energy Network  ######################################################### 
    #------------------------------------------------------------------------------------------------------------------------------------
    # define network
    network                           = RCAIDE.Framework.Networks.Electric() 
    network.charging_power            = 1000
    
    #==================================================================================================================================== 
    # Lift Bus 
    #====================================================================================================================================          
    bus                               = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    bus.tag                           = 'bus'
    bus.number_of_battery_modules     = 1
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus Battery
    #------------------------------------------------------------------------------------------------------------------------------------ 
    battery_module                                                    = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC() 
    battery_module.electrical_configuration.series                    = 150   
    battery_module.electrical_configuration.parallel                  = 270  
    battery_module.geometrtic_configuration.normal_count              = 900
    battery_module.geometrtic_configuration.parallel_count            = 45
    battery_module.geometrtic_configuration.stacking_rows             = 9
    battery_module.origin                                             = [[0.5, 0, 0]]
    bus.battery_modules.append(battery_module)    
    bus.initialize_bus_properties()

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Lift Propulsors 
    #------------------------------------------------------------------------------------------------------------------------------------    
     
    # Define Lift Propulsor Container 
    propulsor                                              = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
    propulsor.tag                                          = 'propulsor'
    propulsor.wing_mounted                                 = True         
              
    # Electronic Speed Controller           
    lift_rotor_esc                                         = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller() 
    lift_rotor_esc.efficiency                              = 0.95     
    lift_rotor_esc.origin                                  = [[-0.073 ,  1.950 , 1.2]] 
    propulsor.electronic_speed_controller                  = lift_rotor_esc 
           
    # Lift Rotor Design              
    g                                                      = 9.81                                     
    Hover_Load                                             = vehicle.mass_properties.takeoff*g *1.1 
    
    lift_rotor                                             = RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor()    
    lift_rotor.active                                      = True           
    lift_rotor.tip_radius                                  = 2.5
    lift_rotor.hub_radius                                  = 0.15 * lift_rotor.tip_radius 
    lift_rotor.number_of_blades                            = 3
    
    lift_rotor.hover.design_altitude                       = 40 * Units.feet  
    lift_rotor.hover.design_thrust                         = Hover_Load/6
    lift_rotor.hover.design_freestream_velocity            = np.sqrt(lift_rotor.hover.design_thrust/(2*1.2*np.pi*(lift_rotor.tip_radius**2)))
    
    lift_rotor.oei.design_altitude                         = 40 * Units.feet  
    lift_rotor.oei.design_thrust                           = Hover_Load/5  
    lift_rotor.oei.design_freestream_velocity              = np.sqrt(lift_rotor.oei.design_thrust/(2*1.2*np.pi*(lift_rotor.tip_radius**2)))
    
    airfoil                                                = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file                                = 'NACA_4412.txt'
    airfoil.polar_files                                    = ['NACA_4412_polar_Re_50000.txt' ,
                                                             'NACA_4412_polar_Re_100000.txt' ,
                                                             'NACA_4412_polar_Re_200000.txt' ,
                                                              'NACA_4412_polar_Re_500000.txt' ,
                                                              'NACA_4412_polar_Re_1000000.txt',
                                                              'NACA_4412_polar_Re_3500000.txt',
                                                              'NACA_4412_polar_Re_5000000.txt',
                                                              'NACA_4412_polar_Re_7500000.txt' ]
    lift_rotor.append_airfoil(airfoil)                         
    lift_rotor.airfoil_polar_stations                      = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
    propulsor.rotor                                        = lift_rotor

    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Motor  
    #------------------------------------------------------------------------------------------------------------------------------------    
    lift_rotor_motor                                       = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    lift_rotor_motor.efficiency                            = 0.9
    lift_rotor_motor.nominal_voltage                       = bus.voltage*3/4    
    lift_rotor_motor.no_load_current                       = 2      
    propulsor.motor                                        = lift_rotor_motor
     
    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Nacelle
    #------------------------------------------------------------------------------------------------------------------------------------     
    nacelle                                                = RCAIDE.Library.Components.Nacelles.Nacelle()
    nacelle.tag                                            = 'rotor_nacelle' 
    nacelle.length                                         = 0.4
    nacelle.diameter                                       = 2.6*2
    nacelle.inlet_diameter                                 = 2.55*2     
    nacelle.orientation_euler_angles                       = [0,-90*Units.degrees,0.]    
    nacelle.flow_through                                   = True  
    propulsor.nacelle                                      = nacelle
    
    if redesign_rotors:
        design_electric_rotor(propulsor)
        save_propulsor(propulsor, os.path.join(local_path, 'lift_rotor_propulsor.res'))
    else:
        regression_prop_rotor_propulsor                                       = deepcopy(propulsor)        
        design_electric_rotor(regression_prop_rotor_propulsor, iterations=2)
        loaded_propulsor                                                      = load_propulsor(os.path.join(local_path, 'lift_rotor_propulsor.res'))

        for key,item in propulsor.prop_rotor.items():
            propulsor.prop_rotor[key]                                         = loaded_propulsor.rotor[key] 
        for key,item in propulsor.motor.items():
            propulsor.motor[key]                                              = loaded_propulsor.motor[key]  

    # Front Rotors Locations
            # foward starboard |foward port |    middle starboard |middle port  | aft starbard  | aft port   
    origins = [[ -1.5,2.6,1.8],[ -1.5,-2.6,1.8], [2.5,6.0,1.8] ,[2.5,-6.,1.8], [6.5,2.6,1.8] ,[6.5,-2.6,1.8]]  
    
    for i in range(len(origins)): 
        propulsor_i                                       = deepcopy(propulsor)
        propulsor_i.tag                                   = 'rotor_propulsor_' + str(i + 1)
        propulsor_i.rotor.tag                             = 'rotor_' + str(i + 1) 
        propulsor_i.rotor.origin                          = [origins[i]]  
        propulsor_i.motor.tag                             = 'rotor_motor_' + str(i + 1)   
        propulsor_i.motor.origin                          = [origins[i]]  
        propulsor_i.electronic_speed_controller.tag       = 'rotor_esc_' + str(i + 1)  
        propulsor_i.electronic_speed_controller.origin    = [origins[i]]  
        propulsor_i.nacelle.tag                           = 'rotor_nacelle_' + str(i + 1)  
        propulsor_i.nacelle.origin                        = [origins[i]]   
        bus.propulsors.append(propulsor_i)    
 
                             
    # Avionics                            
    avionics                                              = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw                                   = 10. # Watts  
    avionics.mass_properties.mass                         = 1.0 * Units.kg
    bus.avionics                                          = avionics    

   
    network.busses.append(bus)       
        
    # append energy network 
    vehicle.append_energy_network(network) 
 
    return vehicle
  
# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------
def configs_setup(vehicle):
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs                        = RCAIDE.Library.Components.Configs.Config.Container()

    base_config                    = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag                = 'base'
    configs.append(base_config)
    
    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------
    config                         = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                     = 'hover'  
    configs.append(config)
    
    # ------------------------------------------------------------------
    #    Configuration
    # ------------------------------------------------------------------
    config                         = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                     = 'vertical_flight'    
    configs.append(config)
    
  
    # ------------------------------------------------------------------
    #    Configuration
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                        = 'vertical_transition'   
    for network in  config.networks: 
        for bus in network.busses: 
            for propulsor in  bus.propulsors: 
                propulsor.rotor.blade_pitch_command   = 3.  * Units.degrees 
    configs.append(config)
      
      
    # ------------------------------------------------------------------
    #    Configuration
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                        = 'descent_transition'   
    for network in  config.networks: 
        for bus in network.busses: 
            for propulsor in  bus.propulsors: 
                propulsor.rotor.blade_pitch_command   = 3.  * Units.degrees 
    configs.append(config)
  
    
    # ------------------------------------------------------------------
    #    Configuration
    # ------------------------------------------------------------------
    config                                           = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                       = 'climb'   
    for network in  config.networks: 
        for bus in network.busses: 
            for propulsor in  bus.propulsors: 
                propulsor.rotor.blade_pitch_command   = 2.  * Units.degrees 
    configs.append(config) 
    
    # ------------------------------------------------------------------
    #    Configuration
    # ------------------------------------------------------------------
    config                                           = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                       = 'forward_flight' 
    for network in  config.networks: 
        for bus in network.busses: 
            for propulsor in  bus.propulsors: 
                propulsor.rotor.blade_pitch_command   = 5.  * Units.degrees 
    configs.append(config)     
    
    return configs

def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

# ------------------------------------------------------------------
# Base Analysis
# ------------------------------------------------------------------
def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses                      = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle              = vehicle 
    
    # ------------------------------------------------------------------
    #  Weights
    weights                       = RCAIDE.Framework.Analyses.Weights.Electric()
    weights.aircraft_type         = "VTOL"
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics                   = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()  
    analyses.append(aerodynamics)
     
    # ------------------------------------------------------------------
    #  Stability Analysis
    stability                      = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method()  
    analyses.append(stability)    

    # ------------------------------------------------------------------
    #  Energy
    energy                         = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet                         = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere                     = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    # done!
    return analyses    



# ------------------------------------------------------------------
#   Baseline Mission Setup
# ------------------------------------------------------------------
def mission_setup(analyses): 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission                             = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag                         = 'mission'

    # unpack Segments module
    Segments                            = RCAIDE.Framework.Mission.Segments  
    base_segment                        = Segments.Segment()
     
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 

    segment                                                          = Segments.Vertical_Flight.Climb(base_segment)
    segment.tag                                                      = "Vertical_Climb" 
    segment.analyses.extend( analyses.vertical_flight) 
    segment.altitude_start                                           = 0.0  * Units.ft 
    segment.altitude_end                                             = 50.  * Units.ft  
    segment.climb_rate                                               = 500. * Units['ft/min']   
    segment.initial_battery_state_of_charge                          = 1.0
    segment.true_course                                              = 0 * Units.degree 
    
    # define flight dynamics to model  
    segment.flight_dynamics.force_z                                  = True 

    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['rotor_propulsor_1','rotor_propulsor_2','rotor_propulsor_3',
                                                                         'rotor_propulsor_4','rotor_propulsor_5','rotor_propulsor_6']]  
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #  First Transition Segment
    # ------------------------------------------------------------------  

    segment                                                          = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                                                      = "Vertical_Transition"  
    segment.analyses.extend( analyses.vertical_transition) 
    segment.altitude                                                 = 50.  * Units.ft       
    segment.air_speed_start                                          = 300. * Units['ft/min'] 
    segment.air_speed_end                                            = 35 * Units['mph']    
    segment.acceleration                                             = 1.0
    segment.pitch_initial                                            = 0. * Units.degrees
    segment.pitch_final                                              = 0. * Units.degrees
    segment.true_course                                              = 0 * Units.degree  
 
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True        
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['rotor_propulsor_1','rotor_propulsor_2','rotor_propulsor_3',
                                                                         'rotor_propulsor_4','rotor_propulsor_5','rotor_propulsor_6']]
    segment.assigned_control_variables.body_angle.active             = True 
    
    mission.append_segment(segment)

    
    # ------------------------------------------------------------------
    #   Climb from vertiport
    # ------------------------------------------------------------------
    segment                                                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                                                      = "Departure_Climb"  
    segment.analyses.extend(analyses.climb) 
    segment.climb_rate                                               = 551. * Units['ft/min']
    segment.air_speed_start                                          = 35.   * Units['mph']
    segment.air_speed_end                                            = 55.  * Units['mph']   
    segment.altitude_end                                             = 500.0 * Units.ft
    segment.true_course                                              = 30 * Units.degree 
        
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True        
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['rotor_propulsor_1','rotor_propulsor_2','rotor_propulsor_3',
                                                                         'rotor_propulsor_4','rotor_propulsor_5','rotor_propulsor_6']]
    segment.assigned_control_variables.body_angle.active             = True 
    mission.append_segment(segment)
    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Circular departure pattern 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "Departure_Pattern_Curve"     
    segment.analyses.extend(analyses.climb) 
    segment.altitude                                      = 500.0 * Units.ft  
    segment.air_speed                                     = 55.  * Units['mph']       
    segment.turn_radius                                   = 3600 * Units.feet  
    segment.true_course                                   = 0 * Units.degree  
    segment.turn_angle                                    = 90 * Units.degree
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True    
    segment.flight_dynamics.force_z                       = True    
    segment.flight_dynamics.force_y                       = True     
    segment.flight_dynamics.moment_y                      = True 
    segment.flight_dynamics.moment_x                      = True
    segment.flight_dynamics.moment_z                      = True 

    # define flight controls              
    segment.assigned_control_variables.throttle.active                          = True
     
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['rotor_propulsor_1','rotor_propulsor_2'],['rotor_propulsor_3'],
                                                                                    ['rotor_propulsor_4','rotor_propulsor_5'],['rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active                        = True     
    segment.assigned_control_variables.bank_angle.active                        = True    
    segment.assigned_control_variables.bank_angle.initial_guess_values          = [[20.0 * Units.degree]]
    
    mission.append_segment(segment) 
 
     
    # ------------------------------------------------------------------
    #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------
    segment                                                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                                                      = "Climb_2"  
    segment.analyses.extend(analyses.climb) 
    segment.climb_rate                                               = 500. * Units['ft/min']
    segment.air_speed_start                                          = 55.   * Units['mph']
    segment.air_speed_end                                            = 75.  * Units['mph']       
    segment.altitude_start                                           = 500.0 * Units.ft     
    segment.altitude_end                                             = 1000.0 * Units.ft
    segment.true_course                                              = 90 * Units.degree  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True        
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['rotor_propulsor_1','rotor_propulsor_2','rotor_propulsor_3',
                                                                         'rotor_propulsor_4','rotor_propulsor_5','rotor_propulsor_6']]
    segment.assigned_control_variables.body_angle.active             = True     
    mission.append_segment(segment)                

    # ------------------------------------------------------------------
    #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                                                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                                      = "Cruise"  
    segment.analyses.extend(analyses.forward_flight)  
    segment.altitude                                                 = 1000.0 * Units.ft      
    segment.air_speed                                                = 75. * Units['mph']      
    segment.distance                                                 = 58*Units.nmi
    segment.true_course                                              = 90 * Units.degree  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True        
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['rotor_propulsor_1','rotor_propulsor_2','rotor_propulsor_3',
                                                                         'rotor_propulsor_4','rotor_propulsor_5','rotor_propulsor_6']]
    segment.assigned_control_variables.body_angle.active             = True          
    mission.append_segment(segment)      
    
                
    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                                                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                                                      = "Descent"  
    segment.analyses.extend(analyses.forward_flight)
    segment.climb_rate                                               = -200. * Units['ft/min']
    segment.air_speed_start                                          = 75. * Units['mph']      
    segment.air_speed_end                                            = 55. * Units['mph']      
    segment.altitude_start                                           = 1000.0 * Units.ft 
    segment.altitude_end                                             = 500.0 * Units.ft
    segment.true_course                                              = 90 * Units.degree  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True        
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['rotor_propulsor_1','rotor_propulsor_2','rotor_propulsor_3',  'rotor_propulsor_4','rotor_propulsor_5','rotor_propulsor_6']]
    segment.assigned_control_variables.body_angle.active             = True     
    mission.append_segment(segment)     
        
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Circular approach pattern 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                                          = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                                      = "Approach_Pattern_Curve"     
    segment.analyses.extend(analyses.climb) 
    segment.altitude                                                 = 500.0 * Units.ft  
    segment.air_speed                                                = 55.  * Units['mph']       
    segment.turn_radius                                              = 3600 * Units.feet  
    segment.true_course                                              = 90 * Units.degree     
    segment.turn_angle                                               = 90 * Units.degree
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                  = True    
    segment.flight_dynamics.force_z                                  = True    
    segment.flight_dynamics.force_y                                  = True     
    segment.flight_dynamics.moment_y                                 = True 
    segment.flight_dynamics.moment_x                                 = True
    segment.flight_dynamics.moment_z                                 = True 

    # define flight controls              
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['rotor_propulsor_1','rotor_propulsor_2'],['rotor_propulsor_3'],
                                                                                    ['rotor_propulsor_4','rotor_propulsor_5'],['rotor_propulsor_6']]  
    segment.assigned_control_variables.body_angle.active                        = True     
    segment.assigned_control_variables.bank_angle.active                        = True    
    segment.assigned_control_variables.bank_angle.initial_guess_values          = [[20.0 * Units.degree]]
    
    mission.append_segment(segment) 
 
 
    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                                                         = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                                                     = "Approach"  
    segment.analyses.extend(analyses.forward_flight)
    segment.climb_rate                                              = -560. * Units['ft/min']
    segment.air_speed_start                                         = 55. * Units['mph']      
    segment.air_speed_end                                           = 35. * Units['mph']      
    segment.altitude_start                                          = 500.0 * Units.ft 
    segment.altitude_end                                            = 50.0 * Units.ft
    segment.true_course                                             = 180 * Units.degree

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                 = True  
    segment.flight_dynamics.force_z                                 = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True        
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['rotor_propulsor_1','rotor_propulsor_2','rotor_propulsor_3',
                                                                         'rotor_propulsor_4','rotor_propulsor_5','rotor_propulsor_6']]
    segment.assigned_control_variables.body_angle.active             = True     
    mission.append_segment(segment)      
 
 
      
    # ------------------------------------------------------------------
    #  Third Transition Segment
    # ------------------------------------------------------------------

    segment                                                          = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                                                      = "Decent_Transition" 
    segment.analyses.extend( analyses.descent_transition) 
    segment.altitude                                                 = 50.  * Units.ft 
    segment.air_speed_start                                          = 35.  * Units['mph'] 
    segment.air_speed_end                                            = 300. * Units['ft/min']
    segment.acceleration                                             = -0.5307 
    segment.pitch_initial                                            = 1. * Units.degrees
    segment.pitch_final                                              = 2. * Units.degrees
    segment.true_course                                              = 180 * Units.degree  

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True        
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['rotor_propulsor_1','rotor_propulsor_2','rotor_propulsor_3',
                                                                         'rotor_propulsor_4','rotor_propulsor_5','rotor_propulsor_6']]
    segment.assigned_control_variables.body_angle.active             = True     
    mission.append_segment(segment)

    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                                                          = Segments.Vertical_Flight.Descent(base_segment)
    segment.tag                                                      = "Vertical_Descent"  
    segment.analyses.extend( analyses.vertical_flight) 
    segment.altitude_start                                           = 50.0  * Units.ft  
    segment.altitude_end                                             = 0.  * Units.ft  
    segment.descent_rate                                             = 300. * Units['ft/min']
    segment.true_course                                              = 180 * Units.degree  

    # define flight dynamics to model  
    segment.flight_dynamics.force_z                                  = True 

    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['rotor_propulsor_1','rotor_propulsor_2','rotor_propulsor_3',
                                                                         'rotor_propulsor_4','rotor_propulsor_5','rotor_propulsor_6']]      
    mission.append_segment(segment)
             
  
    return mission 
 

def missions_setup(mission): 
 
    missions = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions

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
    plot_disc_and_power_loading(results)
     
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



def load_propulsor(filename):
    propulsor =  load(filename)
    return propulsor

def save_propulsor(propulsor, filename):
    save(propulsor, filename)
    return


if __name__ == '__main__': 
    main()    
    plt.show()
