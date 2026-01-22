
# Navion.py
#  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE 
from RCAIDE.Framework.Core                              import Units    
from RCAIDE.Library.Methods.Powertrain.Propulsors.Internal_Combustion_Engine import design_internal_combustion_engine
from RCAIDE.Library.Plots                               import *  

# python imports 
import os 
import numpy as np
import pylab as plt
import  pickle
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main(): 
  
    vehicle  = vehicle_setup()
    vehicle.mass_properties.takeoff = None
   
    # Set up vehicle configs
    configs           = configs_setup(vehicle)

    # create analyses
    analyses          = analyses_setup(configs)

    # mission analyses
    mission           = mission_setup(analyses) 
    # mission            = turn_mission_setup(analyses)

    # create mission instances (for multiple types of missions)
    missions          = missions_setup(mission) 

    # mission analysis 
    results           = missions.base_mission.evaluate()  

    display_stability_derivatives(results.segments[0])  
         
    # plt results
    plot_mission(results)

    return  

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config, configs)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle, configs):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses        = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle =  vehicle
    
    # ------------------------------------------------------------------
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry() 
    geometry.settings.overwrite_reference        = True
    geometry.settings.update_wing_properties     = True
    geometry.settings.print_weight_analysis_report = True
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------
    weights         = RCAIDE.Framework.Analyses.Weights.Conventional_General_Aviation()
    weights.method  = "FLOPS"
    weights.aircraft_type = "General_Aviation"
    weights.settings.FLOPS.fidelity    = "Complex"
    weights.settings.print_weight_analysis_report = True
    weights.settings.update_moment_of_inertia = True
    weights.settings.update_center_of_gravity = False
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics 
    # ------------------------------------------------------------------
    aerodynamics                                      = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()        
    aerodynamics.settings.model_fuselage               = False #True               
    aerodynamics.settings.model_nacelle                = False #True       
    analyses.append(aerodynamics)

    # aerodynamics                                      = RCAIDE.Framework.Analyses.Aerodynamics.Athena_Vortex_Lattice()  
    # aerodynamics.vehicle                              = vehicle      
    # aerodynamics.settings.model_fuselage               = False               
    # aerodynamics.settings.model_nacelle                = True       
    # analyses.append(aerodynamics)

    # aerodynamics                                      = RCAIDE.Framework.Analyses.Aerodynamics.SU2_Euler()  
    # aerodynamics.vehicle                              = vehicle  
    # aerodynamics.settings.run_new_SU2_sim             = True
    # aerodynamics.training.angle_of_attack             = np.array([0, 3]) * Units.deg 
    # aerodynamics.training.Mach                                          = np.array([0.1]) 
    # analyses.append(aerodynamics)


    # ------------------------------------------------------------------
    #  Stability 
    # ------------------------------------------------------------------ 
    stability                                       = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method()  
    stability.settings.model_fuselage               = False #True                
    stability.settings.model_nacelle                = False #True      
    stability.vehicle                               = vehicle
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Energy
    # ------------------------------------------------------------------
    energy     = RCAIDE.Framework.Analyses.Energy.Energy()  
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    # ------------------------------------------------------------------
    planet     = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    # ------------------------------------------------------------------
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    # done!
    return analyses 

def vehicle_setup(): 
       # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------ 
    vehicle     = RCAIDE.Vehicle()
    vehicle.tag = 'Navion' 

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------

    # mass properties
    vehicle.mass_properties.max_takeoff               = 2948 * Units.pounds
    vehicle.mass_properties.takeoff                   = 2948 * Units.pounds
    vehicle.mass_properties.moments_of_inertia.tensor = np.array([[164627.7,0.0,0.0],[0.0,471262.4,0.0],[0.0,0.0,554518.7]])
    vehicle.mass_properties.center_of_gravity         = [[2.087,0,-0.131189711 ]]
    vehicle.mass_properties.max_payload               = 1100 * Units.pounds
    # vehicle.mass_properties.min_payload               = 0 * Units.pounds
    vehicle.mass_properties.max_zero_fuel = 2700 * Units.pounds 
    vehicle.mass_properties.max_fuel = 420 * Units.pounds
     
    vehicle.reference_area                            = 17.112 
    vehicle.number_of_passengers                      = 4
    vehicle.systems.control                           = "fully powered"
    vehicle.systems.accessories                       = "commuter"       
    
    # flight envelope 
    vehicle.flight_envelope.design_cruise_altitude    = 15000*Units.feet
    vehicle.flight_envelope.design_range              = 750 * Units.nmi
    vehicle.flight_envelope.ultimate_load             = 5.7
    vehicle.flight_envelope.positive_limit_load       = 3.8
    vehicle.flight_envelope.design_dynamic_pressure   = 1929.16080736607
    vehicle.flight_envelope.design_mach_number        = 0.1931864244395293
    
    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------   

    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing' 
    wing.sweeps.quarter_chord             = 0.165 * Units.degrees 
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 17.112
    wing.chords.mean_aerodynamic          = 1.74 
    wing.taper                            = 0.54 
    wing.aspect_ratio                     = 6.04  
    wing.spans.projected                  = 10.166
    wing.chords.root                      = 2.1944 
    wing.chords.tip                       = 1.1850
    wing.twists.root                      = 2 * Units.degrees  
    wing.twists.tip                       = -1 * Units.degrees   
    wing.dihedral                         = 7.5 * Units.degrees   
    wing.origin                           = [[1.652555594, 0.,-0.6006666]]
    wing.aerodynamic_center               = [1.852555594, 0., 6006666 ] # INCORRECT 
    wing.vertical                         = False
    wing.xz_plane_symmetric               = True
    wing.high_lift                        = True 
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0    

    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    rel_path                              = os.path.dirname(ospath) + separator + '..' + separator + '..' + separator

    tip_airfoil                           = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    tip_airfoil.NACA_4_Series_code        = '6410'      
    tip_airfoil.coordinate_file           = rel_path + 'Airfoils' + separator + 'NACA_6410.txt' 
   
    root_airfoil                          = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    root_airfoil.NACA_4_Series_code       = '4415'   
    root_airfoil.coordinate_file          = rel_path + 'Airfoils' + separator + 'NACA_4415.txt' 
    
    # Wing Segments 
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root_segment'
    segment.percent_span_location         = 0.0
    segment.twist                         = 2 * Units.degrees  
    segment.root_chord_percent            = 1.0
    segment.dihedral_outboard             = 7.5 * Units.degrees  
    segment.sweeps.quarter_chord          = 0.165 * Units.degrees  
    segment.thickness_to_chord            = .15 
    wing.append_segment(segment)  
         
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.0
    segment.twist                         = -1.0 * Units.degrees
    segment.root_chord_percent            = 0.54  
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 0 * Units.degrees  
    segment.thickness_to_chord            = .12
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)     
                                        
    # control surfaces ------------------------------------------- 
    flap                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                      = 'flap'
    flap.span_fraction_start      = 0.2
    flap.span_fraction_end        = 0.5
    flap.deflection               = 0.0 * Units.degrees 
    flap.chord_fraction           = 0.20
    wing.append_control_surface(flap)  
    

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.7
    aileron.span_fraction_end     = 0.9 
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.2
    wing.append_control_surface(aileron)      

    # add to vehicle
    vehicle.append_component(wing) 
    

    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------       
    wing                                  = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag                              = 'horizontal_stabilizer'  
    wing.sweeps.leading_edge              = 6 * Units.degrees 
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 4   
    wing.spans.projected                  = 4 
    wing.chords.root                      = 1.2394
    wing.chords.mean_aerodynamic          = 1.0484
    wing.chords.tip                       = 0.8304 
    wing.taper                            = wing.chords.tip/wing.chords.root
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                      = 0 * Units.degrees  
    wing.twists.tip                       = 0 * Units.degrees   
    wing.origin                           = [[ 6.54518625 , 0., 0.203859697]]
    wing.aerodynamic_center               = [[ 6.545186254 + 0.25*wing.spans.projected, 0., 0.203859697]] 
    wing.vertical                         = False 
    wing.xz_plane_symmetric               = True
    wing.high_lift                        = False 
    wing.dynamic_pressure_ratio           = 0.9  
    
    elevator                              = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                          = 'elevator'
    elevator.span_fraction_start          = 0.1
    elevator.span_fraction_end            = 0.9
    elevator.deflection                   = 0.0  * Units.deg
    elevator.chord_fraction               = 0.35
    wing.append_control_surface(elevator)
    
    
    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------ 
    wing                                  = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag                              = 'vertical_stabilizer'   
    wing.sweeps.leading_edge              = 20 * Units.degrees 
    wing.thickness_to_chord               = 0.125
    wing.areas.reference                  = 1.163 
    wing.spans.projected                  = 1.4816
    wing.chords.root                      = 1.2176
    wing.chords.tip                       = 0.5870 
    wing.aspect_ratio                     = 1.8874 
    wing.taper                            = 0.4820 
    wing.chords.mean_aerodynamic          = 0.9390 
    wing.twists.root                      = 0 * Units.degrees  
    wing.twists.tip                       = 0 * Units.degrees   
    wing.origin                           = [[ 7.127369987, 0., 0.303750948]]
    wing.aerodynamic_center               = [ 7.49778005775, 0., 0.67416101875] 
    wing.vertical                         = True 
    wing.xz_plane_symmetric               = False
    wing.t_tail                           = False
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0  
    
    rudder                                = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                            = 'rudder'
    rudder.span_fraction_start            = 0.1
    rudder.span_fraction_end              = 0.9
    rudder.deflection                     = 0.0  * Units.deg
    rudder.chord_fraction                 = 0.4
    wing.append_control_surface(rudder) 
    
    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------
    fuselage = RCAIDE.Library.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage'

    # define cabin    
    # cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Cabin() 
    # economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    # economy_class.number_of_seats_abrest              = 2
    # economy_class.number_of_rows                      = 2
    # economy_class.galley_lavatory_percent_x_locations = []  
    # economy_class.emergency_exit_percent_x_locations  = []      
    # economy_class.type_A_exit_percent_x_locations     = [] 
    # economy_class.number_of_seats                     = economy_class.number_of_rows  * economy_class.number_of_seats_abrest 
    # cabin.append_cabin_class(economy_class)
    # fuselage.append_cabin(cabin)
    
    fuselage.lengths.total                      = 8.349950916 
    fuselage.width                              = 1.22028016 
    fuselage.heights.maximum                    = 1.634415138  
    fuselage.areas.wetted                       = 12. # ESTIMATED 
    fuselage.areas.front_projected              = fuselage.width*fuselage.heights.maximum
    fuselage.effective_diameter                 = 1.22028016 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_0'
    segment.percent_x_location                  = 0
    segment.percent_z_location                  = 0
    segment.height                              = 0.529255748
    segment.width                               = 0.575603849
    fuselage.segments.append(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'
    segment.percent_x_location                  =  0.028527593
    segment.percent_z_location                  =  0
    segment.height                              =  0.737072721
    segment.width                               =  0.921265952 
    fuselage.segments.append(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'
    segment.percent_x_location                  = 0.187342754 
    segment.percent_z_location                  = 0 
    segment.height                              = 1.174231852 
    segment.width                               = 1.196956212
    fuselage.segments.append(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'
    segment.percent_x_location                  = 0.242034847 
    segment.percent_z_location                  = 0.011503528 
    segment.height                              = 1.450221906 
    segment.width                               = 1.173932059 
    fuselage.segments.append(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'
    segment.percent_x_location                  = 0.296715183 
    segment.percent_z_location                  = 0.015984303 
    segment.height                              = 1.634415138 
    segment.width                               = 1.22028016 
    fuselage.segments.append(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'
    segment.percent_x_location                  = 0.510275342 
    segment.percent_z_location                  = -0.005
    segment.height                              = 1.082135236 
    segment.width                               = 1.013062774 
    fuselage.segments.append(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'
    segment.percent_x_location                  = 0.833284347 
    segment.percent_z_location                  = 0.014138855 
    segment.height                              = 0.621652157 
    segment.width                               = 0.414134978
    fuselage.segments.append(segment)
 
    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'
    segment.percent_x_location                  = 1
    segment.percent_z_location                  = 0.018978667 
    segment.height                              = 0.092096616 
    segment.width                               = 0.046048308 
    fuselage.segments.append(segment)
    
    # add to vehicle
    vehicle.append_component(fuselage) 

    # ################################################# Landing Gear #############################################################   
    # ------------------------------------------------------------------        
    #  Landing Gear
    # Source: https://www.boeing.com/content/dam/boeing/boeingdotcom/commercial/airports/acaps/747_123sp.pdf 
    # ------------------------------------------------------------------  
    main_gear               = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.tire_diameter = 10.0 * Units.inches
    main_gear.strut_length  = 2.0 * Units.ft 
    main_gear.units         = 2    # Number of main landing gear
    main_gear.wheels        = 1    # Number of wheels on the main landing gear
    main_gear.origin = [[0.5,0,0]]
    vehicle.append_component(main_gear)  

    nose_gear               = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()       
    nose_gear.tire_diameter = 10. * Units.inches
    nose_gear.units         = 1    # Number of nose landing gear
    nose_gear.wheels        = 1    # Number of wheels on the nose landing gear
    nose_gear.strut_length  = 2.0 * Units.ft 
    nose_gear.origin        = [[1.75,0,0]]
    vehicle.append_component(nose_gear)

    # ########################################################  Energy Network  #########################################################  
    net                                         = RCAIDE.Framework.Networks.Fuel()   

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus
    #------------------------------------------------------------------------------------------------------------------------------------  
    fuel_line                                   = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()   

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Fuel Tank & Fuel
    #------------------------------------------------------------------------------------------------------------------------------------       
    fuel_tank                                             = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    fuel_tank.fuel                                        = RCAIDE.Library.Attributes.Propellants.Aviation_Gasoline()  
    fuel_line.fuel_tanks.append(fuel_tank)  
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    ice_prop                                   = RCAIDE.Library.Components.Powertrain.Propulsors.Internal_Combustion_Engine()      
                                                     
    # Engine                     
    engine                                     = RCAIDE.Library.Components.Powertrain.Converters.Engine()

    engine.sea_level_power                     = 185. * Units.horsepower 
    engine.rated_speed                         = 2300. * Units.rpm 
    engine.power_specific_fuel_consumption     = 0.01  * Units['lb/hp/hr']
    ice_prop.engine                            = engine 
    ice_prop.sealevel_static_thrust            = 2500 # N
     
    # Propeller 
    prop                                    = RCAIDE.Library.Components.Powertrain.Converters.Propeller()
    prop.tag                                = 'propeller'
    prop.number_of_blades                   = 2.0
    prop.tip_radius                         = 76./2. * Units.inches
    prop.hub_radius                         = 8.     * Units.inches
    prop.cruise.design_freestream_velocity  = 119.   * Units.knots
    prop.cruise.design_angular_velocity     = 2650.  * Units.rpm
    prop.cruise.design_Cl                   = 0.8
    prop.cruise.design_altitude             = 12000. * Units.feet
    prop.cruise.design_power                = .64 * 180. * Units.horsepower
    prop.variable_pitch                     = True    
    ice_prop.propeller                      = prop

    # design propeller ICE  
    design_internal_combustion_engine(ice_prop)
    
    net.propulsors.append(ice_prop) 
    
    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network      
    fuel_line.assigned_propulsors =  [[ice_prop.tag]]
    
    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network      
    net.fuel_lines.append(fuel_line)        
    
    # Append energy network to aircraft 
    vehicle.append_energy_network(net)    

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------ 
    Wuav                                        = 2. * Units.lbs
    avionics                                    = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.mass_properties.uninstalled        = Wuav
    vehicle.avionics                            = avionics     

    #------------------------------------------------------------------------------------------------------------------------------------ 
    #   Vehicle Definition Complete
    #------------------------------------------------------------------------------------------------------------------------------------ 
     
    return vehicle


# ----------------------------------------------------------------------
#   Define the Configurations
# --------------------------------------------------------------------- 

def configs_setup(vehicle):
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    configs                                                    = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config                                                = RCAIDE.Library.Components.Configs.Config(vehicle) 
    base_config.tag                                            = 'base'
    configs.append(base_config)
    
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------ 
    config                                                     = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                                 = 'cruise' 
    configs.append(config)
    
    
    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------ 
    config                                                     = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                                 = 'takeoff' 
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.V2_VS_ratio                                         = 1.21
    config.maximum_lift_coefficient                            = 2.
    
    configs.append(config)
    
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config                                                     = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                                 = 'landing' 
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.Vref_VS_ratio                                       = 1.23
    config.maximum_lift_coefficient                            = 2.
                                                               
    configs.append(config) 
     
    return configs 

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------
def plot_mission(results):
    
     # Plot Aircraft Stability 
    plot_longitudinal_stability(results)  
    
    plot_lateral_stability(results) 
    
    plot_flight_forces_and_moments(results)
    
    plot_flight_trajectory(results)
    
    plot_aerodynamic_coefficients(results)
      
    return
 
# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
 
 
    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------     
    segment     = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.base )    
    segment.altitude                                                            = 0. * Units.feet
    segment.air_speed                                                           = 100 * Units['knots'] 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                             = True    
    segment.flight_dynamics.force_z                                             = True  
    segment.flight_dynamics.force_y                                             = True
    segment.flight_dynamics.moment_x                                            = True
    segment.flight_dynamics.moment_y                                            = True
    segment.flight_dynamics.moment_z                                            = True
                
    # define flight controls              
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['ice_propeller']]    
    segment.assigned_control_variables.body_angle.active                        = True       
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']] 
    segment.assigned_control_variables.aileron_deflection.active                = True    
    segment.assigned_control_variables.aileron_deflection.assigned_surfaces     = [['aileron']] 
    segment.assigned_control_variables.rudder_deflection.active                 = True    
    segment.assigned_control_variables.rudder_deflection.assigned_surfaces      = [['rudder']] 
    segment.assigned_control_variables.bank_angle.active                        = True    

    
    mission.append_segment(segment)    
  
 
    return mission

def turn_mission_setup(analyses ):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment() 
    # base_segment.state.numerics.solver.type = 'root_finder' 
    
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    # Circular departure pattern 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "Departure_Pattern_Curve"   
    segment.analyses.extend( analyses.base )           
    #segment.state.numerics.solver.type = 'optimize'  
    segment.turn_radius = 320 
    segment.true_course = 0 * Units.degree     
    segment.turn_angle  = 90 * Units.degree
    segment.altitude    = 0. * Units.feet
    segment.air_speed   = 100 * Units['knots'] 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                             = True    
    segment.flight_dynamics.force_z                                             = True    
    segment.flight_dynamics.force_y                                             = True     
    segment.flight_dynamics.moment_y                                            = True 
    segment.flight_dynamics.moment_x                                            = True
    segment.flight_dynamics.moment_z                                            = True 

    # define flight controls              
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['ice_propeller']]  
    segment.assigned_control_variables.body_angle.active                        = True    
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']] 
    #segment.assigned_control_variables.elevator_deflection.initial_guess_values = [[-4.4 * Units.degree]]
    segment.assigned_control_variables.aileron_deflection.active                = True    
    segment.assigned_control_variables.aileron_deflection.assigned_surfaces     = [['aileron']] 
    segment.assigned_control_variables.rudder_deflection.active                 = True    
    segment.assigned_control_variables.rudder_deflection.assigned_surfaces      = [['rudder']] 
    segment.assigned_control_variables.bank_angle.active                        = True 
    
    mission.append_segment(segment)  
    
    return mission


def missions_setup(mission): 
 
    missions     = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions

def display_stability_derivatives(segment):
    """This function displays the stability derivatives of the aircraft."""

    # Get the stability derivatives
    stability_derivatives = segment.conditions.static_stability.derivatives

    # Display the stability derivatives
    print(f"CLift_alpha: {stability_derivatives.Clift_alpha[0,0]:.3f}")
    print(f"CY_beta: {stability_derivatives.CY_beta[0,0]:.3f}")
    print(f"CL_beta: {stability_derivatives.CL_beta[0,0]:.4f}")
    print(f"CM_alpha: {stability_derivatives.CM_alpha[0,0]:.3f}")
    print(f"CN_beta: {stability_derivatives.CN_beta[0,0]:.3f}")
    print(f"CL_p: {stability_derivatives.CL_p[0,0]:.5f}")
    print(f"CL_r: {stability_derivatives.CL_r[0,0]:.5f}")
    print(f"CM_q: {stability_derivatives.CM_q[0,0]:.5f}")
    print(f"CN_p: {stability_derivatives.CN_p[0,0]:.5f}")
    print(f"CN_r: {stability_derivatives.CN_r[0,0]:.5f}")
    print(f"CM_delta_e: {stability_derivatives.CM_delta_e[0,0]:.5f}")
    print(f"CL_delta_a: {stability_derivatives.CL_delta_a[0,0]:.5f}")
    print(f"CN_delta_a: {stability_derivatives.CN_delta_a[0,0]:.5f}")
    print(f"CN_delta_r: {stability_derivatives.CN_delta_r[0,0]:.5f}")
    return

if __name__ == '__main__': 
    main()    
    plt.show()
