''' 
  Embraer_175.py
  
  Created: September 2025, M Carter 

'''

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units      
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan             import design_turbofan 
from RCAIDE.Library.Plots                                              import *    
from RCAIDE.Library.Methods.Performance                                import *  
from RCAIDE.Framework.External_Interfaces.OpenVSP                      import export_vsp_vehicle
from RCAIDE.Library.Methods.Performance.compute_payload_range_diagram  import compute_payload_range_diagram 

# python imports 
import numpy as np  
import os
import matplotlib.pyplot as plt 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(plot_results=True, plot_vehicle=True): 
     
    vehicle                  = vehicle_setup()  

    configs                  = configs_setup(vehicle) 
    analyses                 = analyses_setup(configs) 
    mission                  = mission_setup(analyses)
    missions                 = missions_setup(mission)
    results                  = missions.base_mission.evaluate()  
    plot_mission(results) 

    # run payload range analysis 
    compute_payload_range_diagram(mission = missions.base_mission, fuel_reserve_percentage = 0.05)  
    
    return 

def vehicle_setup():
    
    # ------------------------------------------------------------------------------------------------------------------
    #  Vehicle-level Properties 
    # ------------------------------------------------------------------------------------------------------------------
    vehicle     = RCAIDE.Vehicle()
    vehicle.tag = 'Embraer_E175_Baseline'
 
    # mass properties (http://www.embraercommercialaviation.com/AircraftPDF/E190_Weights.pdf)
    vehicle.mass_properties.max_takeoff               = 37500 # kg
    vehicle.mass_properties.takeoff                   = 37500. # kg
    vehicle.mass_properties.max_zero_fuel             = 31700. # kg
    vehicle.mass_properties.max_fuel                  = 9428. # kg
    vehicle.mass_properties.max_payload               = 10200. # kg
    vehicle.mass_properties.operating_empty           = 21700  #  kg
    vehicle.mass_properties.center_of_gravity         = [[13.76, 0, 0]]
    vehicle.mass_properties.moments_of_inertia.tensor = [[10 ** 5, 0, 0],[0, 10 ** 6, 0,],[0,0, 10 ** 7]] 

    # envelope properties
    vehicle.flight_envelope.ultimate_load             = 3.5 
    vehicle.flight_envelope.ultimate_load             = 3.75
    vehicle.flight_envelope.positive_limit_load       = 2.5 
    vehicle.flight_envelope.design_mach_number        = 0.78 
    vehicle.flight_envelope.design_cruise_altitude    = 41000 *Units.feet
    vehicle.flight_envelope.design_range              = 2000 * Units.nmi
    
    # basic parameters
    vehicle.reference_area                            = 92.
    vehicle.number_of_passengers                      = 84
    vehicle.systems.control                           = "fully powered"
    vehicle.systems.accessories                       = "medium range"

    # ------------------------------------------------------------------        
    #  Landing Gear
    # ------------------------------------------------------------------  
    main_gear                                = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter                  = 44.5 *  Units.inches 
    main_gear.rim_diameter                   = 21   *  Units.inches 
    main_gear.tire_width                     = 16.5  *  Units.inches 
    main_gear.strut_length                   = 1.8  * Units.m  
    main_gear.wheels                         = 4   
    main_gear.number_of_gear_types_in_tandem = 1
    main_gear.number_of_wheels_in_gear_type  = 2  
    main_gear.xz_plane_symmetric             = True
    vehicle.append_component(main_gear)  

    nose_gear                                = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                  = 27    *  Units.inches   
    nose_gear.rim_diameter                   = 15    *  Units.inches 
    nose_gear.tire_width                     = 7.75  *  Units.inches 
    nose_gear.strut_length                   = 1.8   * Units.m  
    nose_gear.wheels                         = 2   
    nose_gear.number_of_gear_types_in_tandem = 1
    nose_gear.number_of_wheels_in_gear_type  = 2    
    vehicle.append_component(nose_gear)
    
    # ----------------------------------------------------------------------------------------------------------------
    #  Wings
    # ----------------------------------------------------------------------------------------------------------------

    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
    wing                         = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                     = 'main_wing'
    wing.areas.reference         = 86.08416
    wing.aspect_ratio            = 8.83336
    wing.chords.root             = 5.82440
    wing.chords.tip              = 1.25967
    wing.sweeps.leading_edge     = 26.28571 * Units.deg
    wing.thickness_to_chord      = 0.1
    wing.taper                   = 0.40187
    wing.dihedral                = 7.00 * Units.deg
    wing.spans.projected         = 27.57557 
    wing.origin                  = [[10.882,0,-0.20]]
    wing.vertical                = False
    wing.xz_plane_symmetric      = True       
    wing.high_lift               = True
    wing.areas.exposed           = 150.93 * wing.areas.wetted        
    wing.twists.root             = 0 * Units.degrees
    wing.twists.tip              = -1.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 1.0
    
    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    rel_path                              = os.path.dirname(ospath) + separator + '..' + separator + '..' + separator + 'Aircraft' + separator 
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0 
    segment.root_chord_percent            = 1.
    segment.thickness_to_chord            = .11
    segment.dihedral_outboard             = 7 * Units.degrees
    segment.sweeps.quarter_chord          = 20.6 * Units.degrees  
    root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    root_airfoil.coordinate_file          = 'transonic_wing_root_section_airfoil.txt'
    segment.append_airfoil(root_airfoil)
    wing.segments.append(segment)    
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'yehudi'
    segment.percent_span_location         = (2*5.23445)/27.57557
    segment.root_chord_percent            = 3.134/5.8244 
    segment.dihedral_outboard             = 7 * Units.degrees
    segment.sweeps.quarter_chord          = 24.1 * Units.degrees 
    yehudi_airfoil                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    yehudi_airfoil.coordinate_file        = 'transonic_wing_inboard_section_airfoil.txt'
    segment.append_airfoil(yehudi_airfoil)
    wing.segments.append(segment)

    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                          = 'section_2'
    segment.percent_span_location        = 0.961
    segment.root_chord_percent           = 0.25 
    segment.dihedral_outboard            = 75. * Units.degrees
    segment.sweeps.quarter_chord         = 26.28571 * Units.degrees 
    mid_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    mid_airfoil.coordinate_file          = 'transonic_wing_outboard_section_airfoil.txt'
    segment.append_airfoil(mid_airfoil)
    wing.segments.append(segment)

    segment = RCAIDE.Library.Components.Wings.Segments.Segment() 
    segment.tag                          = 'Tip'
    segment.percent_span_location        = 1.
    segment.root_chord_percent           = 0.070 
    segment.dihedral_outboard            = 0.
    segment.sweeps.quarter_chord         = 0.  
    tip_airfoil                          =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file          = 'transonic_wing_tip_section_airfoil.txt'
    segment.append_airfoil(tip_airfoil)
    wing.segments.append(segment) 

    # control surfaces -------------------------------------------
    slat                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Slat()
    slat.tag                      = 'slat'
    slat.span_fraction_start      = 0.2
    slat.span_fraction_end        = 0.963
    slat.deflection               = 0.0 * Units.degrees
    slat.chord_fraction           = 0.075
    wing.append_control_surface(slat)

    flap                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                      = 'flap'
    flap.span_fraction_start      = 0.2
    flap.span_fraction_end        = 0.7
    flap.deflection               = 0.0 * Units.degrees
    flap.configuration_type       = 'double_slotted'
    flap.chord_fraction           = 0.14
    wing.append_control_surface(flap)

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.7
    aileron.span_fraction_end     = 0.963
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.25
    wing.append_control_surface(aileron)
        
    spoiler                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Spoiler()
    spoiler.tag                   = 'spoiler'
    spoiler.span_fraction_start   = 0.3
    spoiler.span_fraction_end     = 0.7
    spoiler.deflection            = 0.0 * Units.degrees
    spoiler.chord_fraction        = 0.05
    wing.append_control_surface(spoiler)        
     
    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'
    wing.areas.reference         = 24.96840
    wing.chords.root             = 3.59833
    wing.chords.tip              = 1.27922
    wing.aspect_ratio            = 4.19805
    wing.spans.projected         = 10.23810
    wing.sweeps.quarter_chord    = 35.85714 * Units.deg
    wing.thickness_to_chord      = 0.10
    wing.taper                   = 0.35550
    wing.dihedral                = 0 * Units.degrees
    wing.origin                  = [[26,0,1.900]]
    wing.vertical                = False
    wing.xz_plane_symmetric      = True       
    wing.high_lift               = False   
    wing.areas.exposed           = 41.66 * wing.areas.wetted 
    wing.twists.root             = 0 * Units.degrees
    wing.twists.tip              = 0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 0.90

    # add to vehicle
    vehicle.append_component(wing)     

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing                         = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag                     = 'vertical_stabilizer'
    wing.areas.reference         = 19.19825
    wing.aspect_ratio            = 1.64893
    wing.spans.projected         = 5.62641
    wing.sweeps.quarter_chord    = 40. * Units.deg
    wing.chords.root             = 7.72923
    wing.chords.tip              = 4.18795
    wing.thickness_to_chord      = 0.100
    wing.taper                   = 0.54183
    wing.dihedral                = 0.00
    wing.origin                  = [[21.900,0,2.365]]
    wing.vertical                = True
    wing.xz_plane_symmetric      = False       
    wing.high_lift               = False 
    wing.areas.exposed           = 0.2 * wing.areas.wetted
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 1.00

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tail_root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.leading_edge           = 75  * Units.degrees  
    segment.thickness_to_chord            =.15
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tail_segment_1'
    segment.percent_span_location         = 1.02641/5.62641
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 4.18795/7.72923
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.leading_edge           = 40* Units.degrees   
    segment.thickness_to_chord            = .13
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tail_segment_2'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.50000/7.72923
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0    
    segment.thickness_to_chord            = .12 
    wing.append_segment(segment) 

    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage                                          = RCAIDE.Library.Components.Fuselages.Fuselage() 
    fuselage.origin                                   = [[0,0,0]] 

    cabin                                              = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.origin                                       = [[3, 0, 0.5]]
    first_class                                        = RCAIDE.Library.Components.Fuselages.Cabins.Classes.First() 
    first_class.number_of_seats_abrest                 = 3
    first_class.number_of_rows                         = 2 
    first_class.aisle_width                            = 16 *  Units.inches    
    first_class.galley_lavatory_percent_x_locations    = [0]       
    first_class.type_A_exit_percent_x_locations        = [0.2]
    cabin.append_cabin_class(first_class)  
    
    economy_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 4
    economy_class.number_of_rows                      = 18
    economy_class.aisle_width                         = 14 *  Units.inches   
    economy_class.galley_lavatory_percent_x_locations = [1]      
    economy_class.emergency_exit_percent_x_locations  = [0.1,0.15] 
    economy_class.type_A_exit_percent_x_locations     = [0.99]
    cabin.append_cabin_class(economy_class) 
    fuselage.append_cabin(cabin) 
 
    fuselage.fineness.nose                      = 1.579
    fuselage.fineness.tail                      = 3.48 
    fuselage.lengths.nose                       = 4.75200
    fuselage.lengths.tail                       = 9.0
    fuselage.lengths.cabin                      = 20.02176
    fuselage.lengths.total                      = 31.68 
    fuselage.width                              = 3.01 * Units.meters 
    fuselage.heights.maximum                    = 3.35    
    fuselage.heights.at_quarter_length          = 3.35 
    fuselage.heights.at_three_quarters_length   = 3.35 
    fuselage.heights.at_wing_root_quarter_chord = 3.35  
    fuselage.areas.side_projected               = 239.20
    fuselage.areas.wetted                       = 251.14
    fuselage.areas.front_projected              = np.pi * (fuselage.heights.maximum  / 2) ** 2
    fuselage.effective_diameter                 = 3.18 
    fuselage.differential_pressure              = 10**5 * Units.pascal      
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = 0.0000
    segment.height                              = 0.0000 
    segment.width                               = 0.0000  
    fuselage.segments.append(segment)   
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.01475 
    segment.percent_z_location                  = 0.00334
    segment.height                              = 0.86425
    segment.width                               = 1.22135
    fuselage.segments.append(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.04967
    segment.percent_z_location                  = 0.01412 
    segment.height                              = 2.01612
    segment.width                               = 2.12408
    fuselage.segments.append(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.07476
    segment.percent_z_location                  = 0.02052
    segment.height                              = 2.58071
    segment.width                               = 2.49580
    fuselage.segments.append(segment)   

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.09984	
    segment.percent_z_location                  = 0.02590 
    segment.height                              = 2.99694 
    segment.width                               = 2.76131 
    fuselage.segments.append(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.15000
    segment.percent_z_location                  = 0.03000 
    segment.height                              = 3.35000
    segment.width                               = 3.01000
    fuselage.segments.append(segment)     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.63200
    segment.percent_z_location                  = 0.03000
    segment.height                              = 3.35000
    segment.width                               = 3.01000 
    fuselage.segments.append(segment)             
     
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.74511
    segment.percent_z_location                  = 0.03512
    segment.height                              = 2.99694
    segment.width                               = 2.70820
    fuselage.segments.append(segment)    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.96553 
    segment.percent_z_location                  = 0.05587 
    segment.height                              = 1.08980
    segment.width                               = 0.84963
    fuselage.segments.append(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'     
    segment.percent_x_location                  = 1.000
    segment.percent_z_location                  = 0.0600
    segment.height                              = 0.7
    segment.width                               = 0.4
    fuselage.segments.append(segment)     
      
    # add to vehicle
    vehicle.append_component(fuselage) 
   
   
    
    return vehicle

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs                                        = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config                                    = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag                                = 'base'  
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config                                         = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                     = 'cruise'
    configs.append(config)

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config                                                                                = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                                                            = 'takeoff'
    config.wings['main_wing'].control_surfaces.flap.deflection                            = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection                            = 25. * Units.deg 
    # config.networks.network.converters['starboard_fan'].fan.angular_velocity            =  3470. * Units.rpm
    # config.networks.network.propulsors['port_propulsor'].fan.angular_velocity           =  3470. * Units.rpm 
    # config.networks.network.propulsors['starboard_propulsor'].core_nozzle.exit_velocity =  315.
    # config.networks.network.propulsors['port_propulsor'].core_nozzle.exit_velocity      =  315.
    # config.networks.network.propulsors['starboard_propulsor'].fan_nozzle.exit_velocity  =  415.
    # config.networks.network.propulsors['port_propulsor'].fan_nozzle.exit_velocity       =  415. 
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True 
    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config                                                                                = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                                                            = 'cutback'
    config.wings['main_wing'].control_surfaces.flap.deflection                            = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection                            = 20. * Units.deg
    # config.networks.network.propulsors['starboard_propulsor'].fan.angular_velocity      =  2780. * Units.rpm
    # config.networks.network.propulsors['port_propulsor'].fan.angular_velocity           =  2780. * Units.rpm 
    # config.networks.network.propulsors['starboard_propulsor'].core_nozzle.exit_velocity =  210.
    # config.networks.network.propulsors['port_propulsor'].core_nozzle.exit_velocity      =  210.
    # config.networks.network.propulsors['starboard_propulsor'].fan_nozzle.exit_velocity  =  360.
    # config.networks.network.propulsors['port_propulsor'].fan_nozzle.exit_velocity       =  360. 
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True 
    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Descent Configuration
    # ------------------------------------------------------------------ 

    config                                                         = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                                     = 'descent' 
    config.wings['main_wing'].control_surfaces.spoiler.deflection  = 45. * Units.deg    
    configs.append(config)  
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config                                                                                = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                                                            = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection                            = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection                            = 25. * Units.deg
    # config.networks.network.propulsors['starboard_propulsor'].fan.angular_velocity      =  2030. * Units.rpm
    # config.networks.network.propulsors['port_propulsor'].fan.angular_velocity           =  2030. * Units.rpm
    # config.networks.network.propulsors['starboard_propulsor'].core_nozzle.exit_velocity = 92.
    # config.networks.network.propulsors['port_propulsor'].core_nozzle.exit_velocity      = 92.
    # config.networks.network.propulsors['starboard_propulsor'].fan_nozzle.exit_velocity  = 109.3
    # config.networks.network.propulsors['port_propulsor'].fan_nozzle.exit_velocity       = 109.3 
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True 
    configs.append(config)   
     
    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 
     
    config                                                                           = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                                                       = 'short_field_takeoff'    
    config.wings['main_wing'].control_surfaces.flap.deflection                       = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection                       = 25. * Units.deg
    # config.networks.network.propulsors['starboard_propulsor'].fan.angular_velocity =  3470. * Units.rpm
    # config.networks.network.propulsors['port_propulsor'].fan.angular_velocity      =  3470. * Units.rpm
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True 
    configs.append(config)    

    return configs

def analyses_setup(configs):


    analyses                             = RCAIDE.Framework.Analyses.Analysis.Container()

    for tag,config in configs.items():
        analysis                         = base_analysis(config)
        analyses[tag]                    = analysis

    return analyses

def base_analysis(vehicle):

   
    analyses                                                       = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle                                               = vehicle

    #  Geometry
    geometry                                                       = RCAIDE.Framework.Analyses.Geometry.Geometry() 
    analyses.append(geometry)
 
    #  Weights
    weights                                                        = RCAIDE.Framework.Analyses.Weights.Conventional_Transport() 
    weights.settings.FLOPS.fidelity                                = 'Complex'      
    weights.settings.advanced_composites                           = True
    analyses.append(weights)

    #  Aerodynamics Analysis
    aerodynamics                                                   = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.store_training_data                      = True
    aerodynamics.settings.number_of_spanwise_vortices              = 15
    aerodynamics.settings.number_of_chordwise_vortices             = 2 
    analyses.append(aerodynamics)
 
    #  Energy
    energy                        = RCAIDE.Framework.Analyses.Energy.Energy()
    analyses.append(energy)

    #  Planet Analysis
    planet                        = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    #  Atmosphere Analysis
    atmosphere                    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    return analyses    
    
def mission_setup(analyses):


    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission                                                         = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag                                                     = 'the_mission'
  
    Segments                                                        = RCAIDE.Framework.Mission.Segments 
    base_segment                                                    = Segments.Segment()
    base_segment.state.numerics.solver.type                         = 'root_finder'

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment                                                         = Segments.Ground.Takeoff(base_segment)
    segment.tag                                                     = "Takeoff" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start                                          = 10.* Units.knots
    segment.velocity_end                                            = 125.0 * Units['m/s']
    segment.friction_coefficient                                    = 0.04
    segment.altitude                                                = 0.0  
    segment.hybrid_power_split_ratio                                = 0.05 
    segment.battery_fuel_cell_power_split_ratio                     = 1.0
    segment.initial_battery_state_of_charge                         = 1.0
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment                                                         = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                                     = "climb_1" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_start                                          = 0.0   * Units.km
    segment.altitude_end                                            = 3.0   * Units.km
    segment.air_speed                                               = 125.0 * Units['m/s']
    segment.climb_rate                                              = 6.0   * Units['m/s']  
    segment.hybrid_power_split_ratio                                = 0.001
    segment.battery_fuel_cell_power_split_ratio                     = 1.0
     
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                 = True  
    segment.flight_dynamics.force_z                                 = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                 
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment                                                         = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                                     = "climb_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                            = 8.0   * Units.km
    segment.air_speed                                               = 190.0 * Units['m/s']
    segment.climb_rate                                              = 6.0   * Units['m/s']  
    segment.hybrid_power_split_ratio                                = 0.001
    segment.battery_fuel_cell_power_split_ratio                     = 1.0
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                 = True  
    segment.flight_dynamics.force_z                                 = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                  
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment                                                          = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                                      = "climb_3" 
    segment.analyses.extend( analyses.cruise )  
    segment.altitude_end                                             = 10.5   * Units.km
    segment.air_speed                                                = 226.0  * Units['m/s']
    segment.climb_rate                                               = 3.0    * Units['m/s']  
    segment.hybrid_power_split_ratio                                 = 0.001
    segment.battery_fuel_cell_power_split_ratio                      = 1.0
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment                                                         = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                                     = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                = 10.668 * Units.km  
    segment.air_speed                                               = 230.412 * Units['m/s']
    segment.distance                                                = 1000 * Units.nmi  
    segment.hybrid_power_split_ratio                                = 0.05
    segment.battery_fuel_cell_power_split_ratio                     = 1.0
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                 = True  
    segment.flight_dynamics.force_z                                 = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment                                                          = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                                      = "descent_1" 
    segment.analyses.extend( analyses.descent ) 
    segment.altitude_start                                           = 10.5 * Units.km 
    segment.altitude_end                                             = 6.0    * Units.km
    segment.air_speed                                                = 220.0 * Units['m/s']
    segment.descent_rate                                             = 4.5   * Units['m/s']  
    segment.hybrid_power_split_ratio                                 = 0.001
    segment.battery_fuel_cell_power_split_ratio                      = 1.0
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment                                                          = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                                      = "descent_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                             = 7.0   * Units.km
    segment.air_speed                                                = 210.0 * Units['m/s']
    segment.descent_rate                                             = 5.08   * Units['m/s'] 
    segment.hybrid_power_split_ratio                                 = 0.05
    segment.battery_fuel_cell_power_split_ratio                      = 1.0
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment                                                          = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                                      = "descent_3"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 4.0   * Units.km
    segment.air_speed                                                = 170.0 * Units['m/s']
    segment.descent_rate                                             = 5.0   * Units['m/s'] 
    segment.hybrid_power_split_ratio                                 = 0.001
    segment.battery_fuel_cell_power_split_ratio                      = 1.0 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Fourth Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment                                                          = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                                      = "descent_4" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 2.0   * Units.km
    segment.air_speed                                                = 150.0 * Units['m/s']
    segment.descent_rate                                             = 5.0   * Units['m/s']  
    segment.hybrid_power_split_ratio                                 = 0.001
    segment.battery_fuel_cell_power_split_ratio                      = 1.0
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Fifth Descent Segment:Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment                                                          = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag                                                      = "descent_5" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 0.0   * Units.km
    segment.air_speed                                                = 145.0 * Units['m/s']
    segment.descent_rate                                             = 3.0   * Units['m/s'] 
    segment.hybrid_power_split_ratio                                 = 0.001
    segment.battery_fuel_cell_power_split_ratio                      = 1.0
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Landing Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment                                                                = Segments.Ground.Landing(base_segment)
    segment.tag                                                            = "Landing"

    segment.analyses.extend( analyses.landing ) 
    segment.velocity_end                                                   = 10 * Units.knots 
    segment.friction_coefficient                                           = 0.4
    segment.altitude                                                       = 0.0   
    segment.assigned_control_variables.elapsed_time.active                 = True  
    segment.assigned_control_variables.elapsed_time.initial_guess_values   = [[30.]]  
    segment.hybrid_power_split_ratio                                       = 0.05
    segment.battery_fuel_cell_power_split_ratio                            = 1.0
    mission.append_segment(segment)     

    return mission

def missions_setup(mission):

    missions                           = RCAIDE.Framework.Mission.Missions() 
    mission.tag                        = 'base_mission'
    missions.append(mission)

    return missions

def plot_mission(results):

    plot_altitude_sfc_weight(results)
    
    
    return

if __name__ == '__main__': 
    main()
    plt.show()
    
