# RESEARCH/Aircraft/Elysian_E9X_Model.py
# 
# Created:  Jan 2025, A. Molloy and S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units       
from RCAIDE.Library.Methods.Geometry.Planform               import segment_properties    
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan   import design_turbofan    
from RCAIDE.Library.Plots                                   import *     
 
# python imports 
import numpy as np  
from copy import deepcopy 
import os

def vehicle_setup(): 
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    
    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Elysian_E9X'

    
    # ################################################# Vehicle-level Properties #################################################   
    vehicle.mass_properties.max_takeoff               = 76000 * Units.kilogram  
    vehicle.mass_properties.takeoff                   = 76000 * Units.kilogram    
    vehicle.mass_properties.operating_empty           = 75000 * Units.kilogram  
    vehicle.mass_properties.max_zero_fuel             = 75000 * Units.kilogram 
    vehicle.mass_properties.cargo                     = 9000.  * Units.kilogram  
    vehicle.mass_properties.center_of_gravity         = [[15.75,0, 0, 0]] # CHANGE
    vehicle.flight_envelope.ultimate_load             = 3.75
    vehicle.flight_envelope.positive_limit_load       = 2.5 
    vehicle.flight_envelope.design_mach_number        = 0.45 
    vehicle.flight_envelope.design_cruise_altitude    = 7500
    vehicle.flight_envelope.design_range              = 800 * Units.km
    vehicle.reference_area                            = 150 * Units['meters**2']   
    vehicle.number_of_passengers                      = 90
    vehicle.systems.control                           = "fully powered" 
    vehicle.systems.accessories                       = "medium range"

    # ################################################# Landing Gear #############################################################   
    # ------------------------------------------------------------------        
    #  Landing Gear
    # ------------------------------------------------------------------  
    main_gear                            = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.tire_diameter              = 1.12000 * Units.m
    main_gear.strut_length               = 1.5 * Units.m 
    main_gear.units                      = 2    
    main_gear.wheels                     = 2    
    vehicle.append_component(main_gear)  

    nose_gear                            = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()       
    nose_gear.tire_diameter              = 1.2 * Units.m
    nose_gear.units                      = 1    
    nose_gear.wheels                     = 2    
    nose_gear.strut_length               = 1.3 * Units.m 
    vehicle.append_component(nose_gear)
     

    # ################################################# Wings ##################################################################### 
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
 
    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing' 
    wing.aspect_ratio                     = 11.62
    wing.sweeps.quarter_chord             = 1.92 * Units.deg
    wing.thickness_to_chord               = 0.1
    wing.taper                            = 0.5
    wing.spans.projected                  = 42.0 
    wing.chords.root                      = 5.0 * Units.meter
    wing.chords.tip                       = 2.5 * Units.meter
    wing.chords.mean_aerodynamic          = 3.75 * Units.meter 
    wing.areas.reference                  = 150.0
    wing.areas.wetted                     = 315.0 
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[13.61,0,-0.652]]
    wing.aerodynamic_center               = [0,0,0] 
    wing.vertical                         = False
    wing.xz_plane_symmetric               = True
    wing.high_lift                        = True 
    wing.dynamic_pressure_ratio           = 1.0


    # Wing Segments
    root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    rel_path                              = os.path.dirname(ospath) + separator   
    root_airfoil.coordinate_file          = 'transonic_wing_root_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 4.0 * Units.degrees
    segment.sweeps.quarter_chord          = 1.93 * Units.degrees
    segment.thickness_to_chord            = .1
    segment.append_airfoil(root_airfoil)
    wing.append_segment(segment)

    yehudi_airfoil                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    yehudi_airfoil.coordinate_file        = 'transonic_wing_inboard_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Main_Tip'
    segment.percent_span_location         = 0.9
    segment.twist                         = 0 * Units.deg
    segment.root_chord_percent            = 0.5
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 8.0 * Units.degrees
    segment.sweeps.quarter_chord          = 17.4 * Units.degrees
    segment.append_airfoil(yehudi_airfoil)
    wing.append_segment(segment)

    mid_airfoil                           = RCAIDE.Library.Components.Airfoils.Airfoil()
    mid_airfoil.coordinate_file           = 'transonic_wing_outboard_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Winglet_1'
    segment.percent_span_location         = 0.928
    segment.twist                         = 0.00 * Units.deg
    segment.root_chord_percent            = 0.47
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 10.0 * Units.degrees
    segment.sweeps.quarter_chord          = 37.66 * Units.degrees
    segment.thickness_to_chord            = .1
    segment.append_airfoil(mid_airfoil)
    wing.append_segment(segment)

    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = 'transonic_wing_tip_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Winglet_2'
    segment.percent_span_location         = 0.952
    segment.twist                         = 0. * Units.degrees
    segment.root_chord_percent            = 0.39
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 16.
    segment.sweeps.quarter_chord          = 52.1
    segment.thickness_to_chord            = .1
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)
    
    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = 'transonic_wing_tip_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Winglet_2'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.degrees
    segment.root_chord_percent            = 0.08
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 0.
    segment.sweeps.quarter_chord          = 0.0
    segment.thickness_to_chord            = .1
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)    
    
    

    # control surfaces -------------------------------------------

    flap                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                      = 'flap'
    flap.span_fraction_start      = 0.2
    flap.span_fraction_end        = 0.7
    flap.deflection               = 0.0 * Units.degrees
    flap.configuration_type       = 'double_slotted'
    flap.chord_fraction           = 0.30
    wing.append_control_surface(flap)

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.7
    aileron.span_fraction_end     = 0.963
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.16
    wing.append_control_surface(aileron)

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing  = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 4.63
    wing.sweeps.quarter_chord    = 12.0 * Units.deg  
    wing.thickness_to_chord      = 0.1
    wing.taper                   = 0.5  
    wing.spans.projected         = 10.48 
    wing.chords.root             = 3.0 
    wing.chords.tip              = 1.5 
    wing.chords.mean_aerodynamic = 2.25 
    wing.areas.reference         = 23.58
    wing.areas.exposed           = 48.00    
    wing.areas.wetted            = 48.00     
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees 
    wing.origin                  = [[29.92,0,5.328]]
    wing.aerodynamic_center      = [0,0,0] 
    wing.vertical                = False
    wing.xz_plane_symmetric      = True 
    wing.dynamic_pressure_ratio  = 0.9


    # Wing Segments
    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0.0
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.dihedral_outboard      = 0.0 * Units.degrees
    segment.sweeps.quarter_chord   = 12.0  * Units.degrees 
    segment.thickness_to_chord     = .1
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.5              
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = .1
    wing.append_segment(segment)
    
        

    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.09
    elevator.span_fraction_end     = 0.92
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.3
    wing.append_control_surface(elevator)

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing                         = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag                     = 'vertical_stabilizer'

    wing.aspect_ratio            = 1.64
    wing.sweeps.quarter_chord    = 41.5  * Units.deg   
    wing.thickness_to_chord      = 0.1
    wing.taper                   = 0.5

    wing.spans.projected         = 3.2
    wing.total_length            = wing.spans.projected 
    
    wing.chords.root             = 6.0 
    wing.chords.tip              = 3.0 
    wing.chords.mean_aerodynamic = 4.5

    wing.areas.reference         = 12.5
    wing.areas.wetted            = 26.25 
    
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[25.328,0,2.131]]
    wing.aerodynamic_center      = [0,0,0]

    wing.vertical                = True
    wing.xz_plane_symmetric      = False
    wing.t_tail                  = True

    wing.dynamic_pressure_ratio  = 1.0


    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 61.485 * Units.degrees  
    segment.thickness_to_chord            = .1
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 0.2962
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.45
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.quarter_chord          = 31.2 * Units.degrees   
    segment.thickness_to_chord            = .1
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_2'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.1183 
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0    
    segment.thickness_to_chord            = .1  
    wing.append_segment(segment)
    
    
        

    # add to vehicle
    vehicle.append_component(wing)

    # ################################################# Fuselage ################################################################ 
    
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage() 
    fuselage.number_coach_seats                 = vehicle.number_of_passengers 
    fuselage.seats_abreast                      = 5
    fuselage.seat_pitch                         = 1     * Units.meter 
    fuselage.fineness.nose                      = 2.0
    fuselage.fineness.tail                      = 3.6 
    fuselage.lengths.nose                       = 6.0   * Units.meter
    fuselage.lengths.tail                       = 11.0   * Units.meter
    fuselage.lengths.total                      = 33.0 * Units.meter  
    fuselage.lengths.fore_space                 = 1.    * Units.meter
    fuselage.lengths.aft_space                  = 5.    * Units.meter
    fuselage.width                              = 3.0  * Units.meter
    fuselage.heights.maximum                    = 3.0  * Units.meter
    fuselage.effective_diameter                 = 3.0     * Units.meter
    fuselage.areas.side_projected               = 90.0 * Units['meters**2'] 
    fuselage.areas.wetted                       = 282.0  * Units['meters**2'] 
    fuselage.areas.front_projected              = 9.0    * Units['meters**2']  
    fuselage.differential_pressure              = 5.0e4 * Units.pascal 
    fuselage.heights.at_quarter_length          = 3.0 * Units.meter
    fuselage.heights.at_three_quarters_length   = 3.0 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 3.0 * Units.meter

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = -0.00128
    fuselage.append_segment(segment)   
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.00295 
    segment.percent_z_location                  = 0.00102 
    segment.height                              = 0.571
    segment.width                               = 0.245
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.0077 
    segment.percent_z_location                  = 0.00179
    segment.height                              = 0.68
    segment.width                               = 0.90
    fuselage.append_segment(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.02 
    segment.percent_z_location                  = 0.00459 
    segment.height                              = 1.3 
    segment.width                               = 1.3 
    fuselage.append_segment(segment)   

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.04278 	
    segment.percent_z_location                  = 0.01012 
    segment.height                              = 1.88 
    segment.width                               = 1.88 
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.06834 
    segment.percent_z_location                  = 0.01336 
    segment.height                              = 2.34 
    segment.width                               = 2.34 
    fuselage.append_segment(segment)     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.11857 
    segment.percent_z_location                  = 0.01878 
    segment.height                              = 2.77 
    segment.width                               = 2.77 
    fuselage.append_segment(segment)             
     
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.1911 
    segment.percent_z_location                  = 0.02174 
    segment.height                              = 3.0 
    segment.width                               = 3.00 
    fuselage.append_segment(segment)    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Circle_Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.67 
    segment.percent_z_location                  = 0.02170 
    segment.height                              = 3.0 
    segment.width                               = 3.0 
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Circle_Segment()
    segment.tag                                 = 'segment_9'     
    segment.percent_x_location                  = 0.73 
    segment.percent_z_location                  = 0.02472
    segment.height                              = 2.77
    segment.width                               = 2.77
    fuselage.append_segment(segment)     
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Circle_Segment()
    segment.tag                                 = 'segment_10'     
    segment.percent_x_location                  = 0.89582 
    segment.percent_z_location                  = 0.04072
    segment.height                              = 1.75
    segment.width                               = 1.75
    fuselage.append_segment(segment)   
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'     
    segment.percent_x_location                  = 0.94896 
    segment.percent_z_location                  = 0.04758
    segment.height                              = 1.2
    segment.width                               = 1.2
    fuselage.append_segment(segment)    
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_12'     
    segment.percent_x_location                  = 0.98463
    segment.percent_z_location                  = 0.05364
    segment.height                              = 0.5
    segment.width                               = 0.5
    fuselage.append_segment(segment)             
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_13'     
    segment.percent_x_location                  = 1.0
    segment.percent_z_location                  = 0.05568
    segment.height                              = 0.0
    segment.width                               = 0.0
    fuselage.append_segment(segment)
    
    # add to vehicle
    vehicle.append_component(fuselage)
     

    # ################################################# Energy Network #######################################################          
    
    
    # TO BE ADDED
        
    #------------------------------------------------------------------------------------------------------------------------- 
    # Done ! 
    #------------------------------------------------------------------------------------------------------------------------- 
      
    return vehicle
 
# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs                                  = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config                              = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag                          = 'base' 
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config                           = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                       = 'cruise'
    configs.append(config) 

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config                                                                        = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                                                    = 'takeoff'
    config.wings['main_wing'].control_surfaces.flap.deflection                    = 20. * Units.deg
    # config.networks.fuel.propulsors['starboard_propulsor'].fan.angular_velocity =  3470. * Units.rpm
    # config.networks.fuel.propulsors['port_propulsor'].fan.angular_velocity      =  3470. * Units.rpm 
    config.V2_VS_ratio                                                            = 1.21
    configs.append(config)

    
    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config                                                                        = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                                                    = 'cutback'
    config.wings['main_wing'].control_surfaces.flap.deflection                    = 20. * Units.deg
    # config.networks.fuel.propulsors['starboard_propulsor'].fan.angular_velocity =  2780. * Units.rpm
    # config.networks.fuel.propulsors['port_propulsor'].fan.angular_velocity      =  2780. * Units.rpm 
    configs.append(config)   
    
        
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config                                                                        = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                                                    = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection                    = 30. * Units.deg
    # config.networks.fuel.propulsors['starboard_propulsor'].fan.angular_velocity =  2030. * Units.rpm
    # config.networks.fuel.propulsors['port_propulsor'].fan.angular_velocity      =  2030. * Units.rpm
    config.landing_gears.main_gear.gear_extended                                  = True
    config.landing_gears.nose_gear.gear_extended                                  = True  
    config.Vref_VS_ratio                                                          = 1.23
    configs.append(config)   
     
    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config                                                                       = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                                                   = 'short_field_takeoff'    
    config.wings['main_wing'].control_surfaces.flap.deflection                   = 20. * Units.deg
    #config.networks.fuel.propulsors['starboard_propulsor'].fan.angular_velocity =  3470. * Units.rpm
    #config.networks.fuel.propulsors['port_propulsor'].fan.angular_velocity      =  3470. * Units.rpm 
    config.landing_gears.main_gear.gear_extended                                 = True
    config.landing_gears.nose_gear.gear_extended                                 = True  
    config.V2_VS_ratio                                                           = 1.21 
    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------  

    config                                                      = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                                  = 'reverse_thrust'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.landing_gears.main_gear.gear_extended                = True
    config.landing_gears.nose_gear.gear_extended                = True  
    configs.append(config)    
    

    return configs  
