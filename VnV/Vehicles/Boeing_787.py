# RESEARCH/Aircraft/Boeing_787.py
# 
# 
# Created:  Jan 2025, A. Molloy 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units, Data           
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan    import design_turbofan      
from RCAIDE.Library.Plots                                                 import *     
import RCAIDE.Framework.External_Interfaces.OpenVSP as openvsp

# python imports 
import numpy as np  
from copy import deepcopy
import matplotlib.pyplot as plt  
import os
import sys

def vehicle_setup(vehicle_name = 'Boeing_787-8', number_of_passengers = 248):

    # ------------------------------------------------------------------
    #  File Paths 
    # ------------------------------------------------------------------    
    ospath             = os.path.abspath(__file__)
    separator          = os.path.sep
    rel_path           = os.path.dirname(ospath)   + separator
    airfoil_file_path  = rel_path + separator + 'Airfoils_and_Polars'
    polar_file_path    = rel_path + separator + 'Airfoils_and_Polars' + separator + 'Polars' 
                
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------      
    vehicle = RCAIDE.Vehicle()
    
    # ################################################# Vehicle-level Properties #################################################   
    vehicle.tag = vehicle_name 
    vehicle.mass_properties.max_takeoff               = 227930
    vehicle.mass_properties.takeoff                   = 227930 
    vehicle.mass_properties.max_zero_fuel             = 161025.0 * Units.kilogram   
    vehicle.mass_properties.max_fuel                  = 101323 * Units.kilogram    
    vehicle.mass_properties.max_payload               = 44000
    vehicle.mass_properties.center_of_gravity         = [[27.0, 0, 0]]
    vehicle.flight_envelope.ultimate_load             = 3.5
    vehicle.flight_envelope.positive_limit_load       = 2.5  
    vehicle.flight_envelope.negative_limit_load       = 1
    vehicle.flight_envelope.design_mach_number        = 0.85  
    vehicle.flight_envelope.design_cruise_altitude    = 35000.0*Units.feet 
    vehicle.flight_envelope.design_range              = 7305.0 * Units.nmi
    vehicle.reference_area                            = 395.0 * Units['meters**2']    
    vehicle.number_of_passengers                      = number_of_passengers 
    vehicle.systems.control                           = "fully powered" 
    vehicle.systems.accessories                       = "long range"


    # ------------------------------------------------------------------
    #   Main Wing 
    # ------------------------------------------------------------------

    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing'
    wing.aspect_ratio                     = 8.61650
    wing.sweeps.quarter_chord             = 35 * Units.deg
    wing.thickness_to_chord               = 0.115
    wing.spans.projected                  = 60 * Units.meter
    wing.chords.root                      = 14.0 * Units.meter
    wing.chords.tip                       = 2.1 * Units.meter
    wing.taper                            = wing.chords.tip / wing.chords.root
    wing.chords.mean_aerodynamic          = 5.75 * Units.meter 
    wing.areas.reference                  = 392.27 * Units['meters**2']
    wing.areas.wetted                     = 825.0 * Units['meters**2']
    wing.twists.root                      = 0.0 * Units.degrees 
    wing.twists.tip                       = -3.0 * Units.degrees 
    wing.origin                           = [[16.59,0,-0.492]]
    wing.aerodynamic_center               = [0,0,0] 
    wing.vertical                         = False
    wing.dihedral                         = 9.0 * Units.degrees 
    wing.xz_plane_symmetric               = True 
    wing.high_lift                        = True 
    wing.dynamic_pressure_ratio           = 1.0
        
    # Wing Segments
    root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()  
    root_airfoil.coordinate_file          = airfoil_file_path + 'transonic_wing_root_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Root'
    segment.percent_span_location         = 0.0 
    segment.root_chord_percent            = 1.
    #segment.twist                         = 3.5 * Units.degrees
    segment.dihedral_outboard             = 8.0 * Units.degrees
    segment.sweeps.quarter_chord          = 27.57 * Units.degrees
    segment.thickness_to_chord            = .15
    segment.append_airfoil(root_airfoil)
    wing.append_segment(segment)

    yehudi_airfoil                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    yehudi_airfoil.coordinate_file        = airfoil_file_path + 'transonic_wing_inboard_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Yehudi'
    segment.percent_span_location         = 0.345 
    segment.root_chord_percent            = 0.53 
    segment.dihedral_outboard             = 7.0 * Units.degrees
    segment.sweeps.quarter_chord          = 31. * Units.degrees
    #segment.twist                         = 2.5 * Units.degrees
    segment.thickness_to_chord            = 0.125
    segment.append_airfoil(yehudi_airfoil)
    wing.append_segment(segment)

    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = airfoil_file_path + 'transonic_wing_outboard_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Tip'
    segment.percent_span_location         = 0.95 
    segment.root_chord_percent            = 0.155 
    segment.dihedral_outboard             = 12.0 * Units.degrees
    segment.sweeps.quarter_chord          = 42.0 * Units.degrees
    #segment.twist                         = 1.0 * Units.degrees
    segment.thickness_to_chord            = .1
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)

    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = airfoil_file_path + 'transonic_wing_tip_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Winglet'
    segment.percent_span_location         = 1.00
    #segment.twist                         = 0.0 * Units.degrees
    segment.root_chord_percent            = 0.093 
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0 * Units.degrees
    segment.thickness_to_chord            = .095
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)

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

    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing     = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 5.169
    wing.sweeps.quarter_chord    = 35.0 * Units.deg  
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.237
    wing.spans.projected         = 20.11482 * Units.meter
    wing.chords.root             = 6.22 * Units.meter
    wing.chords.tip              = 1.48 * Units.meter
    wing.chords.mean_aerodynamic = 3.853 * Units.meter
    wing.areas.reference         = 78.26 * Units['meters**2']
    wing.areas.exposed           = 66.52 * Units['meters**2']    # Exposed area of the horizontal tail
    wing.areas.wetted            = 136.69 * Units['meters**2']     # Wetted area of the horizontal tail
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees 
    wing.origin                  = [[46.961, 0, 1.639]]
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
    segment.dihedral_outboard      = 8.0 * Units.degrees
    segment.sweeps.quarter_chord   = 35.785  * Units.degrees 
    segment.thickness_to_chord     = 0.15
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.237               
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = 0.15
    wing.append_segment(segment) 

    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.09
    elevator.span_fraction_end     = 0.92
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.3
    wing.append_control_surface(elevator)
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer' 
    wing.aspect_ratio            = 1.801
    wing.sweeps.quarter_chord    = 40.0  * Units.deg   
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 0.1 
    wing.spans.projected         = 9.77 * Units.meter 
    wing.total_length            = wing.spans.projected  
    wing.chords.root             = 8.7 * Units.meter
    wing.chords.tip              = 0.91 * Units.meter
    wing.chords.mean_aerodynamic = 5.91 * Units.meter 
    wing.areas.reference         = 53.04 * Units['meters**2']
    wing.areas.wetted            = 111.384 * Units['meters**2'] 
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees 
    wing.origin                  = [[43.31, 0, 3.115]]
    wing.aerodynamic_center      = [0,0,0] 
    wing.vertical                = True
    wing.xz_plane_symmetric      = False
    wing.t_tail                  = False 
    wing.dynamic_pressure_ratio  = 1.0

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 6.97 * Units.degrees  
    segment.thickness_to_chord            = 0.15
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 0.107
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.031
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.quarter_chord          = 63.8274 * Units.degrees   
    segment.thickness_to_chord            = 0.15
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_2'
    segment.percent_span_location         = 0.180
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.815
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 41.250 * Units.degrees    
    segment.thickness_to_chord            = 0.15
    wing.append_segment(segment)


    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_3'
    segment.percent_span_location         = 0.94
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.298
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 69.09 * Units.degrees    
    segment.thickness_to_chord            = 0.15
    wing.append_segment(segment)


    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_4'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.105
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0    
    segment.thickness_to_chord            = 0.15
    wing.append_segment(segment)
    

    # control surfaces -------------------------------------------
    rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                   = 'rudder'
    rudder.span_fraction_start   = 0.1 
    rudder.span_fraction_end     = 0.95 
    rudder.deflection            = 0 
    rudder.chord_fraction        = 0.33  
    wing.append_control_surface(rudder)     

    # add to vehicle
    vehicle.append_component(wing)
    
    # ################################################# Fuselage ################################################################ 
        
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage()  
    fuselage.seats_abreast                      = 9
    fuselage.seat_pitch                         = 0.9     * Units.meter 
    fuselage.fineness.nose                      = 2.
    fuselage.fineness.tail                      = 3. 
    fuselage.lengths.nose                       = 8.48   * Units.meter
    fuselage.lengths.tail                       = 16.55   * Units.meter
    fuselage.lengths.total                      = 56.7 * Units.meter  
    fuselage.lengths.fore_space                 = 1.    * Units.meter
    fuselage.lengths.aft_space                  = 10.    * Units.meter
    fuselage.width                              = 5.9  * Units.meter
    fuselage.heights.maximum                    = 5.9  * Units.meter
    fuselage.effective_diameter                 = 5.9     * Units.meter
    fuselage.areas.side_projected               = fuselage.heights.maximum * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.wetted                       = np.pi*fuselage.width/2*(fuselage.width/2+ np.sqrt( fuselage.lengths.nose **2 +(fuselage.width/2)**2)) + \
                                                 np.pi*fuselage.width/2*(fuselage.width/2+ np.sqrt( fuselage.lengths.tail**2 +(fuselage.width/2)**2))+ \
                                                    np.pi * fuselage.width * ( fuselage.lengths.total - (fuselage.lengths.tail+ fuselage.lengths.nose)) * Units['meters**2'] 
    fuselage.areas.front_projected              = np.pi * (fuselage.width/2) **2   
    fuselage.differential_pressure              = 5.0e4 * Units.pascal
    fuselage.heights.at_quarter_length          = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_three_quarters_length   = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = fuselage.heights.maximum* Units.meter
    
    cabin                                              = RCAIDE.Library.Components.Fuselages.Cabins.Cabin() 
    first_class                                        = RCAIDE.Library.Components.Fuselages.Cabins.Classes.First() 
    first_class.number_of_seats_abrest                 = 6
    first_class.number_of_rows                         = 6
    first_class.galley_lavatory_percent_x_locations    = [0]       
    first_class.type_A_exit_percent_x_locations        = [0.01]
    first_class.number_of_seats                      = first_class.number_of_rows  * first_class.number_of_seats_abrest 
    cabin.append_cabin_class(first_class) 

    business_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business() 
    business_class.number_of_seats_abrest              = 8
    business_class.number_of_rows                      = 8  
    business_class.number_of_seats                     = business_class.number_of_rows  * business_class.number_of_seats_abrest 
    cabin.append_cabin_class(business_class) 
    
    economy_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 10
    economy_class.number_of_rows                      = 15
    economy_class.galley_lavatory_percent_x_locations = [1]      
    economy_class.emergency_exit_percent_x_locations  = [0.1,0.15] 
    economy_class.type_A_exit_percent_x_locations     = [0.99]
    economy_class.number_of_seats                     = economy_class.number_of_rows  * economy_class.number_of_seats_abrest 
    cabin.append_cabin_class(economy_class)
    
    fuselage.append_cabin(cabin)          

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = 0.00 
    segment.height                              = 0.000 
    segment.width                               = 0.000  
    fuselage.append_segment(segment)   
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.00410 
    segment.percent_z_location                  = 0.00141 
    segment.height                              = 0.938
    segment.width                               = 1.14
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.00820 
    segment.percent_z_location                  = 0.00256 
    segment.height                              =  1.4
    segment.width                               = 1.47959
    fuselage.append_segment(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.02029 
    segment.percent_z_location                  = 0.00520 
    segment.height                              = 2.36 
    segment.width                               = 2.32 
    fuselage.append_segment(segment)   

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.02715 	
    segment.percent_z_location                  = 0.00720 
    segment.height                              =  2.79
    segment.width                               = 2.73 
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.04244 
    segment.percent_z_location                  = 0.01194 
    segment.height                              = 3.75 
    segment.width                               = 3.41 
    fuselage.append_segment(segment)     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.05931 
    segment.percent_z_location                  = 0.01588 
    segment.height                              = 4.48
    segment.width                               = 4.06 
    fuselage.append_segment(segment)             
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.07737 
    segment.percent_z_location                  = 0.01844 
    segment.height                              = 5.0 
    segment.width                               = 4.598 
    fuselage.append_segment(segment)    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.09544 
    segment.percent_z_location                  = 0.02024 
    segment.height                              = 5.36 
    segment.width                               = 5.05102 
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'     
    segment.percent_x_location                  = 0.1236 
    segment.percent_z_location                  = 0.02180
    segment.height                              = 5.76
    segment.width                               = 5.5289
    fuselage.append_segment(segment)     
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_10'     
    segment.percent_x_location                  = 0.15177 
    segment.percent_z_location                  = 0.02305
    segment.height                              = 5.8890
    segment.width                               = 5.9183
    fuselage.append_segment(segment)   
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'     
    segment.percent_x_location                  = 0.71813 
    segment.percent_z_location                  = 0.02305
    segment.height                              = 5.88980
    segment.width                               = 5.91837
    fuselage.append_segment(segment)    
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_12'     
    segment.percent_x_location                  = 0.77176
    segment.percent_z_location                  = 0.02664
    segment.height                              = 5.41224
    segment.width                               = 5.50771
    fuselage.append_segment(segment)             
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_13'     
    segment.percent_x_location                  = 0.85221 
    segment.percent_z_location                  = 0.03074
    segment.height                              = 4.27768
    segment.width                               = 4.17826
    fuselage.append_segment(segment)               
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_14'     
    segment.percent_x_location                  = 0.93265
    segment.percent_z_location                  = 0.03432
    segment.height                              = 2.38776
    segment.width                               = 2.5
    fuselage.append_segment(segment)
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_15'     
    segment.percent_x_location                  = 0.99488 
    segment.percent_z_location                  = 0.03689
    segment.height                              = 0.74286
    segment.width                               = 1.22145
    fuselage.append_segment(segment)
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_16'     
    segment.percent_x_location                  = 1.0 
    segment.percent_z_location                  = 0.04098
    segment.height                              = 0.0
    segment.width                               = 0.0
    fuselage.append_segment(segment)

    # add to vehicle
    vehicle.append_component(fuselage)
    
    # ################################################# Landing Gear #############################################################   

    main_gear                                = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter                  = 50.0 *  Units.inches 
    main_gear.rim_diameter                   = 22   *  Units.inches 
    main_gear.tire_width                     = 20.0 *  Units.inches 
    main_gear.strut_length                   = 5.5  * Units.ft 
    main_gear.wheels                         = 8   
    main_gear.number_of_gear_types_in_tandem = 2
    main_gear.number_of_wheels_in_gear_type  = 2  
    main_gear.xz_plane_symmetric             = True
    vehicle.append_component(main_gear)  

    nose_gear                                = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                  = 40. *  Units.inches   
    nose_gear.rim_diameter                   = 16  *  Units.inches 
    nose_gear.tire_width                     = 16  *  Units.inches 
    nose_gear.strut_length                   = 9.0 * Units.ft 
    nose_gear.wheels                         = 2   
    nose_gear.number_of_gear_types_in_tandem = 1
    nose_gear.number_of_wheels_in_gear_type  = 2    
    vehicle.append_component(nose_gear)

    # ------------------------------------------------------------------
    # Carbo Bays 
    # ------------------------------------------------------------------ 
    cargo_bay = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
    cargo_bay.cargo.mass_properties.mass  = 4500 * Units.kilogram
    vehicle.cargo_bays.append(cargo_bay) 

    # ################################################# Energy Network #######################################################          
    #------------------------------------------------------------------------------------------------------------------------- 
    #  Turbofan Network
    #-------------------------------------------------------------------------------------------------------------------------   
    net                                         = RCAIDE.Framework.Networks.Fuel() 

    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                   = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Starboard Propulsor
    # Sources: https://www.researchgate.net/publication/320798360_Performance_Analysis_of_Cold_Sections_of_High_BYPASS_Ratio_Turbofan_Aeroengine/figures?lo=1

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Propulsor: Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan1                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()   
    turbofan1.origin                             = [[17.818, 10.000,-0.953 ]]
    turbofan1.tag                                = 'propulsor_1'    
    turbofan1.length                             = 4.928                      
    turbofan1.diameter                           = 2.822                  
    turbofan1.bypass_ratio                       = 9.1                        
    turbofan1.design_altitude                    = 36000*Units.ft             
    turbofan1.design_mach_number                 = 0.85                     
    turbofan1.design_thrust                      = 80000* Units.N
    
    # working fluid                   
    turbofan1.working_fluid                      = RCAIDE.Library.Attributes.Gases.Air() 
    
    # Ram inlet 
    ram                                          = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                      = 'ram' 
    turbofan1.ram                                = ram 
            
    # inlet nozzle          
    inlet_nozzle                                = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                            = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency          = 0.98                                      
    inlet_nozzle.pressure_ratio                 = 1
    inlet_nozzle.compressibility_effects        = False
    turbofan1.inlet_nozzle                       = inlet_nozzle
    
    # fan                
    fan                                         = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                     = 'fan'
    fan.polytropic_efficiency                   = 0.98                 
    fan.pressure_ratio                          = 1.4                    
    turbofan1.fan                                = fan        

    # low pressure compressor    
    low_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                   = 'lpc'
    low_pressure_compressor.polytropic_efficiency = 0.98                  
    low_pressure_compressor.pressure_ratio        = 1.3                     
    turbofan1.low_pressure_compressor              = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.98                        
    high_pressure_compressor.pressure_ratio        = 23.9                    
    turbofan1.high_pressure_compressor              = high_pressure_compressor

    # low pressure turbine  
    low_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                       ='lpt'
    low_pressure_turbine.mechanical_efficiency     = 0.99                     
    low_pressure_turbine.polytropic_efficiency     = 0.98                    
    turbofan1.low_pressure_turbine                  = low_pressure_turbine
    
    # high pressure turbine     
    high_pressure_turbine                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                      ='hpt'
    high_pressure_turbine.mechanical_efficiency    = 0.99                     
    high_pressure_turbine.polytropic_efficiency    = 0.98                     
    turbofan1.high_pressure_turbine                 = high_pressure_turbine 

    # combustor  
    combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                  = 'Comb'
    combustor.efficiency                           = 0.997                    
    combustor.turbine_inlet_temperature            = 1440                  
    combustor.pressure_ratio                       = 0.94                     
    combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    turbofan1.combustor                             = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.98                     
    core_nozzle.pressure_ratio                     = 0.995
    core_nozzle.diameter                           = 1.5  # may be incorrect 
    turbofan1.core_nozzle                          = core_nozzle
        
    # fan nozzle             
    fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                 = 'fan nozzle'
    fan_nozzle.polytropic_efficiency               = 0.98                     # CHECKED Ref. [2] Page 9
    fan_nozzle.pressure_ratio                      = 0.995 
    fan_nozzle.diameter                            = 2.822 # may be incorrect         
    turbofan1.fan_nozzle                           = fan_nozzle
    
    # design turbofan
    design_turbofan(turbofan1)

    # Nacelle 
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                            = 3.556
    nacelle.length                              = 4.9
    nacelle.tag                                 = 'nacelle_1'
    nacelle.inlet_diameter                      = 2.5
    nacelle.origin                              = [[17.818, 10.000,-0.953]] 
    nacelle.areas.wetted                        = np.pi*nacelle.diameter*nacelle.length
    nacelle_airfoil                             = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code          = '0010'
    nacelle.append_airfoil(nacelle_airfoil) 
    turbofan1.nacelle                            = nacelle
    
    net.propulsors.append(turbofan1)

        #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Propulsor 2 (Inner Port Side)
    #------------------------------------------------------------------------------------------------------------------------------------      
    # copy turbofan
    turbofan2                                  = deepcopy(turbofan1)
    turbofan2.active_fuel_tanks                = ['fuel_tank'] 
    turbofan2.tag                              = 'propulsor_2' 
    turbofan2.origin                           = [[17.818, -10.000,-0.953]]
    turbofan2.nacelle.origin                   = [[17.818, -10.000,-0.953]]
        
    # append propulsor to distribution line 
    net.propulsors.append(turbofan2)

    #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #------------------------------------------------------------------------------------------------------------------------- 
    # fuel tank
    fuel_tank_1                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing) 
    fuel_tank_1.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A1()    
    fuel_tank_1.segments_bounding_tank       = ['root', 'yehudi']  
    fuel_tank_1.segments_percent_chord_start = [0.1, 0.1]
    fuel_tank_1.segments_percent_chord_end   = [0.7, 0.7] 
    fuel_line.fuel_tanks.append(fuel_tank_1)
    
    fuel_tank_2                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing) 
    fuel_tank_2.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A1()    
    fuel_tank_2.segments_bounding_tank       = ['yehudi','tip']  
    fuel_tank_2.segments_percent_chord_start = [0.1, 0.1]
    fuel_tank_2.segments_percent_chord_end   = [0.7, 0.7] 
    fuel_line.fuel_tanks.append(fuel_tank_2)    
 

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network      
    fuel_line.assigned_propulsors     =  [['propulsor_1', 'propulsor_2']]

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network      
    net.fuel_lines.append(fuel_line)        
 
    # Append energy network to aircraft 
    vehicle.append_energy_network(net)   
    
    return vehicle 

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    """This function sets up vehicle configurations for use in different parts of the mission.
    Here, this is mostly in terms of high lift settings."""

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base' 
    configs.append(base_config)



    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    config = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag = 'idle' 
    configs.append(config) 
    
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    configs.append(config)
     
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------ 
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'descent' 
    config.wings['main_wing'].control_surfaces.spoiler.deflection  = 45. * Units.deg    
    configs.append(config) 

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'takeoff'    
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 30. * Units.deg 
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity      =  3470. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  3470. * Units.rpm 
    config.networks.fuel.propulsors['propulsor_1'].emission_indices.NOx      = 34.77 /1000      
    config.networks.fuel.propulsors['propulsor_2'].emission_indices.NOx      = 34.77 /1000       
    config.networks.fuel.propulsors['propulsor_1'].fan_nozzle.exit_velocity    = 315.
    config.networks.fuel.propulsors['propulsor_2'].fan_nozzle.exit_velocity    = 315.
    config.networks.fuel.propulsors['propulsor_1'].core_nozzle.exit_velocity   = 415.
    config.networks.fuel.propulsors['propulsor_2'].core_nozzle.exit_velocity   = 415. 
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True     
    
    config.V2_VS_ratio = 1.21
    configs.append(config)


    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 20. * Units.deg
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity    =  2780. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity    =  2780. * Units.rpm 
    config.networks.fuel.propulsors['propulsor_1'].emission_indices.NOx    = 20.87 /1000      
    config.networks.fuel.propulsors['propulsor_2'].emission_indices.NOx    = 20.87 /1000   
    config.networks.fuel.propulsors['propulsor_1'].fan_nozzle.exit_velocity  = 210.
    config.networks.fuel.propulsors['propulsor_2'].fan_nozzle.exit_velocity  = 210.
    config.networks.fuel.propulsors['propulsor_1'].core_nozzle.exit_velocity = 360.
    config.networks.fuel.propulsors['propulsor_2'].core_nozzle.exit_velocity = 360.

    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True         
    configs.append(config)   



    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity   =  2780. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity   =  2780. * Units.rpm
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    config.networks.fuel.propulsors['propulsor_1'].emission_indices.NOx    = 11.02/1000              
    config.networks.fuel.propulsors['propulsor_2'].emission_indices.NOx    = 11.02 /1000   
    config.networks.fuel.propulsors['propulsor_1'].core_nozzle.exit_velocity = 92.
    config.networks.fuel.propulsors['propulsor_2'].core_nozzle.exit_velocity = 92.
    config.networks.fuel.propulsors['propulsor_1'].fan_nozzle.exit_velocity  = 109.3
    config.networks.fuel.propulsors['propulsor_2'].fan_nozzle.exit_velocity  = 109.3 

    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True         
    config.Vref_VS_ratio = 1.23
    configs.append(config)   

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'    
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  3470. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  3470. * Units.rpm 
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    config.V2_VS_ratio = 1.21 
    configs.append(config)

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------  

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'reverse_thrust'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg 
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True     
    configs.append(config)
    
    return configs