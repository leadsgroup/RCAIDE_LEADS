# ATR_72.py

# Created: 2025, M. Clarke, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from   RCAIDE.Framework.Core import Units
from   RCAIDE.Library.Plots  import *  
from   RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop  import design_turboprop    
from RCAIDE.Library.Methods.Performance import *   

# python imports 
import numpy as np  
from   copy import deepcopy
import matplotlib.pyplot as plt
import os
import sys

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(): 

    # Step 1: design a vehicle
    vehicle  = vehicle_setup()   
      
    # Step 2: create aircraft configuration based on vehicle 
    configs  = configs_setup(vehicle)
    
    # Step 3: set up analysis
    analyses = analyses_setup(configs)
    
    # Step 4: set up a flight mission
    mission  = mission_setup(analyses)
    missions = missions_setup(mission) 
    
    # Step 5: execute flight profile
    results = missions.base_mission.evaluate()  
    
    # Step 6: plot results 
    plot_mission(results)   
         
    return  
# ----------------------------------------------------------------------
#   Define Vehicle
# --------------------------------------------------------------------- 
def vehicle_setup():

    # ------------------------------------------------------------------
    #  File Paths 
    # ------------------------------------------------------------------    
    ospath             = os.path.abspath(__file__)
    separator          = os.path.sep
    rel_path           = os.path.dirname(ospath)   + separator
    airfoil_file_path  = rel_path + separator + 'Airfoils'
    polar_file_path    = rel_path + separator + 'Airfoils' + separator + 'Polars'
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'ATR_72' 
    vehicle.mass_properties.max_takeoff               = 23000 
    vehicle.mass_properties.takeoff                   = 23000  
    vehicle.mass_properties.operating_empty           = 13010
    vehicle.mass_properties.max_zero_fuel             = 20000 
    vehicle.mass_properties.max_payload               = 7100 
    vehicle.mass_properties.center_of_gravity         = [[13.00381052,0,1.026]]  
    vehicle.mass_properties.max_fuel                  = 5450   
    # envelope properties
    vehicle.flight_envelope.design_mach_number        = 0.43 
    vehicle.flight_envelope.design_range              = 890 * Units.nmi
    vehicle.flight_envelope.design_cruise_altitude    = 25000 * Units.feet
    vehicle.flight_envelope.ultimate_load             = 3.75
    vehicle.flight_envelope.positive_limit_load       = 1.5
    vehicle.flight_envelope.design_dynamic_pressure   = 5e4
              
    # basic parameters              
    vehicle.reference_area                            = 61.0  
    vehicle.number_of_passengers                      = 72
    vehicle.systems.control                           = "fully powered"
    vehicle.systems.accessories                       = "short range"  


    # ################################################# Landing Gear #############################################################    
    
    main_gear                                = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter                  = 34  *  Units.inches 
    main_gear.rim_diameter                   = 16  *  Units.inches 
    main_gear.tire_width                     = 10  *  Units.inches 
    main_gear.strut_length                   = 1 *  Units.meter 
    main_gear.wheels                         = 4   
    main_gear.number_of_gear_types_in_tandem = 1
    main_gear.number_of_wheels_in_gear_type  = 2  
    main_gear.xz_plane_symmetric             = True
    vehicle.append_component(main_gear)  

    nose_gear                                 = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                   = 17   *  Units.inches   
    nose_gear.rim_diameter                    = 7    *  Units.inches 
    nose_gear.tire_width                      = 17   *  Units.inches 
    nose_gear.strut_length                    = 1 *  Units.meter 
    nose_gear.wheels                          = 2   
    nose_gear.number_of_gear_types_in_tandem  = 1
    nose_gear.number_of_wheels_in_gear_type   = 2    
    vehicle.append_component(nose_gear)
 
    # ################################################# Wings #############################################################   
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing'
    wing.areas.reference                  = 61.0  
    wing.spans.projected                  = 27.12
    wing.aspect_ratio                     = (wing.spans.projected**2) /  wing.areas.reference
    wing.sweeps.quarter_chord             = 0.0 
    wing.thickness_to_chord               = 0.1 
    wing.chords.root                      = 2.7 
    wing.chords.tip                       = 1.35 
    wing.total_length                     = wing.chords.root  
    wing.taper                            = wing.chords.tip/wing.chords.root 
    wing.chords.mean_aerodynamic          = wing.chords.root * 2/3 * (( 1 + wing.taper + wing.taper**2 )/( 1 + wing.taper )) 
    wing.areas.exposed                    = 2 * wing.areas.reference
    wing.areas.wetted                     = 2 * wing.areas.reference  
    wing.origin                           = [[11.52756129,0,2.009316366]]  
    wing.aerodynamic_center               = [11.52756129 + 0.25*wing.chords.root ,0,2.009316366]  
    wing.vertical                         = False   
    wing.xz_plane_symmetric               = True    
    wing.dynamic_pressure_ratio           = 1.0 

    # Wing Segments   
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0.0 * Units.deg
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.0  * Units.degrees
    segment.sweeps.quarter_chord          = 0.0 * Units.degrees
    segment.thickness_to_chord            = .15 
    wing.append_segment(segment)
  
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.324
    segment.twist                         = 0.0 * Units.deg
    segment.root_chord_percent            = 1.0
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = 4.7 * Units.degrees
    segment.thickness_to_chord            = .13 
    wing.append_segment(segment) 
 
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 0. * Units.degrees
    segment.root_chord_percent            = wing.taper 
    segment.thickness_to_chord            = 0.12
    segment.dihedral_outboard             = 0.
    segment.sweeps.quarter_chord          = 0. 
    wing.append_segment(segment)   

    flap                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                      = 'flap'
    flap.span_fraction_start      = 0.1
    flap.span_fraction_end        = 0.727
    flap.deflection               = 0.0 * Units.degrees
    flap.configuration_type       = 'double_slotted'
    flap.chord_fraction           = 0.25
    wing.append_control_surface(flap)

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.727
    aileron.span_fraction_end     = 1
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.4
    wing.append_control_surface(aileron) 
    
    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------ 
    wing                         = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag                     = 'horizontal_stabilizer'  
    wing.spans.projected         = 3.61*2 
    wing.areas.reference         = 15.2 
    wing.aspect_ratio            = (wing.spans.projected**2) /  wing.areas.reference
    wing.sweeps.leading_edge     = 11.56*Units.degrees  
    wing.thickness_to_chord      = 0.12  
    wing.chords.root             = 2.078645129 
    wing.chords.tip              = 0.953457347 
    wing.total_length            = wing.chords.root  
    wing.taper                   = wing.chords.tip/wing.chords.root  
    wing.chords.mean_aerodynamic = wing.chords.root * 2/3 * (( 1 + wing.taper + wing.taper**2 )/( 1 + wing.taper ))
    wing.twists.root             = 0 * Units.degrees  
    wing.twists.tip              = 0 * Units.degrees   
    wing.origin                  = [[25.505088,0,5.510942426]]  
    wing.aerodynamic_center      = [25.505088+ 0.25*wing.chords.root,0,2.009316366] 
    wing.vertical                = False  
    wing.xz_plane_symmetric      = True  
    wing.dynamic_pressure_ratio  = 0.95 

    # Wing Segments
    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0.0
    segment.twist                  = 4. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.sweeps.leading_edge    = wing.sweeps.leading_edge 
    segment.thickness_to_chord     = .12
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 4. * Units.deg
    segment.root_chord_percent     = wing.taper          
    segment.thickness_to_chord     = .12
    wing.append_segment(segment)      

    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.1
    elevator.span_fraction_end     = 0.9
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.3
    wing.append_control_surface(elevator)

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------ 
    wing                                   = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag                               = 'vertical_stabilizer'   
    wing.spans.projected                   = 4.5  
    wing.areas.reference                   = 12.7
    wing.sweeps.quarter_chord              = 54 * Units.degrees  
    wing.thickness_to_chord                = 0.1  
    wing.aspect_ratio                      = (wing.spans.projected**2) /  wing.areas.reference    
    wing.chords.root                       = 8.75
    wing.chords.tip                        = 1.738510759 
    wing.total_length                      = wing.chords.root  
    wing.taper                             = wing.chords.tip/wing.chords.root  
    wing.chords.mean_aerodynamic           = wing.chords.root * 2/3 * (( 1 + wing.taper + wing.taper**2 )/( 1 + wing.taper )) 
    wing.areas.exposed                     = 2 * wing.areas.reference
    wing.areas.wetted                      = 2 * wing.areas.reference 
    wing.twists.root                       = 0 * Units.degrees  
    wing.twists.tip                        = 0 * Units.degrees   
    wing.origin                            = [[17.34807199,0,1.3]]  
    wing.aerodynamic_center                = [17.34807199,0,1.3+ 0.25*wing.chords.root]   
    wing.vertical                          = True  
    wing.xz_plane_symmetric                = False  
    wing.t_tail                            = True  
    wing.dynamic_pressure_ratio            = 1.0  
 
    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0.0
    segment.root_chord_percent            = 1.0
    segment.dihedral_outboard             = 0.0
    segment.sweeps.leading_edge           = 75 * Units.degrees  
    segment.thickness_to_chord            = 0.10
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_2'
    segment.percent_span_location         = 1.331360381/wing.spans.projected
    segment.twist                         = 0.0
    segment.root_chord_percent            = 4.25/wing.chords.root  
    segment.dihedral_outboard             = 0   
    segment.sweeps.leading_edge           = 54 * Units.degrees   
    segment.thickness_to_chord            = 0.15
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_3'
    segment.percent_span_location         = 3.058629978/wing.spans.projected
    segment.twist                         = 0.0
    segment.root_chord_percent            = 2.35/wing.chords.root    
    segment.dihedral_outboard             = 0 
    segment.sweeps.leading_edge           = 31 * Units.degrees   
    segment.thickness_to_chord            = 0.13
    wing.append_segment(segment)
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_4'
    segment.percent_span_location         = 4.380739035/wing.spans.projected
    segment.twist                         = 0.0
    segment.root_chord_percent            = 2.190082795/wing.chords.root  
    segment.dihedral_outboard             = 0
    segment.sweeps.leading_edge           = 52 * Units.degrees   
    segment.thickness_to_chord            = 0.13
    wing.append_segment(segment)    
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_5'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0.0
    segment.root_chord_percent            = 1.3/wing.chords.root  
    segment.dihedral_outboard             = 0  
    segment.sweeps.leading_edge           = 0 * Units.degrees   
    segment.thickness_to_chord            = 0.13
    wing.append_segment(segment)   

    # control surfaces -------------------------------------------
    rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                   = 'rudder'
    rudder.span_fraction_start   = 0.09
    rudder.span_fraction_end     = 1
    rudder.deflection            = 0.0  * Units.deg
    rudder.chord_fraction        = 0.3
    wing.append_control_surface(rudder)
    
    # add to vehicle
    vehicle.append_component(wing) 

    ## ########################################################## Landing Gear Pods  ################################################################           
    landing_gear_pod                                    = RCAIDE.Library.Components.Fuselages.Fuselage()
    landing_gear_pod.tag                                = 'landing_gear_pod' 
    landing_gear_pod.origin                             = [[ 10, 0,  -0.082]]    
    landing_gear_pod.lengths.total                      = 6 
    landing_gear_pod.width                              = 3.5  
    landing_gear_pod.heights.maximum                    = 1.30 
    landing_gear_pod.heights.at_quarter_length          = 1.05    
    landing_gear_pod.heights.at_three_quarters_length   = 1.05
    landing_gear_pod.effective_diameter                 = 3.5
    landing_gear_pod.areas.wetted                       = 8.6715
    landing_gear_pod.areas.front_projected              = np.pi *( 1.30 / 2)*( 3.5   / 2) 
    landing_gear_pod.differential_pressure              = 0.   
    
    # Segment  
    segment                           = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                       = 'segment_1'   
    segment.percent_x_location        = 0
    segment.percent_z_location        = 0 
    segment.height                    = 0.01  
    segment.width                     = 2.35
    landing_gear_pod.append_segment(segment)           
    
    # Segment                                   
    segment                           = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                       = 'segment_2'   
    segment.percent_x_location        = 0.2
    segment.percent_z_location        = -0.25 / landing_gear_pod.lengths.total 
    segment.height                    = 1.05   
    segment.width                     = 3.15   
    landing_gear_pod.append_segment(segment)

    # Segment                                   
    segment                           = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                       = 'segment_3'   
    segment.percent_x_location        = 0.5
    segment.percent_z_location        = -0.25 /landing_gear_pod.lengths.total   
    segment.height                    = 1.30 
    segment.width                     = 3.5 
    landing_gear_pod.append_segment(segment)


    # Segment                                   
    segment                           = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                       = 'segment_4'   
    segment.percent_x_location        = 0.8
    segment.percent_z_location        =  -0.25 /landing_gear_pod.lengths.total
    segment.height                    = 1.05 
    segment.width                     = 3.15 
    landing_gear_pod.append_segment(segment)     
    
    # Segment                                   
    segment                           = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                       = 'segment_5'    
    segment.percent_x_location        = 1
    segment.percent_z_location        = 0 
    segment.height                    = 0.01  
    segment.width                     = 2.35 
    landing_gear_pod.append_segment(segment) 
     
    # add to vehicle
    vehicle.append_component(landing_gear_pod)   
     

    # ########################################################## Fuselage  ################################################################   
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------ 
    fuselage = RCAIDE.Library.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage' 
    fuselage.seats_abreast                      = 4 
    fuselage.seat_pitch                         = 18  
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2. 
    fuselage.lengths.total                      = 27.12   
    fuselage.lengths.nose                       = 3.375147531 
    fuselage.lengths.tail                       = 9.2 
    fuselage.effective_diameter                 = 2.985093814  
    fuselage.lengths.cabin                      = fuselage.lengths.total- (fuselage.lengths.nose + fuselage.lengths.tail  )
    fuselage.width                              = 2.985093814  
    fuselage.heights.maximum                    = 2.755708426  
    fuselage.areas.side_projected               = fuselage.heights.maximum * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.wetted                       = np.pi * fuselage.width   * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.front_projected              = np.pi * (fuselage.width/2) **2   
    fuselage.differential_pressure              = 5.0e4 * Units.pascal
    fuselage.heights.at_quarter_length          = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_three_quarters_length   = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = fuselage.heights.maximum* Units.meter

    # define cabin    
    cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.origin                                      = [[2,0,0]] 
    economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 4
    economy_class.number_of_rows                      = 18
    economy_class.galley_lavatory_percent_x_locations = [0, 9]  
    economy_class.emergency_exit_percent_x_locations  = []      
    economy_class.type_A_exit_percent_x_locations     = [0.01,1] 
    economy_class.number_of_seats                     = economy_class.number_of_rows  * economy_class.number_of_seats_abrest 
    cabin.append_cabin_class(economy_class)
    fuselage.append_cabin(cabin)
    
     # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = 0.0000
    segment.height                              = 0.0
    segment.width                               = 0.0  
    fuselage.append_segment(segment)   
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_2'    
    segment.percent_x_location                  = 0.08732056/fuselage.lengths.total  
    segment.percent_z_location                  = 0.0000
    segment.height                              = 0.459245202 
    segment.width                               = 0.401839552 
    fuselage.append_segment(segment)   
  
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_3'    
    segment.percent_x_location                  = 0.197094977/fuselage.lengths.total  
    segment.percent_z_location                  = 0.001
    segment.height                              = 0.688749197
    segment.width                               = 0.918490404  
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_4'    
    segment.percent_x_location                  = 0.41997031/fuselage.lengths.total 
    segment.percent_z_location                  = 0.0000 
    segment.height                              = 0.975896055   
    segment.width                               = 1.320329956 
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_5'    
    segment.percent_x_location                  = 0.753451685/fuselage.lengths.total
    segment.percent_z_location                  = 0.0014551442477876075 # this is given as a percentage of the fuselage length i.e. location of the center of the cross section/fuselage length
    segment.height                              = 1.320329956 
    segment.width                               = 1.664763858 
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_6'    
    segment.percent_x_location                  = 1.14389933/fuselage.lengths.total
    segment.percent_z_location                  = 0.0036330994100294946
    segment.height                              = 1.607358208   
    segment.width                               = 2.009316366 
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_7'    
    segment.percent_x_location                  = 1.585491874/fuselage.lengths.total
    segment.percent_z_location                  = 0.008262262758112099
    segment.height                              = 2.18141471 
    segment.width                               = 2.411155918 
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_8'    
    segment.percent_x_location                  = 2.031242539/fuselage.lengths.total
    segment.percent_z_location                  = 0.013612882669616513
    segment.height                              = 2.468442962  
    segment.width                               = 2.698065563  
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_9'    
    segment.percent_x_location                  = 2.59009412/fuselage.lengths.total
    segment.percent_z_location                  = 0.01636321766224188
    segment.height                              = 2.640659912   
    segment.width                               = 2.812876863 
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_10'    
    segment.percent_x_location                  = 3.375147531/fuselage.lengths.total
    segment.percent_z_location                  = 0.01860240047935103
    segment.height                              = 2.755708426
    segment.width                               = 2.985093814 
    fuselage.append_segment(segment)   

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_11'    
    segment.percent_x_location                  = 17.01420312/fuselage.lengths.total 
    segment.percent_z_location                  = 0.01860240047935103
    segment.height                              = 2.755708426
    segment.width                               = 2.985093814 
    fuselage.append_segment(segment)   
 
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_12'    
    segment.percent_x_location                  = 18.64210783/fuselage.lengths.total
    segment.percent_z_location                  = 0.01860240047935103
    segment.height                              = 2.698302776 
    segment.width                               = 2.927925377  
    fuselage.append_segment(segment)    
     
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_13'    
    segment.percent_x_location                  = 22.7416002/fuselage.lengths.total 
    segment.percent_z_location                  = 0.043363795685840714
    segment.height                              = 1.779575158  
    segment.width                               = 1.722050901 
    fuselage.append_segment(segment)     
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_14'    
    segment.percent_x_location                  = 1.
    segment.percent_z_location                  = 0.06630560070058995
    segment.height                              = 0.401839552 
    segment.width                               = 0.401839552  
    fuselage.append_segment(segment) 
    
    # add to vehicle
    vehicle.append_component(fuselage)


    # ########################################################  Energy Network  #########################################################  
    net                                              = RCAIDE.Framework.Networks.Fuel()    

    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                       = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  
 
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------    
    starboard_propulsor                              = RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop()        
    starboard_propulsor.tag                          = 'starboard_propulsor'  
    starboard_propulsor.origin                       = [[ 9.559106394 ,4.219315295, 1.616135105]]
    starboard_propulsor.working_fluid                = RCAIDE.Library.Attributes.Gases.Air()               
    starboard_propulsor.gearbox.efficiency           = 0.99                                             
    starboard_propulsor.design_thrust                = 15000.0 * Units.N 
    starboard_propulsor.design_altitude              = 25000*Units.ft                                
    starboard_propulsor.design_freestream_velocity   = 270 * Units.kts  

    #Propeller Design              
    propeller                                        = RCAIDE.Library.Components.Powertrain.Converters.Propeller()   
    propeller.tag                                    = 'starboard_propulsor_propeller' 
    propeller.origin                                 = [[9.1,4.219315295, 1.616135105 ]]
    propeller.active                                 = True          
    propeller.tip_radius                             = 2.8/2
    propeller.hub_radius                             = 0.1 
    propeller.number_of_blades                       = 3   
    propeller.design_efficiency                      = 0.83      
    propeller.design_angular_velocity                = 3000.0 * Units.rpm        
    propeller.design_thrust                          = starboard_propulsor.design_thrust              
    propeller.design_altitude                        = starboard_propulsor.design_altitude                                        
    propeller.design_freestream_velocity             = starboard_propulsor.design_freestream_velocity                                                               
    starboard_propulsor.propeller                    = propeller     
    
    # Ram inlet 
    ram                                              = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                          = 'ram' 
    starboard_propulsor.ram                          = ram 
          
    # inlet nozzle          
    inlet_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                                 = 'inlet nozzle'                                       
    inlet_nozzle.pressure_ratio                      = 0.98
    inlet_nozzle.compressibility_effects             = False
    starboard_propulsor.inlet_nozzle                 = inlet_nozzle
                                                     
    # compressor                        
    compressor                                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    compressor.tag                                   = 'lpc'                   
    compressor.pressure_ratio                        = 10                   
    starboard_propulsor.compressor                   = compressor
    
    # combustor      
    combustor                                        = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                    = 'Comb'
    combustor.efficiency                             = 0.99                   
    combustor.turbine_inlet_temperature              = 1370                    
    combustor.pressure_ratio                         = 0.96                    
    combustor.fuel_data                              = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    starboard_propulsor.combustor                    = combustor
        
    # high pressure turbine         
    high_pressure_turbine                            = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                        ='hpt'
    high_pressure_turbine.mechanical_efficiency      = 0.99                       
    starboard_propulsor.high_pressure_turbine        = high_pressure_turbine 
        
    # low pressure turbine      
    low_pressure_turbine                             = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                         ='lpt'
    low_pressure_turbine.mechanical_efficiency       = 0.99                      
    starboard_propulsor.low_pressure_turbine         = low_pressure_turbine
    
    # core nozzle    
    core_nozzle                                      = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                  = 'core nozzle'          
    core_nozzle.pressure_ratio                       = 0.99
    starboard_propulsor.core_nozzle                  = core_nozzle
    
    # design starboard_propulsor
    design_turboprop(starboard_propulsor)
    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    #   Nacelles
    #------------------------------------------------------------------------------------------------------------------------------------ 
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    nacelle.tag                                 = 'nacelle_1'
    nacelle.length                              = 5
    nacelle.diameter                            = 0.85 
    nacelle.areas.wetted                        = 1.0   
    nacelle.origin                              = [[8.941625295,4.219315295, 1.616135105 ]]
    nacelle.flow_through                        = False     

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_1'
    nac_segment.percent_x_location              = 0.0 
    nac_segment.height                          = 0.0
    nac_segment.width                           = 0.0
    nacelle.append_segment(nac_segment)   

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_2'
    nac_segment.percent_x_location              = 0.2 /  nacelle.length
    nac_segment.percent_z_location              = 0 
    nac_segment.height                          = 0.4 
    nac_segment.width                           = 0.4  
    nacelle.append_segment(nac_segment)   

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_3'
    nac_segment.percent_x_location              = 0.6 /  nacelle.length
    nac_segment.percent_z_location              = 0 
    nac_segment.height                          = 0.52 
    nac_segment.width                           = 0.700 
    nac_segment.curvature                       = 3  
    nacelle.append_segment(nac_segment)  

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_4'
    nac_segment.percent_x_location              = 0.754 /  nacelle.length
    nac_segment.percent_z_location              = -0.15 /  nacelle.length
    nac_segment.height                          = 0.9	 
    nac_segment.width                           = 0.85 
    nac_segment.curvature                       = 3  
    nacelle.append_segment(nac_segment)  

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_5'
    nac_segment.percent_x_location              = 1.154  /  nacelle.length
    nac_segment.percent_z_location              = -0.1/  nacelle.length
    nac_segment.height                          = 1 
    nac_segment.width                           = 0.85 
    nac_segment.curvature                       = 4  
    nacelle.append_segment(nac_segment)   

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_6'
    nac_segment.percent_x_location              = 3.414   / nacelle.length
    nac_segment.percent_z_location              = -0.1 /  nacelle.length 
    nac_segment.height                          = 0.9 
    nac_segment.width                           = 0.85 
    nac_segment.curvature                       = 4  
    nacelle.append_segment(nac_segment)

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_6'
    nac_segment.percent_x_location              = 0.96 
    nac_segment.percent_z_location              = 0.05/  nacelle.length 
    nac_segment.height                          = 0.6
    nac_segment.width                           = 0.5
    nac_segment.curvature                       = 4  
    nacelle.append_segment(nac_segment)    

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_7'
    nac_segment.percent_x_location              = 1.0 
    nac_segment.percent_z_location              = 0.15/  nacelle.length  	
    nac_segment.height                          = 0.4
    nac_segment.width                           = 0.2
    nac_segment.curvature                       = 4  
    nacelle.append_segment(nac_segment) 

    starboard_propulsor.nacelle =  nacelle
    
    net.propulsors.append(starboard_propulsor)    

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------      
    port_propulsor                                  = deepcopy(starboard_propulsor) 
    port_propulsor.tag                              = 'port_propulsor' 
    port_propulsor.origin                           = [[ 9.559106394 ,-4.219315295, 1.616135105]]  # change origin 
    port_propulsor.nacelle.tag                      = 'port_propulsor_nacelle' 
    port_propulsor.nacelle.origin                   = [[8.941625295,-4.219315295, 1.616135105 ]]
    port_propulsor.propeller.tag                    = 'port_propulsor_propeller' 
    port_propulsor.propeller.origin                 = [[9.1,-4.219315295, 1.616135105 ]]
         
    # append propulsor to distribution line 
    net.propulsors.append(port_propulsor) 

    #------------------------------------------------------------------------------------------------------------------------- 
    # Energy Source: Fuel Tank
    #-------------------------------------------------------------------------------------------------------------------------  
    inboard_tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    inboard_tank.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A()
    inboard_tank.segments_bounding_tank       = ['root','inboard']   
    fuel_line.fuel_tanks.append(inboard_tank)
    
    outboard_tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    outboard_tank.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A()
    outboard_tank.segments_bounding_tank       = ['inboard', 'tip']  
    fuel_line.fuel_tanks.append(outboard_tank)    

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line    
    fuel_line.assigned_propulsors =  [[starboard_propulsor.tag, port_propulsor.tag]]
    
    # Append fuel line to Network      
    net.fuel_lines.append(fuel_line)   

    # Append energy network to aircraft 
    vehicle.append_energy_network(net)     

    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------
def configs_setup(vehicle):

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs         = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config     = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'  
    configs.append(base_config)    
    return configs

# ----------------------------------------------------------------------
#   Define the Analyses
# ----------------------------------------------------------------------
def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

# ----------------------------------------------------------------------
#   Define the Base Analysis
# ----------------------------------------------------------------------
def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle 
    analyses.vehicle = vehicle

    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.overwrite_reference        = True
    geometry.settings.update_wing_properties     = True 
    analyses.append(geometry)


    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------     
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_Transport() 
    weights.settings.update_moment_of_inertia = True 
    weights.method                                                       = "Raymer" 
    weights.settings.weight_correction_factors.empty.systems.apu         = 0.0
    weights.settings.weight_correction_factors.empty.systems.furnishings = 0.5
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    # ------------------------------------------------------------------     
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    analyses.append(aerodynamics) 

    # ------------------------------------------------------------------
    #  Stability Analysis
    # ------------------------------------------------------------------     
    stability                                = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method() 
    stability.settings.compute_neutral_point = True 
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Energy
    # ------------------------------------------------------------------     
    energy= RCAIDE.Framework.Analyses.Energy.Energy()
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    # ------------------------------------------------------------------     
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    # ------------------------------------------------------------------     
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    # done!
    return analyses 

# ----------------------------------------------------------------------
#   Define the Missions
# ----------------------------------------------------------------------
def missions_setup(mission): 
 
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions 

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission' 

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment() 
    base_segment.state.numerics.solver.type                      = "root_finder"
      
    # ------------------------------------------------------------------
    #   Takeoff
    # ------------------------------------------------------------------      
    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff"  
    segment.analyses.extend( analyses.base) 
    segment.velocity_start                                   = 10 *  Units.knots
    segment.velocity_end                                     = 120 *  Units.knots
    segment.friction_coefficient                             = 0.04   
    segment.throttle                                         = 0.8    
    mission.append_segment(segment) 
  
    # ------------------------------------------------------------------
    #   Departure End of Runway Segment Flight 1 : 
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = 'Departure_End_of_Runway'       
    segment.analyses.extend( analyses.base )  
    segment.altitude_start                                           = 0.0 * Units.feet
    segment.altitude_end                                             = 50.0 * Units.feet
    segment.air_speed_start                                          = 120 *  Units.knots
    segment.air_speed_end                                            = 125 *  Units.knots
                       
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                   
       
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Initial Climb Area Segment Flight 1  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = 'Initial_CLimb_Area' 
    segment.analyses.extend( analyses.base )   
    segment.altitude_start                                           = 50.0 * Units.feet
    segment.altitude_end                                             = 500.0 * Units.feet 
    segment.air_speed_end                                            = 130 *  Units.knots
    segment.climb_rate                                               = 600 * Units['ft/min']   
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                
          
    mission.append_segment(segment)  
             
    # ------------------------------------------------------------------
    #   Climb Segment Flight 1 
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = 'Climb_1'        
    segment.analyses.extend( analyses.base)      
    segment.altitude_start                                           = 500.0 * Units.feet
    segment.altitude_end                                             = 2500 * Units.feet  
    segment.air_speed_end                                            = 200    * Units.kts
    segment.climb_rate                                               = 500* Units['ft/min']  
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True
    
    mission.append_segment(segment)   
        
    # ------------------------------------------------------------------
    #   Climb 1 : constant Speed, constant rate segment 
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_2"
    segment.analyses.extend( analyses.base)
    segment.altitude_start                                           = 2500.0  * Units.feet
    segment.altitude_end                                             = 22000   * Units.feet  
    segment.air_speed_end                                            = 270    * Units.kts
    segment.climb_rate                                               = 500.034 * Units['ft/min']   
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True       
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "Cruise" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                                 = 22000  * Units.feet 
    segment.air_speed                                                = 270    * Units.kts
    segment.distance                                                 = 862.   * Units.nautical_mile  
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.74]]
    segment.assigned_control_variables.body_angle.active             = True     
    segment.assigned_control_variables.body_angle.initial_guess_values = [[4.2 * Units.degree]]                
          
    mission.append_segment(segment)    

    # ------------------------------------------------------------------
    #   Descent Segment Flight 1   
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = "Descent" 
    segment.analyses.extend( analyses.base)       
    segment.altitude_start                                           = 22000   * Units.feet 
    segment.altitude_end                                             = 1000 * Units.feet  
    segment.air_speed_end                                            = 200    * Units.kts
    segment.climb_rate                                               = -2000 * Units['ft/min']  
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                      
          
    mission.append_segment(segment)   
               
    # ------------------------------------------------------------------
    #  Downleg_Altitude Segment Flight 1 
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = 'Downleg'
    segment.analyses.extend(analyses.base)   
    segment.distance                                                 = 6000 * Units.feet
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                        
            
    mission.append_segment(segment)     
     
    # ------------------------------------------------------------------
    #  Baseleg Segment Flight 1  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = 'Baseleg'
    segment.analyses.extend( analyses.base)   
    segment.altitude_start                                = 1000 * Units.feet
    segment.altitude_end                                  = 500.0 * Units.feet
    segment.air_speed_end                                 = 120 *  Units.knots
    segment.climb_rate                                    = -350 * Units['ft/min'] 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True

    # ------------------------------------------------------------------
    #  Final Approach Segment Flight 1  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment_name = 'Final_Approach'
    segment.tag = segment_name          
    segment.analyses.extend( analyses.base)      
    segment.altitude_start                                = 500.0 * Units.feet
    segment.altitude_end                                  = 0.0 * Units.feet
    segment.air_speed_end                                 = 110 *  Units.knots
    segment.climb_rate                                    = -300 * Units['ft/min']   
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True
    
    mission.append_segment(segment)  

    # ------------------------------------------------------------------
    #   Landing  
    # ------------------------------------------------------------------  
    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "Landing"   
    segment.analyses.extend( analyses.base)  
    segment.velocity_end                                                  = 120 * Units.knots * 0.1  
    segment.friction_coefficient                                          = 0.4
    segment.altitude                                                      = 0.0 
    segment.assigned_control_variables.elapsed_time.active                = True   
    segment.assigned_control_variables.elapsed_time.initial_guess_values  = [[30.]]   
    
    mission.append_segment(segment)
      
    return mission
 
# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------
def plot_mission(results):

    # Plot Flight Conditions 
    plot_flight_conditions(results)
    
    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results)
    
    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results)
    
    # Drag Components
    plot_drag_components(results)
    
    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results)
    
    # Plot Velocities 
    plot_aircraft_velocities(results)  
    
    # Plot Trajectory
    plot_flight_trajectory(results)
    
    # Plot throttles
    plot_propulsor_throttles(results)

    return 

if __name__ == '__main__': 
    main()    
    plt.show()
