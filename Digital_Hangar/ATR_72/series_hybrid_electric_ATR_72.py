# ATR_72.py

# Created: 2025, M. Clarke, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots import *     
from RCAIDE.Library.Methods.Powertrain.Converters.Turboelectric_Generator   import design_turboelectric_generator 
from RCAIDE.Library.Plots.Mission.plot_flight_conditions                    import plot_flight_conditions 
from RCAIDE.Library.Plots import *

# python imports 
import numpy as np  
from   copy import deepcopy
import matplotlib.pyplot as plt 
import os

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main(solver_type = "optimize",solver_objective=None,plot_results=True): 

    vehicle  = vehicle_setup()
    # plot_3d_vehicle(vehicle)
    
    configs  = configs_setup(vehicle)
     
    analyses = analyses_setup(configs)

    missions = missions_setup(analyses,solver_type,solver_objective) 

    results  = missions.base_mission.evaluate()   
   
    if plot_results:
        plot_mission(results)    
        
    return results

def vehicle_setup():
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'ATR_72'

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------

    # mass properties
    vehicle.mass_properties.max_takeoff               = 27000 
    #vehicle.mass_properties.takeoff                   = 23000  
    vehicle.mass_properties.operating_empty           = 13600  
    vehicle.mass_properties.max_zero_fuel             = 21000  
    vehicle.mass_properties.max_payload               = 7400
    #vehicle.mass_properties.payload                   = 5
    vehicle.mass_properties.center_of_gravity         = [[0,0,0]] # Unknown 
    vehicle.mass_properties.moments_of_inertia.tensor = [[0,0,0]] # Unknown 
    vehicle.mass_properties.max_fuel                  = 5000
    vehicle.design_mach_number                        = 0.41 
    vehicle.design_range                              = 5471000 *Units.meter  
    vehicle.design_cruise_alt                         = 15000 *Units.feet

    # envelope properties
    vehicle.flight_envelope.design_mach_number        = 0.43 
    vehicle.flight_envelope.design_range              = 890 * Units.nmi
    vehicle.flight_envelope.design_cruise_altitude    = 15000 * Units.feet
    vehicle.flight_envelope.ultimate_load             = 3.75
    vehicle.flight_envelope.positive_limit_load       = 1.5
              
    # basic parameters              
    vehicle.reference_area                            = 61.0  
    vehicle.number_of_passengers                      = 72
    vehicle.systems.control                           = "fully powered"
    vehicle.systems.accessories                       = "short range"  



    # ################################################# Landing Gear #############################################################    
    
    main_gear                   = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter     = 34  *  Units.inches 
    main_gear.rim_diameter      = 16  *  Units.inches 
    main_gear.tire_width        = 10  *  Units.inches 
    main_gear.strut_length      = 1 *  Units.meter 
    main_gear.wheels            = 4   
    main_gear.number_of_gear_types_in_tandem  = 1
    main_gear.number_of_wheels_in_gear_type  = 2  
    main_gear.xz_plane_symmetric= True
    vehicle.append_component(main_gear)  

    nose_gear                   = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter     = 17   *  Units.inches   
    nose_gear.rim_diameter      = 7    *  Units.inches 
    nose_gear.tire_width        = 17   *  Units.inches 
    nose_gear.strut_length      = 1 *  Units.meter 
    nose_gear.wheels            = 2   
    nose_gear.number_of_gear_types_in_tandem  = 1
    nose_gear.number_of_wheels_in_gear_type  = 2    
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
    wing.high_lift                        = True
    wing.dynamic_pressure_ratio           = 1.0 

    # Wing Segments   
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0.0 * Units.deg
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.0  * Units.degrees
    segment.sweeps.quarter_chord          = 0.0 * Units.degrees
    segment.thickness_to_chord            = .15
    wing.append_segment(segment)
  
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'outboard'
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
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.sweeps.leading_edge    = wing.sweeps.leading_edge 
    segment.thickness_to_chord     = .12
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
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
    segment.thickness_to_chord            = 0.15
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
    #battery_casing                                    = RCAIDE.Library.Components.Booms.Boom()
    #battery_casing.tag                                = 'battery_casing' 
    #battery_casing.origin                             = [[ 6, 0, -0.5]]    
    #battery_casing.lengths.total                      = 10 
    #battery_casing.width                              = 2.   
    #battery_casing.heights.maximum                    = 0.5
    #battery_casing.heights.at_quarter_length          = battery_casing.heights.maximum
    #battery_casing.heights.at_three_quarters_length   = battery_casing.heights.maximum
    #battery_casing.effective_diameter                 = battery_casing.width  
    #battery_casing.differential_pressure              = 0.   
    
    ## Segment  
    #segment                           = RCAIDE.Library.Components.Booms.Segments.Segment() 
    #segment.tag                       = 'segment_1'   
    #segment.percent_x_location        = 0
    #segment.percent_z_location        = 0 
    #segment.height                    = 0
    #segment.width                     = 0
    #battery_casing.append_segment(segment)           
    
    ## Segment                                   
    #segment                           = RCAIDE.Library.Components.Booms.Segments.Segment()
    #segment.tag                       = 'segment_2'   
    #segment.percent_x_location        = 0
    #segment.percent_z_location        = 0
    #segment.height                    = battery_casing.heights.maximum 
    #segment.width                     = battery_casing.width
    #segment.curvature                 =  5
    #battery_casing.append_segment(segment)
    
    ## Segment                                   
    #segment                           = RCAIDE.Library.Components.Booms.Segments.Segment()
    #segment.tag                       = 'segment_3'   
    #segment.percent_x_location        = 1
    #segment.percent_z_location        = 0
    #segment.height                    = battery_casing.heights.maximum 
    #segment.width                     = battery_casing.width
    #segment.curvature                 =  5
    #battery_casing.append_segment(segment)
  
    
    ## Segment                                   
    #segment                           = RCAIDE.Library.Components.Booms.Segments.Segment()
    #segment.tag                       = 'segment_4'    
    #segment.percent_x_location        = 1
    #segment.percent_z_location        = 0 
    #segment.height                    = 0  
    #segment.width                     = 0 
    #battery_casing.append_segment(segment) 
     
    ## add to vehicle
    #vehicle.append_component(battery_casing)
    

    ## ########################################################## Landing Gear Pods  ################################################################           
    landing_gear_pod                                    = RCAIDE.Library.Components.Fuselages.Fuselage()
    landing_gear_pod.tag                                = 'landing_gear_pod' 
    landing_gear_pod.origin                             = [[ 3, 0,  -0.082]]    
    landing_gear_pod.lengths.total                      = 16 
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
    fuselage.areas.wetted                       = np.pi * fuselage.width/2 * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.front_projected              = np.pi * fuselage.width/2      * Units['meters**2']  
    fuselage.differential_pressure              = 5.0e4 * Units.pascal
    fuselage.heights.at_quarter_length          = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_three_quarters_length   = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = fuselage.heights.maximum* Units.meter
    
     # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = 0.0000
    segment.height                              = 1E-3
    segment.width                               = 1E-3  
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

    # ##------------------------------------------------------------------------------------------------------------------------------------  
    # # Coolant Line
    # #------------------------------------------------------------------------------------------------------------------------------------  
    # coolant_line                                           = RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line(bus)
    # coolant_line.tag                                       = 'liquid_cooled_coolant_line'
    # net.coolant_lines.append(coolant_line)
    # HAS                                                    = RCAIDE.Library.Components.Thermal_Management.Batteries.Liquid_Cooled_Wavy_Channel(coolant_line)
    # HAS.design_altitude                                    = 15000. * Units.feet  
    # atmosphere                                             = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976() 
    # atmo_data                                              = atmosphere.compute_values(altitude = HAS.design_altitude)     
    # HAS.coolant_inlet_temperature                          = atmo_data.temperature[0,0]  
    # HAS.design_battery_operating_temperature               = 313
    # HAS.design_heat_removed                                = 15000
    # HAS                                                    = design_wavy_channel(HAS,bat_module) 
    
    # for battery_module in bus.battery_modules:
    #     coolant_line.battery_modules[battery_module.tag].append(HAS)
        
    # # Battery Heat Exchanger               
    # HEX                                                    = RCAIDE.Library.Components.Thermal_Management.Heat_Exchangers.Cross_Flow_Heat_Exchanger() 
    # HEX.design_altitude                                    = 15000. * Units.feet 
    # HEX.inlet_temperature_of_cold_fluid                    = atmo_data.temperature[0,0]   
    # HEX                                                    = design_cross_flow_heat_exchanger(HEX,coolant_line,bat_module)     
    # coolant_line.heat_exchangers.append(HEX)
    
    # # Reservoir for Battery TMS
    # RES                                                    = RCAIDE.Library.Components.Thermal_Management.Reservoirs.Reservoir()
    # coolant_line.reservoirs.append(RES) 
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Network Initialization
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    network                                 = RCAIDE.Framework.Networks.Network()   

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Systems definition
    #------------------------------------------------------------------------------------------------------------------------------------  

    # Hydraulic System
             
    hydraulics                              = RCAIDE.Library.Components.Powertrain.Systems.Hydraulic_System()
    hydraulics.tag                          = 'hydraulics'
    hydraulics.power_draw                   = 10000. # Watts
                 
    # Avionic System

    avionics                                = RCAIDE.Library.Components.Powertrain.Systems.Avionics_System()
    avionics.tag                            = 'avionics'
    avionics.power_draw                     = 10000. # Watts

    # Environmental Control System
             
    ecs                                     = RCAIDE.Library.Components.Powertrain.Systems.Environmental_Control_System()
    ecs.tag                                 = 'ecs'
    ecs.power_draw                          = 100000 # Watts

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Modulators definition
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    # Transformer Rectifier Unit 1

    tru_1                                   = RCAIDE.Library.Components.Powertrain.Modulators.Transformer_Rectifier_Unit()
    tru_1.tag                               = 'tru_1'

    # Transformer Rectifier Unit 2
 
    tru_2                                   = RCAIDE.Library.Components.Powertrain.Modulators.Transformer_Rectifier_Unit()
    tru_2.tag                               = 'tru_2'

    # Electronic Speed Controller 1
     
    esc_1                                   = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    esc_1.tag                               = 'esc_1'
    esc_1.efficiency                        = 0.95 
    esc_1.origin                            = [[ 9.559106394 ,4.219315295, 1.616135105]]
    esc_1.bus_voltage                       = 1008
    
    # Electronic Speed Controller 2
    esc_2                                   = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    esc_2.tag                               = 'esc_2'
    esc_2.efficiency                        = 0.95 
    esc_2.origin                            = [[ 9.559106394 ,4.219315295, 1.616135105]]
    esc_2.bus_voltage                       = 1008

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Converters definition
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    # Generator 

    generator                               = RCAIDE.Library.Components.Powertrain.Converters.Generator()
    generator.tag                           = 'generator'
    generator.generator_type                = 'DC'
    generator.efficiency                    = 0.95
    generator.design_power                  = 90. * 10**5 
    generator.nominal_voltage               = 1008
    generator.design_angular_velocity       = 30000.0 * Units.rpm 
    generator.number_of_turns               = 80
    generator.no_load_current               = 100  
    generator.stator_outer_diameter         = 0.35  
    generator.stator_inner_diameter         = 0.16  
    generator.mu_0                          = 4 * np.pi * 1e-7
    generator.mu_r                          = 4 * np.pi * 1e-7

    # Ram 
    
    ram                                     = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                 = 'ram'  

    # Inlet Nozzle 
                             
    inlet_nozzle                            = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                        = 'inlet_nozzle'
    inlet_nozzle.polytropic_efficiency      = 0.98
    inlet_nozzle.pressure_ratio             = 0.98 

    # High Pressure Compressor 

    compressor                              = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    compressor.tag                          = 'compressor'
    compressor.polytropic_efficiency        = 0.91
    compressor.pressure_ratio               = 6.716    
    compressor.mass_flow_rate               = 1.9 

    # Combustor 

    combustor                               = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                           = 'combustor'
    combustor.efficiency                    = 0.99 
    combustor.alphac                        = 1.0     
    combustor.turbine_inlet_temperature     = 1550
    combustor.pressure_ratio                = 0.95
    combustor.fuel_data                     = RCAIDE.Library.Attributes.Propellants.Jet_A()  
 
    # High Pressure Turbine 
    
    high_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                       = 'hpt'
    high_pressure_turbine.mechanical_efficiency     = 0.99
    high_pressure_turbine.polytropic_efficiency     = 0.93
 
    # Low Pressure Turbine 
 
    low_pressure_turbine                            = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                        = 'lpt'
    low_pressure_turbine.mechanical_efficiency      = 0.99
    low_pressure_turbine.polytropic_efficiency      = 0.93  
 
    # Core Nozzle 
            
    core_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                 = 'core_nozzle'
    core_nozzle.polytropic_efficiency               = 0.95
    core_nozzle.pressure_ratio                      = 0.99  
    core_nozzle.diameter                            = 0.92 

    # Propeller 1
                  
    propeller_1                                     = RCAIDE.Library.Components.Powertrain.Converters.Propeller() 
    propeller_1.tag                                 = 'propeller_1'  
    propeller_1.tip_radius                          = 13*Units.ft / 2
    propeller_1.number_of_blades                    = 6
    propeller_1.hub_radius                          = 20.  * Units.inches / 2 
    propeller_1.cruise.design_freestream_velocity   = 270 * Units.kts  
    propeller_1.cruise.design_angular_velocity      = 1200 *  Units.rpm 
    propeller_1.cruise.design_altitude              = 15000. * Units.feet 
    propeller_1.cruise.design_thrust                = 10000 * Units.N  
    propeller_1.origin                              = [[ 9.559106394 ,4.219315295, 1.616135105]] 
    ospath                                          = os.path.abspath(__file__)
    separator                                       = os.path.sep
    rel_path                                        = os.path.dirname(ospath)   + separator + '..' + separator 
    airfoil                                         = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                                     = 'NACA_4412' 
    airfoil.coordinate_file                         = rel_path + 'Airfoils' + separator + 'NACA_4412.txt'   # absolute path   
    airfoil.polar_files                             = [rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_50000.txt',
                                                       rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_100000.txt',
                                                       rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_200000.txt',
                                                       rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_500000.txt',
                                                       rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_1000000.txt']   
    propeller_1.append_airfoil(airfoil)                      
    propeller_1.airfoil_polar_stations              = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]   

    # Propeller 2  
               
    propeller_2                                     = RCAIDE.Library.Components.Powertrain.Converters.Propeller() 
    propeller_2.tag                                 = 'propeller_2'  
    propeller_2.tip_radius                          = 13 * Units.ft / 2
    propeller_2.number_of_blades                    = 6
    propeller_2.hub_radius                          = 20.  * Units.inches / 2 
    propeller_2.cruise.design_freestream_velocity   = 270 * Units.kts  
    propeller_2.cruise.design_angular_velocity      = 1200 *  Units.rpm 
    propeller_2.cruise.design_altitude              = 15000. * Units.feet 
    propeller_2.cruise.design_thrust                = 10000 * Units.N  
    propeller_2.origin                              = [[ 9.559106394 , -4.219315295, 1.616135105]] 
    ospath                                          = os.path.abspath(__file__)
    separator                                       = os.path.sep
    rel_path                                        = os.path.dirname(ospath)   + separator + '..' + separator 
    airfoil                                         = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                                     = 'NACA_4412' 
    airfoil.coordinate_file                         = rel_path + 'Airfoils' + separator + 'NACA_4412.txt'   # absolute path   
    airfoil.polar_files                             = [rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_50000.txt',
                                                       rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_100000.txt',
                                                       rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_200000.txt',
                                                       rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_500000.txt',
                                                       rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_1000000.txt']   
    propeller_2.append_airfoil(airfoil)                       
    propeller_2.airfoil_polar_stations              = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]   
              
    # Motor 1  
        
    motor_1                                         = RCAIDE.Library.Components.Powertrain.Converters.Motor()
    motor_1.tag                                     = 'motor_1'
    motor_1.type                                    = 'DC'
    motor_1.efficiency                              = 0.98
    motor_1.origin                                  = [[ 9.559106394 ,4.219315295, 1.616135105]]
    motor_1.nominal_voltage                         = 705
    motor_1.no_load_current                         = 1
    motor_1.rotor_radius                            = propeller_1.tip_radius
    motor_1.design_torque                           = propeller_1.cruise.design_torque 
    motor_1.design_angular_velocity                 = propeller_1.cruise.design_angular_velocity # Horse power of gas engine variant  750 * Units['hp'] 
     
    # Motor 2 
         
    motor_2                                         = RCAIDE.Library.Components.Powertrain.Converters.Motor()
    motor_2.tag                                     = 'motor_2'
    motor_2.type                                    = 'DC'
    motor_2.efficiency                              = 0.98
    motor_2.origin                                  = [[ 9.559106394 , -4.219315295, 1.616135105]]
    motor_2.nominal_voltage                         = 705
    motor_2.no_load_current                         = 1
    motor_2.rotor_radius                            = propeller_2.tip_radius
    motor_2.design_torque                           = propeller_2.cruise.design_torque 
    motor_2.design_angular_velocity                 = propeller_2.cruise.design_angular_velocity # Horse power of gas engine variant  750 * Units['hp'] 

    # Turboshaft
    
    turboshaft                                                 = RCAIDE.Library.Components.Powertrain.Converters.Turboshaft()
    turboshaft.working_fluid                                   = RCAIDE.Library.Attributes.Gases.Air() 
    turboshaft.design_altitude                                 = 15000. * Units.feet 
    turboshaft.design_mach_number                              = 0.5   
    turboshaft.design_power                                    = 9E6 *Units.W       
    turboshaft.design_angular_velocity                         = 30000.0*Units.rpm 

    turboshaft.assigned_converters.ram_tag                     = [[ram.tag]]
    turboshaft.assigned_converters.inlet_nozzle_tag            = [[inlet_nozzle.tag]]
    turboshaft.assigned_converters.compressor_tag              = [[compressor.tag]]
    turboshaft.assigned_converters.combustor_tag               = [[combustor.tag]]
    turboshaft.assigned_converters.high_pressure_turbine_tag   = [[high_pressure_turbine.tag]]
    turboshaft.assigned_converters.low_pressure_turbine_tag    = [[low_pressure_turbine.tag]]
    turboshaft.assigned_converters.core_nozzle_tag             = [[core_nozzle.tag]]

    # Turboelectric Generator

    turboelectric_generator                                    = RCAIDE.Library.Components.Powertrain.Converters.Turboelectric_Generator() 
    turboelectric_generator.origin                             = [[ 15 ,0, 1.616135105]]  
    turboelectric_generator.length                             = 0.945 

    turboelectric_generator.assigned_converters.generator_tag  = [[generator.tag]]
    turboelectric_generator.assigned_converters.turboshaft_tag = [[turboshaft.tag]]

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Propulsors definition
    #------------------------------------------------------------------------------------------------------------------------------------  

    starboard_propulsor                               = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()  
    starboard_propulsor.tag                           = 'starboard_propulsor'    
    
    starboard_propulsor.assigned_converters.motor_tag = [[motor_1.tag]]
    starboard_propulsor.assigned_converters.rotor_tag = [[propeller_1.tag]]
    starboard_propulsor.assigned_modulators.esc_tag   = [[esc_1.tag]]
 
    port_propulsor                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()  
    port_propulsor.tag                                = 'port_propulsor'
    
    port_propulsor.assigned_converters.motor_tag      = [[motor_2.tag]]
    port_propulsor.assigned_converters.rotor_tag      = [[propeller_2.tag]]
    port_propulsor.assigned_modulators.esc_tag        = [[esc_2.tag]]

    # Nacelles ---------------------------------------------------

    # Nacelle 1
     
    nacelle_1                                   = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    nacelle_1.tag                               = 'nacelle_1'
    nacelle_1.length                            = 5
    nacelle_1.diameter                          = 0.85 
    nacelle_1.areas.wetted                      = 1.0   
    nacelle_1.origin                            = [[8.941625295,4.219315295, 1.616135105 ]]
    nacelle_1.flow_through                      = False     

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_1'
    nac_segment.percent_x_location              = 0.0 
    nac_segment.height                          = 0.0
    nac_segment.width                           = 0.0
    nacelle_1.append_segment(nac_segment)   

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_2'
    nac_segment.percent_x_location              = 0.2 /  nacelle_1.length
    nac_segment.percent_z_location              = 0 
    nac_segment.height                          = 0.4 
    nac_segment.width                           = 0.4  
    nacelle_1.append_segment(nac_segment)   

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_3'
    nac_segment.percent_x_location              = 0.6 / nacelle_1.length
    nac_segment.percent_z_location              = 0 
    nac_segment.height                          = 0.52 
    nac_segment.width                           = 0.700 
    nac_segment.curvature                       = 3  
    nacelle_1.append_segment(nac_segment)  

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_4'
    nac_segment.percent_x_location              = 0.754 / nacelle_1.length
    nac_segment.percent_z_location              = -0.15 / nacelle_1.length
    nac_segment.height                          = 0.9	 
    nac_segment.width                           = 0.85 
    nac_segment.curvature                       = 3  
    nacelle_1.append_segment(nac_segment)  

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_5'
    nac_segment.percent_x_location              = 1.154 / nacelle_1.length
    nac_segment.percent_z_location              = -0.1 / nacelle_1.length
    nac_segment.height                          = 1 
    nac_segment.width                           = 0.85 
    nac_segment.curvature                       = 4  
    nacelle_1.append_segment(nac_segment)   

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_6'
    nac_segment.percent_x_location              = 3.414  / nacelle_1.length
    nac_segment.percent_z_location              = -0.1 / nacelle_1.length 
    nac_segment.height                          = 0.9 
    nac_segment.width                           = 0.85 
    nac_segment.curvature                       = 4  
    nacelle_1.append_segment(nac_segment)

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_6'
    nac_segment.percent_x_location              = 0.96 
    nac_segment.percent_z_location              = 0.05 / nacelle_1.length 
    nac_segment.height                          = 0.6
    nac_segment.width                           = 0.5
    nac_segment.curvature                       = 4  
    nacelle_1.append_segment(nac_segment)    

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                             = 'segment_7'
    nac_segment.percent_x_location              = 1.0 
    nac_segment.percent_z_location              = 0.15 / nacelle_1.length  	
    nac_segment.height                          = 0.4
    nac_segment.width                           = 0.2
    nac_segment.curvature                       = 4  
    nacelle_1.append_segment(nac_segment)

    starboard_propulsor.nacelle = nacelle_1

    # Nacelle 2
     
    nacelle_2          = deepcopy(nacelle_1)
    nacelle_2.tag      = 'nacelle_2'
    nacelle_2.origin   = [[8.941625295,-4.219315295, 1.616135105 ]]

    port_propulsor.nacelle = nacelle_2   

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Sources definition
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    # Battery Module 

    generic_bat_module_1                                             = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC()
    generic_bat_module_1.electrical_configuration.series             = 20 
    generic_bat_module_1.electrical_configuration.parallel           = 210  *  2
    generic_bat_module_1.cell.nominal_capacity                       = 3.8 
    generic_bat_module_1.geometrtic_configuration.normal_count       = 42 
    generic_bat_module_1.geometrtic_configuration.parallel_count     = 100  *  2

    generic_bat_module_2                                             = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC()
    generic_bat_module_2.electrical_configuration.series             = 20 
    generic_bat_module_2.electrical_configuration.parallel           = 210  *  2
    generic_bat_module_2.cell.nominal_capacity                       = 3.8 
    generic_bat_module_2.geometrtic_configuration.normal_count       = 42 
    generic_bat_module_2.geometrtic_configuration.parallel_count     = 100  *  2

    # Fuel Tank

    fuel_tank                                   = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)
    fuel_tank.tag                               = 'fuel_tank' 
    fuel                                        = RCAIDE.Library.Attributes.Propellants.Jet_A()   
    fuel_tank.fuel                              = fuel
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Distributors definition
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    # AC Bus 1

    ac_bus_1                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    ac_bus_1.tag                                = 'ac_bus_1'
    ac_bus_1.type                               = 'AC'
    ac_bus_1.voltage_phase_to_neutral           = 115.0 
    ac_bus_1.voltage_phase_to_phase             = 200.0
    ac_bus_1.frequency                          = 400. 
    ac_bus_1.efficiency                         = 0.95 
     
    # AC Bus 2     
     
    ac_bus_2                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    ac_bus_2.tag                                = 'ac_bus_2'
    ac_bus_2.type                               = 'AC'
    ac_bus_2.voltage_phase_to_neutral           = 115.0 
    ac_bus_2.voltage_phase_to_phase             = 200.0
    ac_bus_2.frequency                          = 400. 
    ac_bus_2.efficiency                         = 0.95   
     
    # DC Bus 1     
     
    dc_bus_1                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    dc_bus_1.tag                                = 'dc_bus_1'
    dc_bus_1.type                               = 'DC'
    dc_bus_1.voltage                            = 28
    dc_bus_1.frequency                          = 0
    dc_bus_1.efficiency                         = 0.95  
     
    # DC Bus 2     
     
    dc_bus_2                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    dc_bus_2.tag                                = 'dc_bus_2'
    dc_bus_2.type                               = 'DC'
    dc_bus_2.voltage                            = 28
    dc_bus_2.frequency                          = 0
    dc_bus_2.efficiency                         = 0.95 
     
    # DC Ess Bus 1     
     
    dc_ess_1                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    dc_ess_1.tag                                = 'dc_ess_1'
    dc_ess_1.type                               = 'DC'
    dc_ess_1.voltage                            = 28
    dc_ess_1.frequency                          = 0
    dc_ess_1.efficiency                         = 0.95  
     
    # DC Ess Bus 2     
         
    dc_ess_2                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    dc_ess_2.tag                                = 'dc_ess_2'
    dc_ess_2.type                               = 'DC'
    dc_ess_2.voltage                            = 28
    dc_ess_2.frequency                          = 0
    dc_ess_2.efficiency                         = 0.95  

    # Fuel Line 
    
    fuel_line                                   = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line() 
    fuel_line.tag                               = 'fuel_line'

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Assign distributors to components
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    # Systems

    hydraulics.assigned_distributors              = [[dc_bus_1.tag]]
    avionics.assigned_distributors                = [[dc_bus_1.tag]]
    ecs.assigned_distributors                     = [[dc_bus_1.tag]]

    # Modulators

    tru_1.assigned_distributors                   = [[ac_bus_1.tag,
                                                      dc_bus_1.tag]]
    tru_2.assigned_distributors                   = [[ac_bus_2.tag,
                                                      dc_bus_2.tag]]
    # Converters

    turboelectric_generator.assigned_distributors = [[fuel_line.tag,
                                                      ac_bus_1.tag,
                                                      ac_bus_2.tag]]

    # Propulsors

    starboard_propulsor.assigned_distributors     = [[dc_bus_1.tag]]

    port_propulsor.assigned_distributors          = [[dc_bus_2.tag]]

    # Sources

    fuel_tank.assigned_distributors               = [[fuel_line.tag]]
    generic_bat_module_1.assigned_distributors    = [[dc_ess_1.tag]]
    generic_bat_module_2.assigned_distributors    = [[dc_ess_2.tag]]

    for _ in range(12):
        bat_copy_1 = deepcopy(generic_bat_module_1)
        bat_copy_1.assigned_distributors = [[dc_ess_1.tag]]
        dc_ess_1.battery_modules.append(bat_copy_1)
    dc_ess_1.battery_module_electric_configuration = 'Series'  

    for _ in range(12):
        bat_copy_2 = deepcopy(generic_bat_module_2)
        bat_copy_2.assigned_distributors = [[dc_ess_2.tag]]
        dc_ess_2.battery_modules.append(bat_copy_2)
    dc_ess_2.battery_module_electric_configuration = 'Series'  

    # Distributors
    
    dc_bus_1.assigned_distributors                = [[dc_ess_1.tag]]
    dc_bus_2.assigned_distributors                = [[dc_ess_2.tag]] 
    dc_ess_1.assigned_distributors                = [[dc_bus_1.tag]] 
    dc_ess_2.assigned_distributors                = [[dc_bus_2.tag]] 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append systems to the network
    #------------------------------------------------------------------------------------------------------------------------------------  
     
    network.systems.append(hydraulics)
    network.systems.append(avionics)
    network.systems.append(ecs)    
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append modulators to the network
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    network.modulators.append(tru_1)
    network.modulators.append(tru_2) 
    network.modulators.append(esc_1)
    network.modulators.append(esc_2)
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append converters to the network
    #------------------------------------------------------------------------------------------------------------------------------------  
    
    network.converters.append(ram)
    network.converters.append(inlet_nozzle)
    network.converters.append(compressor)
    network.converters.append(combustor)
    network.converters.append(high_pressure_turbine)
    network.converters.append(low_pressure_turbine)
    network.converters.append(core_nozzle)
    network.converters.append(turboshaft)
    network.converters.append(generator)
    network.converters.append(turboelectric_generator)
    network.converters.append(propeller_1)
    network.converters.append(propeller_2)
    network.converters.append(motor_1)
    network.converters.append(motor_2)

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append propulsors to the network
    #------------------------------------------------------------------------------------------------------------------------------------  
  
    network.propulsors.append(starboard_propulsor)     
    network.propulsors.append(port_propulsor)

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append sources to the network
    #------------------------------------------------------------------------------------------------------------------------------------  

    network.sources.append(generic_bat_module_1) 
    network.sources.append(generic_bat_module_2)   
    network.sources.append(fuel_tank)

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append distributors to the network
    #------------------------------------------------------------------------------------------------------------------------------------  
  
    network.distributors.append(ac_bus_1)
    network.distributors.append(ac_bus_2) 
    network.distributors.append(dc_bus_1) 
    network.distributors.append(dc_bus_2) 
    network.distributors.append(dc_ess_1) 
    network.distributors.append(dc_ess_2) 
    network.distributors.append(fuel_line)

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Design components
    #------------------------------------------------------------------------------------------------------------------------------------  

    design_turboelectric_generator(turboelectric_generator, network) 
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append network to the vehicle
    #------------------------------------------------------------------------------------------------------------------------------------  
  
    vehicle.append_energy_network(network)     

    #------------------------------------------------------------------------------------------------------------------------------------  
    # vehicle_setup complete!
    #------------------------------------------------------------------------------------------------------------------------------------      

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
   
    # append vehicle 
    analyses.vehicle = vehicle 

    #  Geometry
    analyses.geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    analyses.geometry.settings.overwrite_reference          = True
    analyses.geometry.settings.update_wing_properties       = True
    analyses.geometry.settings.print_weight_analysis_report = True
    analyses.geometry.settings.overwrite_fuel_volume        = True 
 
    #  Weights 
    analyses.weights = RCAIDE.Framework.Analyses.Weights.Conventional()  
    analyses.weights.settings.update_center_of_gravity                            = True
    analyses.weights.settings.update_moment_of_inertia                            = True
    analyses.weights.settings.print_weight_analysis_report                        = True
    analyses.weights.method                                                       = "FLOPS"
    analyses.weights.settings.FLOPS.fidelity                                      = "Complex" 
    analyses.weights.settings.weight_correction_factors.empty.structurals.main_wing            = 0.3  
    analyses.weights.settings.weight_correction_factors.empty.structurals.empennage            = 0.3  
    analyses.weights.settings.weight_correction_factors.empty.structurals.fuselage             = 0.3  
    analyses.weights.settings.weight_correction_factors.empty.structurals.structural           = 0.3  
    analyses.weights.settings.weight_correction_factors.empty.structurals.systems              = 0.3   
    analyses.weights.settings.weight_correction_factors.empty.structurals.nacelle              = 0.3
 
    #  Aerodynamics 
    analyses.aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()

    #  Aerodynamics 
    analyses.stability = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method()       

    #  Energy
    analyses.energy = RCAIDE.Framework.Analyses.Energy.Energy() 

    #  Planet Analysis
    analyses.planet = RCAIDE.Framework.Analyses.Planets.Earth() 

    #  Atmosphere Analysis
    analyses.atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976() 

    # done!
    return analyses 

# ----------------------------------------------------------------------
#   Define the Missions
# ----------------------------------------------------------------------

def missions_setup(analyses,solver_type,solver_objective): 

    # create base mission 
    mission          = mission_setup(analyses,solver_type,solver_objective)
    
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions 

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses,solver_type,solver_objective):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission' 

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment()
    base_segment.state.numerics.solver.type       = solver_type
    base_segment.state.numerics.solver.objective  = solver_objective 
         
    # ------------------------------------------------------------------
    #   Takeoff
    # ------------------------------------------------------------------      
    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff"  
    segment.analyses.extend( analyses.base) 
    segment.velocity_start                                           = 10 *  Units.knots
    segment.velocity_end                                             = 120 *  Units.knots
    segment.friction_coefficient                                     = 0.04   
    segment.throttle                                                 = 0.8       
    segment.initial_battery_state_of_charge                          = 1.0 
    segment.hybrid_power_split_ratio                                 = 0.1
    segment.battery_fuel_cell_power_split_ratio                      = 1.0
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
    segment.initial_battery_state_of_charge                          = 1.0    
    segment.hybrid_power_split_ratio                                 = 0.3
    segment.battery_fuel_cell_power_split_ratio                      = 1.0
    
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                             = True  
    segment.flight_dynamics.force_z                                             = True   
    segment.flight_dynamics.moment_y                                            = True 
    
    # define flight controls      
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']]
    segment.assigned_control_variables.elevator_deflection.initial_guess_values = [[0.0]]
    segment.assigned_control_variables.elevator_deflection.bounds               = [[-30 *Units.degree, 30 *Units.degree]]  
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.throttle.initial_guess_values            = [[0.75]]
    segment.assigned_control_variables.body_angle.active                        = True    
    segment.assigned_control_variables.body_angle.initial_guess_values          = [[6.5 * Units.degree]] 
       
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Initial Climb Area Segment Flight 1  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = 'Initial_CLimb_Area' 
    segment.analyses.extend( analyses.base )   
    segment.altitude_start                                           = 50.0 * Units.feet
    segment.altitude_end                                             = 500.0 * Units.feet 
    segment.air_speed_start                                          = 120 *  Units.knots
    segment.air_speed_end                                            = 130 *  Units.knots
    segment.climb_rate                                               = 600 * Units['ft/min']      
    segment.hybrid_power_split_ratio                                 = 0.3
    segment.battery_fuel_cell_power_split_ratio                      = 1.0
    
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                             = True  
    segment.flight_dynamics.force_z                                             = True   
    segment.flight_dynamics.moment_y                                            = True 
    
    # define flight controls      
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']]
    segment.assigned_control_variables.elevator_deflection.initial_guess_values = [[0.0]]
    segment.assigned_control_variables.elevator_deflection.bounds               = [[-30 *Units.degree, 30 *Units.degree]]  
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active                        = True                    
          
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
    segment.hybrid_power_split_ratio                                 = 0.3
    segment.battery_fuel_cell_power_split_ratio                      = 1.0
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                             = True  
    segment.flight_dynamics.force_z                                             = True   
    segment.flight_dynamics.moment_y                                            = True 
    
    # define flight controls      
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']]
    segment.assigned_control_variables.elevator_deflection.initial_guess_values = [[0.0]]
    segment.assigned_control_variables.elevator_deflection.bounds               = [[-30 *Units.degree, 30 *Units.degree]]  
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active                        = True       
    
    mission.append_segment(segment)   
        
    # ------------------------------------------------------------------
    #   Climb 1 : constant Speed, constant rate segment 
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_2"
    segment.analyses.extend( analyses.base)
    segment.altitude_start                                           = 2500.0  * Units.feet
    segment.altitude_end                                             = 15000   * Units.feet  
    segment.air_speed_end                                            = 270    * Units.kts
    segment.climb_rate                                               = 500.034 * Units['ft/min']    
    segment.hybrid_power_split_ratio                                 = 0.3
    segment.battery_fuel_cell_power_split_ratio                      = 1.0
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                             = True  
    segment.flight_dynamics.force_z                                             = True   
    segment.flight_dynamics.moment_y                                            = True 
    
    # define flight controls      
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']]
    segment.assigned_control_variables.elevator_deflection.initial_guess_values = [[0.0]]
    segment.assigned_control_variables.elevator_deflection.bounds               = [[-30 *Units.degree, 30 *Units.degree]]  
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active                        = True       
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "Cruise" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                                 = 15000  * Units.feet 
    segment.air_speed                                                = 270    * Units.kts
    segment.distance                                                 = 100.   * Units.nautical_mile    
    segment.hybrid_power_split_ratio                                 = 0.3
    segment.battery_fuel_cell_power_split_ratio                      = 1.0     

    
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                             = True  
    segment.flight_dynamics.force_z                                             = True   
    segment.flight_dynamics.moment_y                                            = True 
    
    # define flight controls      
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']]
    segment.assigned_control_variables.elevator_deflection.initial_guess_values = [[0.0]]
    segment.assigned_control_variables.elevator_deflection.bounds               = [[-30 *Units.degree, 30 *Units.degree]]  
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.throttle.initial_guess_values            = [[0.7]]
    segment.assigned_control_variables.body_angle.active                        = True 
    
    
    mission.append_segment(segment)    

    # ------------------------------------------------------------------
    #   Descent Segment Flight 1   
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = "Descent"  
    segment.analyses.extend( analyses.base)       
    segment.altitude_start                                           = 15000   * Units.feet 
    segment.altitude_end                                             = 1000 * Units.feet  
    segment.air_speed_end                                            = 200    * Units.kts
    segment.climb_rate                                               = -200 * Units['ft/min']     
    segment.hybrid_power_split_ratio                                 = 0.3
    segment.battery_fuel_cell_power_split_ratio                      = 1.0
    
    
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                             = True  
    segment.flight_dynamics.force_z                                             = True   
    segment.flight_dynamics.moment_y                                            = True 
    
    # define flight controls      
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']]
    segment.assigned_control_variables.elevator_deflection.initial_guess_values = [[0.0]]
    segment.assigned_control_variables.elevator_deflection.bounds               = [[-30 *Units.degree, 30 *Units.degree]]  
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active                        = True 
                    
          
    mission.append_segment(segment)   
               
    # ------------------------------------------------------------------
    #  Downleg_Altitude Segment Flight 1 
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = 'Downleg'
    segment.analyses.extend(analyses.base)   
    segment.distance                                                 = 3000 * Units.feet  
    segment.hybrid_power_split_ratio                                 = 0.3
    segment.battery_fuel_cell_power_split_ratio                      = 1.0
               
    
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                             = True  
    segment.flight_dynamics.force_z                                             = True   
    segment.flight_dynamics.moment_y                                            = True 
    
    # define flight controls      
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']]
    segment.assigned_control_variables.elevator_deflection.initial_guess_values = [[0.0]]
    segment.assigned_control_variables.elevator_deflection.bounds               = [[-30 *Units.degree, 30 *Units.degree]]  
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active                        = True 
                        
            
    mission.append_segment(segment)     
     
    # ------------------------------------------------------------------
    #  Baseleg Segment Flight 1  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = 'Baseleg'
    segment.analyses.extend( analyses.base)   
    segment.altitude_start                                           = 1000 * Units.feet
    segment.altitude_end                                             = 500.0 * Units.feet
    segment.air_speed_end                                            = 120 *  Units.knots
    segment.climb_rate                                               = -300 * Units['ft/min']    
    segment.hybrid_power_split_ratio                                 = 0.3
    segment.battery_fuel_cell_power_split_ratio                      = 1.0
               
    
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                             = True  
    segment.flight_dynamics.force_z                                             = True   
    segment.flight_dynamics.moment_y                                            = True 
    
    # define flight controls      
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']]
    segment.assigned_control_variables.elevator_deflection.initial_guess_values = [[0.0]]
    segment.assigned_control_variables.elevator_deflection.bounds               = [[-30 *Units.degree, 30 *Units.degree]]  
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active                        = True 
    

    mission.append_segment(segment) 

    # ------------------------------------------------------------------
    #  Final Approach Segment Flight 1  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment_name = 'Final_Approach'
    segment.tag = segment_name          
    segment.analyses.extend( analyses.base)      
    segment.altitude_start                                           = 500.0 * Units.feet
    segment.altitude_end                                             = 0.0 * Units.feet
    segment.air_speed_end                                            = 110 *  Units.knots
    segment.climb_rate                                               = -300 * Units['ft/min']      
    segment.hybrid_power_split_ratio                                 = 0.3
    segment.battery_fuel_cell_power_split_ratio                      = 1.0
    
    
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                             = True  
    segment.flight_dynamics.force_z                                             = True   
    segment.flight_dynamics.moment_y                                            = True 
    
    # define flight controls      
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']]
    segment.assigned_control_variables.elevator_deflection.initial_guess_values = [[0.0]]
    segment.assigned_control_variables.elevator_deflection.bounds               = [[-30 *Units.degree, 30 *Units.degree]]  
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active                        = True 
    
    
    mission.append_segment(segment)  

    ## ------------------------------------------------------------------
    ##   Landing  
    ## ------------------------------------------------------------------  
    #segment = Segments.Ground.Landing(base_segment)
    #segment.tag = "Landing"   
    #segment.analyses.extend( analyses.base)  
    #segment.velocity_end                                                  = 10 *  Units.knots
    #segment.friction_coefficient                                          = 0.4
    #segment.altitude                                                      = 0.0    
    #segment.hybrid_power_split_ratio                                      = 0.1
    #segment.battery_fuel_cell_power_split_ratio                      = 1.0
    #segment.assigned_control_variables.elapsed_time.active                = True   
    #segment.assigned_control_variables.elapsed_time.initial_guess_values  = [[30.]]   
    
    #mission.append_segment(segment)  
      
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
    # plot_altitude_sfc_weight(results)

    # Plot Velocities 
    plot_aircraft_velocities(results)  

    # Plot Trajectory
    plot_flight_trajectory(results)

    # Plot throttles
    plot_propulsor_throttles(results)

    # Plot Aircraft Stability 
    plot_longitudinal_stability(results)  
    
    plot_lateral_stability(results) 
    
    plot_flight_forces_and_moments(results)     

    plot_powertrain_diagram(results)
    

    return 

if __name__ == '__main__': 
    main()    
    plt.show()