# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units   
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan        import design_turbofan     
from RCAIDE.Library.Plots                 import *   
from RCAIDE.Library.Methods.Performance import *    

# python imports 
import numpy as np  
from copy import deepcopy
import matplotlib.pyplot as plt  
import os   

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main(plot_results=True, plot_vehicle=False, fuel='Jet_A1'): 
    
    # Step 1 design a vehicle
    vehicle  = vehicle_setup(fuel)
    
    # plot vehicle
    plot_3d_vehicle(vehicle)           
    
    # Step 2 create aircraft configuration based on vehicle 
    configs  = configs_setup(vehicle)
    
    # Step 3 set up analysis
    analyses = analyses_setup(configs)
    
    # Step 4 set up a flight mission
    #mission = mission_setup(analyses)
    mission = aviation_mission_setup(analyses)
    missions = missions_setup(mission) 
    
    # Step 5 execute flight profile
    results = missions.base_mission.evaluate()  
    
    # Step 6 plot results
    if plot_results:
        plot_mission(results)
    
    return results

def payload_range(plot_results=True, plot_vehicle=True): 
     
    # Step 1 design a vehicle
    vehicle  = vehicle_setup()    
     
    # Step 2 create aircraft configuration based on vehicle 
    configs  = configs_setup(vehicle)
    
    # Step 3 set up analysis
    analyses = analyses_setup(configs)
    
    # Step 4 set up a flight mission
    mission = simplified_mission_setup(analyses)
    missions= missions_setup(mission)
    
    payload_range_data =  compute_payload_range_diagram(mission = missions.base_mission, cruise_segment_tag = "cruise", fuel_reserve_percentage=0.05, plot_diagram = plot_results, fuel_name=None)
    
    return payload_range_data


def vehicle_setup(fuel):
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle   https://www.boeing.com/content/dam/boeing/boeingdotcom/commercial/airports/acaps/777-200-200ER-300_Rev_E.pdf
    # ------------------------------------------------------------------    
    
    vehicle     = RCAIDE.Vehicle()
    vehicle.tag = 'Boeing_777-200er'

    
    # ################################################# Vehicle-level Properties #################################################   
    vehicle.mass_properties.max_takeoff               = 297550 * Units.kilogram
    vehicle.mass_properties.takeoff                   = 297550  * Units.kilogram  
    vehicle.mass_properties.max_payload               = 55308 
    vehicle.mass_properties.max_fuel                  = 141000 # 171170 * Units.kilogram
    vehicle.mass_properties.payload                   = 120000 * Units.lbs
    vehicle.mass_properties.operating_empty           = 145510 * Units.kilogram
    vehicle.mass_properties.max_zero_fuel             = 199580 * Units.kilogram
    vehicle.mass_properties.fuel                      = 137454 * Units.kilogram  
    vehicle.mass_properties.center_of_gravity         = [[25, 0, 0]]
    vehicle.flight_envelope.ultimate_load             = 3.75 
    vehicle.flight_envelope.positive_limit_load       = 2.5  
    vehicle.flight_envelope.negative_limit_load       = 0.75
    vehicle.flight_envelope.design_mach_number        = 0.84
    vehicle.flight_envelope.design_cruise_altitude    = 35000*Units.feet
    vehicle.flight_envelope.design_range              = 6500 * Units.nmi
    vehicle.reference_area                            = 436.80 * Units['meters**2']   
    vehicle.number_of_passengers                      = 280
    vehicle.systems.control                           = "fully powered" 
    vehicle.systems.accessories                       = "medium range" 
    
    # ################################################# Wings ##################################################################### 
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing'
    wing.sweeps.quarter_chord             = 25 * Units.deg
    wing.thickness_to_chord               = 0.10  
    wing.spans.projected                  = 64.8 
    wing.chords.root                      = 15.1335 * Units.meter
    wing.chords.tip                       = 1.8055 * Units.meter
    wing.taper                            = wing.chords.tip / wing.chords.root
    wing.chords.mean_aerodynamic          = 15.1335 / 2 # INCORRECT 
    wing.areas.reference                  = 436.80 
    wing.areas.exposed                    = 2*wing.areas.reference *0.8
    wing.areas.wetted                     = 2*wing.areas.reference *0.8 
    wing.total_length                     = wing.chords.root 
    wing.aspect_ratio                     = (wing.spans.projected ** 2) /  wing.areas.reference 
    wing.twists.root                      = 4.0 * Units.degrees 
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.percent_span_unexposed           = 6.2 / wing.spans.projected 
    wing.origin                           = [[25 ,0, -0.75]]
    wing.aerodynamic_center               = [25+ 0.25*wing.chords.root ,0, -0.75]  
    wing.vertical                         = False
    wing.dihedral                         = 7.5 * Units.degrees 
    wing.xz_plane_symmetric               = True 
    wing.high_lift                        = True 
    wing.dynamic_pressure_ratio           = 1.0
        
    # Wing Segments
    root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()

    
    
    root_airfoil.coordinate_file          = 'transonic_wing_root_section_airfoil.txt'
    segment                               =  RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Root'
    segment.percent_span_location         = 0.0
    segment.root_chord_percent            = 1.0
    segment.thickness_to_chord            = 0.07
    segment.dihedral_outboard             = 7.5 * Units.degrees 
    segment.sweeps.leading_edge           = 35.214 * Units.deg 
    segment.append_airfoil(root_airfoil)
    wing.append_segment(segment)

    inboard_airfoil                       = RCAIDE.Library.Components.Airfoils.Airfoil()
    inboard_airfoil.coordinate_file       = 'transonic_wing_inboard_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.26382
    segment.root_chord_percent            = 0.635424
    segment.thickness_to_chord            = 0.07
    segment.dihedral_outboard             = 7.5 * Units.degrees 
    segment.sweeps.leading_edge           = 35.214 * Units.deg
    segment.append_airfoil(inboard_airfoil)
    wing.append_segment(segment)
    

    outboard_airfoil                      = RCAIDE.Library.Components.Airfoils.Airfoil()
    outboard_airfoil.coordinate_file      = 'transonic_wing_outboard_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'outboard'
    segment.percent_span_location         = 0.3772
    segment.root_chord_percent            = 0.5154
    segment.thickness_to_chord            = 0.07
    segment.dihedral_outboard             = 7.5 * Units.degrees 
    segment.sweeps.leading_edge           = 35.214 * Units.deg 
    segment.append_airfoil(outboard_airfoil)
    wing.append_segment(segment) 
 

    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = 'transonic_wing_tip_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Tip'
    segment.percent_span_location         = 1.0 
    segment.root_chord_percent            = 0.11930 
    segment.thickness_to_chord            = 0.07  
    segment.dihedral_outboard             = 0 * Units.degrees      
    segment.sweeps.quarter_chord          = 0 * Units.degrees     
    segment.append_airfoil(tip_airfoil) 
    wing.append_segment(segment)
    

    # control surfaces -------------------------------------------
    slat                                  = RCAIDE.Library.Components.Wings.Control_Surfaces.Slat()
    slat.tag                              = 'slat'
    slat.span_fraction_start              = 0.2
    slat.span_fraction_end                = 0.963
    slat.deflection                       = 0.0 * Units.degrees
    slat.chord_fraction                   = 0.075
    wing.append_control_surface(slat)

    inboard_flap                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    inboard_flap.tag                      = 'inboard_flap'
    inboard_flap.span_fraction_start      = 0.05
    inboard_flap.span_fraction_end        = 0.2
    inboard_flap.deflection               = 0.0 * Units.degrees
    inboard_flap.configuration_type       = 'tripple_slotted'
    inboard_flap.chord_fraction           = 0.30
    wing.append_control_surface(inboard_flap)
    

    flap                                  = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                              = 'flap'
    flap.span_fraction_start              = 0.4
    flap.span_fraction_end                = 0.8
    flap.deflection                       = 0.0 * Units.degrees
    flap.configuration_type               = 'double_slotted'
    flap.chord_fraction                   = 0.30
    wing.append_control_surface(flap) 

    aileron                               = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                           = 'aileron'
    aileron.span_fraction_start           = 0.8
    aileron.span_fraction_end             = 0.963
    aileron.deflection                    = 0.0 * Units.degrees
    aileron.chord_fraction                = 0.16
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

    wing                             = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag                         = 'horizontal_stabilizer' 
    wing.sweeps.leading_edge         = 38.6598082 * Units.degrees 
    wing.thickness_to_chord          = 0.12 
    wing.spans.projected             = 10.79076233 *2 
    wing.chords.root                 = 6.9423519 
    wing.chords.tip                  = 2.36018797  
    wing.taper                       = wing.chords.tip  / wing.chords.root
    wing.chords.mean_aerodynamic     = 6.942 /2 # Incorrect  
    wing.areas.reference             = ( wing.chords.root +  wing.chords.tip) *wing.spans.projected / 2   
    wing.areas.exposed               = 2*wing.areas.reference *0.9
    wing.areas.wetted                = 2*wing.areas.reference *0.9 
    wing.total_length                = wing.chords.root 
    wing.aspect_ratio                = (wing.spans.projected ** 2) /  wing.areas.reference 
    wing.twists.root                 = 1.0 * Units.degrees
    wing.twists.tip                  = 1.0 * Units.degrees  
    wing.percent_span_unexposed      = 6.2 / wing.spans.projected 
    wing.dihedral                    = 7.6 * Units.degrees 
    wing.origin                      = [[63.03027968  , 0  ,  2.1745719]] 
    wing.aerodynamic_center          = [63.03027968 + 0.25*wing.chords.root, 0 , 2.1745719]
    wing.vertical                    = False
    wing.xz_plane_symmetric          = True
    wing.dynamic_pressure_ratio      = 1.0
 
    # Wing Segments
    segment                        =  RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0.0
    segment.root_chord_percent     = 1.0
    segment.dihedral_outboard      = 7.6 * Units.degrees 
    segment.sweeps.leading_edge    = 38.6598082  * Units.degrees 
    segment.thickness_to_chord     = .12
    wing.append_segment(segment)

    segment                        =  RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.root_chord_percent     = wing.taper            
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = .12
    wing.append_segment(segment)
    
    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.1 
    elevator.span_fraction_end     = 0.9  
    elevator.deflection            = 0 
    elevator.chord_fraction        = 0.33  
    wing.append_control_surface(elevator)

    # add to vehicle
    vehicle.append_component(wing)
    

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------ 
    wing                         =  RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag                     =  'vertical_stabilizer' 
    wing.sweeps.leading_edge     = 48.37  * Units.degrees 
    wing.thickness_to_chord      = 0.12  
    wing.spans.projected         = 10.92735426 
    wing.chords.root             = 9.43826157 
    wing.chords.tip              = 2.36081054 
    wing.total_length            = 9.43826157  
    wing.areas.reference         = ( wing.chords.root +  wing.chords.tip) *wing.spans.projected / 2   
    wing.aspect_ratio            = (wing.spans.projected ** 2) /  wing.areas.reference 
    wing.taper                   = wing.chords.tip  / wing.chords.root
    wing.chords.mean_aerodynamic = 9.43826157 /2  
    wing.areas.exposed           = 2*wing.areas.reference *0.9
    wing.areas.wetted            = 2*wing.areas.reference *0.9  
    wing.percent_span_unexposed  = 6.2 / wing.spans.projected 
    wing.aspect_ratio            = (wing.spans.projected ** 2) /  wing.areas.reference 
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees 
    wing.origin                  = [[59.90743535, 0 , 2.45865471 ]]  
    wing.aerodynamic_center      = [59.90743535 , 0 , 2.45865471 +  0.25*wing.chords.root]  
    wing.vertical                = True
    wing.xz_plane_symmetric      = False
    wing.t_tail                  = False 
    wing.dynamic_pressure_ratio  = 1.0
     

    # Wing Segments
    segment                        =  RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0.0
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.leading_edge    = 48.37  * Units.degrees 
    segment.thickness_to_chord     = .12
    wing.append_segment(segment)

    segment                        =  RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = wing.taper            
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = .12
    wing.append_segment(segment) 
        

    # control surfaces -------------------------------------------
    rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                   = 'rudder'
    rudder.span_fraction_start   = 0.1 
    rudder.span_fraction_end     = 0.9  
    rudder.deflection            = 0 
    rudder.chord_fraction        = 0.33  
    wing.append_control_surface(rudder) 

    # add to vehicle
    vehicle.append_component(wing)    
    
    
    # ################################################# Fuselage ################################################################ 
        
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage() 
    fuselage.number_coach_seats                 = vehicle.number_of_passengers

    fuselage.seats_abreast                      = 6
    fuselage.seat_pitch                         = 1     * Units.meter 
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2. 
    fuselage.lengths.nose                       = 8.9  * Units.meter
    fuselage.lengths.tail                       = 19   * Units.meter
    fuselage.lengths.total                      = 73.08 * Units.meter  
    fuselage.lengths.fore_space                 = 6.    * Units.meter
    fuselage.lengths.aft_space                  = 5.    * Units.meter 
    fuselage.width                              = 6.2   * Units.meter
    fuselage.heights.maximum                    = 6.2  * Units.meter
    fuselage.effective_diameter                 = 6.2   * Units.meter
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
    segment.percent_x_location                  = 0.0 
    segment.percent_z_location                  = -4.5718274092838756e-05 
    segment.height                              = 0.0 
    segment.width                               = 0.0 
    fuselage.append_segment(segment)   
     

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_2'    
    segment.percent_x_location                  = 0.0006849315068493151 
    segment.percent_z_location                  = 0.00023201116822421539 
    segment.height                              = 0.506612607796538 
    segment.width                               = 0.29486232497524273 
    fuselage.append_segment(segment)
    

    ## Segment  
    #segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    #segment.tag                                 = 'segment_3'    
    #segment.percent_x_location                  = 0.0013698630136986301 
    #segment.percent_z_location                  = 0.0005097406105412695 
    #segment.height                              = 1.0105126311743573 
    #segment.width                               = 0.5860983155721387 
    #fuselage.append_segment(segment)
    

    ## Segment  
    #segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    #segment.tag                                 = 'segment_4'    
    #segment.percent_x_location                  = 0.0027397260273972603 
    #segment.percent_z_location                  = 0.00106519949517538 
    #segment.height                              = 1.1902153719637647 
    #segment.width                               = 0.9369298914718769 
    #fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_5'    
    segment.percent_x_location                  = 0.00410958904109589 
    segment.percent_z_location                  = 0.0015206583798094884 
    segment.height                              = 1.3699181127531718 
    segment.width                               = 1.1328207660995513 
    fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_6'    
    segment.percent_x_location                  = 0.00684931506849315 
    segment.percent_z_location                  = 0.0016650708841355363 
    segment.height                              = 1.7736138256504295 
    segment.width                               = 1.5246025153549005 
    fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_7'    
    segment.percent_x_location                  = 0.00958904109589041 
    segment.percent_z_location                  = 0.0014382635613239057 
    segment.height                              = 1.9377114437855858 
    segment.width                               = 1.8057694327532068 
    fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_8'    
    segment.percent_x_location                  = 0.0136986301369863 
    segment.percent_z_location                  = 0.002076051294030489 
    segment.height                              = 2.2895051006880056 
    segment.width                               = 2.1590037827079334 
    fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_9'    
    segment.percent_x_location                  = 0.0410958904109589 
    segment.percent_z_location                  = 0.0054206755627905015 
    segment.height                              = 4.137475539696899 
    segment.width                               = 3.7836714348574154 
    fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_10'    
    segment.percent_x_location                  = 0.0684931506849315 
    segment.percent_z_location                  = 0.007361830512116725 
    segment.height                              = 5.180529836017472 
    segment.width                               = 5.038003416722632 
    fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_11'    
    segment.percent_x_location                  = 0.0958904109589041 
    segment.percent_z_location                  = 0.008342605799103458 
    segment.height                              = 5.779624874430869 
    segment.width                               = 5.926942646043528 
    fuselage.append_segment(segment)

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_12'    
    segment.percent_x_location                  = 0.1095890410958904 
    segment.percent_z_location                  = 0.008852308594930632 
    segment.height                              = 5.936562615398982 
    segment.width                               = 6.198721376573906 
    fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_13'    
    segment.percent_x_location                  = 0.1232876712328767 
    segment.percent_z_location                  = 0.009418759029858625 
    segment.height                              = 6.019264378898468 
    segment.width                               = 6.283228699551572 
    fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_14'    
    segment.percent_x_location                  = 0.136986301369863 
    segment.percent_z_location                  = 0.009773342446019904 
    segment.height                              = 6.071033557658016 
    segment.width                               = 6.283228699551572 
    fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_15'    
    segment.percent_x_location                  = 0.15753424657534246 
    segment.percent_z_location                  = 0.010291172676447542 
    segment.height                              = 6.14663677130045 
    segment.width                               = 6.283228699551572 
    fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_16'    
    segment.percent_x_location                  = 0.7534246575342466
    segment.percent_z_location                  = 0.010385204131418418
    segment.height                              = 6.132908178874702
    segment.width                               = 6.283228699551572
    fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_17'    
    segment.percent_x_location                  =  0.7808219178082192
    segment.percent_z_location                  =  0.011034594257023024
    segment.height                              =  6.009141025548721
    segment.width                               =  6.179361462712953
    fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_18'    
    segment.percent_x_location                  =  0.7945205479452054
    segment.percent_z_location                  =  0.011784853626199271
    segment.height                              =  5.796015915164266
    segment.width                               =  6.063584270320871
    fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_19'    
    segment.percent_x_location                  = 0.8493150684931506
    segment.percent_z_location                  = 0.017830396113581042
    segment.height                              = 4.328719183639479
    segment.width                               = 5.218170689274808
    fuselage.append_segment(segment)
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_20'    
    segment.percent_x_location                  = 1.0
    segment.percent_z_location                  = 0.03287176935416379
    segment.height                              = 0.6643988043360767
    segment.width                               = 0.3578777817066474
    fuselage.append_segment(segment)       


    # add to vehicle
    vehicle.append_component(fuselage)


    # ################################################# Landing Gear #############################################################   
    # ------------------------------------------------------------------        
    #  Landing Gear
    # Source: https://www.boeing.com/content/dam/boeing/boeingdotcom/commercial/airports/acaps/787.pdf
    # ------------------------------------------------------------------  
    main_gear               = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.tire_diameter = 1.12000 * Units.m  
    main_gear.strut_length  = 1.8 * Units.m 
    main_gear.units         = 2    # Number of main landing gear
    main_gear.wheels        = 6    # Number of wheels on the main landing gear
    vehicle.append_component(main_gear)  

    nose_gear               = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()       
    nose_gear.tire_diameter = 0.6858 * Units.m  
    nose_gear.units         = 1    # Number of nose landing gear
    nose_gear.wheels        = 2    # Number of wheels on the nose landing gear
    nose_gear.strut_length  = 1.3 * Units.m 
    vehicle.append_component(nose_gear)
    
    # ################################################# Energy Network #######################################################         
    # Step 1: Define network
    # Step 2: Define Distribution Type
    # Step 3: Define Propulsors 
    # Step 4: Define Enegy Source 

    #------------------------------------------------------------------------------------------------------------------------- 
    #  Turbofan Network
    #-------------------------------------------------------------------------------------------------------------------------   
    net                                        = RCAIDE.Framework.Networks.Fuel() 
    
    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                  = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  
     
    #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #-------------------------------------------------------------------------------------------------------------------------  
    inboard_tank                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    inboard_tank.fuel                          = RCAIDE.Library.Attributes.Propellants.Jet_A()
    inboard_tank.segments_bounding_tank        = ['root','inboard']   
    fuel_line.fuel_tanks.append(inboard_tank)
    
    outboard_tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    outboard_tank.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A()
    outboard_tank.segments_bounding_tank       = ['inboard', 'outboard']  
    fuel_line.fuel_tanks.append(outboard_tank)    

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() 
    turbofan.tag                                = 'propulsor_1'
    turbofan.active_fuel_tanks                  = ['b777_fuel_tank']   
    turbofan.origin                             = [[ 25.72797886 , 9.69802 , -2.04  ]]
    turbofan.mass_properties.mass               = 7893
    turbofan.engine_length                      = 7.29
    turbofan.bypass_ratio                       = 8.1
    turbofan.diameter                           = 3.124
    turbofan.design_altitude                    = 39000.0 * Units.ft
    turbofan.design_mach_number                 = 0.84   
    turbofan.design_thrust                      = 70000 * Units.N  


    # fan                
    fan                                         = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                     = 'fan'
    fan.polytropic_efficiency                   = 0.98
    fan.pressure_ratio                          = 1.7   
    turbofan.fan                                = fan        

    # working fluid                   
    turbofan.working_fluid                      = RCAIDE.Library.Attributes.Gases.Air() 

    
    # Ram inlet 
    ram                                         = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                     = 'ram' 
    turbofan.ram                                = ram 
          
    # inlet nozzle          
    inlet_nozzle                                = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                            = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency          = 0.98
    inlet_nozzle.pressure_ratio                 = 0.98 
    turbofan.inlet_nozzle                       = inlet_nozzle 

    # low pressure compressor    
    low_pressure_compressor                        = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                    = 'lpc'
    low_pressure_compressor.polytropic_efficiency  = 0.98
    low_pressure_compressor.pressure_ratio         = 1.9   
    turbofan.low_pressure_compressor               = low_pressure_compressor 

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.98
    high_pressure_compressor.pressure_ratio        = 12.38 
    turbofan.high_pressure_compressor              = high_pressure_compressor
    

    # low pressure turbine  
    low_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                       ='lpt'
    low_pressure_turbine.mechanical_efficiency     = 0.98
    low_pressure_turbine.polytropic_efficiency     = 0.93 
    turbofan.low_pressure_turbine                  = low_pressure_turbine
   
    # high pressure turbine     
    high_pressure_turbine                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                      ='hpt'
    high_pressure_turbine.mechanical_efficiency    = 0.98
    high_pressure_turbine.polytropic_efficiency    = 0.93 
    turbofan.high_pressure_turbine                 = high_pressure_turbine 

    # combustor  
    combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                  = 'Comb'
    combustor.efficiency                           = 0.95 
    combustor.alphac                               = 1.0     
    combustor.turbine_inlet_temperature            = 1550
    combustor.pressure_ratio                       = 0.95
    combustor.volume                               = 0.0035         # [m**3] Combustor volume
    combustor.length                               = 0.2            # [m] Combustor Length
    combustor.number_of_combustors                 = 1              # [-] Number of Combustors for one engine
    combustor.F_SC                                 = 1              # [-] Fuel scale factor
    combustor.N_PZ                                 = 21             # [-] Number of PSR in the Primary Zone
    combustor.L_PZ                                 = 0.05           # [m] Primary Zone length  
    combustor.S_PZ                                 = 0.39           # [-] Mixing parameter in the Primary Zone  
    combustor.design_equivalence_ratio_PZ          = 1.71           # [-] Design Equivalence Ratio in Primary Zone at Maximum Throttle  
    combustor.N_SZ                                 = 500            # [-] Number of discritizations in the Secondary Zone
    combustor.f_SM                                 = 0.6            # [-] Slow mode fraction
    combustor.l_SA_SM                              = 0.4            # [-] Secondary air length fraction (of L_SZ) in slow mode
    combustor.l_SA_FM                              = 0.05           # [-] Secondary air length fraction (of L_SZ) in fast mode
    combustor.l_DA_start                           = 0.95           # [-] Dilution air start length fraction (of L_SZ)
    combustor.l_DA_end                             = 1.0            # [-] Dilution air end length fraction (of L_SZ)
    combustor.joint_mixing_fraction                = 0.6            # [-] Joint mixing fraction
    combustor.design_equivalence_ratio_SZ          = 0.61            # [-] Design Equivalence Ratio in Secondary Zone at Maximum Throttle
    combustor.air_mass_flow_rate_take_off          = 40             # [kg/s] Air mass flow rate at take-off

    combustor.air_data                             = RCAIDE.Library.Attributes.Gases.Air()
    if fuel == 'Jet_A1':
        combustor.fuel_to_air_ratio_take_off              = 0.025   # [-]
        combustor.fuel_data                               = RCAIDE.Library.Attributes.Propellants.Jet_A1()
        combustor.fuel_data.stoichiometric_fuel_air_ratio = 0.068
        combustor.fuel_data.heat_of_vaporization          = 360000
        combustor.fuel_data.fuel_surrogate_S1             = {'NC12H26':0.404, 'IC8H18':0.295, 'TMBENZ' : 0.073,'NPBENZ':0.228, 'C10H8':0.02}
        combustor.fuel_data.temperature                   = 298.15
        combustor.fuel_data.pressure                      = 101325
        combustor.fuel_data.kinetic_mechanism             = 'Fuel.yaml'
    elif fuel == 'Liquid_Hydrogen':
        combustor.fuel_to_air_ratio_take_off              = 0.00977 
        combustor.fuel_data                               = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
        combustor.fuel_data.stoichiometric_fuel_air_ratio = 0.02941
        combustor.fuel_data.heat_of_vaporization          = 447000
        combustor.fuel_data.fuel_surrogate_S1             = {'H2': 1}
        combustor.fuel_data.temperature                   = 20
        combustor.fuel_data.pressure                      = 101325
        combustor.fuel_data.kinetic_mechanism             = 'Fuel.yaml'
    elif fuel == 'Liquid_Petroleum_Gas':
        combustor.fuel_to_air_ratio_take_off              = 0.02144  
        combustor.fuel_data                               = RCAIDE.Library.Attributes.Propellants.Liquid_Petroleum_Gas()
        combustor.fuel_data.stoichiometric_fuel_air_ratio = 0.06451
        combustor.fuel_data.heat_of_vaporization          = 426200
        combustor.fuel_data.fuel_surrogate_S1             = {'C3H8':0.822, 'NC4H10':0.178}
        combustor.fuel_data.temperature                   = 298.15
        combustor.fuel_data.pressure                      = 810600
        combustor.fuel_data.kinetic_mechanism             = 'Fuel.yaml'
    elif fuel == 'Liquid_Natural_Gas':
        combustor.fuel_to_air_ratio_take_off              = 0.0193226
        combustor.fuel_data                               = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()
        combustor.fuel_data.stoichiometric_fuel_air_ratio = 0.058139
        combustor.fuel_data.heat_of_vaporization          = 510000
        combustor.fuel_data.fuel_surrogate_S1             = {'CH4':0.9, 'C2H6':0.05, 'C3H8':0.05}
        combustor.fuel_data.temperature                   = 111
        combustor.fuel_data.pressure                      = 101325
        combustor.fuel_data.kinetic_mechanism             = 'Fuel.yaml' 
    turbofan.combustor                                    = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.98
    core_nozzle.pressure_ratio                     = 0.99  
    turbofan.core_nozzle                           = core_nozzle
             
    # fan nozzle             
    fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                 = 'fan nozzle'
    fan_nozzle.polytropic_efficiency               = 0.98
    fan_nozzle.pressure_ratio                      = 0.99 
    turbofan.fan_nozzle                            = fan_nozzle 
    
    # design turbofan
    design_turbofan(turbofan)
    
    # Nacelle 
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                            = 3.124
    nacelle.length                              = 5.27571429 
    nacelle.tag                                 = 'nacelle_1'
    nacelle.inlet_diameter                      = 3.124
    nacelle.origin                              = [[26.72797886 , 9.69802 , -2.04 ]] 
    nacelle.areas.wetted                        = 1.1*np.pi*nacelle.diameter*nacelle.length
    nacelle_airfoil                             = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code          = '2410'
    nacelle.append_airfoil(nacelle_airfoil) 
    turbofan.nacelle                            = nacelle
    
    net.propulsors.append(turbofan)
    

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------      
    # copy turbofan
    turbofan_2                                  = deepcopy(turbofan)
    turbofan_2.active_fuel_tanks                = ['b777_fuel_tank'] 
    turbofan_2.tag                              = 'propulsor_2' 
    turbofan_2.origin                           = [[ 25.72797886 , -9.69802 , -2.04  ]]   # change origin 
    turbofan_2.nacelle.origin                   = [[26.72797886 , -9.69802 , -2.04 ]]  
    
    fuel_line.assigned_propulsors =  [[turbofan.tag, turbofan_2.tag]]

    # append propulsor to distribution line 
    net.propulsors.append(turbofan_2)
  
     #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to network      
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

    configs         = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config     = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base' 
    configs.append(base_config)



    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    config          = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag      = 'idle'     
    configs.append(config) 
    
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config          = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag      = 'cruise'
    configs.append(config)
     
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------ 
    config          = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag      = 'descent' 
    config.wings['main_wing'].control_surfaces.spoiler.deflection  = 45. * Units.deg    
    configs.append(config) 

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config          = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag      = 'takeoff'     
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity      = 3470. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      = 3470. * Units.rpm             
    config.networks.fuel.propulsors['propulsor_1'].fan.rotation              = 3470. #N1 speed
    config.networks.fuel.propulsors['propulsor_2'].fan.rotation              = 3470. #N1 speed
    config.networks.fuel.propulsors['propulsor_1'].fan_nozzle.noise_speed    = 315.
    config.networks.fuel.propulsors['propulsor_2'].fan_nozzle.noise_speed    = 315.
    config.networks.fuel.propulsors['propulsor_1'].core_nozzle.noise_speed   = 415.
    config.networks.fuel.propulsors['propulsor_2'].core_nozzle.noise_speed   = 415. 
    
    config.V2_VS_ratio = 1.21
    configs.append(config)


    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config          = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag      = 'cutback'
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity    =  2780. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity    =  2780. * Units.rpm       
    config.networks.fuel.propulsors['propulsor_1'].fan.rotation            = 2780.  
    config.networks.fuel.propulsors['propulsor_2'].fan.rotation            = 2780. 
    config.networks.fuel.propulsors['propulsor_1'].fan_nozzle.noise_speed  = 210.
    config.networks.fuel.propulsors['propulsor_2'].fan_nozzle.noise_speed  = 210.
    config.networks.fuel.propulsors['propulsor_1'].core_nozzle.noise_speed = 360.
    config.networks.fuel.propulsors['propulsor_2'].core_nozzle.noise_speed = 360.
    
    configs.append(config)   



    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config     = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity      =  2780. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  2780. * Units.rpm
    config.landing_gears.main_gear.gear_extended                             = True
    config.landing_gears.nose_gear.gear_extended                             = True  
    # config.networks.fuel.propulsors['propulsor_1'].emission_indices.NOx    = 11.02/1000              
    # config.networks.fuel.propulsors['propulsor_2'].emission_indices.NOx    = 11.02 /1000      
    config.networks.fuel.propulsors['propulsor_1'].fan.rotation              = 2780.  
    config.networks.fuel.propulsors['propulsor_2'].fan.rotation              = 2780. 
    config.networks.fuel.propulsors['propulsor_1'].core_nozzle.noise_speed   = 92.
    config.networks.fuel.propulsors['propulsor_2'].core_nozzle.noise_speed   = 92.
    config.networks.fuel.propulsors['propulsor_1'].fan_nozzle.noise_speed    = 109.3
    config.networks.fuel.propulsors['propulsor_2'].fan_nozzle.noise_speed    = 109.3 
    
    config.Vref_VS_ratio = 1.23
    configs.append(config)   

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config      = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag  = 'short_field_takeoff'    
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity      =  3470. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  3470. * Units.rpm 
    config.landing_gears.main_gear.gear_extended                             = True
    config.landing_gears.nose_gear.gear_extended                             = True  
    config.V2_VS_ratio = 1.21 
    configs.append(config)

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------  

    config      = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag  = 'reverse_thrust'
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    configs.append(config)
    
    return configs


# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses


def base_analysis(vehicle):
    """This is the baseline set of analyses to be used with this vehicle. Of these, the most
    commonly changed are the weights and aerodynamics methods."""

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle

    # ------------------------------------------------------------------
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    analyses.append(geometry)
    
    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_Transport()
    weights.aircraft_type                                                    =  "Transport"
    weights.settings.FLOPS.fidelity                                          = 'Complex'  
    weights.settings.weight_correction_additions.empty.structural.paint      = 450 
    weights.settings.weight_correction_additions.operational_items.ETOPS     = 7.7 * vehicle.number_of_passengers
    weights.settings.weight_correction_factors.empty.structural.landing_gear = 1.1   
    weights.settings.update_mass_properties                                  = True
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    # ------------------------------------------------------------------
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.number_of_spanwise_vortices           = 40
    aerodynamics.settings.number_of_chordwise_vortices          = 2 
    aerodynamics.settings.drag_reduction_factors.parasite_drag  = -0.10 #-0.25
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    # ------------------------------------------------------------------
    energy = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)
    

    # ------------------------------------------------------------------
    # Emissions 
    # ------------------------------------------------------------------
    emissions = RCAIDE.Framework.Analyses.Emissions.Emission_Index_CRN_Method() 
    emissions.settings.use_surrogate = False                 
    analyses.append(emissions)
    

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

    return analyses    


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.solver.type = 'root_finder'


    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------  
    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff_Ground_Run" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start                                           = 20.* Units.knots
    segment.velocity_end                                             = 167.0 * Units['knots']
    segment.friction_coefficient                                     = 0.03
    segment.altitude                                                 = 0.0  
    segment.throttle                                                 = 0.8
    mission.append_segment(segment)

     
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Takeoff_Climb" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_end                                             = 35 * Units['ft']
    segment.air_speed_end                                            = 175.0 * Units['knots']
    segment.climb_rate                                               = 500 * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment) 

    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Inital_Climb" 
    segment.analyses.extend( analyses.cutback )  
    segment.altitude_end                                             = 1500  * Units['feet']
    segment.air_speed                                                = 200.0 * Units['knots']
    segment.climb_rate                                               = 1800   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_1" 
    segment.analyses.extend( analyses.cutback )  
    segment.air_speed_start                                          = 200  * Units['knots']
    segment.altitude_end                                             = 8000 * Units['ft']
    segment.air_speed_end                                            = 300  * Units['knots']
    segment.climb_rate                                               = 800  * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)
    

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_4" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                             = 35000 * Units['ft']
    segment.air_speed                                                = 420   * Units['knots']
    segment.climb_rate                                               = 280   * Units['fpm']  
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)

    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                 = 35000 * Units['ft']  
    segment.air_speed                                                = 450   * Units['knots']
    segment.distance                                                 = 9200  * Units.km   
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.descent ) 
    segment.altitude_end                                             = 10000 * Units.ft
    segment.air_speed                                                = 300   * Units['knots']
    segment.descent_rate                                             = 1850  * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag  = "approach" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 0     * Units.ft
    segment.air_speed                                                = 225.0 * Units['knots']
    segment.descent_rate                                             = 650   * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)

    return mission


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def simplified_mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment() 
    base_segment.state.numerics.solver.type = 'root_finder' 

    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Inital_Climb" 
    segment.analyses.extend( analyses.cutback )  
    segment.altitude_start                                           = 0.0   * Units['knots'] 
    segment.altitude_end                                             = 1000  * Units['feet']
    segment.air_speed                                                = 200.0 * Units['knots']
    segment.climb_rate                                               = 1800  * Units['fpm']  
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_1" 
    segment.analyses.extend( analyses.cutback ) 
    segment.air_speed_start                                          = 200.0 * Units['knots'] 
    segment.altitude_end                                             = 8000  * Units['ft']
    segment.air_speed_end                                            = 300   * Units['knots']
    segment.climb_rate                                               = 1700  * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                             = 16000 * Units['ft']
    segment.air_speed                                                = 350   * Units['knots']
    segment.climb_rate                                               = 1300  * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)


    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_3" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                             = 35000 * Units['ft']
    segment.air_speed                                                = 450   * Units['knots']
    segment.climb_rate                                               = 1000  * Units['fpm']  
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment) 

    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                 = 35000 * Units['ft']  
    segment.air_speed                                                = 480   * Units['knots']
    segment.distance                                                 = 7370  * Units.km   
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.descent ) 
    segment.altitude_end                                             = 10000 * Units.ft
    segment.air_speed                                                = 380   * Units['knots']
    segment.descent_rate                                             = 1850  * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag  = "approach" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 0     * Units.ft
    segment.air_speed                                                = 225.0 * Units['knots']
    segment.descent_rate                                             = 650   * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)

 
    return mission


def approach_mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment() 
    base_segment.state.numerics.number_of_control_points = 10 
 

    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "final_approach"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_start                                           = 120.5   
    segment.altitude_end                                             = 10.0 * Units.ft
    segment.air_speed                                                = 160  * Units['knots']
    segment.descent_angle                                            = 3.   * Units.deg                               
           
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)
 
 
    return mission

def takeoff_mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points = 10  


    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff_Ground_Run" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start                                           = 20.   * Units.knots
    segment.velocity_end                                             = 167.0 * Units['knots']
    segment.friction_coefficient                                     = 0.03
    segment.altitude                                                 = 5.0   
    segment.throttle                                                 = 0.8
    mission.append_segment(segment)

     
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Takeoff_Climb" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_end                                             = 35    * Units['ft']
    segment.air_speed_end                                            = 175.0 * Units['knots']
    segment.climb_rate                                               = 1800  * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment) 

    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Inital_Climb" 
    segment.analyses.extend( analyses.cutback )  
    segment.altitude_end                                             = 1500  * Units['feet']
    segment.air_speed                                                = 200.0 * Units['knots']
    segment.climb_rate                                               = 1800  * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment)
 

    return mission


def LTO_mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment() 
    base_segment.state.numerics.number_of_control_points = 10 
  
    
    segment = Segments.Ground.Test_Stand(base_segment)
    segment.tag = "LTO_Takeoff_Setting" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity                                    = 157.0 * Units['knots']  
    segment.altitude                                    = 5.0   
    segment.throttle                                    = 1
    segment.time                                        = 0.7   * Units.minutes
    mission.append_segment(segment)
 
    
    segment = Segments.Ground.Test_Stand(base_segment)
    segment.tag = "LTO_Climb_Setting" 
    segment.analyses.extend( analyses.cutback )
    segment.velocity                                    = 250.0 * Units['knots']
    segment.altitude                                    = 5.0   
    segment.throttle                                    = 0.85
    segment.time                                        = 2.2   * Units.minutes
    mission.append_segment(segment)
 
    
    segment = Segments.Ground.Test_Stand(base_segment)
    segment.tag = "LTO_Approach_Setting" 
    segment.analyses.extend( analyses.landing )
    segment.velocity                                    = 160  * Units['knots']
    segment.altitude                                    = 5.0   
    segment.throttle                                    = 0.3
    segment.time                                        = 4    * Units.minutes
    mission.append_segment(segment)
 
    
    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "LTO_Taxi_Setting" 
    segment.analyses.extend( analyses.idle ) 
    segment.velocity                                    = 50  * Units['knots']
    segment.altitude                                    = 5.0   
    segment.throttle                                    = 0.07
    segment.time                                        = 26  * Units.minutes
    mission.append_segment(segment) 

    return mission    


def aviation_mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission     = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'aviation_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment                                         = Segments.Segment()
    base_segment.state.numerics.number_of_control_points = 2
    base_segment.state.numerics.solver.type              = 'root_finder'

    #------------------------------------------------------------------------------------------------------------------------------------ 
    #  Takeoff Ground Run
    #------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "takeoff_ground_run" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start                                           = 0.* Units.knots
    segment.velocity_end                                             = 180.0 * Units['knots']
    segment.friction_coefficient                                     = 0.03
    segment.altitude                                                 = 0.0   
    segment.throttle                                                 = 1.0
    mission.append_segment(segment)
     
    #------------------------------------------------------------------------------------------------------------------------------------
    #  Takeoff Climb
    #------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "takeoff_climb" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.air_speed_start                                          = 180.0 * Units['knots']
    segment.altitude_start                                           = 0.0   
    segment.altitude_end                                             = 35    * Units['ft']
    segment.air_speed_end                                            = 200.0 * Units['knots']
    segment.climb_rate                                               = 200   * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment) 

    #------------------------------------------------------------------
    #   Initial Climb
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "initial_climb" 
    segment.analyses.extend( analyses.cutback )  
    segment.altitude_end                                             = 1000  * Units['feet']
    segment.air_speed                                                = 230.0 * Units['knots']
    segment.climb_rate                                               = 1500  * Units['fpm']  
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Climb to Cruise
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_to_cruise" 
    segment.analyses.extend( analyses.cutback ) 
    segment.air_speed_start                                          = 230.0 * Units['knots'] 
    segment.altitude_end                                             = 40000 * Units['ft']
    segment.air_speed_end                                            = 350   * Units['knots']
    segment.climb_rate                                               = 1300  * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)

    # ------------------------------------------------------------------    
    #   Cruise
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                 = 40000 * Units['ft']  
    segment.air_speed                                                = 490   * Units['knots']
    segment.distance                                                 = 7500  * Units.km   
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #  Descent from Cruise
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_from_cruise" 
    segment.analyses.extend( analyses.descent ) 
    segment.altitude_end                                             = 10000 * Units.ft
    segment.air_speed                                                = 330   * Units['knots']
    segment.descent_rate                                             = 1500  * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #  Final Desccent
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag  = "final_descent" 
    segment.analyses.extend( analyses.landing )
    segment.altitude_start                                           = 10000 * Units.ft
    segment.altitude_end                                             = 2000  * Units.ft
    segment.air_speed                                                = 250.0 * Units['knots']
    segment.descent_rate                                             = 500   * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #  Approach
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "approach"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 50.0  * Units.ft
    segment.air_speed                                                = 190.0 * Units['knots']
    segment.descent_rate                                             = 500.0 * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #  Level Off
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "level_off" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 0   * Units.ft
    segment.air_speed                                                = 180 * Units['knots']
    segment.descent_rate                                             = 300 * Units['fpm']  
           
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Landing Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "landing_roll"

    segment.analyses.extend( analyses.reverse_thrust ) 
    segment.velocity_start                                                = 160.0 * Units['knots']
    segment.velocity_end                                                  = 10    * Units.knots 
    segment.friction_coefficient                                          = 0.4
    segment.altitude                                                      = 0.0   
    segment.assigned_control_variables.elapsed_time.active                = True  
    segment.assigned_control_variables.elapsed_time.initial_guess_values  = [[30.]]  
    mission.append_segment(segment)     

    return mission

def missions_setup(mission):
    """This allows multiple missions to be incorporated if desired, but only one is used here."""

    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)

    return missions

def save_aircraft_geometry(geometry,filename,save_dir): 

    # Create full path for pickle file
    pickle_file = os.path.join(save_dir, filename + '.pkl')

    # Save the file
    with open(pickle_file, 'wb') as file:
        pickle.dump(geometry, file) 
    return


def load_aircraft_geometry(filename,load_dir):
    # Create full path for pickle file
    pickle_file = os.path.join(load_dir, filename +'.pkl')

    # Load the file
    with open(pickle_file, 'rb') as file:
        results = pickle.load(file)
    return results

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------
def plot_mission(results):
    """This function plots the results of the mission analysis and saves those results to 
    png files."""

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

    plot_propulsor_throttles(results)
    
    plot_emissions(results)

    return

if __name__ == '__main__': 
    main()    
    plt.show()