# RESEARCH/Aircraft/Boeing_787.py
#  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units           
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan    import design_turbofan   
from RCAIDE.Library.Plots                                     import *   

# python imports 
import numpy as np  
from copy import deepcopy 
import os
import sys 

# ----------------------------------------------------------------------
#   Main 
def main():
    
    # Step 1: design a vehicle
    vehicle  = vehicle_setup()  

    try:
        import vsp as vsp
        from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle 
        export_vsp_vehicle(vehicle, 'Boeing_787_8')
    except ImportError:
        pass 
    
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,export_gltf=True)  
    
    return 

def vehicle_setup() : 

    airfoil_file_path =  os.path.join(os.path.split(sys.path[0])[0], 'Airfoils_and_Polars') + os.sep 
    polar_file_path   =  os.path.join(os.path.join(os.path.split(sys.path[0])[0], 'Airfoils_and_Polars') , 'Polars') + os.sep  
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------      
    vehicle = RCAIDE.Vehicle()
    
    # ################################################# Vehicle-level Properties #################################################   
    vehicle.tag = 'Boeing_787_8' 
    vehicle.mass_properties.max_takeoff               = 227930
    vehicle.mass_properties.takeoff                   = 227930 
    vehicle.mass_properties.max_zero_fuel             = 161025.0 * Units.kilogram   
    vehicle.mass_properties.max_fuel                  = 101323 * Units.kilogram    
    vehicle.mass_properties.fuel                      = 57500 *Units.kilogram
    vehicle.mass_properties.max_payload               = 44000
    vehicle.mass_properties.center_of_gravity         = [[27.0, 0, 0]]
    vehicle.flight_envelope.ultimate_load             = 3.5
    vehicle.flight_envelope.positive_limit_load       = 2.5  
    vehicle.flight_envelope.negative_limit_load       = 1
    vehicle.flight_envelope.design_mach_number        = 0.85  
    vehicle.flight_envelope.design_cruise_altitude    = 35000.0*Units.feet 
    vehicle.flight_envelope.design_range              = 7305.0 * Units.nmi
    vehicle.reference_area                            = 395.0 * Units['meters**2']    
    vehicle.number_of_passengers                      = 248 
    vehicle.systems.control                           = "fully powered" 
    vehicle.systems.accessories                       = "long range" 

    # ################################################# Wings ##################################################################### 
    # ------------------------------------------------------------------
    # Carbo Bays 
    # ------------------------------------------------------------------ 
    forward_cargo_bay = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
    forward_cargo_bay.cargo.mass_properties.mass  = 1850
    forward_cargo_bay.origin                      = [[5.82, 0, -0.6]]
    forward_cargo_bay.length                      = 10
    forward_cargo_bay.width                       = 106 *  Units.inches 
    forward_cargo_bay.height                      = 67 *  Units.inches 
    vehicle.append_component(forward_cargo_bay) 
 
    aft_cargo_bay  = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
    aft_cargo_bay.cargo.mass_properties.mass     = 1440
    aft_cargo_bay.origin                         = [[30, 0, -0.6]]
    aft_cargo_bay.length                         =  10
    aft_cargo_bay.width                          =  106 *  Units.inches 
    aft_cargo_bay.height                         =  67 *  Units.inches 
    vehicle.append_component(aft_cargo_bay)

    # ------------------------------------------------------------------
    #   Main Wing 
    # ------------------------------------------------------------------

    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing'
    wing.aspect_ratio                     = 8.61650
    wing.sweeps.quarter_chord             = 35 * Units.deg
    wing.thickness_to_chord               = 0.115
    wing.spans.projected                  = 58.138 * Units.meter
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
    wing.dynamic_pressure_ratio           = 1.0
        
    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0 
    segment.root_chord_percent            = 1.
    segment.twist                         = 2.5 * Units.degrees
    segment.dihedral_outboard             = 8.0 * Units.degrees
    segment.sweeps.quarter_chord          = 27.57 * Units.degrees 
    root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil() 
    root_airfoil.coordinate_file          = airfoil_file_path+ 'transonic_wing_root_section_airfoil.txt'
    segment.append_airfoil(root_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'yehudi'
    segment.percent_span_location         = 0.345 
    segment.root_chord_percent            = 0.53 
    segment.dihedral_outboard             = 7.0 * Units.degrees
    segment.sweeps.quarter_chord          = 31. * Units.degrees
    segment.twist                         = 1.5 * Units.degrees 
    yehudi_airfoil                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    yehudi_airfoil.coordinate_file        = airfoil_file_path + 'transonic_wing_inboard_section_airfoil.txt'
    segment.append_airfoil(yehudi_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'section_2'
    segment.percent_span_location         = 0.95 
    segment.root_chord_percent            = 0.155 
    segment.dihedral_outboard             = 12.0 * Units.degrees
    segment.sweeps.quarter_chord          = 42.0 * Units.degrees
    segment.twist                         = 1.0 * Units.degrees 
    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = airfoil_file_path+ 'transonic_wing_outboard_section_airfoil.txt'
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.00
    segment.twist                         = 0.0 * Units.degrees
    segment.root_chord_percent            = 0.093 
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0 * Units.degrees 
    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = airfoil_file_path + 'transonic_wing_tip_section_airfoil.txt'
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
    segment.thickness_to_chord     = 0.14
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.237               
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = 0.14
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
    segment.thickness_to_chord            = 0.14
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 0.107
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.031
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.quarter_chord          = 63.8274 * Units.degrees   
    segment.thickness_to_chord            = 0.14
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_2'
    segment.percent_span_location         = 0.180
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.815
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 41.250 * Units.degrees    
    segment.thickness_to_chord            = 0.14
    wing.append_segment(segment)


    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_3'
    segment.percent_span_location         = 0.94
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.298
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 69.09 * Units.degrees    
    segment.thickness_to_chord            = 0.14
    wing.append_segment(segment)


    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_4'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.105
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0    
    segment.thickness_to_chord            = 0.14
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
    fuselage.number_coach_seats                 = vehicle.number_of_passengers 
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
    

    cabin                                           = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.origin                                    =  [[4.5, 0, 0.5]] 
    cabin.wide_body                                 = True     

    first_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.First() 
    first_class.number_of_seats_abrest              = 4
    first_class.number_of_rows                      = 7
    first_class.seat_width                          = 35 *  Units.inches
    first_class.seat_arm_rest_width                 = 3 *  Units.inches
    first_class.seat_length                         = 45 *  Units.inches
    first_class.seat_pitch                          = 50 *  Units.inches
    first_class.aisle_width                          = 18  *  Units.inches  
    first_class.galley_lavatory_percent_x_locations = [0, 1]       
    first_class.type_A_exit_percent_x_locations     = [0, 1]
    cabin.append_cabin_class(first_class) 

    business_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business() 
    business_class.number_of_seats_abrest              = 8
    business_class.number_of_rows                      = 4  
    business_class.seat_arm_rest_width                 = 4 *  Units.inches 
    business_class.seat_width                          = 17 *  Units.inches
    business_class.aisle_width                          = 18  *  Units.inches  
    cabin.append_cabin_class(business_class) 

    economy_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 9
    economy_class.number_of_rows                      = 26
    economy_class.galley_lavatory_percent_x_locations = [0.35, 1.0]       
    economy_class.type_A_exit_percent_x_locations     = [0.35, 1.0]
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
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan1                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()   
    turbofan1.origin                             = [[17.818, 10.000,-0.953 ]]
    turbofan1.tag                                = 'propulsor_1'    
    turbofan1.engine_length                      = 4.928                      
    turbofan1.engine_diameter                    = 2.822                  
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
    turbofan1.combustor                            = combustor

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
    fan_nozzle.diameter                            = 2.822   
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
    turbofan2.tag                              = 'propulsor_2' 
    turbofan2.origin                           = [[17.818, -10.000,-0.953]]
    turbofan2.nacelle.origin                   = [[17.818, -10.000,-0.953]]
        
    # append propulsor to distribution line 
    net.propulsors.append(turbofan2)
 
    #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #-------------------------------------------------------------------------------------------------------------------------  
    inboard_tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    inboard_tank.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A()
    inboard_tank.segments_bounding_tank       = ['root','yehudi']   
    fuel_line.fuel_tanks.append(inboard_tank)
    
    outboard_tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    outboard_tank.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A()
    outboard_tank.segments_bounding_tank       = ['yehudi', 'section_2']  
    fuel_line.fuel_tanks.append(outboard_tank)    
     
    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network      
    fuel_line.assigned_propulsors =  [['propulsor_1', 'propulsor_2']]
 
    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network      
    net.fuel_lines.append(fuel_line)        
 
    # Append energy network to aircraft 
    vehicle.append_energy_network(net)   
    
    return vehicle 
if __name__ == '__main__': 
    main()     