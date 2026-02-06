# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units   
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan    import design_turbofan 
from RCAIDE.Library.Plots                 import *      
from RCAIDE.Library.Methods.Performance   import *  

# python imports 
import numpy as np   
from copy import deepcopy 
import sys 
import os

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    
    # Step 1: design a vehicle
    vehicle  = vehicle_setup()  

    try:
        import vsp as vsp
        from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle 
        export_vsp_vehicle(vehicle, 'Boeing_737_800')
    except ImportError:
        pass
       
    
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,export_gltf=True)  
    
    return 
    

def vehicle_setup(): 
    local_path = sys.path[0] + os.sep
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    
    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Boeing_737-800'

    # ################################################# Vehicle-level Properties #################################################   
    vehicle.mass_properties.max_takeoff               = 79000 * Units.kilogram    
    vehicle.mass_properties.takeoff                   = 79000 * Units.kilogram   
    vehicle.mass_properties.max_fuel                  = 70000 * Units.lbs  
    vehicle.mass_properties.operating_empty           = 41412.0 * Units.lbs
    vehicle.mass_properties.max_payload               = 21000.  * Units.kilogram  
    vehicle.mass_properties.payload                   = 0.5*vehicle.mass_properties.max_payload
    vehicle.mass_properties.fuel                      = 0.5*vehicle.mass_properties.max_fuel
    vehicle.mass_properties.max_zero_fuel             = 62732 * Units.kilogram
    vehicle.mass_properties.center_of_gravity         = [[21.0,0, 0]]
    vehicle.flight_envelope.ultimate_load             = 3.75
    vehicle.flight_envelope.positive_limit_load       = 2.5 
    vehicle.flight_envelope.design_mach_number        = 0.78 
    vehicle.flight_envelope.design_cruise_altitude    = 35000*Units.feet
    vehicle.flight_envelope.design_range              = 3500 * Units.nmi
    vehicle.reference_area                            = 124.862 * Units['meters**2']   
    vehicle.number_of_passengers                      = 166
    vehicle.systems.control                           = "fully powered" 
    vehicle.systems.accessories                       = "medium range"

    # ################################################# Landing Gear #############################################################   
    # ------------------------------------------------------------------        
    #  Landing Gear
    # ------------------------------------------------------------------  
    main_gear                                 = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter                   = 44.5 *  Units.inches 
    main_gear.rim_diameter                    = 21   *  Units.inches 
    main_gear.tire_width                      = 16.5  *  Units.inches 
    main_gear.strut_length                    = 1.8  * Units.m
    main_gear.origin                          = [[18,0, -2]]  
    main_gear.wheels                          = 4   
    main_gear.number_of_gear_types_in_tandem  = 1
    main_gear.number_of_wheels_in_gear_type   = 2  
    main_gear.xz_plane_symmetric              = True
    vehicle.append_component(main_gear)  

    nose_gear                                 = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                   = 27    *  Units.inches   
    nose_gear.rim_diameter                    = 15    *  Units.inches 
    nose_gear.tire_width                      = 7.75  *  Units.inches 
    nose_gear.strut_length                    = 1.8   * Units.m  
    nose_gear.origin                          = [[5,0, -2]]  
    nose_gear.wheels                          = 2   
    nose_gear.number_of_gear_types_in_tandem  = 1
    nose_gear.number_of_wheels_in_gear_type   = 2    
    vehicle.append_component(nose_gear)
     

    # ################################################# Wings ##################################################################### 
    # ------------------------------------------------------------------
    # Carbo Bays 
    # ------------------------------------------------------------------ 
    forward_cargo_bay = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
    forward_cargo_bay.cargo.mass_properties.mass  = 1850
    forward_cargo_bay.origin                      = [[5.82, 0, -0.6]]
    forward_cargo_bay.length                      = 7.82
    forward_cargo_bay.width                       = 2.5
    forward_cargo_bay.height                      = 1.12
    vehicle.append_component(forward_cargo_bay) 
 
    aft_cargo_bay  = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
    aft_cargo_bay.cargo.mass_properties.mass  = 1440
    aft_cargo_bay.origin                      = [[23.43, 0, -0.6]]
    aft_cargo_bay.length                      =  5.5
    aft_cargo_bay.width                       =  2.5
    aft_cargo_bay.height                      =  1.12
    vehicle.append_component(aft_cargo_bay)


    # ################################################# Wings ##################################################################### 
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
 
    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing' 
    wing.aspect_ratio                     = 10.18
    wing.sweeps.quarter_chord             = 25 * Units.deg
    wing.thickness_to_chord               = 0.1
    wing.taper                            = 0.1 
    wing.spans.projected                  = 35.8
    wing.chords.root                      = 7.25 * Units.meter 
    wing.chords.tip                       = 0.782 * Units.meter
    wing.chords.mean_aerodynamic          = 4.235 * Units.meter 
    wing.areas.reference                  = 124.862
    wing.areas.wetted                     = 225.08  
    wing.origin                           = [[13.61,0,-0.5]]
    wing.aerodynamic_center               = [0,0,0] 
    wing.vertical                         = False
    wing.xz_plane_symmetric               = True
    wing.high_lift                        = True
    wing.twists.root                      = 2.5 * Units.degrees 
    wing.twists.tip                       = 3.5  * Units.degrees 
    wing.transition_x_upper               = 0.5
    wing.transition_x_lower               = 0.5
    

    # Wing Segments
    root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()    
    root_airfoil.coordinate_file          = local_path + 'transonic_wing_root_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Root'
    segment.percent_span_location         = 0.0 
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 3.5 * Units.degrees
    segment.twist                         = 2.5   * Units.degrees
    segment.sweeps.quarter_chord          = 28.225 * Units.degrees 
    segment.thickness_to_chord            = 0.11
    segment.append_airfoil(root_airfoil)
    wing.append_segment(segment)

    yehudi_airfoil                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    yehudi_airfoil.coordinate_file        = local_path + 'transonic_wing_inboard_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Yehudi'
    segment.percent_span_location         = 0.324 
    segment.root_chord_percent            = 0.5
    segment.twist                         = 2.5  * Units.degrees
    segment.dihedral_outboard             = 5.5 * Units.degrees
    segment.sweeps.quarter_chord          = 25. * Units.degrees   
    segment.thickness_to_chord            = 0.12
    segment.append_airfoil(yehudi_airfoil)
    wing.append_segment(segment)

    mid_airfoil                           = RCAIDE.Library.Components.Airfoils.Airfoil()
    mid_airfoil.coordinate_file           = local_path + 'transonic_wing_outboard_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Section_2'
    segment.percent_span_location         = 0.963 
    segment.root_chord_percent            = 0.220  
    segment.dihedral_outboard             = 5.5 * Units.degrees
    segment.sweeps.quarter_chord          = 56.75 * Units.degrees 
    segment.twist                         = 0 * Units.degrees 
    segment.thickness_to_chord            = 0.13
    segment.append_airfoil(mid_airfoil)
    wing.append_segment(segment)

    tip_airfoil                           = RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = local_path +  'transonic_wing_tip_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Tip'
    segment.percent_span_location         = 1. 
    segment.root_chord_percent            = 0.10077
    segment.dihedral_outboard             = 0.
    segment.twist                         = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 56.75 * Units.degrees 
    segment.thickness_to_chord            = 0.14
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
    flap.chord_fraction           = 0.30
    wing.append_control_surface(flap)

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.7
    aileron.span_fraction_end     = 0.963
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.16
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

    wing                          = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag                      = 'horizontal_stabilizer'
    wing.aspect_ratio             = 4.99
    wing.sweeps.quarter_chord     = 28.2250 * Units.deg  
    wing.thickness_to_chord       = 0.08
    wing.taper                    = 0.3333  
    wing.spans.projected          = 14.35 
    wing.chords.root              = 4.1
    wing.chords.tip               = 1.4243 
    wing.chords.mean_aerodynamic  = 8.0 
    wing.areas.reference          = 41.49
    wing.areas.exposed            = 59.354     
    wing.areas.wetted             = 71.81      
    wing.twists.root              = 0.0 * Units.degrees
    wing.twists.tip               = 0.0 * Units.degrees 
    wing.origin                   = [[33.02,0,1.466]]
    wing.aerodynamic_center       = [0,0,0]  
    wing.vertical                 = False
    wing.xz_plane_symmetric       = True     


    # Wing Segments
    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0.0
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.dihedral_outboard      = 8.63 * Units.degrees
    segment.sweeps.quarter_chord   = 28.2250  * Units.degrees 
    segment.thickness_to_chord     = .14
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.3333               
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = .14
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
    wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag                              = 'vertical_stabilizer'
    wing.aspect_ratio                     = 2.00411
    wing.sweeps.quarter_chord             = 31.2  * Units.deg   
    wing.thickness_to_chord               = 0.08
    wing.taper                            = 0.1183 
    wing.spans.projected                  = 6.08916
    wing.total_length                     = wing.spans.projected  
    wing.chords.root                      = 6.7216
    wing.chords.tip                       = 1.20 
    wing.chords.mean_aerodynamic          = 4.0 
    wing.areas.reference                  = 34.89
    wing.areas.wetted                     = 57.25  
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[30.299,0,2.25]]
    wing.aerodynamic_center               = [0,0,0] 
    wing.vertical                         = True
    wing.xz_plane_symmetric               = False
    wing.t_tail                           = False 
    wing.dynamic_pressure_ratio           = 1.0
    wing.transition_x_upper               = 0.5
    wing.transition_x_lower               = 0.5

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tail_root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 65  * Units.degrees  
    segment.thickness_to_chord            =.15
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tail_segment_1'
    segment.percent_span_location         = 0.15
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.62
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.quarter_chord          = 32.771* Units.degrees   
    segment.thickness_to_chord            = .13
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tail_segment_2'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.23
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0    
    segment.thickness_to_chord            = .12 
    wing.append_segment(segment) 

    # control surfaces -------------------------------------------
    rudder                                = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                            = 'rudder'
    rudder.span_fraction_start            = 0.09
    rudder.span_fraction_end              = 0.92
    rudder.deflection                     = 0.0  * Units.deg
    rudder.chord_fraction                 = 0.3
    wing.append_control_surface(rudder)

    # add to vehicle
    vehicle.append_component(wing)

    # ################################################# Fuselage ################################################################ 
    
    fuselage                                           = RCAIDE.Library.Components.Fuselages.Fuselage()  
    fuselage.seats_abreast                             = 6
    fuselage.seat_pitch                                = 1     * Units.meter 
    fuselage.fineness.nose                             = 1.6
    fuselage.fineness.tail                             = 2. 
    fuselage.lengths.nose                              = 6.4   * Units.meter
    fuselage.lengths.tail                              = 8.0   * Units.meter
    fuselage.lengths.total                             = 38.02 * Units.meter  
    fuselage.lengths.fore_space                        = 6.    * Units.meter
    fuselage.lengths.aft_space                         = 5.    * Units.meter
    fuselage.width                                     = 3.76  * Units.meter
    fuselage.heights.maximum                           = 3.76  * Units.meter
    fuselage.effective_diameter                        = 3.76     * Units.meter
    fuselage.areas.side_projected                      = 142.1948 * Units['meters**2'] 
    fuselage.areas.wetted                              = 446.718  * Units['meters**2'] 
    fuselage.areas.front_projected                     = np.pi *(fuselage.effective_diameter **2) /4 
    fuselage.differential_pressure                     = 5.0e4 * Units.pascal 
    fuselage.heights.at_quarter_length                 = 3.74 * Units.meter
    fuselage.heights.at_three_quarters_length          = 3.65 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord        = 3.74 * Units.meter
    
    cabin                                              = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.origin                                       = [[5, 0, 0]]
    first_class                                        = RCAIDE.Library.Components.Fuselages.Cabins.Classes.First() 
    first_class.number_of_seats_abrest                 = 4
    first_class.number_of_rows                         = 4
    first_class.galley_lavatory_percent_x_locations    = [0]       
    first_class.type_A_exit_percent_x_locations        = [0.2]
    cabin.append_cabin_class(first_class) 

    business_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business() 
    business_class.number_of_seats_abrest              = 6
    business_class.number_of_rows                      = 3  
    cabin.append_cabin_class(business_class) 
    
    economy_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 6
    economy_class.number_of_rows                      = 18
    economy_class.galley_lavatory_percent_x_locations = [1]      
    economy_class.emergency_exit_percent_x_locations  = [0.1,0.15] 
    economy_class.type_A_exit_percent_x_locations     = [0.99]
    cabin.append_cabin_class(economy_class)
    
    fuselage.append_cabin(cabin)          
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = -0.00144
    fuselage.append_segment(segment)   
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.00576 
    segment.percent_z_location                  = -0.00144 
    segment.height                              = 0.7500
    segment.width                               = 0.6500
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.02017 
    segment.percent_z_location                  = -0.00050
    segment.height                              = 1.59978
    segment.width                               = 1.20043 
    fuselage.append_segment(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.03170 
    segment.percent_z_location                  = 0.00000 
    segment.height                              = 1.96435 
    segment.width                               = 1.52783 
    fuselage.append_segment(segment)   

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.04899 	
    segment.percent_z_location                  = 0.00431 
    segment.height                              = 2.72826 
    segment.width                               = 1.96435 
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.07781 
    segment.percent_z_location                  = 0.00861 
    segment.height                              = 3.49217 
    segment.width                               = 2.61913 
    fuselage.append_segment(segment)     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.10375 
    segment.percent_z_location                  = 0.01005 
    segment.height                              = 3.70130 
    segment.width                               = 3.05565 
    fuselage.append_segment(segment)             
     
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.16427 
    segment.percent_z_location                  = 0.01148 
    segment.height                              = 3.92870 
    segment.width                               = 3.71043 
    fuselage.append_segment(segment)    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Circle_Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.22478 
    segment.percent_z_location                  = 0.01148 
    segment.height                              = 3.92870 
    segment.width                               = 3.92870 
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Circle_Segment()
    segment.tag                                 = 'segment_9'     
    segment.percent_x_location                  = 0.69164 
    segment.percent_z_location                  = 0.01292
    segment.height                              = 3.81957
    segment.width                               = 3.81957
    fuselage.append_segment(segment)     
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Circle_Segment()
    segment.tag                                 = 'segment_10'     
    segment.percent_x_location                  = 0.71758 
    segment.percent_z_location                  = 0.01292
    segment.height                              = 3.81957
    segment.width                               = 3.81957
    fuselage.append_segment(segment)   
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'     
    segment.percent_x_location                  = 0.78098 
    segment.percent_z_location                  = 0.01722
    segment.height                              = 3.49217
    segment.width                               = 3.71043
    fuselage.append_segment(segment)    
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_12'     
    segment.percent_x_location                  = 0.85303
    segment.percent_z_location                  = 0.02296
    segment.height                              = 3.05565
    segment.width                               = 3.16478
    fuselage.append_segment(segment)             
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_13'     
    segment.percent_x_location                  = 0.91931 
    segment.percent_z_location                  = 0.03157
    segment.height                              = 2.40087
    segment.width                               = 1.96435
    fuselage.append_segment(segment)               
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_14'     
    segment.percent_x_location                  = 1.00 
    segment.percent_z_location                  = 0.04593
    segment.height                              = 1.09130
    segment.width                               = 0.21826
    fuselage.append_segment(segment)
    
    
    # add to vehicle
    vehicle.append_component(fuselage)
     

    # ################################################# Energy Network #######################################################          
    #------------------------------------------------------------------------------------------------------------------------- 
    #  Turbofan Network
    #-------------------------------------------------------------------------------------------------------------------------   
    net                                           = RCAIDE.Framework.Networks.Fuel() 
    
    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                      = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan                                       = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() 
    turbofan.tag                                   = 'starboard_propulsor' 
    turbofan.origin                                = [[13.72, 4.38,-1.5]] 
    turbofan.length                                = 2.71     
    turbofan.bypass_ratio                          = 5.4          
    turbofan.diameter                              = 1.5494
    turbofan.design_altitude                       = 35000.0*Units.ft
    turbofan.design_mach_number                    = 0.78   
    turbofan.design_thrust                         = 35000.0* Units.N 
                
    # fan                   
    fan                                            = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                        = 'fan'
    fan.polytropic_efficiency                      = 0.93
    fan.pressure_ratio                             = 1.7   
    turbofan.fan                                   = fan        
                   
    # working fluid                   
    turbofan.working_fluid                         = RCAIDE.Library.Attributes.Gases.Air() 
    ram                                            = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                        = 'ram' 
    turbofan.ram                                   = ram 
              
    # inlet nozzle              
    inlet_nozzle                                   = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                               = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency             = 0.98
    inlet_nozzle.pressure_ratio                    = 0.98 
    turbofan.inlet_nozzle                          = inlet_nozzle 
 
    # low pressure compressor     
    low_pressure_compressor                        = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                    = 'lpc'
    low_pressure_compressor.polytropic_efficiency  = 0.91
    low_pressure_compressor.pressure_ratio         = 1.9   
    turbofan.low_pressure_compressor               = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.91
    high_pressure_compressor.pressure_ratio        = 10.0    
    turbofan.high_pressure_compressor              = high_pressure_compressor

    # low pressure turbine  
    low_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                       ='lpt'
    low_pressure_turbine.mechanical_efficiency     = 0.99
    low_pressure_turbine.polytropic_efficiency     = 0.93 
    turbofan.low_pressure_turbine                  = low_pressure_turbine
   
    # high pressure turbine     
    high_pressure_turbine                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                      ='hpt'
    high_pressure_turbine.mechanical_efficiency    = 0.99
    high_pressure_turbine.polytropic_efficiency    = 0.93 
    turbofan.high_pressure_turbine                 = high_pressure_turbine 

    # combustor  
    combustor                                         = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                     = 'Comb'
    combustor.efficiency                              = 0.99 
    combustor.alphac                                  = 1.0     
    combustor.turbine_inlet_temperature               = 1600
    combustor.pressure_ratio                          = 0.95
    combustor.air_mass_flow_rate_take_off             = 40      
    combustor.air_data                                = RCAIDE.Library.Attributes.Gases.Air() 
    combustor.fuel_to_air_ratio_take_off              = 0.025    # [-]
    combustor.fuel_data                               = RCAIDE.Library.Attributes.Propellants.Jet_A1()
    combustor.fuel_data.stoichiometric_fuel_air_ratio = 0.068
    combustor.fuel_data.heat_of_vaporization          = 360000
    combustor.fuel_data.fuel_surrogate_S1             = {'NC12H26':0.404, 'IC8H18':0.295, 'TMBENZ' : 0.073,'NPBENZ':0.228, 'C10H8':0.02}
    combustor.fuel_data.temperature                   = 298.15
    combustor.fuel_data.pressure                      = 101325
    combustor.fuel_data.kinetic_mechanism             = 'Fuel.yaml' 
    turbofan.combustor                                = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.95
    core_nozzle.pressure_ratio                     = 0.99  
    turbofan.core_nozzle                           = core_nozzle
             
    # fan nozzle             
    fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                 = 'fan nozzle'
    fan_nozzle.polytropic_efficiency               = 0.95
    fan_nozzle.pressure_ratio                      = 0.99 
    turbofan.fan_nozzle                            = fan_nozzle 
    
    # design turbofan
    design_turbofan(turbofan)   
    
    # Nacelle 
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                            = 1.6
    nacelle.length                              = 2.71
    nacelle.tag                                 = 'starboard_nacelle'
    nacelle.inlet_diameter                      = 2.0
    nacelle.origin                              = [[13.5,4.38,-1.5]] 
    nacelle.areas.wetted                        = 1.1*np.pi*nacelle.diameter*nacelle.length 
    nacelle_airfoil                             = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code          = '2410'
    nacelle.append_airfoil(nacelle_airfoil)  
    turbofan.nacelle                            = nacelle

    # append propulsor to network    
    net.propulsors.append(turbofan)  

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    turbofan_2                                  = deepcopy(turbofan) 
    turbofan_2.tag                              = 'port_propulsor' 
    turbofan_2.origin                           = [[13.72,-4.38,-1.1]] 
    turbofan_2.nacelle.tag                      = 'port_nacelle'    
    turbofan_2.nacelle.origin                   = [[13.5,-4.38,-1.5]]
         
    # append propulsor to network
    net.propulsors.append(turbofan_2)
  
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
    fuel_line.assigned_propulsors =  [[turbofan.tag, turbofan_2.tag]]   

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network      
    net.fuel_lines.append(fuel_line)        
    
    # Append energy network to aircraft 
    vehicle.append_energy_network(net)
    
    #------------------------------------------------------------------------------------------------------------------------- 
    # Done ! 
    #-------------------------------------------------------------------------------------------------------------------------  
    return vehicle

if __name__ == '__main__': 
    main()