# test_take_off_field_length.py
#
# Created: Dec 2024, M Clarke   

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# SUave Imports
import RCAIDE
from RCAIDE.Framework.Core   import Data,Units 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan             import design_turbofan 
from RCAIDE.Library.Methods.Performance.estimate_take_off_field_length import estimate_take_off_field_length

# package imports
import numpy as np
import pylab as plt 
import sys
import os
import numpy as np
from  copy import  deepcopy

# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def main():

    # ----------------------------------------------------------------------
    #   Main
    # ----------------------------------------------------------------------    
    vehicle = vehicle_setup()
    configs = configs_setup(vehicle)
    
    # --- Takeoff Configuration ---
    configuration = configs.takeoff
    configuration.wings['main_wing'].flaps_angle =  20. * Units.deg
    configuration.wings['main_wing'].slats_angle  = 25. * Units.deg
    analyses = RCAIDE.Framework.Analyses.Analysis.Container()
    analyses = base_analysis(vehicle)
    analyses.aerodynamics.settings.maximum_lift_coefficient_factor = 0.90

    # CLmax for a given configuration may be informed by user
    # configuration.maximum_lift_coefficient = 2.XX 
    w_vec                = np.linspace(40000.,52000.,10)
    engines              = (2,3,4)
    takeoff_field_length = np.zeros((len(w_vec),len(engines)))
    second_seg_clb_grad  = np.zeros((len(w_vec),len(engines))) 
    compute_clb_grad = 1 # flag for Second segment climb estimation
    
    for network in  configuration.networks:
        for propulsor in  network.propulsors: 
            baseline_propulsor = deepcopy(propulsor)
            
            # delete propulsor 
            del network.propulsors[propulsor.tag] 

        for fuel_line in  network.fuel_lines: 
            fuel_line.assigned_propulsors = []           
    
    for id_eng,engine_number in enumerate(engines):
        propulsor_list = []
        # append propulsors 
        for network in  configuration.networks:
            for i in  range(engine_number):
                propulsor =  deepcopy(baseline_propulsor)
                propulsor.tag = 'propulsor_' +  str(i+1)
                network.propulsors.append(propulsor)
                propulsor_list.append(propulsor.tag) 
                
            for fuel_line in  network.fuel_lines:  
                fuel_line.assigned_propulsors =  [propulsor_list]
                
        for id_w,weight in enumerate(w_vec):
            configuration.mass_properties.takeoff = weight
            takeoff_field_length[id_w,id_eng],second_seg_clb_grad[id_w,id_eng] =  estimate_take_off_field_length(configuration,analyses,compute_2nd_seg_climb = True)
    
        # delete propulsors again 
        for propulsor in  network.propulsors: 
            baseline_propulsor = deepcopy(propulsor) 
            del network.propulsors[propulsor.tag] 
    
        for fuel_line in  network.fuel_lines: 
            fuel_line.assigned_propulsors = []                        
    
    title = "TOFL vs W"
    plt.figure(1); 
    plt.plot(w_vec,takeoff_field_length[:,0], 'k-', label = '2 Engines')
    plt.plot(w_vec,takeoff_field_length[:,1], 'r-', label = '3 Engines')
    plt.plot(w_vec,takeoff_field_length[:,2], 'b-', label = '4 Engines')

    plt.title(title); plt.grid(True)
    plt.plot(w_vec,truth_TOFL[:,0], 'k--o', label = '2 Engines [truth]')
    plt.plot(w_vec,truth_TOFL[:,1], 'r--o', label = '3 Engines [truth]')
    plt.plot(w_vec,truth_TOFL[:,2], 'b--o', label = '4 Engines [truth]')
    legend = plt.legend(loc='lower right')
    plt.xlabel('Weight (kg)')
    plt.ylabel('Takeoff field length (m)')    
    
    title = "2nd Segment Climb Gradient vs W"
    plt.figure(2); 
    plt.plot(w_vec,second_seg_clb_grad[:,0], 'k-', label = '2 Engines')
    plt.plot(w_vec,second_seg_clb_grad[:,1], 'r-', label = '3 Engines')
    plt.plot(w_vec,second_seg_clb_grad[:,2], 'b-', label = '4 Engines')

    plt.title(title); plt.grid(True)
    plt.plot(w_vec,truth_clb_grad[:,0], 'k--o', label = '2 Engines [truth]')
    plt.plot(w_vec,truth_clb_grad[:,1], 'r--o', label = '3 Engines [truth]')
    plt.plot(w_vec,truth_clb_grad[:,2], 'b--o', label = '4 Engines [truth]')
    legend = plt.legend(loc='lower right')
    plt.xlabel('Weight (kg)')
    plt.ylabel('Second Segment Climb Gradient (%)')    
     

    return 

# ----------------------------------------------------------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------------------------------------------------------
def vehicle_setup():

    # ------------------------------------------------------------------
    #  File Paths 
    # ------------------------------------------------------------------    
    ospath             = os.path.abspath(__file__)
    separator          = os.path.sep
    rel_path           = os.path.dirname(ospath)   + separator
    airfoil_file_path  = rel_path + separator + 'Airfoils'
    polar_file_path    = rel_path + separator + 'Airfoils' + separator + 'Polars'
    
    #------------------------------------------------------------------------------------------------------------------------------------
    # ################################################# Vehicle-level Properties ########################################################  
    #------------------------------------------------------------------------------------------------------------------------------------
    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Embraer_E190AR'
 
    # mass properties (http://www.embraercommercialaviation.com/AircraftPDF/E190_Weights.pdf)
    vehicle.mass_properties.max_takeoff               = 51800. # kg
    vehicle.mass_properties.takeoff                   = 51800. # kg
    vehicle.mass_properties.max_zero_fuel             = 40900. # kg
    vehicle.mass_properties.max_fuel                  = 13100. # kg
    vehicle.mass_properties.max_payload               = 12900. # kg
    vehicle.mass_properties.operating_empty           = 27900  #    


    vehicle.mass_properties.center_of_gravity         = [[16.8, 0, 1.6]]
    vehicle.mass_properties.moments_of_inertia.tensor = [[10 ** 5, 0, 0],[0, 10 ** 6, 0,],[0,0, 10 ** 7]] 

    # envelope properties
    vehicle.flight_envelope.ultimate_load             = 3.5 
    vehicle.flight_envelope.ultimate_load             = 3.75
    vehicle.flight_envelope.positive_limit_load       = 2.5 
    vehicle.flight_envelope.design_mach_number        = 0.78 
    vehicle.flight_envelope.design_cruise_altitude    = 35000*Units.feet
    vehicle.flight_envelope.design_range              = 2000 * Units.nmi
    
    # basic parameters
    vehicle.reference_area                            = 92.
    vehicle.number_of_passengers                      = 112
    vehicle.systems.control                           = "fully powered"
    vehicle.systems.accessories                       = "medium range" 

    # ------------------------------------------------------------------
    # Carbo Bays 
    # ------------------------------------------------------------------ 
    forward_cargo_bay = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
    forward_cargo_bay.cargo.mass_properties.mass  = 1850
    forward_cargo_bay.origin                      = [[6.82, 0, -0.5]]
    forward_cargo_bay.length                      = 7.82
    forward_cargo_bay.width                       = 1.57  
    forward_cargo_bay.height                      = 0.88 
    vehicle.cargo_bays.append(forward_cargo_bay) 
 
    aft_cargo_bay  = RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
    aft_cargo_bay.cargo.mass_properties.mass  = 1440
    aft_cargo_bay.origin                      = [[23.43, 0, -0.5]]
    aft_cargo_bay.length                      =  5.5
    aft_cargo_bay.width                       =  1.57 
    aft_cargo_bay.height                      =  0.88 
    vehicle.cargo_bays.append(aft_cargo_bay)


    # ################################################# Landing Gear #############################################################   
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
    main_gear.origin                         = [[17.96, 0, 0]]
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
    nose_gear.origin                         = [[2.49, 0, 0]] 
    vehicle.append_component(nose_gear)
    

    #------------------------------------------------------------------------------------------------------------------------------------
    # ######################################################## Wings ####################################################################  
    #------------------------------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
    wing                         = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                     = 'main_wing'
    wing.areas.reference         = 92.0
    wing.aspect_ratio            = 8.4
    wing.chords.root             = 6.2
    wing.chords.tip              = 1.44
    wing.sweeps.quarter_chord    = 23.0 * Units.deg
    wing.thickness_to_chord      = 0.1
    wing.taper                   = 0.28
    wing.dihedral                = 5.00 * Units.deg
    wing.spans.projected         = 28.72
    wing.origin                  = [[13.3,0,-1.]]
    wing.vertical                = False
    wing.xz_plane_symmetric      = True       
    wing.high_lift               = True
    wing.areas.exposed           = 0.80 * wing.areas.wetted        
    wing.twists.root             = 2.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 1.0
      
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0 
    segment.root_chord_percent            = 1.
    segment.thickness_to_chord            = .11
    segment.dihedral_outboard             = 5. * Units.degrees
    segment.sweeps.quarter_chord          = 20.6 * Units.degrees  
    root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    root_airfoil.coordinate_file          = airfoil_file_path + 'transonic_wing_root_section_airfoil.txt'
    segment.append_airfoil(root_airfoil)
    wing.segments.append(segment)    
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'yehudi'
    segment.percent_span_location         = 0.348
    segment.root_chord_percent            = 0.60 
    segment.dihedral_outboard             = 4 * Units.degrees
    segment.sweeps.quarter_chord          = 24.1 * Units.degrees  
    yehudi_airfoil                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    yehudi_airfoil.coordinate_file        = airfoil_file_path + 'transonic_wing_inboard_section_airfoil.txt' 
    segment.append_airfoil(yehudi_airfoil)
    wing.segments.append(segment)
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                          = 'section_2'
    segment.percent_span_location        = 0.961
    segment.root_chord_percent           = 0.25 
    segment.dihedral_outboard            = 70. * Units.degrees
    segment.sweeps.quarter_chord         = 40. * Units.degrees  
    mid_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    mid_airfoil.coordinate_file          = airfoil_file_path + 'transonic_wing_outboard_section_airfoil.txt'
    segment.append_airfoil(mid_airfoil)
    wing.segments.append(segment)

    segment = RCAIDE.Library.Components.Wings.Segments.Segment() 
    segment.tag                          = 'Tip'
    segment.percent_span_location        = 1.
    segment.root_chord_percent           = 0.070 
    segment.dihedral_outboard            = 0.
    segment.sweeps.quarter_chord         = 0.  
    tip_airfoil                          =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file          = airfoil_file_path + 'transonic_wing_tip_section_airfoil.txt'
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
    wing.areas.reference         = 26.0
    wing.chords.root             = 3.6
    wing.chords.tip              = 0.72
    wing.aspect_ratio            = 5.5
    wing.spans.projected         = 12
    wing.sweeps.quarter_chord    = 34.5 * Units.deg
    wing.thickness_to_chord      = 0.16
    wing.taper                   = 0.2
    wing.dihedral                = 8.4 * Units.degrees
    wing.origin                  = [[31,0,1.5]]
    wing.vertical                = False
    wing.xz_plane_symmetric      = True       
    wing.high_lift               = False   
    wing.areas.exposed           = 0.9 * wing.areas.wetted 
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 0.90

    # add to vehicle
    vehicle.append_component(wing)     

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'
    wing.areas.reference         = 16.0
    wing.aspect_ratio            = 1.7
    wing.spans.projected         = 5.21
    wing.sweeps.quarter_chord    = 35. * Units.deg
    wing.chords.root             = 4.68
    wing.chords.tip              = 1.45
    wing.thickness_to_chord      = 0.16
    wing.taper                   = 0.31
    wing.dihedral                = 0.00
    wing.origin                  = [[30.4,0,1.675]]
    wing.vertical                = True
    wing.xz_plane_symmetric      = False       
    wing.high_lift               = False 
    wing.areas.exposed           = 0.9 * wing.areas.wetted
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees    
    wing.dynamic_pressure_ratio  = 1.00
    
    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage                                          = RCAIDE.Library.Components.Fuselages.Fuselage() 
    fuselage.origin                                   = [[0,0,0]] 
    fuselage.fineness.nose                      = 1.28
    fuselage.fineness.tail                      = 3.48 
    fuselage.lengths.nose                       = 6.0
    fuselage.lengths.tail                       = 9.0
    fuselage.lengths.cabin                      = 21.24
    fuselage.lengths.total                      = 36.24  
    fuselage.width                              = 3.01 * Units.meters 
    fuselage.heights.maximum                    = 3.35    
    fuselage.heights.at_quarter_length          = 3.35 
    fuselage.heights.at_three_quarters_length   = 3.35 
    fuselage.heights.at_wing_root_quarter_chord = 3.35  
    fuselage.areas.side_projected               = 239.20
    fuselage.areas.wetted                       = 327.01
    fuselage.areas.front_projected              = np.pi * (fuselage.heights.maximum  / 2) ** 2
    fuselage.effective_diameter                 = 3.18 
    fuselage.differential_pressure              = 10**5 * Units.pascal    # Maximum differential pressure  

    cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Cabin() 
    economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 4
    economy_class.number_of_rows                      = 28
    economy_class.number_of_passengers                = 106
    economy_class.galley_lavatory_percent_x_locations = [0, 1]      
    economy_class.emergency_exit_percent_x_locations  = [0.5, 0.5]      
    economy_class.type_A_exit_percent_x_locations     = [0, 1]
    cabin.origin                                      = [[4.65, 0, 0]]
    cabin.append_cabin_class(economy_class)
    fuselage.append_cabin(cabin) 
    

    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = -0.00144 
    segment.height                              = 0.0100 
    segment.width                               = 0.0100  
    fuselage.segments.append(segment)   
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.00576 
    segment.percent_z_location                  = -0.00144 
    segment.height                              = 0.7500
    segment.width                               = 0.6500
    fuselage.segments.append(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.02017 
    segment.percent_z_location                  = 0.00000 
    segment.height                              = 1.52783 
    segment.width                               = 1.20043 
    fuselage.segments.append(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.03170 
    segment.percent_z_location                  = 0.00000 
    segment.height                              = 1.96435 
    segment.width                               = 1.52783 
    fuselage.segments.append(segment)   

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.04899 	
    segment.percent_z_location                  = 0.00431 
    segment.height                              = 2.72826 
    segment.width                               = 1.96435 
    fuselage.segments.append(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.07781 
    segment.percent_z_location                  = 0.00861 
    segment.height                              = 3.49217 
    segment.width                               = 2.61913 
    fuselage.segments.append(segment)     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.10375 
    segment.percent_z_location                  = 0.01005 
    segment.height                              = 3.70130 
    segment.width                               = 3.05565 
    fuselage.segments.append(segment)             
     
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.16427 
    segment.percent_z_location                  = 0.01148 
    segment.height                              = 3.92870 
    segment.width                               = 3.71043 
    fuselage.segments.append(segment)    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.22478 
    segment.percent_z_location                  = 0.01148 
    segment.height                              = 3.92870 
    segment.width                               = 3.92870 
    fuselage.segments.append(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'     
    segment.percent_x_location                  = 0.69164 
    segment.percent_z_location                  = 0.01292
    segment.height                              = 3.81957
    segment.width                               = 3.81957
    fuselage.segments.append(segment)     
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_10'     
    segment.percent_x_location                  = 0.71758 
    segment.percent_z_location                  = 0.01292
    segment.height                              = 3.81957
    segment.width                               = 3.81957
    fuselage.segments.append(segment)   
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'     
    segment.percent_x_location                  = 0.78098 
    segment.percent_z_location                  = 0.01722
    segment.height                              = 3.49217
    segment.width                               = 3.71043
    fuselage.segments.append(segment)    
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_12'     
    segment.percent_x_location                  = 0.85303
    segment.percent_z_location                  = 0.02296
    segment.height                              = 3.05565
    segment.width                               = 3.16478
    fuselage.segments.append(segment)             
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_13'     
    segment.percent_x_location                  = 0.91931 
    segment.percent_z_location                  = 0.03157
    segment.height                              = 2.40087
    segment.width                               = 1.96435
    fuselage.segments.append(segment)               
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_14'     
    segment.percent_x_location                  = 1.00 
    segment.percent_z_location                  = 0.04593
    segment.height                              = 1.09130
    segment.width                               = 0.21826
    fuselage.segments.append(segment)       

    # add to vehicle
    vehicle.append_component(fuselage)  
  
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Fuel Network
    #------------------------------------------------------------------------------------------------------------------------------------  
    #initialize the fuel network
    net                                         = RCAIDE.Framework.Networks.Fuel() 
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------------------  
    fuel_line                                   = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line() 
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Fuel Tank & Fuel
    #------------------------------------------------------------------------------------------------------------------------------------   
    inboard_tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    inboard_tank.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A()
    inboard_tank.segments_bounding_tank       = ['root','yehudi']  
    inboard_tank.segments_percent_chord_start = [0.15  ,0.15 ]
    inboard_tank.segments_percent_chord_end   = [0.625 ,0.625]  
    fuel_line.fuel_tanks.append(inboard_tank)
    
    outboard_tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    outboard_tank.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A()
    outboard_tank.segments_bounding_tank       = ['yehudi', 'section_2']  
    outboard_tank.segments_percent_chord_start = [0.15 ,0.15 ]
    outboard_tank.segments_percent_chord_end   = [0.625,0.625]  
    fuel_line.fuel_tanks.append(outboard_tank)    

    fuel_tank                                   = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank()  
    fuel_tank.lengths.external                      = 2 
    fuel_tank.widths.external                       = 2
    fuel_tank.geometry_type                     = 'prismatic'   
    fuel_tank.heights.external                      = 0.5
    fuel_tank.origin                            = [[15.0,0.0, 0.0]]
    fuel_tank.fuel                              = RCAIDE.Library.Attributes.Propellants.Jet_A()
    fuel_tank.fuel.origin                       = [[15.0,0.0, 0.0]]
    fuel_line.fuel_tanks.append(fuel_tank)     
    

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------    
    turbofan                                        = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() 
    turbofan.tag                                    = 'starboard_propulsor'  
    turbofan.length                                 = 145 *  Units.inches
    turbofan.bypass_ratio                           = 5.4   
    turbofan.diameter                               = 53 *  Units.inches
    turbofan.design_altitude                        = 35000.0*Units.ft
    turbofan.design_mach_number                     = 0.8   
    turbofan.design_thrust                          = 35000.0* Units.N#/2 
    turbofan.origin                                 = [[13.15,4.38,-2.1]]
    turbofan.mass_properties.center_of_gravity      = [[turbofan.length /2,0,0]]
     
    # Nacelle 
    nacelle                                         = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                                = 2.05
    nacelle.length                                  = 2.66
    nacelle.tag                                     = 'nacelle_1'
    nacelle.inlet_diameter                          = 2.0
    nacelle.origin                                  = [[12.15,4.38, -2.1]] 
    nacelle.mass_properties.center_of_gravity       = [[nacelle.length /2,0,0 ]]
    nacelle.areas.wetted                            = 1.1*np.pi*nacelle.diameter*nacelle.length
    nacelle_airfoil                                 = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code              = '2410'
    nacelle.append_airfoil(nacelle_airfoil)
    turbofan.nacelle                                = nacelle
                  
    # fan                     
    fan                                             = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                         = 'fan'
    fan.polytropic_efficiency                       = 0.93
    fan.pressure_ratio                              = 1.7   
    turbofan.fan                                    = fan        
                        
    # working fluid                        
    turbofan.working_fluid                          = RCAIDE.Library.Attributes.Gases.Air() 
    ram                                             = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                         = 'ram' 
    turbofan.ram                                    = ram 
               
    # inlet nozzle               
    inlet_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                                = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency              = 0.98
    inlet_nozzle.pressure_ratio                     = 0.98 
    turbofan.inlet_nozzle                           = inlet_nozzle


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
    combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                  = 'Comb'
    combustor.efficiency                           = 0.99 
    combustor.alphac                               = 1.0     
    combustor.turbine_inlet_temperature            = 1550
    combustor.pressure_ratio                       = 0.95
    combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    turbofan.combustor                             = combustor
           
    # core nozzle           
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.95
    core_nozzle.pressure_ratio                     = 0.99  
    core_nozzle.diameter                           = 0.92    
    turbofan.core_nozzle                           = core_nozzle
          
    # fan nozzle          
    fan_nozzle                                  = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                              = 'fan nozzle'
    fan_nozzle.polytropic_efficiency            = 0.95
    fan_nozzle.pressure_ratio                   = 0.99 
    fan_nozzle.diameter                         = 1.659
    turbofan.fan_nozzle                         = fan_nozzle 
    
    # design turbofan
    design_turbofan(turbofan)  
    
    # append propulsor to network
    net.propulsors.append(turbofan)


    #------------------------------------------------------------------------------------------------------------------------------------  
    # Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------     
    
    # copy turbofan
    turbofan_2                             = deepcopy(turbofan)
    turbofan_2.tag                         = 'port_propulsor'  
    turbofan_2.origin                      = [[13.15,-4.38,-2.1]]  # change origin  
    turbofan_2.nacelle.origin              = [[12.15,-4.38,-2.1]]   
    
    # append propulsor to network
    net.propulsors.append(turbofan_2)

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line    
    fuel_line.assigned_propulsors =  [[turbofan.tag, turbofan_2.tag]]

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network      
    net.fuel_lines.append(fuel_line)        
    
    # Append energy network to aircraft 
    vehicle.append_energy_network(net)     
        
    return vehicle

# ---------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
 
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'   
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cruise' 
    configs.append(config)


    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    config.wings['main_wing'].control_surfaces.flap.deflection                       = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection                       = 25. * Units.deg  
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True 
    configs.append(config)

    
    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    config.wings['main_wing'].control_surfaces.flap.deflection                       = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection                       = 20. * Units.deg 
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True 
    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Descent Configuration
    # ------------------------------------------------------------------ 
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'descent' 
    config.wings['main_wing'].control_surfaces.spoiler.deflection  = 45. * Units.deg     
    configs.append(config)  
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg 
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True 
    configs.append(config)   
     
    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------  
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'    
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg 
    for landing_gear in  config.landing_gears:
        landing_gear.gear_extended = True 
    configs.append(config)    

    # done!
    return configs

def base_analysis(vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    analyses.vehicle = vehicle 

    # ------------------------------------------------------------------   
    #  Aerodynamics Analysis
    aerodynamics         = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Weights Analysis
    weights         = RCAIDE.Framework.Analyses.Weights.Conventional_Transport() 
    analyses.append(weights)    
    
    # ------------------------------------------------------------------
    #  Energy Analysis
    energy         = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)
    
    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)    
    
    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)     
    
    # done!
    return analyses     

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()
    plt.show()