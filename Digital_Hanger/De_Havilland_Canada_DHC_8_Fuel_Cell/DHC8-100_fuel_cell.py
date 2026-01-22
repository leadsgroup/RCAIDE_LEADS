# RESEARCH/Aircraft/DHC8-100/DHC8-100_Hybrid.py
# 
# 
# Created:  Mar. 2025, M. Guidotti, A. Molloy

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports 
import RCAIDE
from   RCAIDE.Framework.Core   import Units           
from   RCAIDE.Library.Plots    import *       
from RCAIDE.Library.Methods.Thermal_Management.Heat_Exchangers.Cross_Flow_Heat_Exchanger  import design_cross_flow_heat_exchanger
from RCAIDE.Library.Methods.Thermal_Management.Batteries.Liquid_Cooled_Wavy_Channel       import design_wavy_channel 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor                          import design_electric_rotor  
from   RCAIDE.Library.Methods.Geometry.Planform                                           import segment_properties  

# python imports 
import numpy as np  
from   copy import deepcopy
import matplotlib.pyplot as plt  
import os

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main(): 

    vehicle  = vehicle_setup() 
    
    configs  = configs_setup(vehicle)
     
    analyses = analyses_setup(configs)

    missions = missions_setup(analyses) 

    results  = missions.base_mission.evaluate()   
    
    plot_mission(results)    

    # plot vehicle 
    plot_3d_vehicle(vehicle,
                    min_x_axis_limit            = -0,
                    max_x_axis_limit            = 30,
                    min_y_axis_limit            = -15,
                    max_y_axis_limit            = 15,
                    min_z_axis_limit            = -15,
                    max_z_axis_limit            = 15)     
    return

def vehicle_setup(): 

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------      

    # Sources: https://contentzone.eurocontrol.int/aircraftperformance/details.aspx?ICAO=DH8A&NameFilter=dash
    #          https://customer.aero.bombardier.com/webd/BAG/CustSite/BRAD/RACSDocument.nsf/51aae8b2b3bfdf6685256c300045ff31/ec63f8639ff3ab9d85257c1500635bd8/$FILE/ATT19ELL.pdf/D8100-APM.pdf
 
    vehicle                                        = RCAIDE.Vehicle()       
    vehicle.tag                                    = 'De_Havilland_DHC8-100'
    vehicle.mass_properties.max_takeoff            = 15650. * Units.kilogram 
    vehicle.mass_properties.takeoff                = 15650 * Units.kilogram 
    vehicle.mass_properties.operating_empty        = 10245 * Units.kilogram  
    vehicle.mass_properties.max_zero_fuel          = 14061.0 * Units.kilogram  
    vehicle.mass_properties.max_fuel               = 2576 * Units.kilogram  
    vehicle.mass_properties.payload                = 3814 *Units.kilogram  
    vehicle.mass_properties.center_of_gravity      = [[10.419, 0, 3.0]] 
    vehicle.flight_envelope.ultimate_load          = 3.75 
    vehicle.flight_envelope.positive_limit_load    = 2.5  
    vehicle.flight_envelope.negative_limit_load    = 0.75
    vehicle.flight_envelope.design_mach_number     = 0.38  
    vehicle.flight_envelope.design_cruise_altitude = 25000.0*Units.feet 
    vehicle.flight_envelope.design_range           = 1350.0 * Units.nmi
    vehicle.reference_area                         = 395.0 * Units['meters**2']    
    vehicle.number_of_passengers                             = 37 
    vehicle.systems.control                        = "fully powered" 
    vehicle.systems.accessories                    = "short range"

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Main Wing
    #------------------------------------------------------------------------------------------------------------------------------------
       
    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing'
    wing.aspect_ratio                     = 11.07
    wing.sweeps.quarter_chord             = 1 * Units.deg
    wing.thickness_to_chord               = 0.1
    wing.spans.projected                  = 25.57
    wing.chords.root                      = 2.75 * Units.meter
    wing.chords.tip                       = 1.39 * Units.meter
    wing.taper                            = wing.chords.tip / wing.chords.root
    wing.chords.mean_aerodynamic          = 2.41171 * Units.meter 
    wing.areas.reference                  = 59.06
    wing.areas.wetted                     = 124.026
    wing.twists.root                      = 0.0 * Units.degrees 
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[8.197,0,2.131]]
    wing.aerodynamic_center               = [0,0,0] 
    wing.vertical                         = False
    wing.dihedral                         = 0.5 * Units.degrees 
    wing.xz_plane_symmetric               = True 
    wing.high_lift                        = True 
    wing.dynamic_pressure_ratio           = 1.0
        
    # Wing Segments

    root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    rel_path                              = os.path.dirname(ospath) + separator  + '..'  + separator 
    root_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator + 'transonic_wing_root_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 0 * Units.degrees
    segment.append_airfoil(root_airfoil)
    wing.append_segment(segment)

    mid_airfoil                           = RCAIDE.Library.Components.Airfoils.Airfoil()
    mid_airfoil.coordinate_file           = rel_path+ 'Airfoils' + separator + 'transonic_wing_inboard_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Yehudi'
    segment.percent_span_location         = 0.35
    segment.twist                         = 0.0 * Units.deg
    segment.root_chord_percent            = 1
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 1.5 * Units.degrees
    segment.sweeps.quarter_chord          = 3.357 * Units.degrees
    segment.append_airfoil(mid_airfoil)
    wing.append_segment(segment)

    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = rel_path + 'Airfoils' + separator + 'transonic_wing_tip_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 0 * Units.degrees
    segment.root_chord_percent            = 0.5
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 0 * Units.degrees
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)
    
    # Fill out more segment properties automatically

    wing = segment_properties(wing) 

    # control surfaces
    
    flap                                  = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                              = 'flap'
    flap.span_fraction_start              = 0.15
    flap.span_fraction_end                = 0.8
    flap.deflection                       = 0.0 * Units.degrees
    flap.configuration_type               = 'double_slotted'
    flap.chord_fraction                   = 0.3
    wing.append_control_surface(flap)

    aileron                               = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                           = 'aileron'
    aileron.span_fraction_start           = 0.8
    aileron.span_fraction_end             = 1.0
    aileron.deflection                    = 0.0 * Units.degrees
    aileron.chord_fraction                = 0.25
    wing.append_control_surface(aileron)

    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing     = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio                     = 4.33
    wing.sweeps.quarter_chord             = 5.89 * Units.deg  
    wing.thickness_to_chord               = 0.1
    wing.taper                            = 0.746  
    wing.spans.projected                  = 8.14
    wing.chords.root                      = 2.15833
    wing.chords.tip                       = 1.6
    wing.chords.mean_aerodynamic          = 1.879
    wing.areas.reference                  = 15.3079
    wing.areas.exposed                    = 32.133    # Exposed area of the horizontal tail
    wing.areas.wetted                     = 32.133     # Wetted area of the horizontal tail
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[19.877, 0, 6.242]]
    wing.aerodynamic_center               = [0,0,0] 
    wing.vertical                         = False
    wing.xz_plane_symmetric               = True 
    wing.dynamic_pressure_ratio           = 0.9

    # Segments

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0.0
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.quarter_chord   = 5.8988 * Units.degrees 
    segment.thickness_to_chord     = .1
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.741              
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = .1
    wing.append_segment(segment)
    
    # Fill out more segment properties automatically

    wing = segment_properties(wing)          

    # control surfaces 

    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.05
    elevator.span_fraction_end     = 1.0
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.3
    wing.append_control_surface(elevator)

    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'

    wing.aspect_ratio            = 0.98439
    wing.sweeps.quarter_chord    = 32.5  * Units.deg   
    wing.thickness_to_chord      = 0.08
    wing.taper                   = 0.30

    wing.spans.projected         = 4.48
    wing.total_length            = wing.spans.projected 
    
    wing.chords.root             = 10.15
    wing.chords.tip              = 3.0 
    wing.chords.mean_aerodynamic = 5.215

    wing.areas.reference         = 20.46834
    wing.areas.wetted            = 43.0 
    
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[10.603, 0, 1.810]]
    wing.aerodynamic_center      = [0,0,0]

    wing.vertical                = True
    wing.xz_plane_symmetric      = False
    wing.t_tail                  = True

    wing.dynamic_pressure_ratio  = 1.0

    # Segments

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 75.0 * Units.degrees  
    segment.thickness_to_chord            = .05
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 0.318
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.3793
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.quarter_chord          = 32.53 * Units.degrees   
    segment.thickness_to_chord            = .1
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_2'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.2955
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0    
    segment.thickness_to_chord            = .1  
    wing.append_segment(segment)
      
    # control surfaces

    rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                   = 'rudder'
    rudder.span_fraction_start   = 0.1 
    rudder.span_fraction_end     = 1.0  
    rudder.deflection            = 0 
    rudder.chord_fraction        = 0.33  
    wing.append_control_surface(rudder)   
    
    # Fill out more segment properties automatically

    wing = segment_properties(wing)        

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Fuselage
    # ------------------------------------------------------------------

    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage() 
    fuselage.number_coach_seats                 = vehicle.number_of_passengers 
    fuselage.seats_abreast                      = 4
    fuselage.seat_pitch                         = 0.9 * Units.meter 
    fuselage.fineness.nose                      = 1.5
    fuselage.fineness.tail                      = 2.5 
    fuselage.lengths.nose                       = 2.9412 * Units.meter
    fuselage.lengths.tail                       = 8.24 * Units.meter
    fuselage.lengths.total                      = 20.78 * Units.meter  
    fuselage.lengths.fore_space                 = 0.75 * Units.meter
    fuselage.lengths.aft_space                  = 3 * Units.meter
    fuselage.width                              = 2.75 * Units.meter
    fuselage.heights.maximum                    = 2.65 * Units.meter
    fuselage.effective_diameter                 = 2.7 * Units.meter
    fuselage.areas.side_projected               = fuselage.heights.maximum * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.wetted                       = np.pi * fuselage.width/2 * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.front_projected              = np.pi * fuselage.width/2 * Units['meters**2']  
    fuselage.differential_pressure              = 5.0e4 * Units.pascal
    fuselage.heights.at_quarter_length          = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_three_quarters_length   = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = fuselage.heights.maximum * Units.meter
    
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
    segment.percent_x_location                  = 0.01147
    segment.percent_z_location                  = 0.00254
    segment.height                              = 0.53670
    segment.width                               = 0.63073
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.04508
    segment.percent_z_location                  = 0.01171
    segment.height                              = 1.22
    segment.width                               = 1.31881
    fuselage.append_segment(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.08302
    segment.percent_z_location                  = 0.02101
    segment.height                              = 1.75
    segment.width                               = 1.9454
    fuselage.append_segment(segment)   

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.10807
    segment.percent_z_location                  = 0.03048
    segment.height                              = 2.20183
    segment.width                               = 2.275
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.12066
    segment.percent_z_location                  = 0.03479
    segment.height                              = 2.4
    segment.width                               = 2.4
    fuselage.append_segment(segment)     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.14154
    segment.percent_z_location                  = 0.03901
    segment.height                              = 2.58
    segment.width                               = 2.6
    fuselage.append_segment(segment)             
     
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.17278
    segment.percent_z_location                  = 0.04064
    segment.height                              = 2.6789
    segment.width                               = 2.75
    fuselage.append_segment(segment)    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.55671
    segment.percent_z_location                  = 0.04268
    segment.height                              = 2.65
    segment.width                               = 2.75
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'     
    segment.percent_x_location                  = 0.60348
    segment.percent_z_location                  = 0.04123
    segment.height                              = 2.58
    segment.width                               = 2.75
    fuselage.append_segment(segment)     
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_10'     
    segment.percent_x_location                  = 0.80902
    segment.percent_z_location                  = 0.05797
    segment.height                              = 1.72477
    segment.width                               = 1.94954
    fuselage.append_segment(segment)   
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'     
    segment.percent_x_location                  = 0.95882
    segment.percent_z_location                  = 0.07021
    segment.height                              = 0.95413
    segment.width                               = 0.97477
    fuselage.append_segment(segment)    
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_12'     
    segment.percent_x_location                  = 0.99509
    segment.percent_z_location                  = 0.07511
    segment.height                              = 0.48695
    segment.width                               = 0.34404
    fuselage.append_segment(segment)             
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_13'     
    segment.percent_x_location                  = 1
    segment.percent_z_location                  = 0.08327
    segment.height                              = 0
    segment.width                               = 0
    fuselage.append_segment(segment)               
        
    # add to vehicle
    vehicle.append_component(fuselage)

    # ------------------------------------------------------------------
    #   Landing Gear
    # ------------------------------------------------------------------   

    main_gear               = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.tire_diameter = 0.5 * Units.meter
    main_gear.strut_length  = 1.9 * Units.meter 
    main_gear.units         = 1    # Number of main landing gear
    main_gear.wheels        = 2    # Number of wheels on the main landing gear
    vehicle.append_component(main_gear)  

    nose_gear               = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()       
    nose_gear.tire_diameter = 0.4 * Units.meter
    nose_gear.units         = 1    # Number of nose landing gear
    nose_gear.wheels        = 2    # Number of wheels on the nose landing gear
    nose_gear.strut_length  = 0.5 * Units.meter 
    vehicle.append_component(nose_gear)
    
    # ------------------------------------------------------------------
    #   Nacelles
    # ------------------------------------------------------------------

    nacelle                    = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    nacelle.tag                = 'nacelle_1'
    nacelle.length             = 6
    nacelle.diameter           = 1.178 
    nacelle.areas.wetted       = 0.01*(2*np.pi*0.01/2)
    nacelle.origin             = [[6.50,3.945 , 1.86]]
    nacelle.flow_through       = False  
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_0'
    nac_segment.percent_x_location = 0.0  
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)   

    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_1'
    nac_segment.percent_x_location = 0.09836
    nac_segment.percent_z_location = -0.03184
    nac_segment.height             = 0.92
    nac_segment.width              = 1.52
    nacelle.append_segment(nac_segment)   
        
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_2'
    nac_segment.percent_x_location = 0.75 
    nac_segment.percent_z_location = -0.04327
    nac_segment.height             = 1.357 
    nac_segment.width              = 0.92 
    nacelle.append_segment(nac_segment)   
        
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_3'
    nac_segment.percent_x_location = 0.86112 
    nac_segment.percent_z_location = -0.04449
    nac_segment.height             = 1.06422	 
    nac_segment.width              = 0.57339 
    nacelle.append_segment(nac_segment)  
         
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_4'
    nac_segment.percent_x_location = 0.94094  
    nac_segment.percent_z_location = -0.04368
    nac_segment.height             = 0.80734	 
    nac_segment.width              = 0.34404 
    nacelle.append_segment(nac_segment)  
        
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 1.0  
    nac_segment.percent_z_location = -0.0449
    nac_segment.height             = 0.0 
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)   

    nacelle_2          = deepcopy(nacelle)
    nacelle_2.tag      = 'nacelle_2'
    nacelle_2.origin   = [[6.50,-3.945 , 1.86]] 
    
    # ########################################################  Energy Network  #########################################################  

    net                              = RCAIDE.Framework.Networks.Electric()   

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus and Crogenic Line 
    #------------------------------------------------------------------------------------------------------------------------------------  
    bus = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()   

    fuel_cell_stack   = RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack() 
    fuel_cell_stack.electrical_configuration.series             = 1020
    fuel_cell_stack.electrical_configuration.parallel           = 1
    fuel_cell_stack.geometrtic_configuration.normal_count       = 1020
    fuel_cell_stack.geometrtic_configuration.parallel_count     = 1

    bus.fuel_cell_stacks.append(fuel_cell_stack)   
    
    #------------------------------------------------------------------------------------------------------------------------------------           
    # Battery
    #------------------------------------------------------------------------------------------------------------------------------------  
    bat_module                                             = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC()
    bat_module.electrical_configuration.series             = 20 
    bat_module.electrical_configuration.parallel           = 210 *  4 
    bat_module.cell.nominal_capacity                       = 3.8 
    bat_module.geometrtic_configuration.normal_count       = 42 
    bat_module.geometrtic_configuration.parallel_count     = 100 *  4 

    for _ in range(12):
        bat_copy = deepcopy(bat_module)
        bus.battery_modules.append(bat_copy) 
    bus.battery_module_electric_configuration = 'Series' 
    bus.initialize_bus_properties()
    
          
    ##------------------------------------------------------------------------------------------------------------------------------------  
    # Coolant Line
    #------------------------------------------------------------------------------------------------------------------------------------  
    coolant_line                                           = RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line(bus)
    coolant_line.tag                                       = 'liquid_cooled_coolant_line'
    net.coolant_lines.append(coolant_line)
    HAS                                                    = RCAIDE.Library.Components.Thermal_Management.Batteries.Liquid_Cooled_Wavy_Channel(coolant_line)
    HAS.design_altitude                                    = 15000. * Units.feet  
    atmosphere                                             = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data                                              = atmosphere.compute_values(altitude = HAS.design_altitude)     
    HAS.coolant_inlet_temperature                          = atmo_data.temperature[0,0]  
    HAS.design_battery_operating_temperature               = 313
    HAS.design_heat_removed                                = 50000 /len(bus.battery_modules)
    HAS                                                    = design_wavy_channel(HAS,bat_module) 
    
    for battery_module in bus.battery_modules:
        coolant_line.battery_modules[battery_module.tag].append(HAS)
        
    # Battery Heat Exchanger               
    HEX                                                    = RCAIDE.Library.Components.Thermal_Management.Heat_Exchangers.Cross_Flow_Heat_Exchanger() 
    HEX.design_altitude                                    = 15000. * Units.feet 
    HEX.inlet_temperature_of_cold_fluid                    = atmo_data.temperature[0,0]   
    HEX                                                    = design_cross_flow_heat_exchanger(HEX,coolant_line,bat_module)     
    coolant_line.heat_exchangers.append(HEX)
    
    # Reservoir for Battery TMS
    RES                                                    = RCAIDE.Library.Components.Thermal_Management.Reservoirs.Reservoir()
    coolant_line.reservoirs.append(RES)
    
    

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Crogenic Tank
    #------------------------------------------------------------------------------------------------------------------------------------       
    cryogenic_tank = RCAIDE.Library.Components.Powertrain.Sources.Cryogenic_Tanks.Cryogenic_Tank()  
    bus.cryogenic_tanks.append(cryogenic_tank)    

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    starboard_propulsor                              = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()  
    starboard_propulsor.tag                          = 'starboard_propulsor'
    
    # Electronic Speed Controller       
    esc                                              = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    esc.tag                                          = 'esc_1'
    esc.efficiency                                   = 0.95 
    esc.origin                                       = [[6.50,3.945 , 1.86]]   
    starboard_propulsor.electronic_speed_controller  = esc   
     
    # Propeller              
    propeller                                        = RCAIDE.Library.Components.Powertrain.Converters.Propeller() 
    propeller.tag                                    = 'propeller_1'  
    propeller.tip_radius                             = 13*Units.ft / 2
    propeller.number_of_blades                       = 6
    propeller.hub_radius                             = 20.  * Units.inches / 2 
    propeller.cruise.design_freestream_velocity      = 270 * Units.kts  
    propeller.cruise.design_angular_velocity         = 1200 *  Units.rpm 
    propeller.cruise.design_altitude                 = 25000. * Units.feet 
    propeller.cruise.design_thrust                   = 10000 * Units.N  
    propeller.cruise.design_Cl                       = 0.7
    
    propeller.origin                                 = [[6.50,3.945 , 1.86]]   
    ospath                                           = os.path.abspath(__file__)
    separator                                        = os.path.sep
    rel_path                                         = os.path.dirname(ospath)   + separator + '..' + separator 
    airfoil                                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                                      = 'NACA_4412' 
    airfoil.coordinate_file                          =  rel_path + 'Airfoils' + separator + 'NACA_4412.txt'   # absolute path   
    airfoil.polar_files                              =[ rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_50000.txt',
                                                        rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_100000.txt',
                                                        rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_200000.txt',
                                                        rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_500000.txt',
                                                        rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_1000000.txt']   
    propeller.append_airfoil(airfoil)                       
    propeller.airfoil_polar_stations                 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]  
    starboard_propulsor.rotor                        = propeller   
              
    # DC_Motor       
    motor                                            = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    motor.efficiency                                 = 0.98
    motor.origin                                     = [[6.50,3.945 , 1.86]]   
    motor.nominal_voltage                            = bus.voltage * 0.7
    motor.no_load_current                            = 1  
    starboard_propulsor.motor                        = motor 

    starboard_propulsor.nacelle                      = nacelle
     
    # design starboard propulsor 
    design_electric_rotor(starboard_propulsor)
    
    # append propulsor to distribution line 
    net.propulsors.append(starboard_propulsor) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    port_propulsor                                   = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor() 
    port_propulsor.tag                               = "port_propulsor"
                  
    esc_2                                            = deepcopy(esc)
    esc_2.origin                                     = [[6.50,-3.945 , 1.86]]        
    port_propulsor.electronic_speed_controller       = esc_2  
      
    propeller_2                                      = deepcopy(propeller)
    propeller_2.tag                                  = 'propeller_2' 
    propeller_2.origin                               =  [[6.50,-3.945 , 1.86]]   
    propeller_2.clockwise_rotation                   = False        
    port_propulsor.rotor                             = propeller_2  
                    
    motor_2                                          = deepcopy(motor)
    motor_2.origin                                   =  [[6.50,-3.945 , 1.86]]   
    port_propulsor.motor                             = motor_2  
    port_propulsor.nacelle                           = nacelle_2
    
    # append propulsor to distribution line 
    net.propulsors.append(port_propulsor) 


    #------------------------------------------------------------------------------------------------------------------------------------  
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------  
    avionics                     = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw          = 20. # Watts
    bus.avionics                 = avionics
 
    # Assign propulsors to bus       
    bus.assigned_propulsors =  [[starboard_propulsor.tag, port_propulsor.tag]] 

    # append bus   
    net.busses.append(bus) 
    
    # append network to vehicle 
    vehicle.append_energy_network(net)

    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
    
    return vehicle 

# ----------------------------------------------------------------------
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
 
    return configs

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
    #  Weights
    # ------------------------------------------------------------------  
    weights = RCAIDE.Framework.Analyses.Weights.Conventional()
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    # ------------------------------------------------------------------  
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()   
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    # ------------------------------------------------------------------  
    energy = RCAIDE.Framework.Analyses.Energy.Energy() 
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

    return analyses    

# ----------------------------------------------------------------------
#   Define the Missions
# ----------------------------------------------------------------------
def missions_setup(analyses): 

    # create base mission 
    mission          = mission_setup(analyses)
    
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
    mission        = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag    = 'mission' 

    # unpack Segments module
    Segments       = RCAIDE.Framework.Mission.Segments  
    base_segment   = Segments.Segment() 
 

    # ------------------------------------------------------------------
    #   Climb Segment 1
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Takeoff"
    segment.analyses.extend( analyses.base) 
    segment.altitude_start                                           = 2500.0 * Units.feet
    segment.altitude_end                                             = 25000  * Units.feet  
    segment.air_speed_start                                          = 200    * Units.kts
    segment.air_speed_end                                            = 270    * Units.kts
    segment.climb_rate                                               = 500    * Units['ft/min'] 
    segment.initial_battery_state_of_charge                          = 1.0  
    segment.hybrid_power_split_ratio                                 = 0        # no fuel used 
    segment.battery_fuel_cell_power_split_ratio                      = 0.3 
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True       
    mission.append_segment(segment)
    
        
    # ------------------------------------------------------------------
    #   Climb Segment 1
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb"
    segment.analyses.extend( analyses.base) 
    segment.altitude_start                                           = 2500.0 * Units.feet
    segment.altitude_end                                             = 25000  * Units.feet  
    segment.air_speed_start                                          = 200    * Units.kts
    segment.air_speed_end                                            = 270    * Units.kts
    segment.climb_rate                                               = 500    * Units['ft/min'] 
    segment.initial_battery_state_of_charge                          = 1.0  
    segment.hybrid_power_split_ratio                                 = 0        # no fuel used 
    segment.battery_fuel_cell_power_split_ratio                      = 0.18 
               
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
    segment.altitude                                                 = 25000  * Units.feet 
    segment.air_speed                                                = 270    * Units.kts
    segment.distance                                                 = 100.   * Units.nautical_mile
    segment.hybrid_power_split_ratio                                = 0        # no fuel used 
    segment.battery_fuel_cell_power_split_ratio                     = 0.3 
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.7]]
    segment.assigned_control_variables.body_angle.active             = True     
    mission.append_segment(segment)    

    # ------------------------------------------------------------------
    #   Descent Segment Flight 1   
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = "Descent"  
    segment.analyses.extend( analyses.base)       
    segment.altitude_start                                           = 25000 * Units.feet 
    segment.altitude_end                                             = 1000  * Units.feet  
    segment.air_speed_end                                            = 200   * Units.kts
    segment.climb_rate                                               = -200  * Units['ft/min']  
    segment.hybrid_power_split_ratio                                = 0        # no fuel used 
    segment.battery_fuel_cell_power_split_ratio                     = 0.3 
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                      
          
    mission.append_segment(segment)      
     
    return mission

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

    # Plot Propulsor Throttles
    plot_propulsor_throttles(results)

    return

if __name__ == '__main__': 
    main()    
    plt.show()