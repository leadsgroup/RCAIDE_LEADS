# Twin_Otter.py
# 
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE      
from RCAIDE.Framework.Core import Units  
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor   import design_electric_rotor 
from RCAIDE.Library.Plots                                           import *      

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
        export_vsp_vehicle(vehicle, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Elysian_E9X'))
    except ImportError:
        pass
        
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,save_filename=os.path.join(os.path.dirname(os.path.abspath(__file__)),'Elysian_E9X_Model'),export_gltf=True,show_figure=True)  
    
    return 
 
def vehicle_setup():
    airfoil_file_path =  os.path.join(os.path.split(sys.path[0])[0], 'Airfoils_and_Polars') + os.sep 
    polar_file_path   =  os.path.join(os.path.join(os.path.split(sys.path[0])[0], 'Airfoils_and_Polars') , 'Polars') + os.sep  

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    
    vehicle     = RCAIDE.Vehicle()
    vehicle.tag = 'Elysian_E9X'

    
    # ################################################# Vehicle-level Properties #################################################   
    vehicle.mass_properties.max_takeoff               = 76000 * Units.kilogram  
    vehicle.mass_properties.takeoff                   = 76000 * Units.kilogram    
    vehicle.mass_properties.operating_empty           = 75000 * Units.kilogram  
    vehicle.mass_properties.max_zero_fuel             = 75000 * Units.kilogram 
    vehicle.mass_properties.cargo                     = 9000.  * Units.kilogram  
    vehicle.mass_properties.center_of_gravity         = [[15.75,0, 0, 0]] 
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
    main_gear                                      = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.tire_diameter                        = 30    * Units.inches
    main_gear.rim_diameter                         = 15    * Units.inches
    main_gear.tire_width                           = 8.8   * Units.inches
    main_gear.strut_length                         = 1.6   * Units.m
    main_gear.origin                               = [[15.3, 2.1, -0.5]]
    main_gear.wheels                               = 2
    main_gear.number_of_gear_types_in_tandem       = 1
    main_gear.number_of_wheels_in_gear_type        = 2
    main_gear.xz_plane_symmetric                   = True
    vehicle.append_component(main_gear)  

    nose_gear                                      = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()
    nose_gear.tire_diameter                        = 20    * Units.inches
    nose_gear.rim_diameter                         = 10    * Units.inches
    nose_gear.tire_width                           = 5.5   * Units.inches
    nose_gear.strut_length                         = 1.3   * Units.m
    nose_gear.origin                               = [[3.9, 0, -0.5]]
    nose_gear.wheels                               = 2
    nose_gear.number_of_gear_types_in_tandem       = 1
    nose_gear.number_of_wheels_in_gear_type        = 2
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
    wing.dynamic_pressure_ratio           = 1.0


    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 6.0 * Units.degrees
    segment.sweeps.quarter_chord          = 1.93 * Units.degrees
    segment.thickness_to_chord            = .1
    root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil() 
    root_airfoil.coordinate_file          = airfoil_file_path + 'transonic_wing_root_section_airfoil.txt'
    segment.append_airfoil(root_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.9
    segment.twist                         = 0 * Units.deg
    segment.root_chord_percent            = 0.5 
    segment.dihedral_outboard             = 6.0 * Units.degrees
    segment.sweeps.quarter_chord          = 17.4 * Units.degrees
    yehudi_airfoil                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    yehudi_airfoil.coordinate_file        = airfoil_file_path + 'transonic_wing_inboard_section_airfoil.txt'
    segment.append_airfoil(yehudi_airfoil)
    wing.append_segment(segment)

    mid_airfoil                           = RCAIDE.Library.Components.Airfoils.Airfoil()
    mid_airfoil.coordinate_file           = airfoil_file_path + 'transonic_wing_outboard_section_airfoil.txt'
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'outboard'
    segment.percent_span_location         = 0.928
    segment.twist                         = 0.00 * Units.deg
    segment.root_chord_percent            = 0.47 
    segment.dihedral_outboard             = 45.0 * Units.degrees
    segment.sweeps.quarter_chord          = 10 * Units.degrees 
    segment.append_airfoil(mid_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'winglet_root'
    segment.percent_span_location         = 0.96
    segment.twist                         = 0. * Units.degrees
    segment.root_chord_percent            = 0.39 
    segment.dihedral_outboard             = 60 * Units.degrees
    segment.sweeps.quarter_chord          = 25 * Units.degrees 
    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = airfoil_file_path + 'transonic_wing_tip_section_airfoil.txt'
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.degrees
    segment.root_chord_percent            = 0.1
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.quarter_chord          = 0.0 * Units.degrees
    tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = airfoil_file_path + 'transonic_wing_tip_section_airfoil.txt'
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)     

    # control surfaces ------------------------------------------- 
    flap                                  = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                              = 'flap'
    flap.span_fraction_start              = 0.2
    flap.span_fraction_end                = 0.7
    flap.deflection                       = 0.0 * Units.degrees
    flap.configuration_type               = 'double_slotted'
    flap.chord_fraction                   = 0.30
    wing.append_control_surface(flap)

    aileron                               = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                           = 'aileron'
    aileron.span_fraction_start           = 0.7
    aileron.span_fraction_end             = 0.963
    aileron.deflection                    = 0.0 * Units.degrees
    aileron.chord_fraction                = 0.16
    wing.append_control_surface(aileron)

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing     = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio              = 4.63
    wing.sweeps.quarter_chord      = 12.0 * Units.deg  
    wing.thickness_to_chord        = 0.1
    wing.taper                     = 0.5  
    wing.spans.projected           = 10.48 
    wing.chords.root               = 2.0 
    wing.chords.tip                = 1.5 
    wing.chords.mean_aerodynamic   = 2.25 
    wing.areas.reference           = 23.58
    wing.areas.exposed             = 48.00    
    wing.areas.wetted              = 48.00     
    wing.twists.root               = 0.0 * Units.degrees
    wing.twists.tip                = 0.0 * Units.degrees 
    wing.origin                    = [[30,0,5.2]]
    wing.aerodynamic_center        = [0,0,0] 
    wing.vertical                  = False
    wing.xz_plane_symmetric        = True 
    wing.dynamic_pressure_ratio    = 0.9


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
    segment.sweeps.quarter_chord          = 68 * Units.degrees  
    segment.thickness_to_chord            = .1
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 0.2962
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.5
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.quarter_chord          = 31.2 * Units.degrees   
    segment.thickness_to_chord            = .1
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_2'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.3
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
    fuselage.operational_items.origin = [[fuselage.lengths.total * 0.6, 0, 0]]

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
      # ########################################################  Energy Network  #########################################################  
    net                                         = RCAIDE.Framework.Networks.Electric()   

    #------------------------------------------------------------------------------------------------------------------------------------  
    ##  Systems
    #------------------------------------------------------------------------------------------------------------------------------------  

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------ 
 

    avionics =  RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.origin                   = [[2,0,0]]
    avionics.mass_properties.uninstalled        = 2. * Units.lbs
    net.systems.append(avionics)

    flight_controls =  RCAIDE.Library.Components.Powertrain.Systems.Flight_Controls()
    flight_controls.origin            = [[7,0,0]]
    net.systems.append(flight_controls)

    auxillary_power_unit =  RCAIDE.Library.Components.Powertrain.Systems.Auxiliary_Power_Unit()
    auxillary_power_unit.origin       = [[14,0,0]]
    net.systems.append(auxillary_power_unit)

    electrical =  RCAIDE.Library.Components.Powertrain.Systems.Electrical()
    electrical.origin                 = [[6,0,0]]
    net.systems.append(electrical)

    hydraulics =  RCAIDE.Library.Components.Powertrain.Systems.Hydraulics()
    hydraulics.origin                 = [[7,0,0]]
    net.systems.append(hydraulics)

    environmental_controls =  RCAIDE.Library.Components.Powertrain.Systems.Environmental_Controls()
    environmental_controls.origin     = [[6,0,-0.5]]
    net.systems.append(environmental_controls)

    instruments =  RCAIDE.Library.Components.Powertrain.Systems.Instruments()
    instruments.origin                = [[6,0,0]]
    net.systems.append(instruments)

    furnishings = RCAIDE.Library.Components.Powertrain.Systems.Furnishings()
    furnishings.origin                = [[7,0,0]]
    net.systems.append(furnishings)


    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus
    #------------------------------------------------------------------------------------------------------------------------------------  
    bus                              = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus() 
    
    #------------------------------------------------------------------------------------------------------------------------------------           
    # Battery
    #------------------------------------------------------------------------------------------------------------------------------------  
    battery_pack                                            = RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack()
    bat_module                                             = RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Lithium_Ion_NMC()
    bat_module.electrical_configuration.series             = 200
    bat_module.electrical_configuration.parallel           = 325
    bat_module.geometric_configuration.stacking_rows       = 5
    bat_module.geometric_configuration.normal_count        = 200
    bat_module.geometric_configuration.parallel_count      = 325
    bat_module.orientation_euler_angles                    = [6*Units.degrees,0,0]
    bat_module.origin                                      = [[15,8.8,0.3]]
    battery_pack.append_module(bat_module)

    battery_module_2 = deepcopy(bat_module)
    battery_module_2.origin  = [[15,-8.8,0.3]]
    battery_module_2.orientation_euler_angles             = [-6*Units.degrees,0,0]
    battery_pack.append_module(battery_module_2)
    battery_pack.assigned_distributors = [[bus.tag]]
    net.sources.append(battery_pack)
    battery_pack.initialize(net)

    #------------------------------------------------------------------------------------------------------------------------------------
    # Coolant Line
    #------------------------------------------------------------------------------------------------------------------------------------
    coolant_line                                 = RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line([bus])
    coolant_line.tag                             = 'air_cooled_coolant_line'
    net.distributors.append(coolant_line)
    HAS                                         = RCAIDE.Library.Components.Powertrain.Converters.Air_Cooled_Heat_Aquisition_System()
    for battery_module in battery_pack.modules:
        battery_module.heat_acquisition_system = HAS
        battery_module.assigned_distributors   = [[coolant_line.tag]]

    
     #------------------------------------------------------------------------------------------------------------------------------------  
    #  Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    propulsor                              = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()  
    propulsor.tag                          = 'propulsor'
    
    # Electronic Speed Controller       
    esc                                              = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    esc.tag                                          = 'esc_1'
    esc.efficiency                                   = 0.95 
    esc.origin                                       = [[4.75,2.8129,1.0]]
    esc.nominal_voltage                              = battery_pack.voltage
    propulsor.electronic_speed_controller  = esc   
     
    # Propeller              
    propeller                                        = RCAIDE.Library.Components.Powertrain.Converters.Propeller() 
    propeller.tag                                    = 'propeller_1'  
    propeller.tip_radius                             = 2.59/2
    propeller.number_of_blades                       = 5
    propeller.hub_radius                             = 0.1  
    propeller.cruise.design_freestream_velocity      = 200 * Units.kts  
    propeller.cruise.design_angular_velocity         = 2500.0 * Units.rpm  
    propeller.cruise.design_altitude                 = 10000*Units.ft  
    propeller.cruise.design_thrust                   = 10000.0 * Units.N
    
    propeller.origin                                 =  [[3.9,2.8129,1.]] 
    airfoil                                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                                      = 'NACA_4412' 
    airfoil.coordinate_file                          =  airfoil_file_path + 'NACA_4412.txt'   # absolute path   
    airfoil.polar_files                              =[ polar_file_path + 'NACA_4412_polar_Re_50000.txt',
                                                        polar_file_path + 'NACA_4412_polar_Re_100000.txt',
                                                        polar_file_path + 'NACA_4412_polar_Re_200000.txt',
                                                        polar_file_path + 'NACA_4412_polar_Re_500000.txt',
                                                        polar_file_path + 'NACA_4412_polar_Re_1000000.txt']   
    propeller.append_airfoil(airfoil)                       
    propeller.airfoil_polar_stations                 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
    propulsor.rotor                        = propeller

    # DC_Motor       
    motor                                            = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    motor.efficiency                                 = 0.98
    motor.origin                                     = [[5.0,2.8129,1.0]]
    motor.nominal_voltage                            = battery_pack.voltage
    motor.no_load_current                            = 1
    propulsor.motor                        = motor

    # design propulsor 
    design_electric_rotor(propulsor)
    

    
    #########################################################   Nacelles  ############################################################    
    nacelle                    = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    nacelle.tag                = 'nacelle_1'
    nacelle.length             = 5
    nacelle.diameter           = 0.73480616 
    nacelle.areas.wetted       = 0.01*(2*np.pi*0.01/2)
    nacelle.origin             = [[3.5,2.8129,1]]
    nacelle.flow_through       = False  
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_1'
    nac_segment.percent_x_location = 0.0  
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)   
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_2'
    nac_segment.percent_x_location = 0.042687938 
    nac_segment.percent_z_location = 0.0284/ nacelle.length
    nac_segment.height             = 0.183333333 
    nac_segment.width              = 0.422484315 
    nacelle.append_segment(nac_segment)   
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_3'
    nac_segment.percent_x_location = 0.143080714 
    nac_segment.percent_z_location = 0.046733333/ nacelle.length
    nac_segment.height             = 0.44	 
    nac_segment.width              = 0.685705173 
    nacelle.append_segment(nac_segment)  
     
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_4'
    nac_segment.percent_x_location = 0.170379029  
    nac_segment.percent_z_location = -0.154233333/ nacelle.length
    nac_segment.height             = 0.898333333	 
    nac_segment.width              = 0.73480616 
    nacelle.append_segment(nac_segment)  
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 0.252189893  
    nac_segment.percent_z_location = -0.154233333/ nacelle.length
    nac_segment.height             = 1.008333333 
    nac_segment.width              = 0.736964445
    nacelle.append_segment(nac_segment)   
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_6'
    nac_segment.percent_x_location = 0.383860821   
    nac_segment.percent_z_location = -0.072566667/ nacelle.length
    nac_segment.height             = 0.971666667 
    nac_segment.width              = 0.736964445 
    nacelle.append_segment(nac_segment)  
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_7'
    nac_segment.percent_x_location = 0.551826736  
    nac_segment.percent_z_location = .055066667/ nacelle.length	
    nac_segment.height             = 0.77	 
    nac_segment.width              = 0.736964445  
    nacelle.append_segment(nac_segment)
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_8'
    nac_segment.percent_x_location = 0.809871485   
    nac_segment.percent_z_location = 0.1284/ nacelle.length
    nac_segment.height             = 0.366666667 
    nac_segment.width              = 0.736964445 
    nacelle.append_segment(nac_segment) 

    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_9'
    nac_segment.percent_x_location = 1.0  
    nac_segment.percent_z_location = 0.201733333 / nacelle.length
    nac_segment.height             = 0.036666667	
    nac_segment.width              = 0.0  
    nacelle.append_segment(nac_segment)
    
    propulsor.nacelle =  nacelle 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Append Propulsors
    #------------------------------------------------------------------------------------------------------------------------------------   
    
    propulsor_names = ['inboard_port','outboard_port','center_port','center_starboard','inboard_starboard','outboard_starboard']
    propulsor_origins = [[12.5,-5,-0.25],
                         [12.7,-9, 0.0],
                         [12.9,-13,0.25],
                         [12.5,5,-0.25],
                         [12.7,9, 0.0],
                         [12.9,13,0.25]]
    
    for i in range(len(propulsor_names)):
        propulsor = deepcopy(propulsor)
        propulsor.tag = propulsor_names[i]
        propulsor.electronic_speed_controller.origin = [propulsor_origins[i]]
        propulsor.electronic_speed_controller.tag = f"{propulsor_names[i]}_esc"
        propulsor.rotor.tag      = f"{propulsor_names[i]}_propeller"
        propulsor.rotor.origin   = [propulsor_origins[i]]
        propulsor.motor.tag      = f"{propulsor_names[i]}_motor"
        propulsor.motor.origin   = [propulsor_origins[i]]
        propulsor.nacelle.tag    = f"{propulsor_names[i]}_nacelle"
        propulsor.nacelle.origin = [propulsor_origins[i]]
        propulsor.assigned_distributors = [[bus.tag]]

        # append propulsor to distribution line
        net.propulsors.append(propulsor)

    # append bus
    net.distributors.append(bus)

    vehicle.append_energy_network(net)   
 

    return vehicle



if __name__ == '__main__':
    main() 
