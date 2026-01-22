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
from RCAIDE.Framework.External_Interfaces.OpenVSP.export_vsp_vehicle import export_vsp_vehicle   

from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor                          import design_electric_rotor   
from RCAIDE.Library.Methods.Performance import *  

# python imports 
import numpy as np  
from   copy import deepcopy
import matplotlib.pyplot as plt 
import  os
import  sys 
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main(plot_results=True, plot_vehicle=False): 

    vehicle  = vehicle_setup()

    # if plot_vehicle:
    #     plot_3d_vehicle(vehicle, 
    #                 fuselage_opacity            = 0.25, 
    #                 nacelle_opacity             = 0.5, 
    #                 boom_opacity                = 1.0,
    #                 # fuel_tank_opacity=1.0, 
    #                 # fuel_tank_color= 'lightblue', 
    #               boom_color                  = 'lightgreen')
    
    configs  = configs_setup(vehicle)
     
    analyses = analyses_setup(configs)

    mission  = mission_setup(analyses) 

    missions = missions_setup(mission) 

    results  = missions.base_mission.evaluate()   
   
    if plot_results:
        plot_mission(results)   
    print(f"Standard L/D: {results.segments.cruise.conditions.aerodynamics.coefficients.lift.total[0,0]/results.segments.cruise.conditions.aerodynamics.coefficients.drag.total[0,0]}")
    
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
    
    payload_range_data =  compute_payload_range_diagram(mission = missions.base_mission, cruise_segment_tag = "cruise", fuel_reserve_percentage=0.15, plot_diagram = plot_results, fuel_name=None)
    
    return payload_range_data

 
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
    vehicle.mass_properties.max_takeoff               = 23000 
    # vehicle.mass_properties.takeoff                   = 23000   
    # vehicle.mass_properties.operating_empty           = 13010
    vehicle.mass_properties.max_zero_fuel             = 15651.8 
    vehicle.mass_properties.max_payload               = 7100
    vehicle.mass_properties.min_payload = 0
    # vehicle.mass_properties.cargo                     = 7200
    vehicle.mass_properties.center_of_gravity         = [[13.0038, 0, 0.45]] #[[11.81775, 0, 0.45]]
    vehicle.mass_properties.moments_of_inertia.tensor = [[0,0,0]] # Unknown 

    # envelope properties
    vehicle.flight_envelope.design_mach_number        = 0.43 
    vehicle.flight_envelope.design_range              = 890 * Units.nmi
    vehicle.flight_envelope.design_cruise_altitude    = 25000 * Units.feet
    vehicle.flight_envelope.ultimate_load             = 3.75
    vehicle.flight_envelope.positive_limit_load       = 1.5
    vehicle.flight_envelope.design_dynamic_pressure   = 5e4
              
    # basic parameters              
    vehicle.reference_area                            = 61.0  
    vehicle.number_of_passengers                                = 72
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
    segment.has_fuel_tank                 = True
    wing.append_segment(segment)
  
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'outboard'
    segment.percent_span_location         = 0.324
    segment.twist                         = 0.0 * Units.deg
    segment.root_chord_percent            = 1.0
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.leading_edge           = 4.7 * Units.degrees
    segment.thickness_to_chord            = .13 
    segment.has_fuel_tank                 = True
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
    flap.span_fraction_end        = 0.72
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
    elevator.span_fraction_start   = 0.01
    elevator.span_fraction_end     = 1.0
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
    segment.thickness_to_chord            = 0.08
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
    cabin.offset_x                                    =2  #origin                                      = [[2, 0, 0]]
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
    segment.percent_z_location                  = 0.01076
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
    # segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    # segment.tag                                 = 'segment_11'    
    # segment.percent_x_location                  = 17.01420312/fuselage.lengths.total 
    # segment.percent_z_location                  = 0.01860240047935103
    # segment.height                              = 2.755708426
    # segment.width                               = 2.985093814 
    # fuselage.append_segment(segment)   
 
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
    net                                         = RCAIDE.Framework.Networks.Electric()    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus
    #------------------------------------------------------------------------------------------------------------------------------------  
    bus                              = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus() 
    
    #------------------------------------------------------------------------------------------------------------------------------------           
    # Battery
    #------------------------------------------------------------------------------------------------------------------------------------  
    bat_module                                             = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC()
    bat_module.electrical_configuration.series             = 135
    bat_module.electrical_configuration.parallel           = 250
    bat_module.cell.nominal_capacity                       = 6
    bat_module.cell.mass                                   = 0.03 * Units.kg
    bat_module.geometrtic_configuration.normal_count            = 135
    bat_module.geometrtic_configuration.parallel_count           = 250
    # bat_module.geometrtic_configuration.normal_spacing  = 0.01
    # bat_module.geometrtic_configuration.parallel_spacing  = 0.01  
    for _ in range(4):
        bat_copy = deepcopy(bat_module)
        bus.battery_modules.append(bat_copy)

    bus.battery_module_electric_configuration = 'Parallel' 
    bus.initialize_bus_properties()

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Coolant Line
    #------------------------------------------------------------------------------------------------------------------------------------  
    coolant_line                                 = RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line([bus])
    coolant_line.tag                             = 'air_cooled_coolant_line'
    net.coolant_lines.append(coolant_line)
    HAS                                         = RCAIDE.Library.Components.Thermal_Management.Batteries.Air_Cooled() 
    for battery_module in bus.battery_modules:
        coolant_line.battery_modules[battery_module.tag].append(HAS)


    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    starboard_propulsor                              = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()  
    starboard_propulsor.tag                          = 'starboard_propulsor'
    
    # Electronic Speed Controller       
    esc                                              = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    esc.tag                                          = 'esc_1'
    esc.efficiency                                   = 0.95 
    esc.origin                                       = [[ 9.559106394 ,4.219315295, 1.616135105]]
    esc.bus_voltage                                  = bus.voltage   
    starboard_propulsor.electronic_speed_controller  = esc   
     
    # Propeller              
    propeller                                        = RCAIDE.Library.Components.Powertrain.Converters.Propeller() 
    propeller.tag                                    = 'propeller_1'  
    propeller.tip_radius                             = 13*Units.ft / 2
    propeller.number_of_blades                       = 6
    propeller.hub_radius                             = 20.  * Units.inches / 2 
    propeller.cruise.design_freestream_velocity      = 270 * Units.kts  
    propeller.cruise.design_angular_velocity         = 3000.0 * Units.rpm  
    propeller.cruise.design_altitude                 = 25000*Units.ft  
    propeller.cruise.design_thrust                   = 15000.0 * Units.N
    
    propeller.origin                                 = [[ 9.559106394 ,4.219315295, 1.616135105]] 
    ospath                                           = os.path.abspath(__file__)
    separator                                        = os.path.sep
    rel_path                                         = os.path.dirname(ospath) + separator + '..' + separator + '..' + separator + 'Aircraft' + separator 
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
    motor.origin                                     = [[ 9.559106394 ,4.219315295, 1.616135105]]
    motor.nominal_voltage                            = bus.voltage 
    motor.no_load_current                            = 1
    starboard_propulsor.motor                        = motor

    # design starboard propulsor 
    design_electric_rotor(starboard_propulsor)

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
    # append propulsor to distribution line 
    net.propulsors.append(starboard_propulsor) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    port_propulsor                                     = deepcopy(starboard_propulsor) 
    port_propulsor.tag                                 = "port_propulsor" 
    port_propulsor.electronic_speed_controller.origin  = [[ 9.559106394 ,-4.219315295, 1.616135105]]       
    port_propulsor.electronic_speed_controller.tag     = 'port_propulsor_esc'  
    port_propulsor.rotor.tag                           = 'port_propulsor_propeller' 
    port_propulsor.rotor.origin                        =  [[ 9.559106394 ,-4.219315295, 1.616135105]] 
    port_propulsor.motor.tag                           ='port_propulsor_motor' 
    port_propulsor.motor.origin                        =  [[ 9.559106394 ,-4.219315295, 1.616135105]]  
    port_propulsor.nacelle.origin                     = [[8.941625295,-4.219315295, 1.616135105 ]]  

    
    # append propulsor to distribution line 
    net.propulsors.append(port_propulsor) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------  
    avionics                     = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw          = 30. # Watts
    bus.avionics                 = avionics
    
    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to bus       
    bus.assigned_propulsors =  [[starboard_propulsor.tag, port_propulsor.tag]] 

    # append bus   
    net.busses.append(bus)
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

    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.overwrite_reference        = True
    geometry.settings.update_wing_properties     = True
    # geometry.settings.print_weight_analysis_report = True
    analyses.append(geometry)


    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------     
    weights = RCAIDE.Framework.Analyses.Weights.Electric_Transport()
    weights.settings.method = 'Raymer'
    weights.settings.update_center_of_gravity                            = False
    weights.settings.update_moment_of_inertia = True
    # weights.settings.print_weight_analysis_report                        = True
    weights.settings.weight_correction_factors.empty.systems.apu         = 0.0
    weights.settings.weight_correction_factors.empty.systems.furnishings = 0.50
    # weights.settings.
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    # ------------------------------------------------------------------     
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.settings.store_training_data  = True
    analyses.append(aerodynamics) 

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
    atmosphere.features.planet = planet.features
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
    segment.initial_battery_state_of_charge    = 1.0
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
    segment.air_speed_end                                            = 120.5 *  Units.knots
                       
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
    segment.altitude_end                                             = 20000   * Units.feet  
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
    segment.altitude                                                 = 20000  * Units.feet 
    segment.air_speed                                                = 270    * Units.kts
    segment.distance                                                 = 115.   * Units.nautical_mile  
    # 
                
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
    segment.altitude_start                                           = 20000   * Units.feet 
    segment.altitude_end                                             = 5000 * Units.feet  
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
    segment.air_speed_end                                 = 150 *  Units.knots
    segment.altitude_end                                             = 2000 * Units.feet 
    segment.climb_rate                                               = -1000 * Units['ft/min']   
               
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

    # Battery Conditions
    plot_battery_module_conditions(results)
    plot_battery_cell_conditions(results)

    plot_thermal_management_performance(results)
    return 

if __name__ == '__main__': 
    main()    
    plt.show()
