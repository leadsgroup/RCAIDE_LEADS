# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units    
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor   import design_electric_rotor  
from RCAIDE.Library.Methods.Powertrain.Converters.Cross_Flow_Heat_Exchanger  import design_cross_flow_heat_exchanger
from RCAIDE.Library.Methods.Powertrain.Converters.Liquid_Cooled_Wavy_Channel       import design_wavy_channel
from RCAIDE.Library.Plots                 import *       

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
        export_vsp_vehicle(vehicle, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Twin_Otter_all_electric'))
    except ImportError:
        pass
        
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,save_filename=os.path.join(os.path.dirname(os.path.abspath(__file__)),'Electric_Twin_Otter'),export_gltf=True,show_figure=True)  
    
    return 
 
def vehicle_setup():

    
    airfoil_file_path =  os.path.join(os.path.split(sys.path[0])[0], 'Airfoils_and_Polars') + os.sep 
    polar_file_path   =  os.path.join(os.path.join(os.path.split(sys.path[0])[0], 'Airfoils_and_Polars') , 'Polars') + os.sep  

 
    # ################################################# Vehicle-level Properties ########################################################  

    #------------------------------------------------------------------------------------------------------------------------------------
    #   Initialize the Vehicle
    #------------------------------------------------------------------------------------------------------------------------------------

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'All_Electric_Twin_Otter'

 
    # ################################################# Vehicle-level Properties ########################################################  

    # mass properties
    vehicle.mass_properties.max_takeoff   = 5670. # kg 
    vehicle.mass_properties.takeoff       = 5670. # kg 
    vehicle.mass_properties.max_zero_fuel = 5670. # kg 
    vehicle.mass_properties.max_payload   = 1414. # kg
    vehicle.mass_properties.max_fuel      = 1138. # kg
    vehicle.reference_area                = 39 
    vehicle.number_of_passengers                    = 18
    vehicle.systems.control               = "fully powered"
    vehicle.systems.accessories           = "commuter"    
     
    vehicle.flight_envelope.design_cruise_altitude   = 5000 * Units.feet
    vehicle.flight_envelope.design_dynamic_pressure  = 2130.457961
    vehicle.flight_envelope.design_mach_number       = 0.19
    vehicle.flight_envelope.ultimate_load            = 5.7
    vehicle.flight_envelope.limit_load               = 3.8       
    vehicle.flight_envelope.positive_limit_load      = 2.5  
    vehicle.flight_envelope.design_range             = 3500 * Units.nmi

    
    #------------------------------------------------------------------------------------------------------------------------------------
    # ##################################################### Landing Gear ################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    main_gear                                = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.tire_diameter                  = 22.0 *  Units.inches
    main_gear.rim_diameter                   = 10.0 *  Units.inches
    main_gear.tire_width                     = 8.5  *  Units.inches
    main_gear.strut_length                   = 0.65 * Units.m
    main_gear.origin                         = [[5.7, 2.055, -.5]]
    main_gear.wheels                         = 4
    main_gear.number_of_gear_types_in_tandem = 1
    main_gear.number_of_wheels_in_gear_type  = 1
    main_gear.symmetric                      = True
    vehicle.append_component(main_gear)

    nose_gear                                = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()
    nose_gear.tire_diameter                  = 22.0 *  Units.inches
    nose_gear.rim_diameter                   = 10.0 *  Units.inches
    nose_gear.tire_width                     = 8.5  *  Units.inches
    nose_gear.strut_length                   = 0.65 * Units.m
    nose_gear.origin                         = [[2.0, 0, -.5]]
    nose_gear.wheels                         = 1
    nose_gear.number_of_gear_types_in_tandem = 1
    nose_gear.number_of_wheels_in_gear_type  = 1
    vehicle.append_component(nose_gear)
            

         
     # ##########################################################  Wings ################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Main Wing
    #------------------------------------------------------------------------------------------------------------------------------------
    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing' 
    wing.sweeps.quarter_chord             = 0.0 * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 39 
    wing.spans.projected                  = 19.81
    wing.chords.root                      = 2.04 
    wing.chords.tip                       = 2.02 
    wing.chords.mean_aerodynamic          = 2.03 
    wing.taper                            = wing.chords.root/wing.chords.tip 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 3. * Units.degree 
    wing.twists.tip                       = 0
    wing.origin                           = [[5.38, 0, 1.35]] 
    wing.aerodynamic_center               = [[5.38 + 0.25 *wing.chords.root , 0, 1.35]]  
    wing.vertical                         = False
    wing.xz_plane_symmetric               = True 
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0      
    cg_x                                  = wing.origin[0][0] + 0.25*wing.chords.mean_aerodynamic
    cg_z                                  = wing.origin[0][2] - 0.2*wing.chords.mean_aerodynamic
    vehicle.mass_properties.center_of_gravity = [[cg_x,   0.  ,  cg_z ]]  # SOURCE: Design and aerodynamic analysis of a twin-engine commuter aircraft
    vehicle.mass_properties.center_of_gravity = [[6.3133, 0, 0.38]]
    
    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.0 
    segment.twist                         = 3. * Units.degree 
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0. * Units.degree 
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    airfoil                               = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                           = 'Clark_y' 
    airfoil.coordinate_file               =airfoil_file_path + 'Clark_y.txt'    
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 0
    segment.root_chord_percent            = 0.999
    segment.dihedral_outboard             = 0.
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    airfoil                               = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                           = 'Clark_y' 
    airfoil.coordinate_file               =airfoil_file_path + 'Clark_y.txt'    
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)     
    
    # control surfaces -------------------------------------------
    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.55
    aileron.span_fraction_end     = 0.98
    aileron.deflection            = 0.0  * Units.deg
    aileron.chord_fraction        = 0.25 
    wing.append_control_surface(aileron)  

    # control surfaces -------------------------------------------
    flap                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                   = 'flap'
    flap.span_fraction_start   = 0.15
    flap.span_fraction_end     = 0.55
    flap.deflection            = 0.0  * Units.deg
    flap.chord_fraction        = 0.25 
    wing.append_control_surface(flap)      
    
    # add to vehicle
    vehicle.append_component(wing)


    #------------------------------------------------------------------------------------------------------------------------------------  
    #   Horizontal Tail
    #------------------------------------------------------------------------------------------------------------------------------------    
    wing                                  = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag                              = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord            = 0.01 * Units.degree
    wing.thickness_to_chord               = 0.12 
    wing.areas.reference                  = 9.762 
    wing.spans.projected                  = 6.29   
    wing.chords.root                      = 1.552 
    wing.chords.tip                       = 1.552 
    wing.chords.mean_aerodynamic          = 1.552  
    wing.taper                            = 1 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degree
    wing.twists.tip                       = 0.0 * Units.degree 
    wing.origin                           = [[12.96 , 0 , 1.25]] 
    wing.aerodynamic_center               = [12.96 + wing.chords.root /4 , 0 , 1.25]
    wing.vertical                         = False
    wing.winglet_fraction                 = 0.0  
    wing.xz_plane_symmetric               = True 
    wing.dynamic_pressure_ratio           = 0.9

     # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'yip'
    segment.percent_span_location         = 1.0  
    segment.root_chord_percent            = wing.taper 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0 * Units.deg
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.01
    elevator.span_fraction_end     = 1.00
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.45
    wing.append_control_surface(elevator)

    # add to vehicle
    vehicle.append_component(wing)


    #------------------------------------------------------------------------------------------------------------------------------------  
    #   Vertical Stabilizer
    #------------------------------------------------------------------------------------------------------------------------------------ 
    wing                                  = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag                              = 'vertical_stabilizer'     
    wing.sweeps.quarter_chord             = 23.73 * Units.degree 
    wing.thickness_to_chord               = 0.12 
    wing.areas.reference                  = 8.2 
    wing.spans.projected                  = 3.5
    wing.chords.root                      = 3.0 
    wing.chords.tip                       = 1.68
    wing.chords.mean_aerodynamic          = 2.34 
    wing.taper                            = wing.chords.tip/wing.chords.root 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degree
    wing.twists.tip                       = 0.0 * Units.degree 
    wing.origin                           = [[ 12.222 , 0 , 0.75 ]] 
    wing.aerodynamic_center               = [ 12.222 + 0.25 * wing.chords.root, 0 , 0.385 ]
    wing.vertical                         = True 
    wing.xz_plane_symmetric               = False
    wing.t_tail                           = False
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 23.73 * Units.deg
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.0   
    segment.root_chord_percent            = wing.taper
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0 * Units.deg
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # control surfaces -------------------------------------------
    rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                   = 'rudder'
    rudder.span_fraction_start   = 0.01
    rudder.span_fraction_end     = 1.0
    rudder.deflection            = 0.0  * Units.deg
    rudder.chord_fraction        = 0.44
    wing.append_control_surface(rudder)

    # add to vehicle
    vehicle.append_component(wing)

 
    # ##########################################################   Fuselage  ############################################################    
    fuselage = RCAIDE.Library.Components.Fuselages.Fuselage() 

    # define cabin
    cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Cabin() 
    cabin.origin                                      = [[3.5,0, 0]]
    economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 3
    economy_class.seat_arm_rest_width                 = 0
    economy_class.number_of_rows                      = 6
    economy_class.aisle_width                         = 8  *  Units.inches   
    economy_class.galley_lavatory_percent_x_locations = []  
    economy_class.emergency_exit_percent_x_locations  = []      
    economy_class.type_A_exit_percent_x_locations     = [] 
    cabin.append_cabin_class(economy_class)
    fuselage.append_cabin(cabin) 
        
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2.
    fuselage.lengths.nose                       = 2.95  
    fuselage.lengths.tail                       = 7.57
    fuselage.lengths.cabin                      = 4.62 
    fuselage.lengths.total                      = 15.77  
    fuselage.width                              = 1.75  
    fuselage.heights.maximum                    = 1.50  
    fuselage.heights.at_quarter_length          = 1.50  
    fuselage.heights.at_three_quarters_length   = 1.50  
    fuselage.heights.at_wing_root_quarter_chord = 1.50  
    fuselage.areas.side_projected               = fuselage.lengths.total *fuselage.heights.maximum  # estimate    
    fuselage.areas.wetted                       = 2 * np.pi * fuselage.width *  fuselage.lengths.total +  2 * np.pi * fuselage.width ** 2
    fuselage.areas.front_projected              =  np.pi * fuselage.width ** 2 
    fuselage.effective_diameter                 = 1.75
    fuselage.operational_items.origin            = [[fuselage.lengths.total * 0.6, 0, 0]]

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_0'
    segment.percent_x_location                  = 0
    segment.percent_z_location                  = 0
    segment.height                              = 0.0
    segment.width                               = 0.0
    fuselage.segments.append(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1a'
    segment.percent_x_location                  = 0.00985
    segment.percent_z_location                  = 0 
    segment.height                              = 0.629
    segment.width                               = 0.56185
    fuselage.segments.append(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'
    segment.percent_x_location                  = 0.019706071
    segment.percent_z_location                  = 0.0	 
    segment.height                              = 0.8130
    segment.width                               = 0.7152
    fuselage.segments.append(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'
    segment.percent_x_location                  = 0.054892307
    segment.percent_z_location                  = 0.00152
    segment.height                              = 1.10
    segment.width                               = 1.11
    fuselage.segments.append(segment)  

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'
    segment.percent_x_location                  = 0.11688
    segment.percent_z_location                  = 0.0049
    segment.height                              = 1.47905
    segment.width                               = 1.5
    segment.curvature                           = 2.5
    fuselage.segments.append(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'
    segment.percent_x_location                  = 0.14226 
    segment.percent_z_location                  = 0.00582 
    segment.height                              = 1.6
    segment.width                               = 1.6
    segment.curvature                           = 3
    fuselage.segments.append(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'
    segment.percent_x_location                  = 0.17164 
    segment.percent_z_location                  = 0.01737	 
    segment.height                              = 2.07562 
    segment.width                               = 1.72  
    segment.curvature                           = 3.5
    fuselage.segments.append(segment)
  

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'
    segment.percent_x_location                  = 0.19455   
    segment.percent_z_location                  = 0.01836 	 
    segment.height                              = 2.17	 
    segment.width                               = 1.75 
    segment.curvature                           = 4
    fuselage.segments.append(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'
    segment.percent_x_location                  =  0.54 
    segment.percent_z_location                  = 0.01977  
    segment.height                              = 2.09	 
    segment.width                               = 1.75 
    segment.curvature                           = 4
    fuselage.segments.append(segment)
    
    

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'
    segment.percent_x_location                  = 0.98
    segment.percent_z_location                  = 0.03867	 	 
    segment.height                              = 0.36	 
    segment.width                               = 0.05 
    fuselage.segments.append(segment)
    
    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'
    segment.percent_x_location                  = 1.0
    segment.percent_z_location                  = 0.03586	 
    segment.height                              = 0.0	 
    segment.width                               = 0.0 
    fuselage.segments.append(segment) 
    
    # add to vehicle
    vehicle.append_component(fuselage)
    
    # ########################################################  Energy Network  #########################################################  
    net                                         = RCAIDE.Framework.Networks.Electric()   

    #------------------------------------------------------------------------------------------------------------------------------------  
    ##  Systems
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
    bat_module.electrical_configuration.series             = 10
    bat_module.electrical_configuration.parallel           = 210
    bat_module.cell.nominal_capacity                       = 3.8
    bat_module.geometric_configuration.stacking_rows       = 8
    bat_module.geometric_configuration.normal_count        = 75
    bat_module.geometric_configuration.parallel_count      = 30

    for i in range(12):
        bat_copy = deepcopy(bat_module)
        bat_copy.origin   = [[3 + (i * 0.5) , 0, -0.35]]
        battery_pack.append_module(bat_copy)

    battery_pack.assigned_distributors = [[bus.tag]]
    net.sources.append(battery_pack)
    battery_pack.initialize(net)


    ##------------------------------------------------------------------------------------------------------------------------------------
    # Coolant Line
    #------------------------------------------------------------------------------------------------------------------------------------
    coolant_line                                           = RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line([bus])
    coolant_line.tag                                       = 'liquid_cooled_coolant_line'
    net.distributors.append(coolant_line)
    HAS                                                    = RCAIDE.Library.Components.Powertrain.Converters.Liquid_Cooled_Wavy_Channel(coolant_line)
    HAS.design_altitude                                    = 2500. * Units.feet
    atmosphere                                             = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data                                              = atmosphere.compute_values(altitude = HAS.design_altitude)
    HAS.coolant_inlet_temperature                          = atmo_data.temperature[0,0]
    HAS.design_battery_operating_temperature               = 313
    HAS.design_heat_removed                                = 50000 /len(battery_pack.modules)
    HAS                                                    = design_wavy_channel(HAS,bat_module)

    for battery_module in battery_pack.modules:
        battery_module.heat_acquisition_system = HAS
        battery_module.assigned_distributors   = [[coolant_line.tag]]

    # Battery Heat Exchanger
    HEX                                                    = RCAIDE.Library.Components.Powertrain.Converters.Cross_Flow_Heat_Exchanger()
    HEX.design_altitude                                    = 2500. * Units.feet
    HEX.inlet_temperature_of_cold_fluid                    = atmo_data.temperature[0,0]
    HEX                                                    = design_cross_flow_heat_exchanger(HEX,coolant_line,bat_module)
    coolant_line.heat_exchangers.append(HEX)

    # Reservoir for Battery TMS
    RES                                                    = RCAIDE.Library.Components.Powertrain.Sources.Reservoirs.Reservoir()
    coolant_line.reservoirs.append(RES)
    
     
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    starboard_propulsor                              = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()  
    starboard_propulsor.tag                          = 'starboard_propulsor'
    
    # Electronic Speed Controller       
    esc                                              = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    esc.tag                                          = 'esc_1'
    esc.efficiency                                   = 0.95 
    esc.origin                                       = [[3.8,2.8129,1. ]]
    esc.nominal_voltage                              = battery_pack.voltage
    starboard_propulsor.electronic_speed_controller  = esc   
     
    # Propeller              
    propeller                                        = RCAIDE.Library.Components.Powertrain.Converters.Propeller() 
    propeller.tag                                    = 'propeller_1'  
    propeller.tip_radius                             = 2.59/2
    propeller.number_of_blades                       = 3
    propeller.hub_radius                             = 5.    * Units.inches 
    propeller.cruise.design_freestream_velocity      = 150 * Units.kts      
    speed_of_sound                                   = 343 
    propeller.cruise.design_tip_mach                 = 0.8
    propeller.cruise.design_angular_velocity         = propeller.cruise.design_tip_mach *speed_of_sound/propeller.tip_radius
    propeller.cruise.design_lift_coefficient         = 0.8
    propeller.cruise.design_altitude                 = 10000. * Units.feet 
    propeller.cruise.design_thrust                   = 5000  
    propeller.clockwise_rotation                     = False
    propeller.variable_pitch                         = True  
    propeller.origin                                 = [[3.5,2.8129,1.]]   
    airfoil                                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                                      = 'NACA_4412' 
    airfoil.coordinate_file                          =  airfoil_file_path+ 'NACA_4412.txt'   # absolute path   
    airfoil.polar_files                              =[ polar_file_path + 'NACA_4412_polar_Re_50000.txt',
                                                        polar_file_path + 'NACA_4412_polar_Re_100000.txt',
                                                        polar_file_path + 'NACA_4412_polar_Re_200000.txt',
                                                        polar_file_path + 'NACA_4412_polar_Re_500000.txt',
                                                        polar_file_path + 'NACA_4412_polar_Re_1000000.txt']   
    propeller.append_airfoil(airfoil)                       
    propeller.airfoil_polar_stations                 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
    starboard_propulsor.rotor                        = propeller   
              
    # DC_Motor       
    motor                                            = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    motor.efficiency                                 = 0.98
    motor.origin                                     = [[4.0,2.8129,1. ]]   
    motor.nominal_voltage                            = battery_pack.voltage
    motor.no_load_current                            = 1   
    starboard_propulsor.motor                        = motor
    
    # design starboard propulsor 
    design_electric_rotor(starboard_propulsor)

    
    #########################################################   Nacelles  ############################################################    
    nacelle                    = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    nacelle.tag                = 'nacelle_1'
    nacelle.length             = 4
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
    
    starboard_propulsor.nacelle =  nacelle
    starboard_propulsor.assigned_distributors = [[bus.tag]]
    net.propulsors.append(starboard_propulsor)

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    port_propulsor                                     = deepcopy(starboard_propulsor) 
    port_propulsor.tag                                 = "port_propulsor" 
    port_propulsor.electronic_speed_controller.origin  = [[4.75,-2.8129,1.0]]  
    port_propulsor.electronic_speed_controller.tag     = 'port_propulsor_esc'  
    port_propulsor.rotor.tag                           = 'port_propulsor_propeller' 
    port_propulsor.rotor.origin                        = [[3.9,-2.8129,1.]]
    port_propulsor.motor.tag                           ='port_propulsor_motor' 
    port_propulsor.motor.origin                        = [[5.0,-2.8129,1.]]    
    port_propulsor.nacelle.tag                         = 'nacelle_2'
    port_propulsor.nacelle.origin                      = [[3.5,-2.8129,1]] 
    # append propulsor to distribution line
    net.propulsors.append(port_propulsor)

    # append bus
    net.distributors.append(bus)
    vehicle.append_energy_network(net)   
 
    
    
    return vehicle 

if __name__ == '__main__':
    main() 
