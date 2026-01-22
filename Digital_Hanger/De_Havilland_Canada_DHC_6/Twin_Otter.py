# Twin_Otter.py
# 
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE      
from RCAIDE.Framework.Core import Units  
from   RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop        import design_turboprop   
from RCAIDE.Library.Plots                                           import *       
from RCAIDE.Library.Methods.Performance import *  
from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle

# python imports 
import numpy as np 
from copy import deepcopy
import os 
import matplotlib.pyplot        as plt 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main(plot_results=True, plot_vehicle=True, fuel='Jet_A1'):
 
    
    # vehicle data
    vehicle  = vehicle_setup(fuel)               
    # plot_3d_vehicle(vehicle, fuselage_opacity            = 0.25, front_view = True) 
    # export_vsp_vehicle(vehicle, 'Twin_Otter_Updated')   
    # Set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = analyses_setup(configs)

    # mission analyses
    mission  = aviation_mission_setup(analyses) 
    
    # create mission instances (for multiple types of missions)
    missions = missions_setup(mission) 
     
    # mission analysis 
    results = missions.base_mission.evaluate()  
    
    plot_mission(results)
    
    # plot vehicle 
    # plot_3d_vehicle(vehicle, 
    #                 fuselage_opacity            = 0.25, 
    #                 nacelle_opacity             = 0.5, 
    #                 boom_opacity                = 1.0,
    #                 fuel_tank_opacity=1.0, 
    #                 fuel_tank_color= 'lightblue', 
    #                 boom_color                  = 'lightgreen')
    plt.show()

    return results

def payload_range():
     
    # Step 1 design a vehicle
    vehicle  = vehicle_setup() 
     
    # Step 2 create aircraft configuration based on vehicle 
    configs  = configs_setup(vehicle)
    
    # Step 3 set up analysis
    analyses = analyses_setup(configs)
    
    # Step 4 set up a flight mission
    mission = payload_range_simplified_mission_setup(analyses)
    missions= missions_setup(mission)
    
    payload_range_data =  compute_payload_range_diagram(mission = missions.base_mission, cruise_segment_tag = "cruise", fuel_reserve_percentage=0.05, plot_diagram = True, fuel_name=None)
    
    return payload_range_data 
    
def vehicle_setup(fuel='Jet_A1'): 
    

   #------------------------------------------------------------------------------------------------------------------------------------
    #   Initialize the Vehicle
    #------------------------------------------------------------------------------------------------------------------------------------

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Twin_Otter'

 
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
    main_gear.tire_diameter                  = 6  *  Units.inches 
    main_gear.rim_diameter                   = 3  *  Units.inches 
    main_gear.tire_width                     = 6  *  Units.inches 
    main_gear.strut_length                   = 12  * Units.ft 
    main_gear.wheels                         = 4   
    main_gear.number_of_gear_types_in_tandem = 1
    main_gear.number_of_wheels_in_gear_type  = 2  
    main_gear.symmetric                      = True
    main_gear.origin = [[8.0, 0, 0]]
    vehicle.append_component(main_gear)  

    nose_gear                                = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                  =  5 *  Units.inches   
    nose_gear.rim_diameter                   =  3 *  Units.inches 
    nose_gear.tire_width                     =  5 *  Units.inches 
    nose_gear.strut_length                   =  6.* Units.ft 
    nose_gear.origin = [[2.0, 0, 0]]
    nose_gear.wheels                         = 2   
    nose_gear.number_of_gear_types_in_tandem = 1
    nose_gear.number_of_wheels_in_gear_type  = 2    
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
    wing.high_lift                        = True 
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0  
    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    rel_path                              = os.path.dirname(ospath) + separator + '..' + separator 
    #airfoil                               = RCAIDE.Library.Components.Airfoils.Airfoil()
    #airfoil.tag                           = 'Clark_y' 
    #airfoil.coordinate_file               = rel_path + 'Airfoils' + separator + 'Clark_y.txt'   # absolute path     
    cg_x                                  = wing.origin[0][0] + 0.25*wing.chords.mean_aerodynamic
    cg_z                                  = wing.origin[0][2] - 0.2*wing.chords.mean_aerodynamic
    vehicle.mass_properties.center_of_gravity = [[6.3133,   0.  ,  0.38 ]]  # SOURCE: Design and aerodynamic analysis of a twin-engine commuter aircraft

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.0 
    segment.twist                         = 3. * Units.degree 
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0. * Units.degree 
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    #segment.append_airfoil(airfoil)
    wing.append_segment(segment)
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 0
    segment.root_chord_percent            = 0.999
    segment.dihedral_outboard             = 0.
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    #segment.append_airfoil(airfoil)
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
    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    aileron.tag                   = 'flap'
    aileron.span_fraction_start   = 0.15
    aileron.span_fraction_end     = 0.55
    aileron.deflection            = 0.0  * Units.deg
    aileron.chord_fraction        = 0.25 
    wing.append_control_surface(aileron)      
    
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
    wing.high_lift                        = False 
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
    # cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Cabin() 
    # cabin.offset_x = 3.5
    # economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    # economy_class.number_of_seats_abrest              = 3
    # economy_class.seat_arm_rest_width = 0
    # economy_class.number_of_rows                      = 6
    # economy_class.aisle_width                          = 8  *  Units.inches  

    # economy_class.galley_lavatory_percent_x_locations = []  
    # economy_class.emergency_exit_percent_x_locations  = []      
    # economy_class.type_A_exit_percent_x_locations     = [] 
    # cabin.append_cabin_class(economy_class)
    # fuselage.append_cabin(cabin) 
        
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
    net                                         = RCAIDE.Framework.Networks.Fuel()    

    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                       = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  
 
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------    
    starboard_propulsor                              = RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop()    
    starboard_propulsor.tag                          = 'starboard_propulsor' 
    starboard_propulsor.origin                       = [[3.5,2.8129,1.22 ]]
    starboard_propulsor.design_altitude              = 10000*Units.ft                                   # [-]         Design Altitude
    starboard_propulsor.design_mach_number           = 0.27                                              # [-]         Design Mach number
    starboard_propulsor.design_thrust                = 3500.0 * Units.N                                  # [-]         Design Thrust 
    starboard_propulsor.working_fluid                = RCAIDE.Library.Attributes.Gases.Air()          
    starboard_propulsor.gearbox.efficiency           = 0.99   
    starboard_propulsor.design_power                 = 462334                                          # [-]         Design Gearbox Efficiency
    starboard_propulsor.specific_fuel_consumption_reduction_factor = -3.5

    #Propeller Design              
    propeller                                        = RCAIDE.Library.Components.Powertrain.Converters.Propeller()   
    propeller.tag                                    = 'starboard_propulsor_propeller' 
    propeller.origin                                 = [[3.75, 2.8129,1.22 ]]
    propeller.active                                 = True          
    propeller.tip_radius                             = 2.59/2
    propeller.hub_radius                             = 0.1 
    propeller.number_of_blades                       = 3   
    propeller.design_efficiency                      = 0.83      
    propeller.design_angular_velocity                = 2200.0 * Units.rpm        # https://paracleteaviation.com/wp-content/uploads/2017/10/Twin-Otter-Type-Data-Sheet.pdf
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
    # combustor.volume                                 = 0.0023         # [m**3] Combustor volume
    # combustor.length                                 = 0.2            # [m] Combustor Length
    # combustor.number_of_combustors                   = 1              # [-] Number of Combustors for one engine
    # combustor.F_SC                                   = 1              # [-] Fuel scale factor
    # combustor.N_PZ                                    = 21             # [-] Number of PSR in the Primary Zone
    # combustor.L_PZ                                    = 0.05           # [m] Primary Zone length  
    # combustor.S_PZ                                    = 0.39           # [-] Mixing parameter in the Primary Zone  
    # combustor.design_equivalence_ratio_PZ             = 1.71           # [-] Design Equivalence Ratio in Primary Zone at Maximum Throttle  
    # combustor.N_SZ                                    = 500            # [-] Number of discritizations in the Secondary Zone
    # combustor.f_SM                                    = 0.6            # [-] Slow mode fraction
    # combustor.l_SA_SM                                 = 0.4            # [-] Secondary air length fraction (of L_SZ) in slow mode
    # combustor.l_SA_FM                                 = 0.05           # [-] Secondary air length fraction (of L_SZ) in fast mode
    # combustor.l_DA_start                              = 0.95           # [-] Dilution air start length fraction (of L_SZ)
    # combustor.l_DA_end                                = 1.0            # [-] Dilution air end length fraction (of L_SZ)
    # combustor.joint_mixing_fraction                   = 0.6            # [-] Joint mixing fraction
    # combustor.design_equivalence_ratio_SZ             = 0.61            # [-] Design Equivalence Ratio in Secondary Zone at Maximum Throttle
    combustor.air_mass_flow_rate_take_off             = 40             # [kg/s] Air mass flow rate at take-off
    combustor.fuel_to_air_ratio_take_off              = 0.025          # [-] Fuel to air ratio at take-off
    combustor.air_data                                = RCAIDE.Library.Attributes.Gases.Air()          # [-] Air object
    combustor.fuel_data                               = RCAIDE.Library.Attributes.Propellants.Jet_A1()       # [-] Fuel object
    if fuel == 'Jet_A1':
        combustor.fuel_to_air_ratio_take_off           = 0.025   # [-]
        combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A1()
        combustor.fuel_data.stoichiometric_fuel_air_ratio = 0.068
        combustor.fuel_data.heat_of_vaporization = 360000
        combustor.fuel_data.fuel_surrogate_S1 = {'NC12H26':0.404, 'IC8H18':0.295, 'TMBENZ' : 0.073,'NPBENZ':0.228, 'C10H8':0.02}
        combustor.fuel_data.temperature = 298.15
        combustor.fuel_data.pressure    = 101325
        combustor.fuel_data.kinetic_mechanism             = 'Fuel.yaml'
    elif fuel == 'Liquid_Hydrogen':
        combustor.fuel_to_air_ratio_take_off = 0.00977 
        combustor.fuel_data = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
        combustor.fuel_data.stoichiometric_fuel_air_ratio = 0.02941
        combustor.fuel_data.heat_of_vaporization = 447000
        combustor.fuel_data.fuel_surrogate_S1 = {'H2': 1}
        combustor.fuel_data.temperature = 20
        combustor.fuel_data.pressure    = 101325
        combustor.fuel_data.kinetic_mechanism             = 'Fuel.yaml'
    elif fuel == 'Liquid_Petroleum_Gas':
        combustor.fuel_to_air_ratio_take_off = 0.02144  
        combustor.fuel_data = RCAIDE.Library.Attributes.Propellants.Liquid_Petroleum_Gas()
        combustor.fuel_data.stoichiometric_fuel_air_ratio = 0.06451
        combustor.fuel_data.heat_of_vaporization = 426200
        combustor.fuel_data.fuel_surrogate_S1 = {'C3H8':0.822, 'NC4H10':0.178}
        combustor.fuel_data.temperature = 298.15
        combustor.fuel_data.pressure    = 810600
        combustor.fuel_data.kinetic_mechanism             = 'Fuel.yaml'
    elif fuel == 'Liquid_Natural_Gas':
        combustor.fuel_to_air_ratio_take_off = 0.0193226
        combustor.fuel_data = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()
        combustor.fuel_data.stoichiometric_fuel_air_ratio = 0.058139
        combustor.fuel_data.heat_of_vaporization = 510000
        combustor.fuel_data.fuel_surrogate_S1 = {'CH4':0.9, 'C2H6':0.05, 'C3H8':0.05}
        combustor.fuel_data.temperature = 111
        combustor.fuel_data.pressure = 101325
        combustor.fuel_data.kinetic_mechanism             = 'Fuel.yaml' 
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
    
    starboard_propulsor.nacelle = nacelle      
 
    net.propulsors.append(starboard_propulsor) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------      
    # copy turboprop
    port_propulsor                                  = deepcopy(starboard_propulsor)
    port_propulsor.active_fuel_tanks                = ['fuel_tank'] 
    port_propulsor.tag                              = 'port_propulsor' 
    port_propulsor.origin                           = [[3.5, -2.8129,1.22 ]]  # change origin 
    port_propulsor.nacelle.tag                      = 'port_propulsor_nacelle' 
    port_propulsor.nacelle.origin                   = [[3.5, -2.8129,1.22 ]]
    port_propulsor.propeller.origin                 = [[3.75, -2.8129,1.22 ]]
         
    # append propulsor to distribution line 
    net.propulsors.append(port_propulsor) 

    # ------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    # ------------------------------------------------------------------------------------------------------------------------- 
    # fuel tank
    fuel_tank                                             = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank() 
    fuel_tank.origin                                      = vehicle.wings.main_wing.origin  
    
    if fuel == 'Jet_A1':
        fuel_tank.fuel                                = RCAIDE.Library.Attributes.Propellants.Jet_A1()   
    elif fuel == 'Liquid_Hydrogen':
        fuel_tank.fuel                                = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    elif fuel == 'Liquid_Petroleum_Gas':
        fuel_tank.fuel                                = RCAIDE.Library.Attributes.Propellants.Liquid_Petroleum_Gas()
    elif fuel == 'Liquid_Natural_Gas':
        fuel_tank.fuel                                = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()  
    fuel_tank.fuel.mass_properties.mass                   = 0 #1190 *Units.lbs # CHANGE
    fuel_tank.fuel.mass_properties.center_of_gravity      = wing.mass_properties.center_of_gravity
    fuel_tank.internal_volume                             = fuel_tank.fuel.mass_properties.mass/fuel_tank.fuel.density   
    fuel_line.fuel_tanks.append(fuel_tank) 

    fuel_line.assigned_propulsors =  [[starboard_propulsor.tag, port_propulsor.tag]]

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to network      
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

    configs         = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config     = RCAIDE.Library.Components.Configs.Config(vehicle)
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
    configs.append(config)

    
    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    configs.append(config)   

    # ------------------------------------------------------------------
    #   Descent Configuration
    # ------------------------------------------------------------------ 
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'descent' 
    configs.append(config) 
    
        
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'
    configs.append(config)   
     
    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------  
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'    
    configs.append(config)    

 
    return configs

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
    base_segment.temperature_deviation  = 2.5
    base_segment.state.numerics.number_of_control_points  = 4 
    
    # ## ------------------------------------------------------------------
    # ##   Takeoff
    # ## ------------------------------------------------------------------      
    # #segment = Segments.Ground.Takeoff(base_segment)
    # #segment.tag = "Takeoff"  
    # #segment.analyses.extend( analyses.base )   
    # #segment.velocity_start                                   = Vstall*0.1 
    # #segment.velocity_end                                     = Vstall*1.2  
    # #segment.friction_coefficient                             = 0.04   
    # #segment.throttle                                         = 0.8   
    
    # #segment.flight_dynamics.force_x                           = True 
    # #segment.assigned_control_variables.elapsed_time.active               = True         
   
    # #mission.append_segment(segment) 
  
    
    # # ------------------------------------------------------------------
    # #   Departure End of Runway Segment Flight 1 : 
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    # segment.tag = 'Departure_End_of_Runway'       
    # segment.analyses.extend( analyses.base )   
    # segment.altitude_start                                = 0.0 * Units.feet
    # segment.altitude_end                                  = 50.0 * Units.feet
    # Vstall = 100 * Units['mph']
    # segment.air_speed_start                               = Vstall *1.2  
    # segment.air_speed_end                                 = Vstall *1.25
    # segment.initial_battery_state_of_charge    = 1.0
            
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                  
       
    # mission.append_segment(segment)
    
    # # ------------------------------------------------------------------
    # #   Initial Climb Area Segment Flight 1  
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    # segment.tag = 'Initial_CLimb_Area' 
    # segment.analyses.extend( analyses.base )   
    # segment.altitude_start                                = 50.0 * Units.feet
    # segment.altitude_end                                  = 500.0 * Units.feet 
    # segment.air_speed_start                               = Vstall *1.25 
    # segment.air_speed_end                                 = Vstall *1.3 
    # segment.climb_rate                                    = 600 * Units['ft/min']   
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                  
          
    # mission.append_segment(segment)  
   
             
    # # ------------------------------------------------------------------
    # #   Climb Segment Flight 1 
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    # segment.tag = 'Climb_1'        
    # segment.analyses.extend( analyses.base )     
    # segment.altitude_start                                = 500.0 * Units.feet
    # segment.altitude_end                                  = 2500 * Units.feet  
    # segment.air_speed_end                                 = 120 * Units.kts 
    # segment.climb_rate                                    = 500* Units['ft/min']  
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                 
           
    # mission.append_segment(segment)
    
        
    # # ------------------------------------------------------------------
    # #   Climb 1 : constant Speed, constant rate segment 
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    # segment.tag = "Climb_2"
    # segment.analyses.extend( analyses.base )   
    # segment.altitude_start                                = 2500.0  * Units.feet
    # segment.altitude_end                                  = 5000   * Units.feet  
    # segment.air_speed_end                                 = 130 * Units.kts 
    # segment.climb_rate                                    = 700.034 * Units['ft/min']   
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                 
            
    # mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "Cruise" 
    segment.analyses.extend( analyses.base )   
    segment.altitude                                      = 10000   * Units.feet 
    segment.air_speed                                     = 146 * Units.kts
    segment.distance                                      = 146.   * Units.nautical_mile  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor', 'port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                  
          
    mission.append_segment(segment)    


    # # ------------------------------------------------------------------
    # #   Descent Segment Flight 1   
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    # segment.tag = "Descent"  
    # segment.analyses.extend( analyses.base )       
    # segment.altitude_start                                = 5000   * Units.feet 
    # segment.altitude_end                                  = 1000 * Units.feet  
    # segment.air_speed_end                                 = 100 * Units['mph']   
    # segment.climb_rate                                    = -2000 * Units['ft/min']  
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                 
          
    # mission.append_segment(segment)   
               
    # # ------------------------------------------------------------------
    # #  Downleg_Altitude Segment Flight 1 
    # # ------------------------------------------------------------------

    # segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    # segment.tag = 'Downleg'
    # segment.analyses.extend( analyses.base )   
    # segment.air_speed                                     = 100 * Units['mph']   
    # segment.distance                                      = 6000 * Units.feet 
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                   
            
    # mission.append_segment(segment)     
    
    # ## ------------------------------------------------------------------
    # ##  Reserve Climb 
    # ## ------------------------------------------------------------------ 
    # #segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment) 
    # #segment.tag = 'Reserve_Climb'        
    # #segment.analyses.extend( analyses.base )      
    # #segment.altitude_end                                  = 5000 * Units.feet
    # #segment.air_speed                                     = 120 * Units['mph']
    # #segment.climb_rate                                    = 500* Units['ft/min']  
    
    # ## define flight dynamics to model 
    # #segment.flight_dynamics.force_x                       = True  
    # #segment.flight_dynamics.force_z                       = True     
    
    # ## define flight controls 
    # #segment.assigned_control_variables.throttle.active               = True           
    # #segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # #segment.assigned_control_variables.body_angle.active             = True                
        
    # #mission.append_segment(segment)
    
    # ## ------------------------------------------------------------------
    # ##  Researve Cruise Segment 
    # ## ------------------------------------------------------------------ 
    # #segment = Segments.Cruise.Constant_Speed_Constant_Altitude_Loiter(base_segment) 
    # #segment.tag = 'Reserve_Cruise'  
    # #segment.analyses.extend( analyses.base )   
    # #segment.altitude                                      = 5000 * Units.feet
    # #segment.air_speed                                     = 130 * Units.kts
    # #segment.time                                          = 60*30 * Units.sec  
    
    # ## define flight dynamics to model 
    # #segment.flight_dynamics.force_x                       = True  
    # #segment.flight_dynamics.force_z                       = True     
    
    # ## define flight controls 
    # #segment.assigned_control_variables.throttle.active               = True           
    # #segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # #segment.assigned_control_variables.body_angle.active             = True                  
       
    # #mission.append_segment(segment)     
    
    # ## ------------------------------------------------------------------
    # ##  Researve Descent
    # ## ------------------------------------------------------------------ 
    # #segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment) 
    # #segment.tag = 'Reserve_Descent'
    # #segment.analyses.extend( analyses.hex_descent_operation)    
    # #segment.altitude_end                                  = 1000 * Units.feet 
    # #segment.air_speed                                     = 110 * Units['mph']
    # #segment.descent_rate                                  = 300 * Units['ft/min']   
    
    # ## define flight dynamics to model 
    # #segment.flight_dynamics.force_x                       = True  
    # #segment.flight_dynamics.force_z                       = True     
    
    # ## define flight controls 
    # #segment.assigned_control_variables.throttle.active               = True           
    # #segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # #segment.assigned_control_variables.body_angle.active             = True                
    # #mission.append_segment(segment)  

    
    # # ------------------------------------------------------------------
    # #  Baseleg Segment Flight 1  
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    # segment.tag = 'Baseleg'
    # segment.analyses.extend( analyses.base )  
    # segment.altitude_start                                = 1000 * Units.feet
    # segment.altitude_end                                  = 500.0 * Units.feet
    # segment.air_speed_end                                 = 90 * Units['mph']  
    # segment.climb_rate                                    = -350 * Units['ft/min'] 
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                
    # mission.append_segment(segment) 

    # # ------------------------------------------------------------------
    # #  Final Approach Segment Flight 1  
    # # ------------------------------------------------------------------ 
    # segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    # segment.tag = 'Final_Approach'
    # segment.analyses.extend( analyses.base )      
    # segment.altitude_start                                = 500.0 * Units.feet
    # segment.altitude_end                                  = 00.0 * Units.feet
    # segment.air_speed_end                                 = 80 * Units['mph']  
    # segment.climb_rate                                    = -300 * Units['ft/min']   
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    # segment.assigned_control_variables.body_angle.active             = True                      
    # mission.append_segment(segment)  

    
    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------ 
    return mission



# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def payload_range_simplified_mission_setup(analyses):
    

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission' 

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment()
    base_segment.temperature_deviation  = 2.5
    base_segment.state.numerics.number_of_control_points  = 4 
    
    ## ------------------------------------------------------------------
    ##   Takeoff
    ## ------------------------------------------------------------------      
    #segment = Segments.Ground.Takeoff(base_segment)
    #segment.tag = "Takeoff"  
    #segment.analyses.extend( analyses.base )   
    #segment.velocity_start                                   = Vstall*0.1 
    #segment.velocity_end                                     = Vstall*1.2  
    #segment.friction_coefficient                             = 0.04   
    #segment.throttle                                         = 0.8   
    
    #segment.flight_dynamics.force_x                           = True 
    #segment.assigned_control_variables.elapsed_time.active               = True         
   
    #mission.append_segment(segment) 
  
    Vstall = 58 * Units.kts
    # ------------------------------------------------------------------
    #   Departure End of Runway Segment Flight 1 : 
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = 'Departure_End_of_Runway'       
    segment.analyses.extend( analyses.base )   
    segment.altitude_start                                = 0.0 * Units.feet
    segment.altitude_end                                  = 5
    0.0 * Units.feet
    segment.air_speed_start                               = Vstall *1.2  
    segment.air_speed_end                                 = Vstall *1.25
    segment.initial_battery_state_of_charge    = 1.0
            
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
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
    segment.altitude_start                                = 50.0 * Units.feet
    segment.altitude_end                                  = 500.0 * Units.feet 
    segment.air_speed_start                               = Vstall *1.25 
    segment.air_speed_end                                 = Vstall *1.3 
    segment.climb_rate                                    = 600 * Units['ft/min']   
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
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
    segment.analyses.extend( analyses.base )     
    segment.altitude_start                                = 500.0 * Units.feet
    segment.altitude_end                                  = 2500 * Units.feet  
    segment.air_speed_end                                 = 120 * Units.kts 
    segment.climb_rate                                    = 500* Units['ft/min']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
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
    segment.analyses.extend( analyses.base )   
    segment.altitude_start                                = 2500.0  * Units.feet
    segment.altitude_end                                  = 5000   * Units.feet  
    segment.air_speed_end                                 = 130 * Units.kts 
    segment.climb_rate                                    = 700.034 * Units['ft/min']   
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
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
    segment.analyses.extend( analyses.base )   
    segment.altitude                                      = 5000   * Units.feet 
    segment.air_speed                                     = 130 * Units.kts
    segment.distance                                      = 400.   * Units.nautical_mile  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                  
          
    mission.append_segment(segment)    


    # ------------------------------------------------------------------
    #   Descent Segment Flight 1   
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = "Descent"  
    segment.analyses.extend( analyses.base )       
    segment.altitude_start                                = 5000   * Units.feet 
    segment.altitude_end                                  = 1000 * Units.feet  
    segment.air_speed_end                                 = 100 * Units['mph']   
    segment.climb_rate                                    = -2000 * Units['ft/min']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
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
    segment.analyses.extend( analyses.base )   
    segment.air_speed                                     = 100 * Units['mph']   
    segment.distance                                      = 6000 * Units.feet 
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                   
            
    mission.append_segment(segment)     
    
    ## ------------------------------------------------------------------
    ##  Reserve Climb 
    ## ------------------------------------------------------------------ 
    #segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment) 
    #segment.tag = 'Reserve_Climb'        
    #segment.analyses.extend( analyses.base )      
    #segment.altitude_end                                  = 5000 * Units.feet
    #segment.air_speed                                     = 120 * Units['mph']
    #segment.climb_rate                                    = 500* Units['ft/min']  
    
    ## define flight dynamics to model 
    #segment.flight_dynamics.force_x                       = True  
    #segment.flight_dynamics.force_z                       = True     
    
    ## define flight controls 
    #segment.assigned_control_variables.throttle.active               = True           
    #segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    #segment.assigned_control_variables.body_angle.active             = True                
        
    #mission.append_segment(segment)
    
    ## ------------------------------------------------------------------
    ##  Researve Cruise Segment 
    ## ------------------------------------------------------------------ 
    #segment = Segments.Cruise.Constant_Speed_Constant_Altitude_Loiter(base_segment) 
    #segment.tag = 'Reserve_Cruise'  
    #segment.analyses.extend( analyses.base )   
    #segment.altitude                                      = 5000 * Units.feet
    #segment.air_speed                                     = 130 * Units.kts
    #segment.time                                          = 60*30 * Units.sec  
    
    ## define flight dynamics to model 
    #segment.flight_dynamics.force_x                       = True  
    #segment.flight_dynamics.force_z                       = True     
    
    ## define flight controls 
    #segment.assigned_control_variables.throttle.active               = True           
    #segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    #segment.assigned_control_variables.body_angle.active             = True                  
       
    #mission.append_segment(segment)     
    
    ## ------------------------------------------------------------------
    ##  Researve Descent
    ## ------------------------------------------------------------------ 
    #segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment) 
    #segment.tag = 'Reserve_Descent'
    #segment.analyses.extend( analyses.hex_descent_operation)    
    #segment.altitude_end                                  = 1000 * Units.feet 
    #segment.air_speed                                     = 110 * Units['mph']
    #segment.descent_rate                                  = 300 * Units['ft/min']   
    
    ## define flight dynamics to model 
    #segment.flight_dynamics.force_x                       = True  
    #segment.flight_dynamics.force_z                       = True     
    
    ## define flight controls 
    #segment.assigned_control_variables.throttle.active               = True           
    #segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    #segment.assigned_control_variables.body_angle.active             = True                
    #mission.append_segment(segment)  

    
    # ------------------------------------------------------------------
    #  Baseleg Segment Flight 1  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = 'Baseleg'
    segment.analyses.extend( analyses.base )  
    segment.altitude_start                                = 1000 * Units.feet
    segment.altitude_end                                  = 500.0 * Units.feet
    segment.air_speed_end                                 = 90 * Units['mph']  
    segment.climb_rate                                    = -350 * Units['ft/min'] 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    mission.append_segment(segment) 

    # ------------------------------------------------------------------
    #  Final Approach Segment Flight 1  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = 'Final_Approach'
    segment.analyses.extend( analyses.base )      
    segment.altitude_start                                = 500.0 * Units.feet
    segment.altitude_end                                  = 00.0 * Units.feet
    segment.air_speed_end                                 = 80 * Units['mph']  
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
    #   Mission definition complete    
    # ------------------------------------------------------------------ 
    return mission

def missions_setup(mission): 
 
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions  


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


def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle
    analyses.vehicle.mass_properties.takeoff = None

    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.overwrite_reference        = True
    geometry.settings.update_wing_properties     = True
    geometry.settings.print_weight_analysis_report = True
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Stability Analysis
    # ------------------------------------------------------------------     
    stability                                           = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method() 
    stability.settings.compute_neutral_point = True
    stability.settings.model_fuselage   = True
    analyses.append(stability)


    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------     
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_General_Aviation()
    weights.settings.update_center_of_gravity                            = False
    weights.settings.update_moment_of_inertia = True
    weights.settings.print_weight_analysis_report                        = True
    weights.method                                                       = "FLOPS"
    weights.settings.weight_correction_factors.empty.systems.furnishings = 0.5
    # weights.settings.FLOPS.fidelity                                      = "Simple"
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    
    # Calculate extra drag from landing gear: 
    main_wheel_width  = 4. * Units.inches
    main_wheel_height = 12. * Units.inches
    nose_gear_height  = 10. * Units.inches
    nose_gear_width   = 4. * Units.inches 
    total_wheel       = 2*main_wheel_width*main_wheel_height + nose_gear_width*nose_gear_height 
    main_gear_strut_height = 2. * Units.inches
    main_gear_strut_length = 24. * Units.inches
    nose_gear_strut_height = 12. * Units.inches
    nose_gear_strut_width  = 2. * Units.inches 
    total_strut = 2*main_gear_strut_height*main_gear_strut_length + nose_gear_strut_height*nose_gear_strut_width 
    drag_area = 1.4*( total_wheel + total_strut)
    
    
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.settings.drag_coefficient_increment = drag_area/vehicle.reference_area
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy()
    analyses.append(energy)

    # # ------------------------------------------------------------------
    # # Emissions 
    # # ------------------------------------------------------------------
    # emissions = RCAIDE.Framework.Analyses.Emissions.Emission_Index_CRN_Method() 
    # emissions.settings.use_surrogate     = False             
    # analyses.append(emissions)
    
    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   
 
    return analyses


def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def aviation_mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'aviation_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points = 2
    base_segment.state.numerics.solver.type = 'root_finder'

    #------------------------------------------------------------------------------------------------------------------------------------ 
    #  Takeoff Ground Run
    #------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "takeoff_ground_run" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start                                           = 0.* Units.knots
    segment.velocity_end                                             = 70.0 * Units['knots']
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
    segment.air_speed_start                                          = 80.0 * Units['knots']
    segment.altitude_start                                           = 0.0   
    segment.altitude_end                                             = 35 * Units['ft']
    segment.air_speed_end                                            = 100.0 * Units['knots']
    segment.climb_rate                                               = 150 * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment) 

    #------------------------------------------------------------------
    #   Initial Climb
    # ------------------------------------------------------------------

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "initial_climb" 
    segment.analyses.extend( analyses.cutback )  
    segment.altitude_start                                          = 35 * Units['feet'] 
    segment.altitude_end                                             = 1000  * Units['feet']
    segment.air_speed_start                                          = 80.0 * Units['knots']
    segment.air_speed_end                                                = 120.0 * Units['knots']
    segment.climb_rate                                               = 600   * Units['fpm']  
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Climb to Cruise
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_to_cruise" 
    segment.analyses.extend( analyses.cutback ) 
    segment.air_speed_start                                          = 130.0 * Units['knots'] 
    segment.altitude_end                                             = 10000   * Units['ft']
    segment.air_speed_end                                            = 140 * Units['knots']
    segment.climb_rate                                               = 600   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)

    # ------------------------------------------------------------------    
    #   Cruise
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                 = 10000 * Units['ft']  
    segment.air_speed                                                = 150 * Units['knots']
    segment.distance                                                 = 1200 * Units.km   
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Descent from Cruise
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_from_cruise" 
    segment.analyses.extend( analyses.descent ) 
    segment.altitude_start                                           = 10000 * Units.ft
    segment.altitude_end                                             = 3000   * Units.ft
    segment.air_speed                                                = 140 * Units['knots']
    segment.descent_rate                                             = 1000   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Final Desccent
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag  = "final_descent" 
    segment.analyses.extend( analyses.landing )
    segment.altitude_start                                           = 3000 * Units.ft
    segment.altitude_end                                             = 1500 * Units.ft
    segment.air_speed                                                = 120.0 * Units['knots']
    segment.descent_rate                                             = 400  * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #  Approach
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "approach"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 50.0   * Units.ft
    segment.air_speed                                                = 100.0 * Units['knots']
    segment.descent_rate                                             = 300.0   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #  Level Off
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "level_off" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 0  * Units.ft
    segment.air_speed                                                = 80 * Units['knots']
    segment.descent_rate                                             = 200   * Units['fpm']  
           
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Landing Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "landing_roll"

    segment.analyses.extend( analyses.landing ) 
    segment.velocity_start                                                = 80.0 * Units['knots']
    segment.velocity_end                                                  = 10 * Units.knots 
    segment.friction_coefficient                                          = 0.4
    segment.altitude                                                      = 0.0   
    segment.assigned_control_variables.elapsed_time.active                = True  
    segment.assigned_control_variables.elapsed_time.initial_guess_values  = [[30.]]  
    mission.append_segment(segment)     

    return mission

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()
    plt.show()
