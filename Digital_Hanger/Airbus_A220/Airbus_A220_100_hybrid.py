# Airbus A220-100
# 
# 
# Created:  Mar 2024, S. Shekar 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core                                                 import Units
# from RCAIDE.Library.Methods.Geometry.Planform import compute_segment_volume, wing_planform
from RCAIDE.Library.Methods.Powertrain.Converters.Motor import design_optimal_motor
from RCAIDE.Library.Plots                                                  import *     
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Integral_Tank.compute_integral_tank_volume import compute_segmented_wing_integral_tank_fuel_volume
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan                 import design_turbofan  
from RCAIDE.Framework.External_Interfaces.OpenVSP.export_vsp_vehicle import export_vsp_vehicle

from RCAIDE.Library.Methods.Performance import *  
from RCAIDE.Library.Plots import *  

# Python imports 
import numpy                                               as np
import matplotlib.pyplot                                   as plt
from copy                                                  import deepcopy 
import os
import sys

# sys.path.append(os.path.abspath(os.path.join(os.path.join(sys.path[0]), "../../Aircraft/Airbus_A220")))
from .Airbus_A220_100 import vehicle_setup   as  baseline_vehicle_setup 
from .Airbus_A220_100 import configs_setup   as  configs_setup 
from .Airbus_A220_100 import analyses_setup  as  analyses_setup
from .Airbus_A220_100 import missions_setup  as  missions_setup 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():

    hybrid_power_split_ratio = .1

    fuel_type                = RCAIDE.Library.Attributes.Propellants.Jet_A1()  

    # Step 1 design a vehicle
    vehicle  = vehicle_setup(fuel_type, hybrid_power_split_ratio)     

    # export_vsp_vehicle(vehicle, 'A220_hybrid') 
    
    # Step 2 create aircraft configuration based on vehicle 
    configs  = configs_setup(vehicle)
    
    # Step 3 set up analysis
    analyses = analyses_setup(configs)
    
    # Step 4 set up a flight mission
    mission = mission_setup(analyses, hybrid_power_split_ratio=hybrid_power_split_ratio)
    missions = missions_setup(mission) 

    # Step 5 execute flight profile
    results = missions.base_mission.evaluate()  
    
    # Step 6 plot results 
    plot_mission(results)    

    # # plot vehicle 
    # plot_3d_vehicle(vehicle,
    #                 side_view = True )   
    
    return

def vehicle_setup(fuel_type,hybrid_power_split_ratio): 

    # IMPORT CONVENTIONAL AIRCRAFT
    vehicle = baseline_vehicle_setup()
    tank = Data()
    tank.segment = Data()
    tank.segment.percent_chord_start_location = 0.2
    tank.segment.percent_chord_end_location   = 0.75
            
    wing_volume_possible = compute_segmented_wing_integral_tank_fuel_volume(vehicle.wings.main_wing,
                                                                            vehicle.wings.main_wing.segments.root,
                                                                            vehicle.wings.main_wing.segments.tip, tank)
    # REMOVE NETWORK 
    vehicle.networks.clear()

    # ################################################# Energy Network #######################################################         

    #------------------------------------------------------------------------------------------------------------------------- 
    #  Turbofan Network
    #-------------------------------------------------------------------------------------------------------------------------   
    net                                         = RCAIDE.Framework.Networks.Hybrid() 
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus
    #------------------------------------------------------------------------------------------------------------------------------------  
    bus                                        = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus() 

    #------------------------------------------------------------------------------------------------------------------------------------           
    # Battery
    #------------------------------------------------------------------------------------------------------------------------------------  
    bat_module                                             = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC()
    bat_module.electrical_configuration.series             = 150
    bat_module.electrical_configuration.parallel           = 700
    bat_module.cell.nominal_capacity                       = 8
    bat_module.cell.mass                                   = 0.03
    # bat_module.geometrtic_configuration.normal_count       = 42 
    # bat_module.geometrtic_configuration.parallel_count     = 100 * 50

    for _ in range(2):
        bat_copy = deepcopy(bat_module)
        bus.battery_modules.append(bat_copy)

    bus.battery_module_electric_configuration = 'Parallel' 
    bus.initialize_bus_properties()
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------  
    avionics                     = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw          = 20. # Watts
    bus.avionics                 = avionics

    # append bus   
    net.busses.append(bus)

    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                   = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() 
    turbofan.tag                                = 'starboard_propulsor'
    turbofan.active_fuel_tanks                  = ['fuel_tank'] 
    turbofan.origin                             = [[ 10.150,  5.435, -1.087]] 
    turbofan.engine_length                      = 3.175    
    turbofan.eninge_diameter                    = 2.086
    turbofan.bypass_ratio                       = 12.5   
    turbofan.design_altitude                    = 40000.0*Units.ft
    turbofan.design_mach_number                 = 0.78   
    turbofan.design_thrust                      = 17000.0* Units.N  

    # fan                
    fan                                         = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                     = 'fan'
    fan.polytropic_efficiency                   = 0.93
    fan.pressure_ratio                          = 1.7   
    turbofan.fan                                = fan        

    # working fluid                   
    turbofan.working_fluid                      = RCAIDE.Library.Attributes.Gases.Air() 
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
    low_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                   = 'lpc'
    low_pressure_compressor.polytropic_efficiency = 0.91
    low_pressure_compressor.pressure_ratio        = 1.9      

    low_pressure_compressor.motor                  = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    low_pressure_compressor.motor.tag              =  "starboard_propulsor_low_pressure_compressor_motor"
    low_pressure_compressor.motor.nominal_voltage  = bus.voltage 
    low_pressure_compressor.motor.no_load_current  = 1 
    low_pressure_compressor.motor.resistance       = 0.002
    low_pressure_compressor.motor.efficiency       = 0.98 
    
    low_pressure_compressor.motor.design_angular_velocity = 7200*Units.rpm #
    low_pressure_compressor.motor.design_torque = .8e6 * hybrid_power_split_ratio/low_pressure_compressor.motor.design_angular_velocity #https://mgm-compro.com/electric-motor/mgm-bldcin-400kw/#scrollDown

    design_optimal_motor(low_pressure_compressor.motor)

    turbofan.low_pressure_compressor              = low_pressure_compressor

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
    combustor.turbine_inlet_temperature            = 1600
    combustor.pressure_ratio                       = 0.95
    combustor.fuel_data                            = fuel_type
    turbofan.combustor                             = combustor

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
    nacelle.diameter                            = 2.1
    nacelle.length                              = 3.258
    nacelle.tag                                 = 'nacelle_1'
    nacelle.inlet_diameter                      = 2.08
    nacelle.origin                              = [[ 10.150, 5.435, -1.087]] 
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
    turbofan_2.active_fuel_tanks                = ['fuel_tank'] 
    turbofan_2.tag                              = 'port_propulsor' 
    turbofan_2.low_pressure_compressor.motor.tag = "port_propulsor_low_pressure_compressor_motor"
    turbofan_2.origin                           = [[10.150, -5.435, -1.087]]  
    turbofan_2.nacelle.origin                   = [[10.150, -5.435, -1.087]]
         
    # append propulsors to distribution line 
    fuel_line.assigned_propulsors = [['starboard_propulsor', 'port_propulsor']]
    net.propulsors.append(turbofan_2)

    net.converters.append(turbofan_2.low_pressure_compressor.motor)    
    net.converters.append(turbofan.low_pressure_compressor.motor)

    bus.assigned_converters  = [["starboard_propulsor_low_pressure_compressor_motor" ,"port_propulsor_low_pressure_compressor_motor" ]]       
   
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Fuel Tank & Fuel
    #------------------------------------------------------------------------------------------------------------------------------------   
    inboard_tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    inboard_tank.fuel                         = fuel_type
    inboard_tank.segments_bounding_tank       = ['root','yehudi']  
    inboard_tank.segments_percent_chord_start = [0.15  ,0.15 ]
    inboard_tank.segments_percent_chord_end   = [0.625 ,0.625]  
    fuel_line.fuel_tanks.append(inboard_tank)
    
    outboard_tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    outboard_tank.fuel                         = fuel_type
    outboard_tank.segments_bounding_tank       = ['yehudi', 'section_2']  
    outboard_tank.segments_percent_chord_start = [0.15 ,0.15 ]
    outboard_tank.segments_percent_chord_end   = [0.625,0.625]  
    fuel_line.fuel_tanks.append(outboard_tank)        
 
    # Append fuel line to Network      
    net.fuel_lines.append(fuel_line)   

    # Append energy network to aircraft 
    vehicle.append_energy_network(net)   


    # Battery Mass and Volume Checks 
    # ------------------------------------
    # Cell volume in m^3
    # ------------------------------------
    width_cm = 2.8
    height_cm = 2.8
    length_cm = 6.5

    V_cell = (width_cm * 1e-2) * (height_cm * 1e-2) * (length_cm * 1e-2)
    V_pack =   V_cell* bat_module.electrical_configuration.series * bat_module.electrical_configuration.parallel*2     
    if V_pack> wing_volume_possible:
        print('you screwed')
    
    #------------------------------------------------------------------------------------------------------------------------- 
    # Done ! 
    #------------------------------------------------------------------------------------------------------------------------- 

    return vehicle
 
def mission_setup(analyses, hybrid_power_split_ratio):
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
    segment.velocity_start           = 30.* Units.knots
    segment.velocity_end             = 167.0 * Units['knots']
    segment.friction_coefficient     = 0.03
    segment.altitude                 = 0.0   
    segment.throttle                 = 1.0
    segment.initial_battery_state_of_charge = 1.0
    segment.hybrid_power_split_ratio  = hybrid_power_split_ratio
    segment.battery_fuel_cell_power_split_ratio = 1 

    segment.assigned_control_variables.ground_velocity.active  = True  
    segment.assigned_control_variables.ground_velocity.bounds  = [[-2, 120]]

    mission.append_segment(segment)
      
    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    
    
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Takeoff_Climb" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_start                                           = 0.0 * Units['knots'] 
    segment.altitude_end                                             = 35 * Units['ft']
    segment.air_speed_end                                            = 167.0 * Units['knots']
    segment.air_speed_start                                          = 175.0 * Units['knots']
    segment.climb_rate                                               = 250 * Units['fpm'] 
    segment.hybrid_power_split_ratio  = hybrid_power_split_ratio 
    segment.battery_fuel_cell_power_split_ratio = 1 
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment) 

    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Inital_Climb" 
    segment.analyses.extend( analyses.cutback )  
    segment.air_speed_start                                            = 0.0 * Units['knots'] 
    segment.altitude_end                                             = 1000  * Units['feet']
    segment.air_speed                                                = 200.0 * Units['knots']
    segment.climb_rate                                               = 1800   * Units['fpm'] 
    segment.hybrid_power_split_ratio  = hybrid_power_split_ratio 
    segment.battery_fuel_cell_power_split_ratio = 1 
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_1" 
    segment.analyses.extend( analyses.cutback ) 
    segment.air_speed_start                                          = 200.0 * Units['knots'] 
    segment.altitude_end                                             = 8000   * Units['ft']
    segment.air_speed_end                                            = 300 * Units['knots']
    segment.climb_rate                                               = 1700   * Units['fpm'] 
    segment.hybrid_power_split_ratio  = hybrid_power_split_ratio 
    segment.battery_fuel_cell_power_split_ratio = 1 
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                             = 16000   * Units['ft']
    segment.air_speed                                                = 350 * Units['knots']
    segment.climb_rate                                               = 1300   * Units['fpm']  
    segment.hybrid_power_split_ratio  = hybrid_power_split_ratio
    segment.battery_fuel_cell_power_split_ratio = 1 
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)


    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_3" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                             = 35000   * Units['ft']
    segment.air_speed                                                = 450 * Units['knots']
    segment.climb_rate                                               = 700   * Units['fpm']  
    segment.hybrid_power_split_ratio  = hybrid_power_split_ratio
    segment.battery_fuel_cell_power_split_ratio = 1 
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment) 

    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                 = 35000 * Units['ft']  
    segment.air_speed                                                = 450 * Units['knots']
    segment.distance                                                 = 2700*Units.nmi
    segment.hybrid_power_split_ratio  = hybrid_power_split_ratio
    segment.battery_fuel_cell_power_split_ratio = 1 
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.descent ) 
    segment.altitude_end                                             = 10000   * Units.ft
    segment.air_speed                                                = 380 * Units['knots']
    segment.descent_rate                                             = 1850   * Units['fpm']  
    segment.hybrid_power_split_ratio  = hybrid_power_split_ratio
    segment.battery_fuel_cell_power_split_ratio = 1 
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag  = "approach" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 2000 * Units.ft
    segment.air_speed                                                = 225.0 * Units['knots']
    segment.descent_rate                                             = 650  * Units['fpm']
    segment.hybrid_power_split_ratio  = hybrid_power_split_ratio  
    segment.battery_fuel_cell_power_split_ratio = 1 
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "final_approach"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = .0   * Units.ft
    segment.air_speed                                                = 175.0 * Units['knots']
    segment.descent_rate                                             = 600.0   * Units['fpm']
    segment.hybrid_power_split_ratio  = hybrid_power_split_ratio  
    segment.battery_fuel_cell_power_split_ratio = 1 
            
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
    segment.tag = "Landing"

    segment.analyses.extend( analyses.reverse_thrust ) 
    segment.velocity_start                                                = 160.0 * Units['knots']
    segment.velocity_end                                                  = 30 * Units.knots 
    segment.friction_coefficient                                          = 0.4
    segment.altitude                                                      = 0.0   
    segment.assigned_control_variables.elapsed_time.active                = True  
    segment.assigned_control_variables.elapsed_time.initial_guess_values  = [[30.]]  
    segment.hybrid_power_split_ratio  = hybrid_power_split_ratio
    segment.battery_fuel_cell_power_split_ratio = 1 
    mission.append_segment(segment)     

 
    return mission
# --------------------------
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

    plot_battery_cell_conditions(results)
        
    return

if __name__ == '__main__': 
    main()
    plt.show()