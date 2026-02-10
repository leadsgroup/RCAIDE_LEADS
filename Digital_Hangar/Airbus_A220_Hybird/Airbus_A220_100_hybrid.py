# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core                                                 import Units
from RCAIDE.Library.Methods.Powertrain.Converters.Motor                    import design_optimal_motor
from RCAIDE.Library.Plots                                                  import *     
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Integral_Tank.compute_integral_tank_volume import compute_segmented_wing_integral_tank_fuel_volume
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan                 import design_turbofan  
from RCAIDE.Framework.External_Interfaces.OpenVSP.export_vsp_vehicle       import export_vsp_vehicle
from RCAIDE.Library.Methods.Performance                                    import *  
from RCAIDE.Library.Plots                                                  import *  

# Python imports 
import numpy                                 as np
import matplotlib.pyplot                     as plt
from copy                                    import deepcopy 
import os
import sys

from .Airbus_A220_100 import vehicle_setup   as  baseline_vehicle_setup 
from .Airbus_A220_100 import configs_setup   as  configs_setup 
from .Airbus_A220_100 import analyses_setup  as  analyses_setup
from .Airbus_A220_100 import missions_setup  as  missions_setup 


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():

    hybrid_power_split_ratio = .1

    fuel_type = RCAIDE.Library.Attributes.Propellants.Jet_A1()  

    # Step 1 design a vehicle
    vehicle   = vehicle_setup(fuel_type, hybrid_power_split_ratio)     
    
    # Step 2 create aircraft configuration based on vehicle 
    configs   = configs_setup(vehicle)
    
    # Step 3 set up analysis
    analyses  = analyses_setup(configs)
    
    # Step 6 plot results 
    plot_mission(results)    

    
    return

def vehicle_setup(fuel_type,hybrid_power_split_ratio):

    ospath      = os.path.abspath(__file__)
    separator   = os.path.sep
    airfoil_path    = os.path.dirname(ospath) + separator  + '..' + separator  
    local_path  = os.path.dirname(ospath) + separator       
        

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
    net                                                    = RCAIDE.Framework.Networks.Hybrid() 
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus
    #------------------------------------------------------------------------------------------------------------------------------------  
    bus                                                    = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus() 

    #------------------------------------------------------------------------------------------------------------------------------------           
    # Battery
    #------------------------------------------------------------------------------------------------------------------------------------  
    bat_module                                             = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC()
    bat_module.electrical_configuration.series             = 150
    bat_module.electrical_configuration.parallel           = 700
    bat_module.cell.nominal_capacity                       = 8
    bat_module.cell.mass                                   = 0.03

    for _ in range(2):
        bat_copy = deepcopy(bat_module)
        bus.battery_modules.append(bat_copy)

    bus.battery_module_electric_configuration = 'Parallel' 
    bus.initialize_bus_properties()
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------  
    avionics                     = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw          = 20. 
    bus.avionics                 = avionics

    # append bus   
    net.busses.append(bus)

    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                             = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan                                              = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() 
    turbofan.tag                                          = 'starboard_propulsor'
    turbofan.active_fuel_tanks                            = ['fuel_tank'] 
    turbofan.origin                                       = [[ 10.150,  5.435, -1.087]] 
    turbofan.engine_length                                = 3.175    
    turbofan.eninge_diameter                              = 2.086
    turbofan.bypass_ratio                                 = 12.5   
    turbofan.design_altitude                              = 40000.0*Units.ft
    turbofan.design_mach_number                           = 0.78   
    turbofan.design_thrust                                = 17000.0* Units.N  

    # fan                
    fan                                                   = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                               = 'fan'
    fan.polytropic_efficiency                             = 0.93
    fan.pressure_ratio                                    = 1.7   
    turbofan.fan                                          = fan        

    # working fluid                   
    turbofan.working_fluid                                = RCAIDE.Library.Attributes.Gases.Air() 
    ram                                                   = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                               = 'ram' 
    turbofan.ram                                          = ram 

    # inlet nozzle          
    inlet_nozzle                                          = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                                      = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency                    = 0.98
    inlet_nozzle.pressure_ratio                           = 0.98 
    turbofan.inlet_nozzle                                 = inlet_nozzle 

    # low pressure compressor    
    low_pressure_compressor                               = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                           = 'lpc'
    low_pressure_compressor.polytropic_efficiency         = 0.91
    low_pressure_compressor.pressure_ratio                = 1.9      

    low_pressure_compressor.motor                         = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    low_pressure_compressor.motor.tag                     =  "starboard_propulsor_low_pressure_compressor_motor"
    low_pressure_compressor.motor.nominal_voltage         = bus.voltage 
    low_pressure_compressor.motor.no_load_current         = 1 
    low_pressure_compressor.motor.resistance              = 0.002
    low_pressure_compressor.motor.efficiency              = 0.98 
    
    low_pressure_compressor.motor.design_angular_velocity = 7200*Units.rpm #
    low_pressure_compressor.motor.design_torque           = .8e6 * hybrid_power_split_ratio/low_pressure_compressor.motor.design_angular_velocity #https://mgm-compro.com/electric-motor/mgm-bldcin-400kw/#scrollDown

    design_optimal_motor(low_pressure_compressor.motor)

    turbofan.low_pressure_compressor                      = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                              = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                          = 'hpc'
    high_pressure_compressor.polytropic_efficiency        = 0.91
    high_pressure_compressor.pressure_ratio               = 10.0    
    turbofan.high_pressure_compressor                     = high_pressure_compressor

    # low pressure turbine  
    low_pressure_turbine                                  = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                              = 'lpt'
    low_pressure_turbine.mechanical_efficiency            = 0.99
    low_pressure_turbine.polytropic_efficiency            = 0.93 
    turbofan.low_pressure_turbine                         = low_pressure_turbine

    # high pressure turbine     
    high_pressure_turbine                                 = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                             = 'hpt'
    high_pressure_turbine.mechanical_efficiency           = 0.99
    high_pressure_turbine.polytropic_efficiency           = 0.93 
    turbofan.high_pressure_turbine                        = high_pressure_turbine 

    # combustor  
    combustor                                             = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                         = 'Comb'
    combustor.efficiency                                  = 0.99 
    combustor.alphac                                      = 1.0     
    combustor.turbine_inlet_temperature                   = 1600
    combustor.pressure_ratio                              = 0.95
    combustor.fuel_data                                   = fuel_type
    turbofan.combustor                                    = combustor

    # core nozzle
    core_nozzle                                           = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                       = 'core nozzle'
    core_nozzle.polytropic_efficiency                     = 0.95
    core_nozzle.pressure_ratio                            = 0.99  
    turbofan.core_nozzle                                  = core_nozzle

    # fan nozzle             
    fan_nozzle                                            = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                        = 'fan nozzle'
    fan_nozzle.polytropic_efficiency                      = 0.95
    fan_nozzle.pressure_ratio                             = 0.99 
    turbofan.fan_nozzle                                   = fan_nozzle 

    # design turbofan
    design_turbofan(turbofan)   

    # Nacelle 
    nacelle                                               = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                                      = 2.1
    nacelle.length                                        = 3.258
    nacelle.tag                                           = 'nacelle_1'
    nacelle.inlet_diameter                                = 2.08
    nacelle.origin                                        = [[ 10.150, 5.435, -1.087]] 
    nacelle.areas.wetted                                  = 1.1*np.pi*nacelle.diameter*nacelle.length
    
    nacelle_airfoil                                       = RCAIDE.Library.Components.Airfoils.Airfoil()   
    nacelle_airfoil.coordinate_file                       = airfoil_path + 'Airfoils' + separator + 'NACA_2410.txt'
    nacelle_airfoil.polar_files                           = [airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_2410_polar_Re_50000.txt' ,
                                                             airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_2410_polar_Re_100000.txt' ,
                                                              airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_2410_polar_Re_200000.txt' ,
                                                              airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_2410_polar_Re_500000.txt' ,
                                                              airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_2410_polar_Re_1000000.txt',
                                                              airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_2410_polar_Re_3500000.txt',
                                                              airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_2410_polar_Re_5000000.txt',
                                                              airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_2410_polar_Re_7500000.txt' ]
    nacelle.append_airfoil(nacelle_airfoil) 

    
    
    turbofan.nacelle                                      = nacelle
    net.propulsors.append(turbofan)

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------      
    # copy turbofan
    turbofan_2                                     = deepcopy(turbofan)
    turbofan_2.active_fuel_tanks                   = ['fuel_tank'] 
    turbofan_2.tag                                 = 'port_propulsor' 
    turbofan_2.low_pressure_compressor.motor.tag   = "port_propulsor_low_pressure_compressor_motor"
    turbofan_2.origin                              = [[10.150, -5.435, -1.087]]  
    turbofan_2.nacelle.origin                      = [[10.150, -5.435, -1.087]]
         
    # append propulsors to distribution line 
    fuel_line.assigned_propulsors                  = [['starboard_propulsor', 'port_propulsor']]
    net.propulsors.append(turbofan_2)

    net.converters.append(turbofan_2.low_pressure_compressor.motor)    
    net.converters.append(turbofan.low_pressure_compressor.motor)

    bus.assigned_converters                        = [["starboard_propulsor_low_pressure_compressor_motor" ,"port_propulsor_low_pressure_compressor_motor" ]]       
   
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Fuel Tank & Fuel
    #------------------------------------------------------------------------------------------------------------------------------------   
    inboard_tank                                   = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    inboard_tank.fuel                              = fuel_type
    inboard_tank.segments_bounding_tank            = ['root','yehudi']  
    inboard_tank.segments_percent_chord_start      = [0.15  ,0.15 ]
    inboard_tank.segments_percent_chord_end        = [0.625 ,0.625]  
    fuel_line.fuel_tanks.append(inboard_tank)
    
    outboard_tank                                  = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    outboard_tank.fuel                             = fuel_type
    outboard_tank.segments_bounding_tank           = ['yehudi', 'section_2']  
    outboard_tank.segments_percent_chord_start     = [0.15 ,0.15 ]
    outboard_tank.segments_percent_chord_end       = [0.625,0.625]  
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

if __name__ == '__main__': 
    main()
    plt.show()