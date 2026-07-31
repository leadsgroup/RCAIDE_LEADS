# VnV/Vehicles/Isolated_Battery_Cell.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE  
from RCAIDE.Framework.Core                                    import Units 
 
# ----------------------------------------------------------------------------------------------------------------------
#  Build the Vehicle
# ----------------------------------------------------------------------------------------------------------------------   

def vehicle_setup(current,C_rat,cell_chemistry,electrical_config): 

    vehicle                       = RCAIDE.Vehicle() 
    vehicle.tag                   = 'battery'   
    vehicle.reference_area        = 1
 
    # ################################################# Vehicle-level Properties #####################################################   
    # mass properties
    vehicle.mass_properties.takeoff         = 1 * Units.kg 
    vehicle.mass_properties.max_takeoff     = 1 * Units.kg 
         
    net                              = RCAIDE.Framework.Networks.Electric()
    net.charging_power               = 20 # Watt
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus
    #------------------------------------------------------------------------------------------------------------------------------------  
    bus                                       = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    bus.tag                                   = 'bus'

    battery_pack                                     = RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack()
    battery_pack.battery_module_electric_configuration = electrical_config
    if cell_chemistry == 'lithium_ion_nmc':
        battery = RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Lithium_Ion_NMC()
    elif cell_chemistry == 'lithium_ion_lfp':
        battery = RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Lithium_Ion_LFP()
    battery_pack.append_module(battery)
    battery_pack.assigned_distributors = [[bus.tag]]
    net.sources.append(battery_pack)
    battery_pack.initialize(net)

    #------------------------------------------------------------------------------------------------------------------------------------
    # Systems
    #------------------------------------------------------------------------------------------------------------------------------------
    systems                     = RCAIDE.Library.Components.Powertrain.Systems.Systems()
    systems.tag                 = 'systems'
    systems.power_draw          = current * battery_pack.voltage
    systems.assigned_distributors = [[bus.tag]]
    net.systems.append(systems)

    # append bus
    net.distributors.append(bus)
    
    # append network 
    vehicle.append_energy_network(net)
    
    return vehicle


def configs_setup(vehicle): 
    configs         = RCAIDE.Library.Components.Configs.Config.Container()  
    discharge_config     = RCAIDE.Library.Components.Configs.Config(vehicle)
    discharge_config.tag = 'discharge' 
    configs.append(discharge_config)
    
    charge_config     = RCAIDE.Library.Components.Configs.Config(vehicle)
    charge_config.tag = 'charge'
    charge_config.networks.electric.systems.systems.power_draw =  0
    configs.append(charge_config)
   
    
    return configs 