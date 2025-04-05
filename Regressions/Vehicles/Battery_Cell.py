# Regressions/Vehicles/Isolated_Battery_Cell.py
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
    
    bus.battery_module_electric_configuration = electrical_config
    if cell_chemistry == 'lithium_ion_nmc': 
        battery = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC()
    elif cell_chemistry == 'lithium_ion_lfp': 
        battery = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_LFP()    
    bus.battery_modules.append(battery)  
    bus.initialize_bus_properties()
    
    #------------------------------------------------------------------------------------------------------------------------------------           
    # Payload 
    #------------------------------------------------------------------------------------------------------------------------------------  
    payload                      = RCAIDE.Library.Components.Payloads.Payload()
    payload.power_draw           = current * bus.voltage  
    payload.mass_properties.mass = 1.0 * Units.kg
    bus.payload                  = payload  
      
    # append bus   
    net.busses.append(bus) 
    
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
    charge_config.networks.electric.busses.bus.payload.power_draw =  0
    configs.append(charge_config)
   
    
    return configs 