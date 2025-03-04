
#RCAIDE/Library/Methods/Thermal_Management/Batteries/Liquid_Cooled_Wavy_Channel/wavy_channel_geometry_setup.py
#
# Created: Apr 2024, S. Shekar 2024

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core        import Data

# ----------------------------------------------------------------------------------------------------------------------  
#  Wavy Channel Geometry Setup 
# ----------------------------------------------------------------------------------------------------------------------   

def wavy_channel_geometry_setup(HAS,battery): 
    """ Defines a dummy vehicle for wavy channel geometry optimization.
          
          Inputs:  
             HAS   - HAS data structure             [None] 
             battery - Battery data structure
         Outputs:  
             configs - configuration used in optimization    [None]
              
          Assumptions: 
             N/A 
        
          Source:
             None
    """     
    vehicle                                                   = RCAIDE.Vehicle()  
    net                                                       = RCAIDE.Framework.Networks.Electric()
    bus                                                       = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    bus.battery_modules.append(battery)
    coolant_line                                                          = RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line(bus)     
    coolant_line.battery_modules[battery.tag].thermal_management_system   = Data() 

    coolant_line.battery_modules[battery.tag].thermal_management_system.heat_acquisition_system = HAS 

    net.busses.append(bus)
    net.coolant_lines.append(coolant_line) 
    vehicle.append_energy_network(net) 
    
    configs                                                     = RCAIDE.Library.Components.Configs.Config.Container()
    base_config                                                 = RCAIDE.Library.Components.Configs.Config(vehicle)  
    config                                                      = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                                                  = 'optimized'  
    configs.append(config)         
    return configs 