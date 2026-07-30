# RCAIDE/Library/Methods/Thermal_Management/Heat_Exchangers/Cross_Flow_Heat_Exchanger/cross_flow_heat_exchanger_geometry_setup.py
#
# Created: Jun 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from copy import  deepcopy

# ----------------------------------------------------------------------------------------------------------------------  
#  Cross Flow Heat Exchanger Geometry Setup 
# ----------------------------------------------------------------------------------------------------------------------   
def cross_flow_heat_exchanger_geometry_setup(HEX,coolant_line_base): 
    """ Modifies geometry of Cross Flow Heat Exchanger  
          
          Inputs:  
             nexus     - RCAIDE optmization framework with Wavy Channel geometry data structure [None]
              
          Outputs:   
             procedure - optimization methodology                                       
              
          Assumptions: 
             N/A 
        
          Source:
             None
    """            
    vehicle                = RCAIDE.Vehicle()
    net                    = RCAIDE.Framework.Networks.Electric()
    coolant_line           = deepcopy(coolant_line_base)

    HEX.coolant_temperature_of_hot_fluid                  = 323
    HEX.design_heat_removed                               = 100000
    coolant_line.heat_exchangers.append(HEX)

    net.distributors.append(coolant_line)
    vehicle.append_energy_network(net)
    

    
    configs                             = RCAIDE.Library.Components.Configs.Config.Container()
    base_config                         = RCAIDE.Library.Components.Configs.Config(vehicle)  
    config                              = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                          = 'optimized'  
    configs.append(config)         
    return configs 