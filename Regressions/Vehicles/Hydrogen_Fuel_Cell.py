# Regressions/Vehicles/Hydrogen_Fuel_Cell.py
# 
# 
# Created:   Jan 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core  import Units ,  Data 
 
# python imports 
import numpy as np 
from copy import deepcopy
import os
# ----------------------------------------------------------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------------------------------------------------------
def vehicle_setup(fuel_cell_model):  

    vehicle                       = RCAIDE.Vehicle() 
    vehicle.tag                   = 'hydrogen_fuel_cell'   
    vehicle.reference_area        = 1
  
    # mass properties
    vehicle.mass_properties.takeoff         = 1 * Units.kg 
    vehicle.mass_properties.max_takeoff     = 1 * Units.kg 
         
    net                              = RCAIDE.Framework.Networks.Electric()  

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus and Crogenic Line 
    #------------------------------------------------------------------------------------------------------------------------------------  
    bus = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()  
    if fuel_cell_model == 'PEM': 
        bus.fuel_cell_stack_electric_configuration =  "Series"
        bus.identical_fuel_cell_stacks             = False
        fuel_cell_stack = RCAIDE.Library.Components.Powertrain.Converters.Proton_Exchange_Membrane_Fuel_Cell() 
        bus.fuel_cell_stacks.append(fuel_cell_stack)  
    if fuel_cell_model == 'Larminie':
        bus.fuel_cell_stack_electric_configuration =  "Parallel"
        
        fuel_cell_stack_1 = RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack()  
        bus.fuel_cell_stacks.append(fuel_cell_stack_1)

        fuel_cell_stack_2 = RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack()  
        bus.fuel_cell_stacks.append(fuel_cell_stack_2)
        
    bus.initialize_bus_properties()
        
    #------------------------------------------------------------------------------------------------------------------------------------           
    # Payload 
    #------------------------------------------------------------------------------------------------------------------------------------  
    payload                      = RCAIDE.Library.Components.Payloads.Payload()
    payload.power_draw           = 50
    payload.mass_properties.mass = 1.0 * Units.kg
    bus.payload                  = payload  
       
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Crogenic Tank
    #------------------------------------------------------------------------------------------------------------------------------------       
    cryogenic_tank = RCAIDE.Library.Components.Powertrain.Sources.Cryogenic_Tanks.Cryogenic_Tank()  
    bus.cryogenic_tanks.append(cryogenic_tank)     

    # append bus   
    net.busses.append(bus)
    
    # append network 
    vehicle.append_energy_network(net) 
  
    return vehicle

# ---------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):

    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------  
    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'discharge'  
    configs.append(base_config)   
    
    # done!
    return configs
