# VnV/Vehicles/Hydrogen_Fuel_Cell.py
# 
# 
# Created:   Jan 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core  import Units ,  Data
from RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Common import design_fuel_cell

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
    vehicle.mass_properties.operating_empty = 1 * Units.kg

    net                              = RCAIDE.Framework.Networks.Electric()

    #------------------------------------------------------------------------------------------------------------------------------------
    # Bus and Cryogenic Line
    #------------------------------------------------------------------------------------------------------------------------------------
    bus = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()

    fuel_line               = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    fuel_line.working_fluid = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()

    fuel_cell_stacks = []
    if fuel_cell_model == 'PEM':
        fuel_cell_stack = RCAIDE.Library.Components.Powertrain.Converters.Proton_Exchange_Membrane_Fuel_Cell()
        fuel_cell_stacks.append(fuel_cell_stack)
    if fuel_cell_model == 'Larminie':
        fuel_cell_stack_1 = RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack()
        fuel_cell_stack_1.tag = 'fuel_cell_stack_1'
        fuel_cell_stack_1.power_split_ratio = 0.5
        fuel_cell_stacks.append(fuel_cell_stack_1)

        fuel_cell_stack_2 = RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack()
        fuel_cell_stack_2.tag = 'fuel_cell_stack_2'
        fuel_cell_stack_2.power_split_ratio = 0.5
        fuel_cell_stacks.append(fuel_cell_stack_2)

    for stack in fuel_cell_stacks:
        design_fuel_cell(stack)
        # linked to both the fuel line (hydrogen supply) and the electrical bus (power extraction)
        stack.assigned_distributors = [[fuel_line.tag, bus.tag]]
        net.converters.append(stack)

    bus_nominal_voltage = fuel_cell_stacks[0].voltage

    #------------------------------------------------------------------------------------------------------------------------------------
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------
    avionics                       = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw            = 50. # Watts
    avionics.assigned_distributors = [[bus.tag]]
    net.systems.append(avionics)

    #------------------------------------------------------------------------------------------------------------------------------------
    # Cryogenic Tank
    #------------------------------------------------------------------------------------------------------------------------------------
    cryogenic_tank                      = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank()
    cryogenic_tank.fuel                 = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    cryogenic_tank.assigned_distributors = [[fuel_line.tag]]
    net.sources.append(cryogenic_tank)

    # append bus and fuel line
    net.distributors.append(bus)
    net.distributors.append(fuel_line)

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
