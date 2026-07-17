# RCAIDE/Library/Methods/Powertrain/Converters/Pump/compute_pump_performance.py

# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import  RCAIDE
from RCAIDE.Framework.Core import  Units

# package imports 
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
#  compute_pump_performance
# ----------------------------------------------------------------------------------------------------------------------   
def compute_pump_performance(pump,state,fuel_line,bus):
    """Computes the performance of the pump""" 

    # Unpack
    pump_conditions = state.conditions.energy.converters[pump.tag]
    
    # mass flow 
    m_dot  = state.conditions.energy.fuel_lines[fuel_line.tag].fuel_mass_flow_rate * pump.distributor_split

    # compute delta P
    pressure_rise = pump.design_outlet_pressure - pump.design_inlet_pressure     

    # volumetric flow rate
    Q  = m_dot /pump.working_fluid.density
    
    # mass of pump 
    total_efficiency =  pump.pump_efficiency * pump.turbine_efficiency    

    # hydraulic power
    hydraulic_power  =  Q * pressure_rise     
    
    # shaft power 
    shaft_power      =  hydraulic_power /total_efficiency 
     
    pump_conditions.inputs.power  = shaft_power     # shaft power 
    pump_conditions.outputs.power = hydraulic_power # hydraulic power
    
    if state.conditions.energy.fuel_lines[fuel_line.tag].fuel_mass_flow_rate[0,0] <0.3:
        fuel_efficiency = 0.75 
    else:
        fuel_efficiency =  pump.fuel_cell_efficiency

    fuel_cellpower = shaft_power / fuel_efficiency
    fuel_cell_mass_flow = fuel_cellpower * pump.fuel_cell_flow_rate_multipier
    pump_conditions.fuel_mass_flow_rate[:] = fuel_cell_mass_flow
    
    return 0,shaft_power,None,None