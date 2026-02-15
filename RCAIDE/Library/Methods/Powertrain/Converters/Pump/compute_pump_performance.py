# RCAIDE/Library/Methods/Powertrain/Converters/Pump/compute_pump_performance.py
#
# 
# Created:  Sep. 2025, M. Guidotti

def compute_pump_performance(pump, line, state):

    pump_conditions = state.conditions.converters[pump.tag]
    pump_conditions.inputs.pressure = line.pressure
    pump_conditions.outputs.pressure = line.pressure + pump.delta_pressure
    pump_conditions.outputs.power = pump.mass_flow_rate * pump.delta_pressure / (line.density * pump.efficiency)

    stored_results_flag            = True
    stored_converter_tag           = pump.tag  

    pump_conditions.power.propulsive               = 0.0 * state.ones_row(1)
    pump_conditions.power.mechanical               = 0.0 * state.ones_row(1)
    pump_conditions.power.electrical               = pump_conditions.inputs.power 
    pump_conditions.power.chemical                 = 0.0 * state.ones_row(1)
    pump_conditions.power.pneumatic                = 0.0 * state.ones_row(1)
    pump_conditions.power.hydraulic                = pump_conditions.outputs.power 
    pump_conditions.power.thermal                  = 0.0 * state.ones_row(1)

    #"""Computes the performance of the pump""" 

    ## Unpack
    #pump_conditions = state.conditions.energy.converters[pump.tag]
    
    ## mass flow 
    #m_dot  = state.conditions.energy.fuel_lines[fuel_line.tag].fuel_mass_flow_rate * pump.distributor_split

    ## compute delta P
    #pressure_rise = pump.design_outlet_pressure - pump.design_inlet_pressure     

    ## volumetric flow rate
    #Q  = m_dot /pump.working_fluid.density
    
    ## mass of pump 
    #total_efficiency =  pump.pump_efficiency * pump.turbine_efficiency    

    ## hydraulic power
    #hydraulic_power  =  Q * pressure_rise     
    
    ## shaft power 
    #shaft_power      =  hydraulic_power /total_efficiency 
     
    #pump_conditions.inputs.power  = shaft_power     # shaft power 
    #pump_conditions.outputs.power = hydraulic_power # hydraulic power
    
    ## compute additional mdot required to produce power assuming it comes
    #m_dot_increment = shaft_power / pump.working_fluid.specific_energy
    
    #pump_conditions.fuel_mass_flow_rate = m_dot_increment
     
    return  pump_conditions.power, stored_results_flag, stored_converter_tag 
