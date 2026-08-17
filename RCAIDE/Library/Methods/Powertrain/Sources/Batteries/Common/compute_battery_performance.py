# RCAIDE/Methods/Powertrain/Sources/Batteries/Common/compute_battery_pack_performance.py
# 
# 
# Created:  Feb 2024, M. Clarke
# Modified: Sep 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
# compute_nmc_cell_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_battery_performance(battery,state,network):
    """Computes battery performance for the electrical distributor it is assigned to.

    Notes
    -----
    Looks up its own distributor's psi (rather than a vehicle-wide value)
    since a battery may share the vehicle with other electrically-isolated
    buses resolved to a different split. A battery is always processed
    after everything on its bus (sources run last in
    ``Network.evaluate()``), so that bus's demand is already fully known --
    no solver unknown is needed, the same as ``Fuel_Tank`` reading its fuel
    line's demand directly.
    """
    battery_conditions = state.conditions.energy.sources[battery.tag]

    electrical_distributor_tag = None
    for d_tag in battery.assigned_distributors[0]:
        if network.distributors[d_tag].domain == 'electrical':
            electrical_distributor_tag = d_tag
    psi = state.conditions.energy.battery_fuel_cell_power_split_ratio[electrical_distributor_tag]
    
    # Reset accumulators before summing across modules
    battery_conditions.energy[:,0]                = 0.0
    battery_conditions.heat_energy_generated[:,0] = 0.0

    stored_module_tag = None
    for m_i, module in enumerate(battery.modules):
        if module.active:
            # determine system voltages for battery and module 
            battery_voltage = battery.voltage * state.ones_row(1) 
            if battery.battery_module_electric_configuration == 'Series': 
                module_voltage          =  battery_voltage / len(battery.modules)
            elif battery.battery_module_electric_configuration == 'Parallel':    
                module_voltage          = battery_voltage 
            
            # determine power flow
            if state.conditions.energy.recharging:
                battery_conditions.outputs.power.electrical             = (battery.nominal_capacity * battery.charging_c_rate* battery_voltage*battery_conditions.power_split_ratio)
                battery_conditions[module.tag].inputs.power.electrical  = battery_conditions.outputs.power.electrical/ battery.number_of_active_modules
                battery_conditions[module.tag].current_draw             = battery_conditions[module.tag].inputs.power.electrical  / module_voltage
            else:
                total_electrical_demand = state.conditions.energy.distributors[electrical_distributor_tag].outputs.power.electrical
                battery_conditions.outputs.power.electrical              = total_electrical_demand * battery_conditions.power_split_ratio * psi
                battery_conditions[module.tag].outputs.power.electrical  = battery_conditions.outputs.power.electrical / battery.number_of_active_modules
                battery_conditions[module.tag].current_draw              = battery_conditions[module.tag].outputs.power.electrical / module_voltage
            
            # compute battery module performance 
            battery_conditions[module.tag].power_draw   = battery_conditions[module.tag].outputs.power.electrical -  battery_conditions[module.tag].inputs.power.electrical      
            if (battery.identical_modules == False or m_i == 0):  
                module_inputs, module_outputs, stored_results_flag, stored_module_tag = module.compute_performance(battery,state,network)
            else: 
                module_inputs, module_outputs = module.reuse_stored_data(state,battery.tag,stored_module_tag)                
            
            # aggregate module conditions to battery 
            battery_conditions.temperature              =   battery_conditions[module.tag].temperature 
            battery_conditions.energy                   +=  battery_conditions[module.tag].energy
            battery_conditions.state_of_charge          =   battery_conditions[module.tag].state_of_charge  
            battery_conditions.heat_energy_generated    +=  battery_conditions[module.tag].heat_energy_generated       

            if battery.battery_module_electric_configuration == 'Series': 
                battery_conditions.voltage_open_circuit  +=  battery_conditions[module.tag].voltage_open_circuit 
                battery_conditions.voltage_under_load    +=  battery_conditions[module.tag].voltage_under_load 
            elif battery.battery_module_electric_configuration == 'Parallel': 
                battery_conditions.voltage_open_circuit  = battery_conditions[module.tag].voltage_open_circuit 
                battery_conditions.voltage_under_load    = battery_conditions[module.tag].voltage_under_load  
       
            if state.conditions.energy.recharging:
                fully_charged = battery_conditions.state_of_charge                    == 1
                battery_conditions.charging_current[fully_charged]                    = 0
                battery_conditions[module.tag].power_draw[fully_charged]              = 0
                battery_conditions[module.tag].current[fully_charged]                 = 0
                battery_conditions[module.tag].inputs.power.electrical[fully_charged] = 0
            
    stored_results_flag     = True
    stored_battery_tag      = battery.tag
    
    return battery_conditions.inputs, battery_conditions.outputs, stored_results_flag, stored_battery_tag 
 