# RCAIDE/Methods/Powertrain/Converters/Fuel_Cells/Common/compute_stack_properties.py
# 
# 
# Created:   Nov 2024, M. Clarke 
# Modified:  Jan 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Larminie_Model import  compute_power, compute_voltage
from RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Proton_Exchange_Membrane.compute_fuel_cell_performance import  evaluate_PEM ,  evaluate_max_gross_power, set_rated_current_density 
import  scipy as  sp
import  numpy as  np
import matplotlib.pyplot as  plt
import matplotlib.cm as cm

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Stack Properties
# ----------------------------------------------------------------------------------------------------------------------  
def compute_stack_properties(fuel_cell_stack):  
    """Calculate fuel_cell_stack level properties of battery module using cell 
    properties and module configuraton
    
    Assumptions: 
    Inputs:
    mass              
    fuel_cell_stack.cell
      nominal_capacity        [amp-hours]            
      nominal_voltage         [volts]
      pack_config             [unitless]
      mass                    [kilograms]
                          
    Outputs:              
     fuel_cell_stack.             
       maximum_energy         [watt-hours]
       maximum_power              [watts]
       initial_maximum_energy [watt-hours]
       specific_energy        [watt-hours/kilogram]
       charging_voltage       [volts]
       mass_properties.    
        mass                  [kilograms] 
    """
     
    
    series_e           = fuel_cell_stack.electrical_configuration.series
    parallel_e         = fuel_cell_stack.electrical_configuration.parallel
    n_total            = parallel_e *series_e
    normal_count       = fuel_cell_stack.geometrtic_configuration.normal_count  
    parallel_count     = fuel_cell_stack.geometrtic_configuration.parallel_count
    stacking_rows      = fuel_cell_stack.geometrtic_configuration.stacking_rows

    if int(parallel_e*series_e) != int(normal_count*parallel_count):
        raise Exception('Number of cells in gemetric layout not equal to number of cells in electric circuit configuration ')
         
    normal_spacing     = fuel_cell_stack.geometrtic_configuration.normal_spacing   
    parallel_spacing   = fuel_cell_stack.geometrtic_configuration.parallel_spacing
    volume_factor      = fuel_cell_stack.volume_packaging_factor 
    euler_angles       = fuel_cell_stack.orientation_euler_angles
    fuel_cell_length   = fuel_cell_stack.fuel_cell.length 
    fuel_cell_width    = fuel_cell_stack.fuel_cell.width   
    fuel_cell_height   = fuel_cell_stack.fuel_cell.height    
    
    x1 =  normal_count * (fuel_cell_length +  normal_spacing) * volume_factor # distance in the module-level normal direction
    x2 =  parallel_count *  (fuel_cell_width +parallel_spacing) * volume_factor # distance in the module-level parallel direction
    x3 =  fuel_cell_height * volume_factor # distance in the module-level height direction 

    length = x1 / stacking_rows
    width  = x2
    height = x3 *stacking_rows     
    
    if  euler_angles[0] == (np.pi / 2):
        x1prime      = x2
        x2prime      = -x1
        x3prime      = x3 
    if euler_angles[1] == (np.pi / 2):
        x1primeprime = -x3prime
        x2primeprime = x2prime
        x3primeprime = x1prime
    if euler_angles[2] == (np.pi / 2):
        length       = x1primeprime
        width        = x3primeprime
        height       = -x2primeprime

    # store length, width and height
    fuel_cell_stack.length = length
    fuel_cell_stack.width  = width
    fuel_cell_stack.height = height 
    fuel_cell              = fuel_cell_stack.fuel_cell 
     
    if type(fuel_cell_stack) == RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack:
        lb                                     = 0.0001/(Units.cm**2.)    #lower bound on fuel cell current density
        ub                                     = 1.2/(Units.cm**2.)
        sign                                   = -1. # used to minimize -power 
        maximum_current_density                = sp.optimize.fminbound(compute_power, lb, ub, args=(fuel_cell, sign)) 
        P_fuel_cell                            = compute_power(maximum_current_density,fuel_cell)  
        V_fuel_cell                            = compute_voltage(fuel_cell,maximum_current_density)  # useful voltage vector 
        efficiency                             = np.divide(V_fuel_cell, fuel_cell.ideal_voltage)
        mdot_H2                                = np.divide(P_fuel_cell,np.multiply(fuel_cell.propellant.specific_energy,efficiency))
        
        # store properties
        fuel_cell.rated_current_density       = maximum_current_density
        fuel_cell.volume                       = fuel_cell.interface_area*fuel_cell.wall_thickness
        fuel_cell.mass                         = fuel_cell.volume*fuel_cell.cell_density*fuel_cell.porosity_coefficient  
        fuel_cell.density                      = fuel_cell.mass/fuel_cell.volume                      
        fuel_cell.specific_power               = fuel_cell.max_power/fuel_cell.mass  
  
        fuel_cell_stack.mass_properties.mass   = n_total*fuel_cell.mass 
        fuel_cell_stack.voltage                = V_fuel_cell  * series_e
        fuel_cell_stack.maximum_voltage        = V_fuel_cell  * series_e
        fuel_cell_stack.maximum_power          = P_fuel_cell * series_e 
        fuel_cell_stack.maximum_current        = fuel_cell_stack.maximum_power / fuel_cell_stack.maximum_voltage
        fuel_cell_stack.maximum_fuel_flow_rate = mdot_H2 * n_total
    
    elif type(fuel_cell_stack) == RCAIDE.Library.Components.Powertrain.Converters.Proton_Exchange_Membrane_Fuel_Cell: 
    
        # check if mach number and temperature are passed
        design_altitude =  0
        
        # call the atmospheric model to get the conditions at the specified altitude
        atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        atmo_data  = atmosphere.compute_values(design_altitude) 
         
        segment                          = RCAIDE.Framework.Mission.Segments.Segment()
        segment.hybrid_power_split_ratio = 1.0
        segment.state.conditions         = RCAIDE.Framework.Mission.Common.Results()   
        bus                              = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus() 
        bus.fuel_cell_stacks.append(fuel_cell_stack)   
        
        bus.append_operating_conditions(segment)
        for fuel_cell_stack in  bus.fuel_cell_stacks: 
            fuel_cell_stack.append_operating_conditions(segment,bus)      
            
        for tag, bus_item in  bus.items():  
            if issubclass(type(bus_item), RCAIDE.Library.Components.Component):
                bus_item.append_operating_conditions(segment,bus)
 
        for cryogenic_tank in  bus.cryogenic_tanks: 
            cryogenic_tank.append_operating_conditions(segment,bus)
      
        # compute fuel cell performance             
        t_idx                                                                    =  0
        fuel_cell_stack_conditions                                               = segment.state.conditions.energy[bus.tag].fuel_cell_stacks[fuel_cell_stack.tag]
        fuel_cell_stack_conditions.fuel_cell.stagnation_temperature[t_idx, 0]    = atmo_data.temperature   
        fuel_cell_stack_conditions.fuel_cell.stagnation_pressure[t_idx, 0]       = atmo_data.pressure   
        fuel_cell_stack_conditions.fuel_cell.pressure_drop[t_idx, 0]             = fuel_cell.rated_p_drop_fc
        fuel_cell_stack_conditions.fuel_cell.stack_temperature[t_idx, 0]         = fuel_cell.stack_temperature 
        rated_current_density, rated_power_density                               = evaluate_max_gross_power(fuel_cell_stack,fuel_cell_stack_conditions,t_idx)
        set_rated_current_density(fuel_cell_stack, rated_current_density, rated_power_density)
        
        fuel_cell_stack_conditions.fuel_cell.current_density[t_idx] = rated_current_density
        m_dot_H2, V_fuel_cell, P_fuel_cell, _, _, _, _, _,_  =  evaluate_PEM(fuel_cell_stack,fuel_cell_stack_conditions, t_idx)
   
        # store properties
        area_square_meters                     = fuel_cell.interface_area * 0.0001
        fuel_cell.volume                       = area_square_meters*fuel_cell.wall_thickness
        fuel_cell.mass                         = fuel_cell.volume*fuel_cell.cell_density*fuel_cell.porosity_coefficient  
        fuel_cell.density                      = fuel_cell.mass/fuel_cell.volume                      
        fuel_cell.specific_power               = fuel_cell.max_power/fuel_cell.mass  
        fuel_cell_stack.mass_properties.mass   = n_total*fuel_cell.mass 
        fuel_cell_stack.voltage                = V_fuel_cell  * series_e
        fuel_cell_stack.maximum_voltage        = V_fuel_cell  * series_e
        fuel_cell_stack.maximum_power          = P_fuel_cell * n_total 
        fuel_cell_stack.maximum_current        = fuel_cell_stack.maximum_power / fuel_cell_stack.maximum_voltage
        fuel_cell_stack.maximum_fuel_flow_rate = m_dot_H2 * n_total         
         
    return 