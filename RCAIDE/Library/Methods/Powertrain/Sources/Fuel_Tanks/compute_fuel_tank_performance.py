# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/compute_fuel_tank_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE 

# package imports 
import numpy as np  
# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------  
def compute_fuel_tank_performance(tank,state,network):
    """ Computes fuel comsumtion of tanks
    """
    # unpack  
    I    = state.numerics.time.integrate
    fuel = tank.fuel
     
    tank_conditions = state.conditions.energy.sources[tank.tag]      
    if type(tank.fuel) == RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen:
        '''needs updating'''
        # unpack
        T_amb   = state.conditions.freestream.temperature  
        
        T_s     =  tank_conditions.surface_temperature 
        h       =  0  
        
        # unpack tank properties
        epsilon = 0  
        h_fg    = 0  
        sigma   = 0  
        
        # compute head added o system (tank) 
        Q_radianton  =  epsilon * sigma * (T_amb ** 4 -  T_s ** 4)
        Q_convection =  h * (T_amb - T_s) 
        Q_total      = Q_convection + Q_radianton
        
        m_dot_boil_off = 0  
         
        tank_conditions.boil_off_flow_rate =  m_dot_boil_off 
     
    net_chemical_flow_rate          = tank_conditions.outputs.power.chemical +  tank_conditions.inputs.power.chemical
    net_fuel_mass_flow_rate         = net_chemical_flow_rate /  fuel.lower_heating_value 
    m_0_fuel                        = state.conditions.weights.components.mass[fuel.tag][0,0] 
    mass_flow_rate                  = net_fuel_mass_flow_rate +  tank_conditions.secondary_mass_flow_rate +  tank_conditions.boil_off_flow_rate 
    tank_conditions.mass_flow_rate  = mass_flow_rate
    if len(mass_flow_rate) > 1:
        # update mass 
        state.conditions.weights.components.mass[fuel.tag][:,0]  = m_0_fuel +  np.dot(I, -mass_flow_rate).flatten()
    
    stored_results_flag     = True
    stored_fuel_tank_tag    = tank.tag
    
    return tank_conditions.inputs, tank_conditions.outputs, stored_results_flag, stored_fuel_tank_tag 