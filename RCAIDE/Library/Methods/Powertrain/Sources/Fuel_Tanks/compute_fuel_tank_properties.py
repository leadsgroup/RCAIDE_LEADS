# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/compute_fuel_tank_properties.py
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
def compute_fuel_tank_properties(tank,state,distributor):
    """ Computes fuel comsumtion of tanks
    """
    # unpack  
    I    = state.numerics.time.integrate
    fuel = tank.fuel
    
    # pull out distributor
    if type(distributor) == RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus:
        distributor_conditions = state.conditions.energy.busses[distributor.tag] 
    elif  type(distributor) == RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line: 
        distributor_conditions = state.conditions.energy.fuel_lines[distributor.tag]         
    
    tank_conditions = distributor_conditions.fuel_tanks[tank.tag]      
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
     
    #m_0_fuel                                       = state.conditions.weights.components.mass[fuel.tag][0,0] 
    mass_flow_rate                                 = tank.fuel_selector_ratio*distributor_conditions.fuel_mass_flow_rate +  tank_conditions.secondary_mass_flow_rate #tank_conditions.boil_off_flow_rate 
    tank_conditions.mass_flow_rate                 = mass_flow_rate
    #if len(mass_flow_rate) > 1:
        # update mass 
        #state.conditions.weights.components.mass[fuel.tag][:,0]  = m_0_fuel +  np.dot(I, -mass_flow_rate).flatten()
         
    return 