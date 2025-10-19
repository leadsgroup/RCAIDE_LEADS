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
    '''
    SAI HEADER
    ''' 
    # unpack  
    I  = state.numerics.time.integrate
    
    # pull out distributor
    if type(distributor) == RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus:
        distributor_conditions = state.conditions.energy.busses[distributor.tag] 
    elif  type(distributor) == RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line: 
        distributor_conditions = state.conditions.energy.distributors[distributor.tag]         
    
    tank_conditions = state.conditions.energy.sources[tank.tag]      
    if type(tank.fuel) == RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen:
        # unpack
        T_amb  = state.conditions.freestream.temperature  
        
        T_s =  tank_conditions.surface_temperature

        h   =  0 # NEED TO UPDATE 
        
        # unpack tank properties
        epsilon = 0 # tant.  NEED TO UPDATE 
        h_fg    = 0 #  tank.fuel  NEED TO UPDATE 
        sigma   = 0 #  NEED TO UPDATE  
                        
        
        # compute head added o system (tank) 
        Q_radianton  =  epsilon * sigma * (T_amb ** 4 -  T_s ** 4)
        Q_convection =  h * (T_amb - T_s) 
        Q_total      = Q_convection + Q_radianton
        
        m_dot_boil_off = 0 #Q_dot_liquid / h_fg
         
        tank_conditions.boil_off_flow_rate =  m_dot_boil_off 

    Press_tank                                     = tank.pressure 
    m_0_fuel                                       = tank_conditions.fuel_mass[0,0]
    mass_flow_rate                                 = tank.fuel_selector_ratio*distributor_conditions.fuel_mass_flow_rate + tank_conditions.boil_off_flow_rate +  tank_conditions.secondary_mass_flow_rate             
    tank_conditions.mass_flow_rate                 = mass_flow_rate
    if len(mass_flow_rate) > 1: 
        tank_conditions.fuel_mass[:,0]  = m_0_fuel +  np.dot(I, -mass_flow_rate).flatten()  

    stored_results_flag            = True
    stored_source_tag              = tank.tag  

    tank_conditions.power.propulsive               = 0.0 * state.ones_row(1)
    tank_conditions.power.mechanical               = 0.0 * state.ones_row(1)
    tank_conditions.power.electrical               = 0.0 * state.ones_row(1)
    tank_conditions.power.chemical                 = mass_flow_rate * tank.fuel.lower_heating_value
    tank_conditions.power.pneumatic                = 0.0 * state.ones_row(1)
    tank_conditions.power.hydraulic                = 0.0 * state.ones_row(1)
    tank_conditions.power.thermal                  = 0.0 * state.ones_row(1)

    return tank_conditions.power
