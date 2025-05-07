# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/compute_fuel_tank_properties.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------  
def compute_fuel_tank_properties(tank,state,distributor):
    '''
    SAI HEADER
    ''' 
    
    if type(distributor) == RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus:
        distributor_conditions = state.conditions.energy.busses[distributor.tag] 
    elif  type(distributor) == RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line: 
        distributor_conditions = state.conditions.energy.fuel_lines[distributor.tag]         
    
    tank_conditions = distributor_conditions.fuel_tanks[tank.tag]      
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
                    
    tank_conditions.mass_flow_rate  = tank.fuel_selector_ratio*distributor_conditions.fuel_flow_rate + tank_conditions.boil_off_flow_rate +  tank_conditions.secondary_fuel_flow_rate
    tank_conditions.mass -= tank_conditions.mass_flow_rate 
  
    return 