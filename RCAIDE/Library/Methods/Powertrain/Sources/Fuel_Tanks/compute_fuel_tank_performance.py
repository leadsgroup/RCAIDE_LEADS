# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/compute_fuel_tank_properties.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.update_moments_of_inertia import update_fuel_tank_moment_of_inertia

# package imports 
import numpy as np  
# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------  
def compute_fuel_tank_performance(tank,state,distributor):
    """ Computes fuel comsumtion of tanks
    """
    # unpack  
    I    = state.numerics.time.integrate
    fuel = tank.fuel
     
    tank_conditions     = state.conditions.energy.sources[tank.tag]
    chemical_power      = tank_conditions.outputs.power.chemical
    fuel_mass_flow_rate =  chemical_power / tank.fuel.lower_heating_value
    
    if type(tank.fuel) == RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen:
        # unpack
        T_amb  = state.conditions.freestream.temperature  
        
        T_s =  tank_conditions.surface_temperature 
        h   =  0 # NEED TO UPDATE 
        
        # unpack tank properties
        epsilon = 0 # tant.  NEED TO UPDATE # check this
        h_fg    = 0 #  tank.fuel  NEED TO UPDATE 
        sigma   = 0 #  NEED TO UPDATE  
                        
        
        # compute head added o system (tank) 
        Q_radianton  =  epsilon * sigma * (T_amb ** 4 -  T_s ** 4)
        Q_convection =  h * (T_amb - T_s) 
        Q_total      = Q_convection + Q_radianton
        
        m_dot_boil_off = 0 # Q_dot_liquid / h_fg
         
        tank_conditions.boil_off_flow_rate =  m_dot_boil_off 
     
    m_0_fuel                               = state.conditions.weights.components.mass[fuel.tag][0,0]  
    total_mass_flow_rate                   = tank_conditions.power_split_ratio * fuel_mass_flow_rate + tank_conditions.boil_off_flow_rate +  tank_conditions.secondary_mass_flow_rate             
    tank_conditions.mass_flow_rate         = total_mass_flow_rate
    tank_conditions.outputs.power.chemical = total_mass_flow_rate * tank.fuel.lower_heating_value
    
    if len(total_mass_flow_rate) > 1: 
        tank_conditions.fuel_mass[:,0]  = m_0_fuel +  np.dot(I, -total_mass_flow_rate).flatten()  

    stored_results_flag            = True
    stored_source_tag              = tank.tag   
    return tank_conditions.inputs, tank_conditions.outputs, stored_results_flag, stored_source_tag
 