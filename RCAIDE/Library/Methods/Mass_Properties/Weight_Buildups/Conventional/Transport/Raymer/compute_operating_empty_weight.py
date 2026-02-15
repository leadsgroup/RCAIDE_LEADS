# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Transport/Raymer/operating_empty_weight.py
# 
# Created: Sep 2024, M. Clarke 
# Modifed:  Feb 2025, A. Molloy, S. Shekar


# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
import RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Raymer as Raymer
from RCAIDE.Framework.Core import Data

# python imports 
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
# Operating Empty Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_operating_empty_weight(vehicle, settings=None):
    """
    """
    Wings = RCAIDE.Library.Components.Wings  

    ##-------------------------------------------------------------------------------             
    # Payload Weight
    ##-------------------------------------------------------------------------------     
    payload = RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Common.compute_payload_weight(vehicle)  

    ##-------------------------------------------------------------------------------             
    # Operating Items Weight
    ##------------------------------------------------------------------------------- 
    W_oper = RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Common.compute_operating_items_weight(vehicle) 

    ##-------------------------------------------------------------------------------         
    # System Weight
    ##------------------------------------------------------------------------------- 
    W_systems = Raymer.compute_systems_weight(vehicle)

    ##-------------------------------------------------------------------------------                 
    # Propulsion Weight 
    ##-------------------------------------------------------------------------------
    output                                      = Data()
    output.empty                                = Data() 
    output.empty.propulsion                     = Data() 
    output.empty.propulsion.total               = 0
    output.empty.propulsion.engines             = 0
    output.empty.propulsion.thrust_reversers    = 0
    output.empty.propulsion.miscellaneous       = 0
    output.empty.propulsion.fuel_system         = 0

    W_energy_network                   = Data()
    W_energy_network.total             = 0
    W_energy_network.W_engine          = 0 
    W_energy_network.W_thrust_reverser = 0 
    W_energy_network.W_engine_controls = 0 
    W_energy_network.W_starter         = 0 
    W_energy_network.W_fuel_system     = 0 
    W_energy_network.W_motors          = 0 
    W_energy_network.W_nacelle         = 0 
    W_energy_network.W_battery         = 0
    W_energy_network.W_motor           = 0
    number_of_engines                  = 0
    number_of_tanks                    = 0
    W_energy_network_cumulative        = 0 

    for network in vehicle.networks: 
        W_energy_network_total   = 0 
        # Fuel-Powered Propulsors  
   
        W_propulsion                        = Raymer.compute_propulsion_system_weight(vehicle, network, settings) 
        W_energy_network_total              += W_propulsion.W_prop 
        W_energy_network.W_engine           += W_propulsion.W_engine
        W_energy_network.W_thrust_reverser  += W_propulsion.W_thrust_reverser
        W_energy_network.W_engine_controls  += W_propulsion.W_engine_controls
        W_energy_network.W_starter          += W_propulsion.W_starter
        W_energy_network.W_fuel_system      += W_propulsion.W_fuel_system 
        W_energy_network.W_nacelle          += W_propulsion.W_nacelle    
        number_of_engines                   += W_propulsion.number_of_engines
        number_of_tanks                     += W_propulsion.number_of_fuel_tanks
        
    W_energy_network_cumulative += W_energy_network_total
    
    ##-------------------------------------------------------------------------------                 
    # Pod Weight Weight 
    ##-------------------------------------------------------------------------------          
    output.empty.propulsion.total               = W_energy_network_cumulative
    output.empty.propulsion.battery             = 0  
    output.empty.propulsion.motors              = 0  
    output.empty.propulsion.engines             = W_energy_network.W_engine
    output.empty.propulsion.thrust_reversers    = W_energy_network.W_thrust_reverser
    output.empty.propulsion.miscellaneous       = W_energy_network.W_engine_controls + W_energy_network.W_starter
    output.empty.propulsion.fuel_system         = W_energy_network.W_fuel_system

    ##-------------------------------------------------------------------------------                 
    # Wing Weight 
    ##-------------------------------------------------------------------------------  
    num_main_wings      = 0
    W_main_wing        = 0.0
    W_tail_horizontal  = 0.0
    W_tail_vertical    = 0.0
    for wing in vehicle.wings:
        if isinstance(wing, Wings.Main_Wing) or isinstance(wing, Wings.Blended_Wing_Body): 
            num_main_wings += 1
    
    for wing in vehicle.wings:
        if isinstance(wing, Wings.Main_Wing) or isinstance(wing, Wings.Blended_Wing_Body):  
            W_wing = Raymer.compute_main_wing_weight(vehicle, wing, settings) 
            if np.isnan(W_wing):
                W_wing = 0.
            wing.mass_properties.mass = W_wing
            W_main_wing += W_wing
        if isinstance(wing, Wings.Horizontal_Tail):
            W_tail = Raymer.compute_horizontal_tail_weight(vehicle, wing, settings)
            if type(W_tail) == np.ndarray:
                W_tail = sum(W_tail)
         
            # Pack and sum
            wing.mass_properties.mass = W_tail
            W_tail_horizontal += W_tail
        if isinstance(wing, Wings.Vertical_Tail):
            W_tail = Raymer.compute_vertical_tail_weight(vehicle, wing, settings)

            # Pack and sum
            wing.mass_properties.mass = W_tail
            W_tail_vertical += W_tail
        
    ##-------------------------------------------------------------------------------                 
    # Fuselage 
    ##------------------------------------------------------------------------------- 
    W_fuselage_total = 0
    for fuse in vehicle.fuselages:
        W_fuselage = Raymer.compute_fuselage_weight(vehicle, fuse, settings)
        fuse.mass_properties.mass = W_fuselage
        W_fuselage_total += W_fuselage
    
    ##-------------------------------------------------------------------------------                 
    # Landing Gear Weight
    ##------------------------------------------------------------------------------- 
    landing_gear = Raymer.compute_landing_gear_weight(vehicle)
    for LG in vehicle.landing_gears:
        if isinstance(LG, RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear): 
            LG.mass_properties.mass = landing_gear.main 
        elif isinstance(LG, RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear):  
            LG.mass_properties.mass = landing_gear.nose    
    
    ##-------------------------------------------------------------------------------                 
    # Accumulate Structural Weight
    ##-------------------------------------------------------------------------------   
    output.empty.structural                       = Data()
    output.empty.structural.wings                  = W_main_wing 
    output.empty.structural.empennage             = W_tail_horizontal +  W_tail_vertical 
    output.empty.structural.fuselage              = W_fuselage_total
    output.empty.structural.landing_gear          = landing_gear.main +  landing_gear.nose  
    output.empty.structural.nacelle               = W_energy_network.W_nacelle
    output.empty.structural.total                 = W_main_wing +   W_tail_horizontal +  W_tail_vertical + W_fuselage_total + landing_gear.main +  landing_gear.nose + W_energy_network.W_nacelle

    ##-------------------------------------------------------------------------------                 
    # Accumulate Systems Weight
    ##-------------------------------------------------------------------------------
    output.empty.systems                        = Data()
    output.empty.systems.control_systems        = W_systems.W_flight_control
    output.empty.systems.apu                    = W_systems.W_apu
    output.empty.systems.electrical             = W_systems.W_electrical
    output.empty.systems.avionics               = W_systems.W_avionics
    output.empty.systems.hydraulics             = W_systems.W_hyd_pnu
    output.empty.systems.furnishings            = W_systems.W_furnish
    output.empty.systems.air_conditioner        = W_systems.W_ac + W_systems.W_anti_ice # Anti-ice is sometimes included in ECS
    output.empty.systems.instruments            = W_systems.W_instruments
    output.empty.systems.total                  = output.empty.systems.control_systems + output.empty.systems.apu \
                                                    + output.empty.systems.electrical + output.empty.systems.avionics \
                                                    + output.empty.systems.hydraulics + output.empty.systems.furnishings \
                                                    + output.empty.systems.air_conditioner + output.empty.systems.instruments
 
    output.payload    = payload 
    output.operational_items    = Data()
    output.operational_items    = W_oper 
    output.empty.total          = output.empty.structural.total + output.empty.propulsion.total + output.empty.systems.total 
    output.zero_fuel_weight     = output.empty.total + output.operational_items.total + output.payload.total
    output.max_takeoff          = vehicle.mass_properties.max_takeoff 

    return output
