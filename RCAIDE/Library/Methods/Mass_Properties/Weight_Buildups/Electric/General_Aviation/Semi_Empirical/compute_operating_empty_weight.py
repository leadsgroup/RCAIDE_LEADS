# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Electric/General_Aviation/operating_empty_weight.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data ,  Units 
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Common import compute_payload_weight
import RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.General_Aviation.FLOPS as FLOPS
# python imports 
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
# Operating Empty Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_operating_empty_weight(vehicle, settings=None):
    """"


        Properties Used:
            N/A

        Also creates system components and assigns weights to them. These are appended to the vehicle.
    """
    Wings = RCAIDE.Library.Components.Wings  

    # Check vehicle properties:
    if vehicle.flight_envelope.design_mach_number  == None: # Added design mach number
        raise ValueError("FLOPS requires a design mach number for sizing!")
    
    if not hasattr(vehicle, 'flap_ratio'):
        flap_ratio = 0.33
        for wing in vehicle.wings:
            if isinstance(wing, Wings.Main_Wing):
                wing.flap_ratio = flap_ratio 
                
    ##-------------------------------------------------------------------------------             
    # Payload Weight
    ##-------------------------------------------------------------------------------  
    payload = compute_payload_weight(vehicle)    

    ##-------------------------------------------------------------------------------             
    # Operating Items Weight
    ##------------------------------------------------------------------------------- 
    W_oper = FLOPS.compute_operating_items_weight(vehicle)    
    
    ##-------------------------------------------------------------------------------         
    # System Weight
    ##------------------------------------------------------------------------------- 
    W_systems = FLOPS.compute_systems_weight(vehicle) 
    for system in vehicle.systems:
        if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Avionics:
            if system.mass_properties.mass == 0:
                system.mass_properties.mass = W_systems.W_avionics 
        if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Flight_Controls:
            if system.mass_properties.mass == 0:
                system.mass_properties.mass = W_systems.W_flight_control 
        if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Auxiliary_Power_Unit: 
            if system.mass_properties.mass == 0:
                system.mass_properties.mass = W_systems.W_apu 
        if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Electrical: 
            if system.mass_properties.mass == 0:
                system.mass_properties.mass = W_systems.W_electrical 
        if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Hydraulics: 
            if system.mass_properties.mass == 0:
                system.mass_properties.mass = W_systems.W_hyd_pnu 
        if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Environmental_Controls: 
            if system.mass_properties.mass == 0:
                system.mass_properties.mass = W_systems.W_ac + W_systems.W_anti_ice   
        if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Instruments:
            if system.mass_properties.mass == 0:     
                system.mass_properties.mass = W_systems.W_instruments  
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
    W_energy_network_cumulative        = 0 

    for network in vehicle.networks: 
        W_energy_network_total   = 0 
    
        # Electric-Powered Propulsors  
        for distributor in network.distrubutor:
            if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus): 
                # electrical payload 
                W_systems.W_electrical += distributor.systems.mass_properties.mass   
         
                # Avionics Weight 
                W_systems.W_avionics  += distributor.avionics.mass_properties.mass      
        
        for source in network.sources: 
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack):
                W_energy_network_total  += source.mass_properties.mass 
                W_energy_network.W_battery = source.mass_properties.mass 
            
        for propulsor in network.propulsors:
            if 'motor' in propulsor:                           
                W_energy_network.W_motor +=  propulsor.motor.mass_properties.mass
                W_energy_network_total  +=  propulsor.motor.mass_properties.mass
                   
    W_energy_network_cumulative += W_energy_network_total
    
    ##-------------------------------------------------------------------------------                 
    # Pod Weight Weight 
    ##-------------------------------------------------------------------------------         
    WPOD  = 0.0             

    NENG   = number_of_engines
    WTNFA  = W_energy_network.W_engine + W_energy_network.W_thrust_reverser + W_energy_network.W_starter \
            + 0.25 * W_energy_network.W_engine_controls + 0.11 * W_systems.W_instruments + 0.13 * W_systems.W_electrical \
            + 0.13 * W_systems.W_hyd_pnu + 0.25 * W_energy_network.W_fuel_system
    WPOD += WTNFA / np.max([1, NENG]) + W_energy_network.W_nacelle / np.max(
        [1.0, NENG + 1. / 2 * (NENG - 2 * np.floor(NENG / 2.))])
 
    output.empty.propulsion.total               = W_energy_network_cumulative
    output.empty.propulsion.battery             = W_energy_network.W_battery
    output.empty.propulsion.motors              = W_energy_network.W_motor
    output.empty.propulsion.engines             = W_energy_network.W_engine
    output.empty.propulsion.thrust_reversers    = W_energy_network.W_thrust_reverser
    output.empty.propulsion.miscellaneous       = W_energy_network.W_engine_controls + W_energy_network.W_starter
    output.empty.propulsion.fuel_system         = W_energy_network.W_fuel_system
   
    num_main_wings      = 0
    W_main_wing        = 0.0
    W_tail_horizontal  = 0.0
    W_tail_vertical    = 0.0
    for wing in vehicle.wings:
        if isinstance(wing, Wings.Main_Wing) or isinstance(wing, Wings.Blended_Wing_Body): 
            num_main_wings += 1
    
    for wing in vehicle.wings:
        if isinstance(wing, Wings.Main_Wing) or isinstance(wing, Wings.Blended_Wing_Body):  # Main wing
            fidelity = settings.FLOPS.fidelity
            W_wing = FLOPS.compute_wing_weight(vehicle, wing, WPOD, fidelity, settings, num_main_wings)
            if np.isnan(W_wing):
                W_wing = 0.
            # Pack and sum
            wing.mass_properties.mass = W_wing
            W_main_wing += W_wing
        if isinstance(wing, Wings.Horizontal_Tail): # Horizontal tail
            W_tail = FLOPS.compute_horizontal_tail_weight(vehicle)
            if type(W_tail) == np.ndarray:
                W_tail = sum(W_tail)
            # Pack and sum
            wing.mass_properties.mass = W_tail
            W_tail_horizontal += W_tail
        if isinstance(wing, Wings.Vertical_Tail): # Vertical tail
            W_tail = FLOPS.compute_vertical_tail_weight(vehicle, wing)
            # Pack and sum
            wing.mass_properties.mass = W_tail
            W_tail_vertical += W_tail
        
    ##-------------------------------------------------------------------------------                 
    # Fuselage 
    ##------------------------------------------------------------------------------- 
    W_fuselage_total = 0
    for fuse in vehicle.fuselages:
        W_fuselage = FLOPS.compute_fuselage_weight(vehicle)
        W_fuselage = W_fuselage 
        fuse.mass_properties.mass = W_fuselage
        W_fuselage_total += W_fuselage
    
    ##-------------------------------------------------------------------------------                 
    # Landing Gear Weight
    ##------------------------------------------------------------------------------- 
    landing_gear = FLOPS.compute_landing_gear_weight(vehicle) 
    for LG in vehicle.landing_gears:
        if isinstance(LG, RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear): 
            LG.mass_properties.mass = landing_gear.main 
        elif isinstance(LG, RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear):  
            LG.mass_properties.mass = landing_gear.nose   

    
    ##-------------------------------------------------------------------------------                 
    # Accumulate Structural Weight
    ##-------------------------------------------------------------------------------   
    output.empty.structural                      = Data()
    output.empty.structural.wings                = W_main_wing 
    output.empty.structural.empennage             = W_tail_horizontal +  W_tail_vertical 
    output.empty.structural.fuselage             = W_fuselage_total
    output.empty.structural.landing_gear         = landing_gear.main +  landing_gear.nose  
    output.empty.structural.nacelle              = W_energy_network.W_nacelle 
    output.empty.structural.paint                = 0  # TODO reconcile FLOPS paint calculations with Raymer and RCAIDE baseline
    output.empty.structural.total                = output.empty.structural.wings   + output.empty.structural.fuselage + output.empty.structural.landing_gear\
                                                   + output.empty.structural.paint + output.empty.structural.nacelle + output.empty.structural.empennage 

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