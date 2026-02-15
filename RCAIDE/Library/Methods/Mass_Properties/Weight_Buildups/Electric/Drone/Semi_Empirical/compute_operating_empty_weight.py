# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Electric/Drone/operating_empty_weight.py
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
    """ Computes the empty weight breakdown of a drone aircraft: 

        Source:
            None
            
       Inputs:
            vehicle - data dictionary with vehicle properties               [dimensionless]
            
       Outputs:
            output - data dictionary with the weight breakdown of the vehicle
                        -.structures: structural weight
                            -.wing: wing weight
                            -.horizontal_tail: horizontal tail weight
                            -.vertical_tail: vertical tail weight
                            -.fuselage: fuselage weight
                            -.main_landing_gear: main landing gear weight
                            -.nose_landing_gear: nose landing gear weight
                            -.nacelle: nacelle weight
                            -.paint: paint weight
                            -.total: total strucural weight

                        -.propulsion: propulsive system weight
                            -.engines: dry engine weight
                            -.thrust_reversers: thrust reversers weight
                            -.miscellaneous: miscellaneous items includes electrical system for engines and starter engine
                            -.fuel_system: fuel system weight
                            -.total: total propulsive system weight

                        -.systems: system weight
                            -.control_systems: control system weight
                            -.apu: apu weight
                            -.electrical: electrical system weight
                            -.avionics: avionics weight
                            -.hydraulics: hydraulics and pneumatic system weight
                            -.furnish: furnishing weight
                            -.air_conditioner: air conditioner weight
                            -.instruments: instrumentation weight
                            -.anti_ice: anti ice system weight
                            -.total: total system weight

                        -.payload: payload weight
                            -.passengers: passenger weight
                            -.bagage: baggage weight
                            -.cargo: cargo weight
                            -.total: total payload weight

                        -.operational_items: operational items weight
                            -.misc: unusable fuel, engine oil, passenger service weight and cargo containers
                            -.flight_crew: flight crew weight
                            -.flight_attendants: flight attendants weight
                            -.total: total operating items weight

                        -.empty = structures.total + propulsion.total + systems.total
                        -.operating_empty = empty + operational_items.total
                        -.zero_fuel_weight = operating_empty + payload.total
                        -.fuel = vehicle.mass_properties.max_takeoff - zero_fuel_weight


        Properties Used:
            N/A
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
    W_oper = Data()
    W_oper.misc                      = 0.0
    W_oper.flight_crew               = 0.0
    W_oper.flight_attendants         = 0.0
    W_oper.total                     = 0.0
                                      
    
    ##-------------------------------------------------------------------------------         
    # System Weight
    ##-------------------------------------------------------------------------------  
    W_systems                     = Data()
    W_systems.W_flight_control    = 0.0
    W_systems.W_hyd_pnu           = 0.0
    W_systems.W_instruments       = 0.0
    W_systems.W_avionics          = 0.0
    W_systems.W_apu               = 0.0
    W_systems.W_anti_ice          = 0.0
    W_systems.W_electrical        = 0.0
    W_systems.W_ac                = 0.0
    W_systems.W_furnish           = 0.0
    W_systems.total               = 0.0
    
      
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
        for bus in network.busses: 
            # electrical payload 
            try: W_systems.W_electrical  += bus.systems.mass_properties.mass * Units.kg
            except: pass
     
            # Avionics Weight 
            W_systems.W_avionics  += bus.avionics.mass_properties.mass      
    
            for battery in bus.battery_modules: 
                W_energy_network_total  += battery.mass_properties.mass * Units.kg
                W_energy_network.W_battery = battery.mass_properties.mass * Units.kg
                
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
    output.empty.structural.empennage            = W_tail_horizontal +  W_tail_vertical 
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
 
    output.payload              = payload 
    output.operational_items    = Data()
    output.operational_items    = W_oper 
    output.empty.total          = output.empty.structural.total + output.empty.propulsion.total + output.empty.systems.total 
    output.zero_fuel_weight     = output.empty.total + output.operational_items.total + output.payload.total
    output.max_takeoff          = vehicle.mass_properties.max_takeoff 
 
    return output