# RCAIDE/Library/Methods/Weights/Correlation_Buildups/Transport/Raymer/operating_empty_weight.py
# 
# Created: Sep 2024, M. Clarke 
# Modifed:  Feb 2025, A. Molloy, S. Shekar


# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
import RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Raymer as Raymer
from RCAIDE.Framework.Core import Data, Units
from RCAIDE.Library.Attributes.Materials.Aluminum_Alloy import Aluminum_Alloy

# python imports 
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
# Operating Empty Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_operating_empty_weight(vehicle, settings=None):
    """
    """
    
    if settings == None:
        W_factors = Data()
        use_max_fuel_weight = True 
    else:
        use_max_fuel_weight = settings.use_max_fuel_weight 
        
    # Set the factors
    if not hasattr(settings, 'weight_reduction_factors'):
        W_factors              = Data() 
        W_factors.main_wing    = 0.
        W_factors.empennage    = 0.
        W_factors.fuselage     = 0.
        W_factors.structural   = 0.
        W_factors.systems      = 0.
    else:
        W_factors = settings.weight_reduction_factors
        if 'structural' in W_factors and W_factors.structural != 0.:
            print('Overriding individual structural weight factors')
            W_factors.main_wing    = 0.
            W_factors.empennage    = 0.
            W_factors.fuselage     = 0.
            W_factors.systems      = 0.
        else:
            W_factors.structural   = 0.
            W_factors.systems      = 0. 
    
    Wings = RCAIDE.Library.Components.Wings  
    
    payload = RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Common.compute_payload_weight(vehicle) 
    
    
    vehicle.payload.passengers                      = RCAIDE.Library.Components.Component()
    vehicle.payload.passengers.tag                  = 'passengers'
    vehicle.payload.passengers.mass_properties.mass = payload.passengers
    
    vehicle.payload.baggage                         = RCAIDE.Library.Components.Component()
    vehicle.payload.baggage.tag                     = 'baggage'
    vehicle.payload.baggage.mass_properties.mass    = payload.baggage
    
    vehicle.payload.cargo                           = RCAIDE.Library.Components.Component() 
    vehicle.payload.cargo.tag                       = 'cargo'   
    vehicle.payload.cargo.mass_properties.mass      = payload.cargo    

    ##-------------------------------------------------------------------------------             
    # Operating Items Weight
    ##------------------------------------------------------------------------------- 
    W_oper = RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Common.compute_operating_items_weight(vehicle)  # TO ADD

    ##-------------------------------------------------------------------------------         
    # System Weight
    ##------------------------------------------------------------------------------- 
    W_systems = Raymer.compute_systems_weight(vehicle)
        
    for item in W_systems.keys():
        W_systems[item] *= (1. - W_factors.systems)
    
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
   
        W_propulsion                        = Raymer.compute_propulsion_system_weight(vehicle, network) 
        W_energy_network_total              += W_propulsion.W_prop 
        W_energy_network.W_engine           += W_propulsion.W_engine
        W_energy_network.W_thrust_reverser  += W_propulsion.W_thrust_reverser
        W_energy_network.W_engine_controls  += W_propulsion.W_engine_controls
        W_energy_network.W_starter          += W_propulsion.W_starter
        W_energy_network.W_fuel_system      += W_propulsion.W_fuel_system 
        W_energy_network.W_nacelle          += W_propulsion.W_nacelle    
        number_of_engines                   += W_propulsion.number_of_engines
        number_of_tanks                     += W_propulsion.number_of_fuel_tanks
        for propulsor in network.propulsors:
            propulsor.mass_properties.mass = W_energy_network_total / number_of_engines
                 
        # Electric-Powered Propulsors  
        # for bus in network.busses: 
        #     # electrical payload 
        #     W_systems.W_electrical  += bus.payload.mass_properties.mass * Units.kg
     
        #     # Avionics Weight 
        #     W_systems.W_avionics  += bus.avionics.mass_properties.mass      
    
        #     for battery in bus.battery_modules: 
        #         W_energy_network_total  += battery.mass_properties.mass * Units.kg
        #         W_energy_network.W_battery = battery.mass_properties.mass * Units.kg
                
        # for propulsor in network.propulsors:
        #     if 'motor' in propulsor:                           
        #         W_energy_network.W_motor +=  propulsor.motor.mass_properties.mass
        #         W_energy_network_total  +=  propulsor.motor.mass_properties.mass
                   
    W_energy_network_cumulative += W_energy_network_total
    
    ##-------------------------------------------------------------------------------                 
    # Pod Weight Weight 
    ##-------------------------------------------------------------------------------         
    WPOD  = 0.0             
    output.empty.propulsion.total               = W_energy_network_cumulative
    output.empty.propulsion.battery             = 0 #W_energy_network.W_battery
    output.empty.propulsion.motors              = 0 #W_energy_network.W_motor
    output.empty.propulsion.engines             = W_energy_network.W_engine
    output.empty.propulsion.thrust_reversers    = W_energy_network.W_thrust_reverser
    output.empty.propulsion.miscellaneous       = W_energy_network.W_engine_controls + W_energy_network.W_starter
    output.empty.propulsion.fuel_system         = W_energy_network.W_fuel_system

    ##-------------------------------------------------------------------------------                 
    # Wing Weight 
    ##------------------------------------------------------------------------------- 
    Al_rho   = Aluminum_Alloy().density
    Al_sigma = Aluminum_Alloy().yield_tensile_strength      
    
    num_main_wings      = 0
    W_main_wing        = 0.0
    W_tail_horizontal  = 0.0
    W_tail_vertical    = 0.0
    for wing in vehicle.wings:
        if isinstance(wing, Wings.Main_Wing):
            num_main_wings += 1
    
    for wing in vehicle.wings:
        if isinstance(wing, Wings.Main_Wing): 
            W_wing = Raymer.compute_main_wing_weight(vehicle, wing, settings) 
            # Apply weight factor
            W_wing = W_wing * (1. - W_factors.main_wing) * (1. - W_factors.structural)
            if np.isnan(W_wing):
                W_wing = 0.
            wing.mass_properties.mass = W_wing
            W_main_wing += W_wing
        if isinstance(wing, Wings.Horizontal_Tail):
            W_tail = Raymer.compute_horizontal_tail_weight(vehicle, wing, settings)
            if type(W_tail) == np.ndarray:
                W_tail = sum(W_tail)
            # Apply weight factor
            W_tail = W_tail * (1. - W_factors.empennage) * (1. - W_factors.structural)
            # Pack and sum
            wing.mass_properties.mass = W_tail
            W_tail_horizontal += W_tail
        if isinstance(wing, Wings.Vertical_Tail):
            W_tail = Raymer.compute_vertical_tail_weight(vehicle, wing, settings)
            # Apply weight factor
            W_tail = W_tail * (1. - W_factors.empennage) * (1. - W_factors.structural)
            # Pack and sum
            wing.mass_properties.mass = W_tail
            W_tail_vertical += W_tail
        
    ##-------------------------------------------------------------------------------                 
    # Fuselage 
    ##------------------------------------------------------------------------------- 
    W_fuselage_total = 0
    for fuse in vehicle.fuselages:
        W_fuselage = Raymer.compute_fuselage_weight(vehicle, fuse, settings)
        W_fuselage = W_fuselage * (1. - W_factors.fuselage) * (1. - W_factors.structural)
        fuse.mass_properties.mass = W_fuselage
        W_fuselage_total += W_fuselage
    
    ##-------------------------------------------------------------------------------                 
    # Landing Gear Weight
    ##------------------------------------------------------------------------------- 
    landing_gear = Raymer.compute_landing_gear_weight(vehicle)
    
    ##-------------------------------------------------------------------------------                 
    # Accumulate Structural Weight
    ##-------------------------------------------------------------------------------   
    output.empty.structural                       = Data()
    output.empty.structural.wings                  = W_main_wing +   W_tail_horizontal +  W_tail_vertical 
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
    total_fuel_weight           = vehicle.mass_properties.max_takeoff - output.zero_fuel_weight
    
    # assume fuel is equally distributed in fuel tanks
    if use_max_fuel_weight:
        for network in vehicle.networks: 
            for fuel_line in network.fuel_lines:  
                for fuel_tank in fuel_line.fuel_tanks:
                    fuel_weight =  total_fuel_weight/number_of_tanks  
                    fuel_tank.fuel.mass_properties.mass = fuel_weight
                    
    nose_landing_gear = False
    main_landing_gear =  False
    for LG in vehicle.landing_gears:
        if isinstance(LG, RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear):
            LG.mass_properties.mass = landing_gear.main
            main_landing_gear = True
        elif isinstance(LG, RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear):
            LG.mass_properties.mass = landing_gear.nose
            nose_landing_gear = True 
    if nose_landing_gear == False:
        nose_gear = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()  
        nose_gear.mass_properties.mass = landing_gear.nose    
        vehicle.landing_gears.append(nose_gear) 
    if main_landing_gear == False:
        main_gear = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()  
        main_gear.mass_properties.mass = landing_gear.main  
        vehicle.landing_gears.append(main_gear) 

    control_systems                         = RCAIDE.Library.Components.Component()
    control_systems.tag                     = 'control_systems'  
    control_systems.mass_properties.mass    = output.empty.systems.control_systems
    electrical_systems                      = RCAIDE.Library.Components.Component()
    electrical_systems.tag                  = 'electrical_systems'
    electrical_systems.mass_properties.mass = output.empty.systems.electrical
    furnishings                             = RCAIDE.Library.Components.Component()
    furnishings.tag                         = 'furnishings'
    furnishings.mass_properties.mass        = output.empty.systems.furnishings
    air_conditioner                         = RCAIDE.Library.Components.Component() 
    air_conditioner.tag                     = 'air_conditioner'
    air_conditioner.mass_properties.mass    = output.empty.systems.air_conditioner
    apu                                     = RCAIDE.Library.Components.Component()
    apu.tag                                 = 'apu'
    apu.mass_properties.mass                = output.empty.systems.apu
    hydraulics                              = RCAIDE.Library.Components.Component()
    hydraulics.tag                          = 'hydraulics' 
    hydraulics.mass_properties.mass         = output.empty.systems.hydraulics
    avionics                                = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.mass_properties.mass           = output.empty.systems.avionics + output.empty.systems.instruments
    optionals                               = RCAIDE.Library.Components.Component()
    optionals.tag                           = 'optionals'
    optionals                               = RCAIDE.Library.Components.Component()
    optionals.mass_properties.mass          = output.operational_items.misc
    
    # assign components to vehicle
    vehicle.systems.control_systems         = control_systems
    vehicle.systems.electrical_systems      = electrical_systems
    vehicle.systems.avionics                = avionics
    vehicle.systems.furnishings             = furnishings
    vehicle.systems.air_conditioner         = air_conditioner 
    vehicle.systems.apu                     = apu
    vehicle.systems.hydraulics              = hydraulics
    vehicle.systems.optionals               = optionals   

    return output
