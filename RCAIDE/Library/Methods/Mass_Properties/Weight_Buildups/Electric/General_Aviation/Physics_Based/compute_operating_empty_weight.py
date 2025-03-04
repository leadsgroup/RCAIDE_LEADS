# RCAIDE/Library/Methods/Weights/Correlation_Buildups/Transport/operating_empty_weight.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data ,  Units 
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Common import compute_payload_weight
from RCAIDE.Library.Attributes.Materials.Aluminum import Aluminum
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
    W_oper = FLOPS.compute_operating_items_weight(vehicle)
    
    ##-------------------------------------------------------------------------------         
    # System Weight
    ##------------------------------------------------------------------------------- 
    W_systems = FLOPS.compute_systems_weight(vehicle)
   
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
    
        # Electric-Powered Propulsors  
        for bus in network.busses: 
            # electrical payload 
            W_systems.W_electrical  += bus.payload.mass_properties.mass * Units.kg
     
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

    ##-------------------------------------------------------------------------------                 
    # Wing Weights 
    ##------------------------------------------------------------------------------- 
    Al_rho   = Aluminum().density
    Al_sigma = Aluminum().yield_tensile_strength      
    
    num_main_wings      = 0
    W_main_wing        = 0.0
    W_tail_horizontal  = 0.0
    W_tail_vertical    = 0.0
    for wing in vehicle.wings:
        if isinstance(wing, Wings.Main_Wing):
            num_main_wings += 1
    
    for wing in vehicle.wings:
        if isinstance(wing, Wings.Main_Wing): # Main wing
            complexity = settings.FLOPS.complexity
            W_wing = FLOPS.compute_wing_weight(vehicle, wing, WPOD, complexity, settings, num_main_wings)

            # Apply weight factor
            W_wing = W_wing * (1. - W_factors.main_wing) * (1. - W_factors.structural)
            if np.isnan(W_wing):
                W_wing = 0.
            # Pack and sum
            wing.mass_properties.mass = W_wing
            W_main_wing += W_wing
        if isinstance(wing, Wings.Horizontal_Tail): # Horizontal tail
            W_tail = FLOPS.compute_horizontal_tail_weight(vehicle)
            if type(W_tail) == np.ndarray:
                W_tail = sum(W_tail)
            # Apply weight factor
            W_tail = W_tail * (1. - W_factors.empennage) * (1. - W_factors.structural)
            # Pack and sum
            wing.mass_properties.mass = W_tail
            W_tail_horizontal += W_tail
        if isinstance(wing, Wings.Vertical_Tail): # Vertical tail
            W_tail = FLOPS.compute_vertical_tail_weight(vehicle, wing)
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
        W_fuselage = FLOPS.compute_fuselage_weight(vehicle)
        W_fuselage = W_fuselage * (1. - W_factors.fuselage) * (1. - W_factors.structural)
        fuse.mass_properties.mass = W_fuselage
        W_fuselage_total += W_fuselage
    
    ##-------------------------------------------------------------------------------                 
    # Landing Gear Weight
    ##------------------------------------------------------------------------------- 
    landing_gear = FLOPS.compute_landing_gear_weight(vehicle)
    
    ##-------------------------------------------------------------------------------                 
    # Accumulate Structural Weight
    ##-------------------------------------------------------------------------------   
    output.empty.structural                       = Data()
    output.empty.structural.wings                  = W_main_wing +   W_tail_horizontal +  W_tail_vertical 
    output.empty.structural.fuselage              = W_fuselage_total
    output.empty.structural.landing_gear          = landing_gear.main +  landing_gear.nose  
    output.empty.structural.nacelle               = W_energy_network.W_nacelle
    
  
    print('Paint weight is currently ignored in FLOPS calculations.')
    output.empty.structural.paint = 0  # TODO reconcile FLOPS paint calculations with Raymer and RCAIDE baseline
    output.empty.structural.total = output.empty.structural.wings   + output.empty.structural.fuselage + output.empty.structural.landing_gear\
                                    + output.empty.structural.paint + output.empty.structural.nacelle 

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
    

    ##-------------------------------------------------------------------------------                 
    # Create and assign miscellaneous components and weights to aircraft
    ##-------------------------------------------------------------------------------
    # Assign landing gear weights to landing gear components
    nose_landing_gear = False
    main_landing_gear =  False
    for LG in vehicle.landing_gears:
        if isinstance(LG, RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear): # Main landing gear
            LG.mass_properties.mass = landing_gear.main
            main_landing_gear = True
        elif isinstance(LG, RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear): # Nose landing gear
            LG.mass_properties.mass = landing_gear.nose
            nose_landing_gear = True 
    # Append landing gear components to the vehicle if there are none. 
    if nose_landing_gear == False:
        nose_gear = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()  
        nose_gear.mass_properties.mass = landing_gear.nose    
        vehicle.append_component(nose_gear) 
    if main_landing_gear == False:
        main_gear = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()  
        main_gear.mass_properties.mass = landing_gear.main  
        vehicle.append_component(main_gear) 

    # Create components and assign weights for systems
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