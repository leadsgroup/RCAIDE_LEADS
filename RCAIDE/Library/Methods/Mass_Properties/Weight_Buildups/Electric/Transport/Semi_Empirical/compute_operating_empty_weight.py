# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Electric/Transport/operating_empty_weight.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data ,  Units 
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Common import compute_payload_weight 
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Electric.Transport.Semi_Empirical.compute_operating_items_weight import compute_operating_items_weight
import RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Electric.Common as Electric_Common
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Electric.Transport.Semi_Empirical.FLOPS_compute_wing_weight import compute_wing_weight

# python imports 
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
# Operating Empty Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_operating_empty_weight(vehicle, settings=None):
    """ Main function that estimates the zero-fuel weight of a transport aircraft:
        - MTOW = WZFW + FUEL
        - WZFW = WOE + WPAYLOAD
        - WOE = WE + WOPERATING_ITEMS
        - WE = WSTRCT + WPROP + WSYS
        Assumptions:
            1) All nacelles are identical
            2) The number of nacelles is the same as the number of engines 
            3) Aircraft has no APU
            4) Aircraft is fully eelctric (not hybrid)

        Source:
            FLOPS method: The Flight Optimization System Weight Estimation Method
       Inputs:
            vehicle - data dictionary with vehicle properties               [dimensionless]
                -.networks: data dictionary with all the network elements and properties
                    -.total_weight: total weight of the propulsion system   [kg] 
                      (optional, calculated if not included)
                -.fuselages: data dictionary with the fuselage properties of the vehicle
                -.wings: data dictionary with all the wing properties of the vehicle, including horzinotal and vertical stabilizers
                -.wings['main_wing']: data dictionary with main wing properties
                    -.flap_ratio: flap surface area over wing surface area
                -.mass_properties: data dictionary with all the main mass properties of the vehicle including MTOW, ZFW, EW and OEW
            method_type - weight estimation method chosen, available:
                            - FLOPS Simple
                            - FLOPS Complex
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
    if settings.method == 'Raymer':
        import RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.Raymer as Method
    elif settings.method == 'FLOPS':
        import RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport.FLOPS as Method
    
    Wings = RCAIDE.Library.Components.Wings  
    if settings.method == 'FLOPS':
        if vehicle.flight_envelope.design_mach_number  == None: # Added design mach number
            raise ValueError("FLOPS requires a design mach number for sizing!")
        if vehicle.flight_envelope.design_range  == None:
            raise ValueError("FLOPS requires a design range for sizing!")
        if vehicle.flight_envelope.design_cruise_altitude == None:
            raise ValueError("FLOPS requires a cruise altitude for sizing!")
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
    W_oper = compute_operating_items_weight(vehicle) 
    
    ##-------------------------------------------------------------------------------         
    # System Weight
    ##------------------------------------------------------------------------------- 
    W_systems = Method.compute_systems_weight(vehicle) 
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
    W_energy_network.W_TMS             = Data()
    W_energy_network.W_propellers      = 0
    number_of_engines                  = 0
    number_of_tanks                    = 0
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
            if type(propulsor.rotor) == RCAIDE.Library.Components.Powertrain.Converters.Propeller:
                    ''' Propeller Weight '''
                    propeller_mass             = Electric_Common.compute_rotor_weight(propulsor.rotor, propulsor.sealevel_static_thrust) * Units.kg
                    propulsor.rotor.mass_properties.mass  =  propeller_mass
                    W_energy_network.W_propellers += propeller_mass
                    W_energy_network_total     += propeller_mass
        
        # Cable weight, inverter weight, and other electric  powertrain components added here
    
    W_energy_network_cumulative += W_energy_network_total
    
    ##-------------------------------------------------------------------------------                 
    # Pod Weight Weight 
    ##-------------------------------------------------------------------------------         
    WPOD  = 0.0             
    if settings.method == 'FLOPS':
        if settings.FLOPS.fidelity == 'Complex': 
            NENG   = number_of_engines
            WTNFA  = W_energy_network.W_engine + W_energy_network.W_thrust_reverser + W_energy_network.W_starter \
                    + 0.25 * W_energy_network.W_engine_controls + 0.11 * W_systems.W_instruments + 0.13 * W_systems.W_electrical \
                    + 0.13 * W_systems.W_hyd_pnu + 0.25 * W_energy_network.W_fuel_system
            WPOD += WTNFA / np.max([1, NENG]) + W_energy_network.W_nacelle  / np.max(
                [1.0, NENG + 1. / 2 * (NENG - 2 * np.floor(NENG / 2.))])
 
    output.empty.propulsion.total               = W_energy_network_cumulative
    output.empty.propulsion.battery             = W_energy_network.W_battery
    output.empty.propulsion.motors              = W_energy_network.W_motor
    output.empty.propulsion.engines             = W_energy_network.W_engine
    output.empty.propulsion.propellers          = W_energy_network.W_propellers
    output.empty.propulsion.thrust_reversers    = W_energy_network.W_thrust_reverser
    output.empty.propulsion.miscellaneous       = W_energy_network.W_engine_controls + W_energy_network.W_starter
    output.empty.propulsion.fuel_system         = W_energy_network.W_fuel_system

    #-------------------------------------------------------------------------------
    # Thermal Management System Weight
    #-------------------------------------------------------------------------------
    tms_weight = 0.0 
    for coolant_line in network.coolant_lines:
        W_energy_network.W_TMS.battery_module = Data()  # Add container for battery module
        for i, battery_module in enumerate(coolant_line.battery_modules):
            module_key = f'module_{i+1}'  # Create unique key for each module
            W_energy_network.W_TMS.battery_module[module_key] = 0.0  # Initialize weight
            for HAS in battery_module:
                W_energy_network.W_TMS.battery_module[module_key] = HAS.mass_properties.mass
                tms_weight +=  HAS.mass_properties.mass

        for tag, item in coolant_line.items():
            if tag == 'heat_exchangers':
                for heat_exchanger in item:
                    W_energy_network.W_TMS[heat_exchanger.tag] = heat_exchanger.mass_properties.mass
                    tms_weight +=  heat_exchanger.mass_properties.mass
            if tag == 'reservoirs':
                for reservoir in item:
                    W_energy_network.W_TMS[reservoir.tag] = reservoir.mass_properties.mass
                    tms_weight +=  reservoir.mass_properties.mass

    ##-------------------------------------------------------------------------------                 
    # Wing Weight 
    ##-------------------------------------------------------------------------------     
    num_main_wings     = 0
    W_main_wing        = 0.0
    W_tail_horizontal  = 0.0
    W_tail_vertical    = 0.0
    for wing in vehicle.wings:
        if isinstance(wing, Wings.Main_Wing): 
            num_main_wings += 1
    
    for wing in vehicle.wings:
        if isinstance(wing, Wings.Main_Wing): 
            fidelity = settings.FLOPS.fidelity
            try:
                W_wing = compute_wing_weight(vehicle, wing, WPOD, fidelity, settings, num_main_wings) # FLOPS wing weight with added battery ienrtia relief
            except:
                W_wing = Method.compute_main_wing_weight(vehicle, wing, settings)

            if np.isnan(W_wing):
                W_wing = 0.
            wing.mass_properties.mass = W_wing
            W_main_wing += W_wing
        if isinstance(wing, Wings.Horizontal_Tail):
            try:
                W_tail = Method.compute_horizontal_tail_weight(vehicle, wing, settings)
            except:
                W_tail = Method.compute_horizontal_tail_weight(vehicle, wing)
            if type(W_tail) == np.ndarray:
                W_tail = sum(W_tail)

            # Pack and sum
            wing.mass_properties.mass = W_tail
            W_tail_horizontal += W_tail
        if isinstance(wing, Wings.Vertical_Tail):
            try:
                W_tail = Method.compute_vertical_tail_weight(vehicle, wing)
            except:
                W_tail = Method.compute_vertical_tail_weight(vehicle, wing, settings)
            # Pack and sum
            wing.mass_properties.mass = W_tail
            W_tail_vertical += W_tail
        
    ##-------------------------------------------------------------------------------                 
    # Fuselage 
    ##------------------------------------------------------------------------------- 
    W_fuselage_total = 0
    for fuse in vehicle.fuselages:
        try:
            W_fuselage = Method.compute_fuselage_weight(vehicle)
        except:
            W_fuselage = Method.compute_fuselage_weight(vehicle, fuse, settings)
        fuse.mass_properties.mass = W_fuselage
        W_fuselage_total += W_fuselage
    
    ##-------------------------------------------------------------------------------                 
    # Landing Gear Weight
    ##------------------------------------------------------------------------------- 
    landing_gear = Method.compute_landing_gear_weight(vehicle)
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
    output.empty.structural.paint = 0
    output.empty.structural.total = output.empty.structural.wings   + output.empty.structural.fuselage + output.empty.structural.landing_gear\
                                    + output.empty.structural.paint + output.empty.structural.nacelle + output.empty.structural.empennage 

    ##-------------------------------------------------------------------------------                 
    # Accumulate Systems Weight
    ##-------------------------------------------------------------------------------
    output.empty.systems                        = Data()
    output.empty.systems.control_systems        = W_systems.W_flight_control
    output.empty.systems.apu                    = 0.0
    output.empty.systems.electrical             = W_systems.W_electrical
    output.empty.systems.avionics               = W_systems.W_avionics
    output.empty.systems.hydraulics             = W_systems.W_hyd_pnu
    output.empty.systems.furnishings            = W_systems.W_furnish
    output.empty.systems.air_conditioner        = W_systems.W_ac + W_systems.W_anti_ice # Anti-ice is sometimes included in ECS
    output.empty.systems.instruments            = W_systems.W_instruments
    output.empty.systems.total                  = output.empty.systems.control_systems + output.empty.systems.electrical \
                                                    + output.empty.systems.avionics + output.empty.systems.hydraulics \
                                                    + output.empty.systems.furnishings + output.empty.systems.air_conditioner\
                                                    + output.empty.systems.instruments
 
    output.payload    = payload 
    output.operational_items    = Data()
    output.operational_items    = W_oper 
    output.empty.total          = output.empty.structural.total + output.empty.propulsion.total + output.empty.systems.total 
    output.zero_fuel_weight     = output.empty.total + output.operational_items.total + output.payload.total 
    return output