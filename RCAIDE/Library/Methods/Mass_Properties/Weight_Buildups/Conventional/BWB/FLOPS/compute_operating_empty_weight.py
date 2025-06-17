# RCAIDE/Library/Methods/Weights/Correlation_Buildups/BWB/operating_empty_weight.py
# 
# Created: Sep 2024, M. Clarke  

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------
import  RCAIDE
from RCAIDE.Framework.Core import Data , Units 
from .compute_aft_center_body_weight import compute_aft_center_body_weight
from .compute_cabin_weight import compute_cabin_weight
from .compute_systems_weight import compute_systems_weight
from .compute_bwb_wing_weight import compute_wing_weight
from .compute_operating_items import compute_operating_items_weight
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Common import compute_payload_weight
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport import FLOPS
from RCAIDE.Library.Attributes.Materials.Aluminum import Aluminum
from RCAIDE.Library.Methods.Geometry.Planform                          import segment_properties  
 
from copy import deepcopy
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
# Operating Empty Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_operating_empty_weight(vehicle,settings=None):
    """ This is for a BWB aircraft configuration.

    Assumptions:
         Calculated aircraft weight from correlations created per component of historical aircraft
         The wings are made out of aluminum.
         A wing with the tag 'main_wing' exists.

    Source:
        N/A

    Inputs:
        engine - a data dictionary with the fields:
            thrust_sls - sea level static thrust of a single engine                                        [Newtons]

    Parameters
    ----------
    vehicle : Vehicle
        The vehicle instance containing:
            - mass_properties.max_takeoff : float
                Maximum takeoff weight [kg]
            - fuselages : list
                BWB fuselage segments with:
                    - aft_center_body_area : float
                        Planform area of aft section [m²]
                    - aft_center_body_taper : float
                        Taper ratio of aft section
                    - cabin_area : float
                        Pressurized cabin area [m²]
            - wings : list
                Wing surfaces
            - networks : list
                Propulsion systems
    settings : Data, optional
        Configuration settings with:
            - use_max_fuel_weight : bool
                Flag to use maximum fuel capacity
    Returns
    --------
    output : Data
        Container with weight breakdowns:
            - empty : Data
                Structural, propulsion, and systems weights
            - payload : Data
                Passenger, baggage, and cargo weights
            - fuel : float
                Total fuel weight [kg]
            - zero_fuel_weight : float
                Operating empty weight plus payload [kg]
            - total : float
                Total aircraft weight [kg]
    Notes
    -----
    Computes weights for all major aircraft components and systems using methods 
    specific to BWB configurations.

    **Major Assumptions**
        * Calculated aircraft weight from correlations created per component of historical aircraft
        * The wings are made out of aluminum.
        * A wing with the tag 'main_wing' exists.

    References
    ----------
    [1] Bradley, K. R., "A Sizing Methodology for the Conceptual Design of 
        Blended-Wing-Body Transports," NASA/CR-2004-213016, 2004.
    
    See Also
    --------
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.BWB.FLOPS.compute_cabin_weight
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.BWB.FLOPS.compute_aft_center_body_weight
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Common
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

    if vehicle.flight_envelope.design_mach_number  == None: # Added design mach number
        raise ValueError("FLOPS requires a design mach number for sizing!")
    if vehicle.flight_envelope.design_range  == None:
        raise ValueError("FLOPS requires a design range for sizing!")
    if vehicle.flight_envelope.design_cruise_altitude == None:
        raise ValueError("FLOPS requires a cruise altitude for sizing!")
    if not hasattr(vehicle, 'flap_ratio'):
        if vehicle.systems.accessories == None:
            raise ValueError("FLOPS requires systems accessories!")
        if vehicle.systems.accessories == "sst":
            flap_ratio = 0.22
        else:
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
    W_systems = compute_systems_weight(vehicle)
   
    for item in W_systems.keys():
        W_systems[item] *= (1. - W_factors.systems) 
    
    ##-------------------------------------------------------------------------------   
    # Cabin
    ##------------------------------------------------------------------------------- 
    for fuselage in vehicle.fuselages:
        if len(fuselage.cabins) == 0:
            print("No cabin defined for weights method. Defining default cabin.")   
            cabin =  RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
            cabin.mass_properties.mass = (W_oper.total + payload.passengers + output.W_systems)
            fuselage.append_cabin(cabin)
        else: 
            for cabin in fuselage.cabins:
                cabin.mass_properties.mass = (W_oper.total + payload.passengers + output.W_systems) * (cabin.number_of_passengers / fuselage.number_of_passengers )          
    
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

        W_propulsion                         = FLOPS.compute_propulsion_system_weight(vehicle, network)
        W_energy_network_total              += W_propulsion.W_prop 
        W_energy_network.W_engine           += W_propulsion.W_engine
        W_energy_network.W_thrust_reverser  += W_propulsion.W_thrust_reverser
        W_energy_network.W_engine_controls  += W_propulsion.W_engine_controls
        W_energy_network.W_starter          += W_propulsion.W_starter
        W_energy_network.W_fuel_system      += W_propulsion.W_fuel_system 
        W_energy_network.W_nacelle          += W_propulsion.W_nacelle * (1. - W_factors.nacelle)
        number_of_engines                   += W_propulsion.number_of_engines
        number_of_tanks                     += W_propulsion.number_of_fuel_tanks  
        for propulsor in network.propulsors:
            propulsor.mass_properties.mass = W_energy_network_total / number_of_engines
        
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
    if settings.FLOPS.complexity == 'Complex': 
        NENG   = number_of_engines
        WTNFA  = W_energy_network.W_engine + W_energy_network.W_thrust_reverser + W_energy_network.W_starter \
                + 0.25 * W_energy_network.W_engine_controls + 0.11 * W_systems.W_instruments + 0.13 * W_systems.W_electrical \
                + 0.13 * W_systems.W_hyd_pnu + 0.25 * W_energy_network.W_fuel_system
        WPOD += WTNFA / np.max([1, NENG]) + W_energy_network.W_nacelle* (1. - W_factors.nacelle)    / np.max(
            [1.0, NENG + 1. / 2 * (NENG - 2 * np.floor(NENG / 2.))])
 
    output.empty.propulsion.total               = W_energy_network_cumulative
    output.empty.propulsion.battery             = W_energy_network.W_battery
    output.empty.propulsion.motors              = W_energy_network.W_motor
    output.empty.propulsion.engines             = W_energy_network.W_engine
    output.empty.propulsion.thrust_reversers    = W_energy_network.W_thrust_reverser
    output.empty.propulsion.miscellaneous       = W_energy_network.W_engine_controls + W_energy_network.W_starter
    output.empty.propulsion.fuel_system         = W_energy_network.W_fuel_system

    ##-------------------------------------------------------------------------------                 
    # Wing Weight 
    ##-------------------------------------------------------------------------------     
    num_main_wings     = 0
    W_main_wing        = 0.0
    W_tail_horizontal  = 0.0
    W_tail_vertical    = 0.0
    for wing in vehicle.wings:
        if isinstance(wing, Wings.Main_Wing) or isinstance(wing, Wings.Blended_Wing_Body):
            num_main_wings += 1
            bwb_aft_center_body_area  = wing.aft_center_body.area
            bwb_aft_center_body_taper = wing.aft_center_body.taper 
            
    
    for wing in vehicle.wings:
        if isinstance(wing, Wings.Main_Wing) or isinstance(wing, Wings.Blended_Wing_Body):
            complexity = settings.FLOPS.complexity 
            sym_wing = generate_represenative_main_wing(wing, vehicle) 
            W_wing = compute_wing_weight(vehicle, sym_wing, WPOD, complexity, settings, num_main_wings)

            # Apply weight factor
            W_wing = W_wing * (1. - W_factors.main_wing) * (1. - W_factors.structural)
            if np.isnan(W_wing):
                W_wing = 0.
            wing.mass_properties.mass = W_wing
            W_main_wing += W_wing
        if isinstance(wing, Wings.Horizontal_Tail):
            W_tail = FLOPS.compute_horizontal_tail_weight(vehicle, wing)
            if type(W_tail) == np.ndarray:
                W_tail = sum(W_tail)
            # Apply weight factor
            W_tail = W_tail * (1. - W_factors.empennage) * (1. - W_factors.structural)
            # Pack and sum
            wing.mass_properties.mass = W_tail
            W_tail_horizontal += W_tail
        if isinstance(wing, Wings.Vertical_Tail):
            W_tail = FLOPS.compute_vertical_tail_weight(vehicle, wing)
            # Apply weight factor
            W_tail = W_tail * (1. - W_factors.empennage) * (1. - W_factors.structural)
            # Pack and sum
            wing.mass_properties.mass = W_tail
            W_tail_vertical += W_tail 
        
    ##-------------------------------------------------------------------------------                 
    # Fuselage 
    ##-------------------------------------------------------------------------------

        
    TOW                = vehicle.mass_properties.max_takeoff
    W_cabin            = compute_cabin_weight(vehicle,settings) 
    W_aft_center_body   = compute_aft_center_body_weight(number_of_engines,bwb_aft_center_body_area, bwb_aft_center_body_taper, TOW)
     
    
    ##-------------------------------------------------------------------------------                 
    # Landing Gear Weight
    ##------------------------------------------------------------------------------- 
    landing_gear = FLOPS.compute_landing_gear_weight(vehicle) 
 
    ##-------------------------------------------------------------------------------                 
    # Accumulate Structural Weight
    ##-------------------------------------------------------------------------------   
    output.empty.structural                       = Data()
    output.empty.structural.wings                 = W_main_wing +   W_tail_horizontal +  W_tail_vertical 
    output.empty.structural.center_body           = W_cabin
    output.empty.structural.aft_center_body       = W_aft_center_body
    output.empty.structural.landing_gear          = landing_gear.main +  landing_gear.nose  
    output.empty.structural.nacelle               = W_energy_network.W_nacelle* (1. - W_factors.nacelle)
    output.empty.structural.total = output.empty.structural.wings   + output.empty.structural.center_body + output.empty.structural.aft_center_body + output.empty.structural.landing_gear\
                                    + output.empty.structural.nacelle 
    
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
    total_fuel_weight           = vehicle.mass_properties.max_takeoff - output.zero_fuel_weight

    # assume fuel is equally distributed in fuel tanks
    if use_max_fuel_weight:
        for network in vehicle.networks: 
            for fuel_line in network.fuel_lines:  
                for fuel_tank in fuel_line.fuel_tanks:
                    fuel_weight =  total_fuel_weight/number_of_tanks  
                    fuel_tank.fuel.mass_properties.mass = fuel_weight
                    
    nose_landing_gear = False
    main_landing_gear = False
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

    return output


def generate_represenative_main_wing(wing, vehicle):
     
        
    # Compute Wing Weight 
    bwb_vehicle = deepcopy(vehicle) 
    
    bwb_vehicle.wings[wing.tag].segments.clear()
    for fus_segment in vehicle.wings[wing.tag].segments:
        bwb_wing_seg = deepcopy(fus_segment)
        if isinstance(fus_segment, RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment):
            bwb_vehicle.wings[wing.tag].spans.projected = vehicle.wings[wing.tag].spans.projected * bwb_wing_seg.percent_span_location
        else:
            bwb_vehicle.wings[wing.tag].segments.append(bwb_wing_seg)
    bwb_vehicle.wings[wing.tag].spans.projected = vehicle.wings[wing.tag].spans.projected - bwb_vehicle.wings[wing.tag].spans.projected
    for idx, segment in enumerate(bwb_vehicle.wings[wing.tag].segments):
        if idx ==0:
            bwb_vehicle.wings[wing.tag].chords.root  =  bwb_vehicle.wings[wing.tag].chords.root* segment.root_chord_percent  
            starting_span_percentage = bwb_vehicle.wings[wing.tag].segments[segment.tag].percent_span_location
    last_percentage = 1.0
    for segment in bwb_vehicle.wings[wing.tag].segments:
        segment.percent_span_location = (segment.percent_span_location - starting_span_percentage) / (last_percentage - starting_span_percentage)
        segment.root_chord_percent    = (vehicle.wings[wing.tag].segments[segment.tag].root_chord_percent * vehicle.wings[wing.tag].chords.root ) / bwb_vehicle.wings[wing.tag].chords.root
        
        
    bwb_wing = segment_properties(bwb_vehicle.wings[wing.tag],update_ref_areas=True) 
        
    return bwb_wing