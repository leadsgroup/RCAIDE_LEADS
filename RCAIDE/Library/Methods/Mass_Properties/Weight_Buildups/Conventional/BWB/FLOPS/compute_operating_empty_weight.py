# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Conventional/BWB/FLOPS/operating_empty_weight.py
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
from .compute_operating_items_weight import compute_operating_items_weight
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Common import compute_payload_weight
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Transport import FLOPS 
from RCAIDE.Library.Methods.Geometry.Planform                          import segment_properties, wing_planform
 
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

    # Set the factors
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
    for system in vehicle.systems:
        if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Avionics:
            if system.mass_properties.mass == 0 or system.mass_properties.calculated_flag:
                system.mass_properties.mass = W_systems.W_avionics 
                system.mass_properties.calculated_flag = True
            else:
                W_systems.W_avionics = system.mass_properties.mass
        if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Flight_Controls:
            if system.mass_properties.mass == 0 or system.mass_properties.calculated_flag:
                system.mass_properties.mass = W_systems.W_flight_control 
                system.mass_properties.calculated_flag = True
            else:
                W_systems.W_flight_control = system.mass_properties.mass
        if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Auxillary_Power_Unit: 
            if system.mass_properties.mass == 0 or system.mass_properties.calculated_flag:
                system.mass_properties.mass = W_systems.W_apu 
                system.mass_properties.calculated_flag = True
            else:
                W_systems.W_apu = system.mass_properties.mass
        if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Electrical: 
            if system.mass_properties.mass == 0 or system.mass_properties.calculated_flag:
                system.mass_properties.mass = W_systems.W_electrical 
                system.mass_properties.calculated_flag = True
            else:
                W_systems.W_electrical = system.mass_properties.mass
        if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Hydraulics: 
            if system.mass_properties.mass == 0 or system.mass_properties.calculated_flag:
                system.mass_properties.mass = W_systems.W_hyd_pnu 
                system.mass_properties.calculated_flag = True
            else:
                W_systems.W_hyd_pnu = system.mass_properties.mass
        if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Environmental_Controls: 
            if system.mass_properties.mass == 0 or system.mass_properties.calculated_flag:
                system.mass_properties.mass = W_systems.W_ac + W_systems.W_anti_ice   
                system.mass_properties.calculated_flag = True
            else:
                W_systems.W_anti_ice = system.mass_properties.mass * 0.5
                W_systems.W_ac       = system.mass_properties.mass * 0.5
        if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Instruments:
            if system.mass_properties.mass == 0 or system.mass_properties.calculated_flag:     
                system.mass_properties.mass = W_systems.W_instruments 
                system.mass_properties.calculated_flag = True
            else:
                W_systems.W_instruments = system.mass_properties.mass

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

        W_propulsion                        = FLOPS.compute_propulsion_system_weight(vehicle, network, settings)
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
            propulsor.mass_properties.mass = (W_energy_network.W_engine +W_energy_network.W_thrust_reverser+W_energy_network.W_starter +\
                                            W_energy_network.W_engine_controls) / number_of_engines
            propulsor.nacelle.mass_properties.mass = W_energy_network.W_nacelle / number_of_engines
    
        # Electric-Powered Propulsors  
        for bus in network.busses: 
            # electrical payload 
            W_systems.W_electrical  += bus.systems.mass_properties.mass * Units.kg
     
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
            fidelity = settings.FLOPS.fidelity 
            sym_wing = generate_represenative_main_wing(wing, vehicle) 
            W_wing = compute_wing_weight(vehicle, sym_wing, WPOD, fidelity, settings, num_main_wings)
            if np.isnan(W_wing):
                W_wing = 0.
            wing.mass_properties.mass = W_wing
            W_main_wing += W_wing
        if isinstance(wing, Wings.Horizontal_Tail):
            W_tail = FLOPS.compute_horizontal_tail_weight(vehicle, wing)
            if type(W_tail) == np.ndarray:
                W_tail = sum(W_tail)
            # Pack and sum
            wing.mass_properties.mass = W_tail
            W_tail_horizontal += W_tail
        if isinstance(wing, Wings.Vertical_Tail):
            W_tail = FLOPS.compute_vertical_tail_weight(vehicle, wing)
            # Pack and sum
            wing.mass_properties.mass = W_tail
            W_tail_vertical += W_tail 
        
    ##-------------------------------------------------------------------------------                 
    # Fuselage 
    ##------------------------------------------------------------------------------- 
    TOW                 = vehicle.mass_properties.max_takeoff
    W_cabin             = compute_cabin_weight(vehicle,settings) 
    W_aft_center_body   = compute_aft_center_body_weight(number_of_engines,bwb_aft_center_body_area, bwb_aft_center_body_taper, TOW)
    
    ##-------------------------------------------------------------------------------                 
    # Landing Gear Weight
    ##------------------------------------------------------------------------------- 
    landing_gear = FLOPS.compute_landing_gear_weight(vehicle) 
 
    ##-------------------------------------------------------------------------------                 
    # Accumulate Structural Weight
    ##-------------------------------------------------------------------------------   
    output.empty.structural                       = Data()
    output.empty.structural.wings                 = W_main_wing 
    output.empty.structural.empennage             = W_tail_horizontal +  W_tail_vertical 
    output.empty.structural.center_body           = W_cabin
    output.empty.structural.aft_center_body       = W_aft_center_body
    output.empty.structural.landing_gear          = landing_gear.main +  landing_gear.nose  
    output.empty.structural.nacelle               = W_energy_network.W_nacelle
    output.empty.structural.total = output.empty.structural.wings   + output.empty.structural.center_body + output.empty.structural.aft_center_body + output.empty.structural.landing_gear\
                                    + output.empty.structural.nacelle +output.empty.structural.empennage 
    
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
 
    for wing in vehicle.wings:
        if isinstance(wing, Wings.Blended_Wing_Body):     
            wing.aft_center_body.mass_properties.mass = output.empty.structural.aft_center_body  +  output.empty.propulsion.miscellaneous
            wing.center_body.mass_properties.mass     = output.empty.structural.center_body  + output.operational_items.total +  output.empty.systems.furnishings 
    
    #-------------------------------------------------------------------------------                 
    # Assign landing gear weights to landing gear components 
    #-------------------------------------------------------------------------------
    # Assign landing gear weights to landing gear components 
    for LG in vehicle.landing_gears:
        if isinstance(LG, RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear): 
            LG.mass_properties.mass = landing_gear.main 
        elif isinstance(LG, RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear):  
            LG.mass_properties.mass = landing_gear.nose   

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
        segment.chords.reference_area_root = False
        
    bwb_wing = wing_planform(bwb_vehicle.wings[wing.tag]) 
        
    return bwb_wing