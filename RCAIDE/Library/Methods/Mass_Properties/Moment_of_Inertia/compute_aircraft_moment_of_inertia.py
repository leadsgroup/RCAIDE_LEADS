# RCAIDE/Library/Methods/Stability/Moment_of_Inertia/compute_aircraft_moment_of_inertia.py 
# 
# Created:  September 2024, A. Molloy

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia import compute_cuboid_moment_of_inertia, compute_cylinder_moment_of_inertia, compute_wing_moment_of_inertia, compute_LOPA_moment_of_inertia

import RCAIDE
import numpy as  np 
from copy import deepcopy

# ------------------------------------------------------------------        
#  Component moments of inertia (MOI) tensors
# ------------------------------------------------------------------  
def compute_aircraft_moment_of_inertia(vehicle, CG_location, update_MOI=True): 
    ''' sums the moments of inertia of each component in the aircraft. Components summed: fuselages,
    wings (main, horizontal, tail + others), turbofan engines, batteries, motors, batteries, fuel tanks

    Assumptions:
    - All other components than those listed are insignificant

    Source:
 
    Inputs:
    - vehicle
    - Center of gravity

    Outputs:
    - Total aircraft moment of inertia tensor

    Properties Used:
    N/A
    '''    
    
    # ------------------------------------------------------------------        
    # Setup
    # ------------------------------------------------------------------      
    # Array to hold the entire aircraft's inertia tensor
    MOI_tensor = np.zeros((3, 3)) 
    MOI_mass = 0
    
    # ------------------------------------------------------------------        
    #  Fuselage(s)
    # ------------------------------------------------------------------      
    for fuselage in vehicle.fuselages:
        I, mass = fuselage.compute_moment_of_inertia(center_of_gravity = CG_location)
        MOI_tensor += I
        MOI_mass += mass

        #if len(fuselage.cabins) >0:
            #I, mass     = compute_LOPA_moment_of_inertia(vehicle.payload.passengers,fuselage.layout_of_passenger_accommodations, center_of_gravity =CG_location)  
            #MOI_tensor += I
            #MOI_mass   += mass          
    
    # ------------------------------------------------------------------        
    #  Wing(s)
    # ------------------------------------------------------------------      
    for wing in vehicle.wings:

        if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            wing_cabin = deepcopy(wing)
            wing_outboard = deepcopy(wing)
            wing_cabin.segments.clear()
            wing_outboard.segments.clear()
            center_fuse_number = 1
            outboard_segment_number = 1
            for segment in wing.segments:
                if isinstance(segment, RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment):
                    if center_fuse_number == 1:
                        wing_cabin.chords.root = segment.root_chord_percent * wing.chords.root
                        wing_cabin.thickness_to_chord = segment.thickness_to_chord
                        net_x_location = wing_cabin.chords.root * 0.25
                        id_first = segment.tag
                    else:
                        net_x_location += wing.spans.total / 2 * (segment.percent_span_location - wing_cabin.segments[id_first].percent_span_location)*np.sin(wing_cabin.segments[id_first].sweeps.quarter_chord)
                    segment.root_chord_percent = segment.root_chord_percent * wing.chords.root/wing_outboard.chords.root
                    wing_cabin.chords.tip = segment.root_chord_percent * wing.chords.root
                    wing_cabin.spans.total = segment.percent_span_location * wing.spans.total
                    wing_cabin.append_segment(segment)
                    center_fuse_number += 1
                    wing_cabin.sweeps.quarter_chord = np.arctan(net_x_location/wing_cabin.spans.total)
                    tag_last = segment.tag
                    
                else:
                    if outboard_segment_number == 1:
                        wing_outboard.append_segment(wing.segments[tag_last])
                        wing_outboard.chords.root = segment.root_chord_percent * wing.chords.root
                        wing_outboard.thickness_to_chord = segment.thickness_to_chord
                        wing_outboard.origin = np.array(wing_outboard.origin) + np.array([net_x_location, wing_outboard.segments[tag_last].percent_span_location * wing.spans.total, 0])
                        wing_outboard.spans.total = (1 - wing_outboard.segments[tag_last].percent_span_location) * wing.spans.total
                        net_outboard_x_location = wing_outboard.chords.root * 0.25
                        tag_first = segment.tag
                    else:
                        net_outboard_x_location += wing.spans.total / 2 * (segment.percent_span_location - wing_outboard.segments[tag_last].percent_span_location)*np.sin(wing_outboard.segments[tag_last].sweeps.quarter_chord)
                    
                    segment.root_chord_percent = segment.root_chord_percent * wing.chords.root/wing_outboard.chords.root
                    wing_outboard.append_segment(segment)
                    outboard_segment_number += 1
                    wing_outboard.chords.tip = segment.root_chord_percent * wing.chords.root
                    wing_outboard.sweeps.quarter_chord = np.arctan((net_outboard_x_location - net_x_location)/wing_outboard.spans.total)
                    # need to add wing quarter chord sweep


            RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
            I_outboard, mass_outboard  = wing_outboard.compute_moment_of_inertia(mass=vehicle.mass_properties.weight_breakdown.empty.structural.wings, center_of_gravity =CG_location)
            centerbody_mass = vehicle.mass_properties.weight_breakdown.empty.structural.aft_center_body + vehicle.mass_properties.weight_breakdown.empty.structural.center_body
            I_cabin, mass_cabin  = wing_cabin.compute_moment_of_inertia(mass=centerbody_mass, center_of_gravity =CG_location)
            MOI_tensor += I_outboard + I_cabin
            MOI_mass   += mass_outboard + mass_cabin   

        #if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body) and len(wing.cabins) >0:
            #I, mass  = compute_LOPA_moment_of_inertia(vehicle.payload.passengers, wing.layout_of_passenger_accommodations, center_of_gravity =CG_location)  
            #MOI_tensor += I
            #MOI_mass   += mass   

    else:
        I, mass = wing.compute_moment_of_inertia(mass=wing.mass_properties.mass, center_of_gravity =CG_location)
        MOI_tensor += I
        MOI_mass   += mass 
    # ------------------------------------------------------------------        
    #  Wing(s)
    # ------------------------------------------------------------------      
    #for payload in vehicle.cargo_bays:
        #I, mass = wing.compute_moment_of_inertia(payload.mass_properties.mass, payload.length, payload.width, payload.height, 0, 0, 0, CG_location)
        #MOI_tensor += I
        #MOI_mass   += mass     
    
    # ------------------------------------------------------------------        
    #  Energy network
    # ------------------------------------------------------------------      
    I_network = np.zeros([3, 3]) 
    for network in vehicle.networks:
        for propulsor in network.propulsors:
            if isinstance(propulsor,RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor):
                motor   = propulsor.motor 
                I, mass = compute_cylinder_moment_of_inertia(motor.origin,motor.mass_properties.mass, 0, 0, 0,0, CG_location)
                I_network += I
                MOI_mass  += mass
                    
            if isinstance(propulsor,RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan):
                I, mass= compute_cylinder_moment_of_inertia(propulsor.origin, propulsor.mass_properties.mass, propulsor.length, propulsor.nacelle.diameter/2, 0, 0, CG_location)                    
                I_network += I
                MOI_mass += mass
            if isinstance(propulsor,RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop):
                I, mass= compute_cylinder_moment_of_inertia(propulsor.origin, propulsor.mass_properties.mass, propulsor.length, propulsor.diameter/2, 0, 0, CG_location)                    
                I_network += I
                MOI_mass += mass
            if isinstance(propulsor,RCAIDE.Library.Components.Powertrain.Propulsors.Internal_Combustion_Engine) or  isinstance(propulsor,RCAIDE.Library.Components.Powertrain.Propulsors.Constant_Speed_Internal_Combustion_Engine):
                I, mass= compute_cylinder_moment_of_inertia(propulsor.origin, propulsor.mass_properties.mass, propulsor.length, propulsor.diameter/2, 0, 0, CG_location)                    
                I_network += I
                MOI_mass += mass
        
        for bus in network.busses: 
            for battery in bus.battery_modules: 
                I_battery, mass_battery = compute_cuboid_moment_of_inertia(battery.origin, battery.mass_properties.mass, battery.length, battery.width, battery.height, 0, 0, 0, CG_location)
                I_network += I_battery
                MOI_mass  += mass_battery         
                                 
        for fuel_line in network.fuel_lines:
            for fuel_tank in fuel_line.fuel_tanks:
                if isinstance(fuel_tank,RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank): 
                    I, mass = compute_cylinder_moment_of_inertia(fuel_tank.origin, fuel_tank.fuel.mass_properties.mass, fuel_tank.length, fuel_tank.outer_diameter/2, 0, 0, CG_location)
                    I_network += I
                    MOI_mass += mass
                    
                if isinstance(fuel_tank,RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank):
                    I, mass =  compute_wing_moment_of_inertia(vehicle.wings["main_wing"], mass=fuel_tank.fuel.mass_properties.mass, center_of_gravity = CG_location, fuel_flag=True)
                    I_network += I
                    MOI_mass += mass                    
                else:
                    pass # TO DO
                        
    MOI_tensor += I_network    
    
    if update_MOI:
        vehicle.mass_properties.moments_of_inertia.tensor = MOI_tensor  
    return MOI_tensor,MOI_mass     