# RCAIDE/Library/Methods/Stability/Moment_of_Inertia/compute_aircraft_moment_of_inertia.py 
# 
# Created:  September 2024, A. Molloy

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia import compute_cuboid_moment_of_inertia, compute_cylinder_moment_of_inertia,compute_rounded_end_cylinder_moment_of_inertia, compute_wing_moment_of_inertia

import RCAIDE
import numpy as  np

# ------------------------------------------------------------------        
#  Component moments of inertia (MOI) tensors
# ------------------------------------------------------------------  
def compute_aircraft_moment_of_inertia(vehicle, CG_location, update_moment_of_inertia=True): 
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

    C =  RCAIDE.Library.Components
    
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
        MOI_mass   += mass
        
        for cabin in fuselage.cabins: 
            I, mass = cabin.compute_moment_of_inertia(center_of_gravity = CG_location)
            MOI_tensor += I
            MOI_mass   += mass
             
    # ------------------------------------------------------------------        
    #  Wing(s)
    # ------------------------------------------------------------------      
    for wing in vehicle.wings:
        I, mass = wing.compute_moment_of_inertia(center_of_gravity =CG_location)
        MOI_tensor += I
        MOI_mass   += mass

        if isinstance(wing, C.Wings.Blended_Wing_Body): 
            for cabin in wing.cabins:
                I, mass = cabin.compute_moment_of_inertia(center_of_gravity = CG_location)
                MOI_tensor += I
                MOI_mass   += mass 
    
    # ------------------------------------------------------------------        
    # Cargo Bay
    # ------------------------------------------------------------------      
    for cargo_bay in vehicle.cargo_bays:
        I, mass = cargo_bay.compute_moment_of_inertia(center_of_gravity = CG_location)
        MOI_tensor += I
        MOI_mass   += mass 
    
    # ------------------------------------------------------------------        
    # Landing Gear
    # ------------------------------------------------------------------      
    for landing_gear in vehicle.landing_gears: 
        if isinstance(landing_gear,RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear):
            landing_gear.length = landing_gear.strut_length * 1.1
            landing_gear.width  = landing_gear.tire_diameter* 1.1
            landing_gear.height = landing_gear.tire_diameter* 1.1
        else:
            landing_gear.length = landing_gear.tire_diameter* 1.1
            landing_gear.width  = landing_gear.strut_length* 1.1
            landing_gear.height = landing_gear.tire_diameter* 1.1
            
        I, mass = landing_gear.compute_moment_of_inertia()
        MOI_tensor += I
        MOI_mass   += mass
            
    # ------------------------------------------------------------------        
    #  Energy network
    # ------------------------------------------------------------------      
    I_network = np.zeros([3, 3]) 
    for network in vehicle.networks:
        for propulsor in network.propulsors:
            if isinstance(propulsor,C.Powertrain.Propulsors.Electric_Rotor):
                for assigned_converter_tag in propulsor.assigned_converters:
                    if isinstance(network.converters[assigned_converter_tag[0][0]], RCAIDE.Library.Components.Powertrain.Converters.Motor):
                        motor = network.converters[assigned_converter_tag[0][0]]
                I, mass = compute_cylinder_moment_of_inertia(motor.origin,motor.mass_properties.mass, 0, 0, 0,0, CG_location)
                I_network += I
                MOI_mass  += mass
                    
            if isinstance(propulsor,C.Powertrain.Propulsors.Turbofan):
                I, mass= compute_cylinder_moment_of_inertia(propulsor.origin, propulsor.mass_properties.mass, propulsor.length, propulsor.nacelle.diameter/2, 0, 0, CG_location)                    
                I_network += I
                MOI_mass += mass
            if isinstance(propulsor,C.Powertrain.Propulsors.Turboprop):
                I, mass= compute_cylinder_moment_of_inertia(propulsor.origin, propulsor.mass_properties.mass, propulsor.length, propulsor.diameter/2, 0, 0, CG_location)                    
                I_network += I
                MOI_mass += mass
            if isinstance(propulsor,C.Powertrain.Propulsors.Internal_Combustion_Engine) or  isinstance(propulsor,C.Powertrain.Propulsors.Constant_Speed_Internal_Combustion_Engine):
                I, mass= compute_cylinder_moment_of_inertia(propulsor.origin, propulsor.mass_properties.mass, propulsor.length, propulsor.diameter/2, 0, 0, CG_location)                    
                I_network += I
                MOI_mass += mass
        
        for distributor in network.distributors:

            if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):

                for source in network.sources: 
                    assigned_tags = []
                    for distributors_tags in source.assigned_distributors:
                        if isinstance(distributors_tags, (list, tuple, set)):
                            assigned_tags.extend(list(distributors_tags))
                        else:
                            assigned_tags.append(distributors_tags)

                    if distributor.tag in assigned_tags:
                        if issubclass(type(source), RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Generic_Battery_Module):
                            I_battery, mass_battery = compute_cuboid_moment_of_inertia(source.origin, source.mass_properties.mass, source.length, source.width, source.height, 0, 0, 0, CG_location)
                            I_network += I_battery
                            MOI_mass  += mass_battery         

            elif isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):

                for source in network.sources:

                    assigned_tags = []
                    for distributors_tags in source.assigned_distributors:
                        if isinstance(distributors_tags, (list, tuple, set)):
                            assigned_tags.extend(list(distributors_tags))
                        else:
                            assigned_tags.append(distributors_tags)

                    if distributor.tag in assigned_tags:

                        if isinstance(source,C.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank):
                            if source.geometry_type == 'prismatic': 
                                I, mass = compute_cuboid_moment_of_inertia(source.origin, source.fuel.mass_properties.mass, source.outer_length, source.outer_width, source.outer_height,\
                                                                        source.outer_length- 2*source.wall_thickness, source.outer_width- 2*source.wall_thickness, source.outer_height- 2*source.wall_thickness, CG_location)
                                I_network += I
                                MOI_mass += mass
                            else: 
                                I, mass = compute_rounded_end_cylinder_moment_of_inertia(source.origin, source.fuel.mass_properties.mass, source.outer_length,
                                                                                        source.outer_diameter/2, source.outer_length - 2*source.wall_thickness, source.inner_diameter/2, CG_location)
                                I_network += I                    
                                                                            
                        if  isinstance(source,C.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank):                                   
                            I, mass = compute_rounded_end_cylinder_moment_of_inertia(source.origin, source.fuel.mass_properties.mass, source.outer_length,
                                                                                     source.outer_diameter/2, source.outer_length - 2*source.wall_thickness, source.inner_diameter/2, CG_location)
                            I_network += I                    
                                                
                        if isinstance(source,C.Powertrain.Sources.Fuel_Tanks.Integral_Tank):
                            I, mass =  compute_wing_moment_of_inertia(vehicle.wings["main_wing"], mass=source.fuel.mass_properties.mass, center_of_gravity = CG_location, fuel_flag=True)
                            I_network += I
                            MOI_mass += mass   
                        
    MOI_tensor += I_network    
    
    if update_moment_of_inertia:
        vehicle.mass_properties.moments_of_inertia.tensor = MOI_tensor  
    return MOI_tensor,MOI_mass     