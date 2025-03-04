# RCAIDE/Library/Methods/Stability/Moment_of_Inertia/compute_aircraft_moment_of_inertia.py 
# 
# Created:  September 2024, A. Molloy

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia import compute_cuboid_moment_of_inertia, compute_cylinder_moment_of_inertia, compute_wing_moment_of_inertia

import RCAIDE
import numpy as  np 

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
    
    # ------------------------------------------------------------------        
    #  Wing(s)
    # ------------------------------------------------------------------      
    for wing in vehicle.wings:
        I, mass = wing.compute_moment_of_inertia(mass=wing.mass_properties.mass, center_of_gravity =CG_location)
        MOI_tensor += I
        MOI_mass += mass
    
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
                I, mass= compute_cylinder_moment_of_inertia(propulsor.origin, propulsor.mass_properties.mass, propulsor.engine_length, propulsor.nacelle.diameter/2, 0, 0, CG_location)                    
                I_network += I
                MOI_mass += mass
            if isinstance(propulsor,RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop):
                I, mass= compute_cylinder_moment_of_inertia(propulsor.origin, propulsor.mass_properties.mass, propulsor.engine_length, propulsor.engine_diameter/2, 0, 0, CG_location)                    
                I_network += I
                MOI_mass += mass
            if isinstance(propulsor,RCAIDE.Library.Components.Powertrain.Propulsors.ICE_Propeller):
                I, mass= compute_cylinder_moment_of_inertia(propulsor.origin, propulsor.mass_properties.mass, propulsor.engine_length, propulsor.engine_diameter/2, 0, 0, CG_location)                    
                I_network += I
                MOI_mass += mass
        
        for bus in network.busses: 
            for battery in bus.battery_modules: 
                I_battery, mass_battery = compute_cuboid_moment_of_inertia(battery.origin, battery.mass_properties.mass, battery.length, battery.width, battery.height, 0, 0, 0, CG_location)
                I_network += I_battery
                MOI_mass  += mass_battery         
                                 
        for fuel_line in network.fuel_lines:
            for fuel_tank in fuel_line.fuel_tanks:
                if isinstance(fuel_tank,RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Central_Fuel_Tank ): 
                    I, mass = compute_cuboid_moment_of_inertia(fuel_tank.origin, fuel_tank.fuel.mass_properties.mass, fuel_tank.length, fuel_tank.width, fuel_tank.height, 0, 0, 0, CG_location)
                    I_network += I
                    MOI_mass += mass
                if isinstance(fuel_tank,RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Wing_Fuel_Tank): 
                    I, mass =  compute_wing_moment_of_inertia(vehicle.wings["main_wing"], mass=fuel_tank.fuel.mass_properties.mass, center_of_gravity = CG_location, fuel_flag=True)
                    I_network += I
                    MOI_mass += mass                    
                else:
                    pass # TO DO
                        
    MOI_tensor += I_network    
    
    if update_MOI:
        vehicle.mass_properties.moments_of_inertia.tensor = MOI_tensor  
    return MOI_tensor,MOI_mass     