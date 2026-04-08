# RCAIDE/Methods/Powertrain/Distributors/Electrical_Bus/compute_electrical_bus_conditions.py
# 
# 
# Created: Mar 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# imports 
import numpy as np
from RCAIDE.Library.Methods.Powertrain.Converters.Pump.compute_pump_performance import compute_pump_performance
from scipy.optimize import fsolve
 
# ----------------------------------------------------------------------------------------------------------------------
# compute_bus_conditions
# ----------------------------------------------------------------------------------------------------------------------
def compute_fuel_line_conditions(fuel_line, state,network): 
    fuel_line_conditions              = state.conditions.energy.distributors[fuel_line.tag]   
    
    pump = fuel_line.pump
    flow_rate = fuel_line_conditions.mass_flow_rate
    length = fuel_line.length
    inner_diameter = fuel_line.inner_diameter
    surface_roughness = fuel_line.surface_roughness
    p_tank_pa = 1.01e5
    p_engine_pa = 2.02e5 # Pa
    density = fuel.density
    k_viscosity = fuel.viscosity
    delta_z = 5 # in m
    k_factor = 0.9
    
    # Fittings Library (Lookup Table)
    # Values are taken from Crane TP-410 
    K_Factor_Library = {
    '90_deg_bend_standard': 0.9,
    '90_deg_bend_long_radius': 0.6,
    '45_deg_bend': 0.4,
    'gate_valve_fully_open': 0.2,
    'globe_valve_fully_open': 10.0,
    'pipe_entrance_square': 0.5,
    'pipe_entrance_rounded': 0.05,
    'pipe_exit': 1.0
    }

    # 1. Initialization
    Q = flow_rate / density  # m^3/s
    D = inner_diameter #m
    e = surface_roughness / 1000   # m
    area = np.pi * (D/2)**2
    v = Q / area              # velocity (m/s)
    g = 9.81

    # 2. Dynamically Calculate K_total
    k_total = 0
    for fitting_name, count in fittings_count.items():
        if fitting_name in K_Factor_Library:
            k_total += K_Factor_Library[fitting_name] * count
        else:
            print(f"'{fitting_name}' not found in K_Factor_Library. K_total = 0.")
    
    # 3. Reynolds Number
    Re = (v * D) / k_viscosity

    # 4. Friction Factor (f)
    if Re < 2300:
        f = 64 / Re
    else:
        # Implicit Colebrook-White equation: 1/sqrt(f) = -2*log10( (e/D)/3.7 + 2.51/(Re*sqrt(f)) )
        def colebrook(f_initial):
            return 1/np.sqrt(f_initial) + 2*np.log10((e/D)/3.7 + 2.51/(Re*np.sqrt(f_initial)))
        
        # f_initial using Swamee-Jain explicit formula
        f_initial = 0.25 / (np.log10((e/D)/3.7 + 5.74/(Re**0.9)))**2
        f = fsolve(colebrook, f_initial)[0]

    # 5. Head Loss (m)
    h_major     = f * (length / D) * (v**2 / (2 * g))
    h_minor     = k_total * (v**2 / (2 * g))
    h_static    = delta_z
    h_total     = h_major + h_minor + h_static

    # 6. Pressure and Power
    # Pressure to overcome friction/gravity
    delta_p_losses  = h_total * density * g                         # Pa
    # Total pressure pump must add to the fluid
    delta_p_pump    = (p_engine_pa - p_tank_pa) + delta_p_losses
    power_ideal     = Q * delta_p_pump                              # Watts
    power_actual    = power_ideal / pump_efficiency

    pump_conditions = state.conditions.converters[pump.tag]
    pump_conditions.inputs.pressure = 
    pump_conditions.outputs.pressure = 
    pump_conditions.outputs.power = 
    compute_pump_performance(pump, fuel_line, state)

    

    
    Results = {
        'Reynolds': Re,
        'Friction_Factor': f,
        'Total_K_Factor': k_total,
        'Total_Head_Loss': h_total,
        'delta_P of Pipe': delta_p_losses,
        'Pump_Power': power_actual
    }
    
    return Results

# Example
# Flow Rate        : 5 L/s
# Pipe Properties  : Length (20 m), Inner Diameter (50 mm), Material (SS - Roughness = 0.015 mm), Delta Z (5 m), 
#                    3 bends (K=0.9 each)
# Fuel             : Jet A - Density (804 kg/m^3), Kinematic Viscosity (2.1e-6 m^2/s)
# Pump Efficiency  : 0.75
# Pressure Values  : Fuel Tank Pressure (1 atm), Combustion Chamber Pressure (1 atm)

# Define the Pipeline Route
    pipeline_fittings = {
        'pipe_entrance_rounded': 1,
        '90_deg_bend_standard': 3,
        'gate_valve_fully_open': 1,
        'pipe_exit': 1
    }
    
    
    
    
        
    return fuel_line_conditions.inputs, fuel_line_conditions.outputs