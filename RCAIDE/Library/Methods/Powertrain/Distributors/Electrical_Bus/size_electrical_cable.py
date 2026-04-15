# RCAIDE/Methods/Powertrain/Distributors/Electrical_Bus/size_electrical_cable.py
#
# Created:  Apr 2026, S. Sharma

# Python Imports
import numpy as np
from scipy.optimize import fsolve


def size_electrical_cable(bus):
    """
    Cable Sizing:
    Calculates the physical dimensions, mass, and resistance of an electrical cable 
    based on the absolute maximum design power it must carry to prevent thermal failure 
    and dielectric breakdown.

    Parameters
    ----------
    P_max_watts : float
        Absolute maximum design power the cable will carry in Watts
        
    V_sys : float
        Nominal system source voltage in Volts
        
    L : float
        Total length of the cable in meters
        
    theta_a : float, optional
        Ambient environment temperature in Kelvin, defaults to 293 K (20 C - Room Temp)
        
    theta_max : float, optional
        Maximum allowable operating temperature of the cable in Kelvin, defaults to 423 K 
        (150 C - Cable melting temp for materials like PTFE/Teflon)
        
    E0 : float, optional
        Maximum electric field the insulation can withstand in V/m, defaults to 2.5e6
        (Dielectric Strength)
        
    rho_elec : float, optional
        Electrical resistivity of the conductor at operating temperature in Ohm-m, 
        defaults to 1.68e-8 (Copper)
        
    rho_cond : float, optional
        Mass density of the conductor material in kg/m^3, defaults to 8960 (Copper)
        
    rho_insul : float, optional
        Mass density of the insulation material in kg/m^3, defaults to 1280 (Polyimide)
        
    rho_theta_insul : float, optional
        Thermal resistivity of the insulation material in K*m/W, defaults to 5.0
        (1/Thermal_Conductivity)
        
    T4 : float, optional
        External thermal resistance of the environment, defaults to 1.0

    Returns
    -------
    dict
        Dictionary containing the sized cable properties:
        
        - Conductor_Radius_mm : float
            Required radius of the inner conductive wire in millimeters
        - Insulation_Radius_mm : float
            Total outer radius of the cable including insulation in millimeters
        - Cable_Mass_kg : float
            Total computed mass of the cable segment in kilograms
        - Total_Resistance_ohms : float
            Total fixed electrical resistance of the sized cable in Ohms

    Notes
    -----
    The conductor radius is sized iteratively by solving for thermal equilibrium where 
    joule heating equals heat dissipation. The insulation is sized to prevent dielectric 
    breakdown based on the maximum electric field threshold.
    """
    P_max_watts         = bus.design_power
    V_sys               = bus.design_voltage 
    L                   = bus.length 
    theta_a             = bus.design_ambient_temperature
    theta_max           = bus.maximum_temperature 
    rho_elec            = bus.conductor.material.compute_electrical_resistivity(theta_max)
    E0                  = bus.insulator.material.dielectric_strength
    rho_cond            = bus.conductor.material.density
    rho_insul           = bus.insulator.material.density
    rho_theta_insul     = bus.insulator.material.thermal_resistivity
    T4                  = bus.environmental_external_thermal_resistance
    
    # 1. Max design current
    I_max = P_max_watts / V_sys

    # 2. Iteratively solve for conductor radius (r_cond) based on thermal limits
    def thermal_residual(r):
        # Electrical resistance per meter (R_prime)
        R_prime = rho_elec / (np.pi * r**2)
        # Thermal resistance of insulation (T1) using substitution from Eq 18
        T1 = (rho_theta_insul / (2 * np.pi)) * (V_sys / (E0 * r))
        # The residual should be 0 when thermal equilibrium is met
        return (theta_max - theta_a) - (I_max**2 * R_prime * (T1 + T4))
    
    # Using fsolve with an initial guess of 2mm (0.002 meters)
    r_cond_m = fsolve(thermal_residual, 0.002)[0]
    
    # 3. Calculate insulation radius based on dielectric breakdown (Eq 18)
    r_insul_m = r_cond_m * np.exp(V_sys / (E0 * r_cond_m))
    
    # 4. Calculate Total Cable Mass (Eq 22)
    vol_cond = np.pi * L * r_cond_m**2
    vol_insul = np.pi * L * (r_insul_m**2 - r_cond_m**2)
    cond_mass_kg = (vol_cond * rho_cond) 
    ins_mass_kg = (vol_insul * rho_insul)
    cable_mass_kg = cond_mass_kg + ins_mass_kg
    
    # 5. Calculate Fixed Total Resistance for the mission solver
    R_total_ohms = (rho_elec * L) / (np.pi * r_cond_m**2)
    
    bus.conductor.radius        = r_cond_m
    bus.insulator.radius        = r_insul_m
    bus.mass_properties.mass    = cable_mass_kg
    bus.conductor.resistance    = R_total_ohms
    
    return