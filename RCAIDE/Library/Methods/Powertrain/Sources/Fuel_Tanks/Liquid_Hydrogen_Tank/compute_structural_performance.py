# RCAIDE/Library/Components/Powertrain/Energy/Sources/Fuel_Tanks/compute_structural_performance.py
# 
# Created: Aug 2025, S. Shekar
#
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core import Units

# Python imports
from copy import deepcopy
import numpy as np
from scipy.optimize import minimize

# ----------------------------------------------------------------------------------------------------------------------
#  Structural Solver
# ----------------------------------------------------------------------------------------------------------------------    
def compute_structural_performance(fuel_tank):
    """
    Compute the structural sizing and mass of a cryogenic fuel tank using 
    thick-walled cylinder theory and von Mises failure criteria.  

    Parameters
    ----------
    fuel_tank : Fuel_Tank
        Fuel tank object containing geometric, thermal, and material properties.  

    Returns
    -------
    None
        Updates the following attributes of the fuel_tank object in place:
            - mass : float
                Tank mass [kg].     
            - internal_volume : float
                Total internal volume including ullage [m³].  
            - inner_diameter : float
                Internal diameter [m].  
            - inner_length : float
                Internal cylindrical length [m].  

    Notes
    -----
    * Uses Lame’s equations for stress distribution in thick-walled cylinders.  
    * Hoop, radial, and axial stresses are reduced to an equivalent von Mises stress.  
    * Iteratively adjusts tank radius until structural equilibrium is satisfied.  

    **Major Assumptions**
        * Tank approximated as a cylindrical shell with hemispherical end caps.  
        * Fuel saturation pressure determines design internal pressure.  
        * No allowance for creep or fatigue (static strength only).  
        * Symmetry doubles the usable fuel volume if specified.  
    """
    
    # Constants
    safety_factor   = 1.6          # structural factor of safety
    pressure_factor = 5.0          # internal pressure multiplier for sizing
    T_inlet         = fuel_tank.design_inlet_temperature

    # Saturation and design pressures
    P_sat = fuel_tank.fuel.propellant_properties(T_inlet, "Pressure (MPa)") * Units.MPa
    P_internal = pressure_factor * P_sat
    P_external = fuel_tank.design_external_pressure

    # Initial fuel volume guess
    if fuel_tank.xz_plane_symmetric:
        V_guess = deepcopy(fuel_tank.volume_properties.gross_volume  * 0.45)
    else:
        V_guess = deepcopy(fuel_tank.volume_properties.gross_volume * 0.75)

    # Iterative solver loop
    tol       = 1e-5
    error     = 1e2
    alpha     = 0.5
    iteration = 0
    max_iter  = 10000

    while abs(error) > tol and iteration < max_iter:
        # Compute internal tank geometry
        V_total = V_guess / (1 - fuel_tank.ullage_volume_fraction)  
        r_inner = ( V_total/(2*np.pi*(fuel_tank.aspect_ratio-1/3)) )**(1/3)
        L_inner = (2 * r_inner * fuel_tank.aspect_ratio)-2*r_inner

        # Optimize wall thickness ratio (ro/ri) using von Mises criterion
        ro_ri = minimize(
            tank_width,
            x0=(1 + 1e-3),
            method='L-BFGS-B',
            tol=1e-5,
            args=(P_internal, P_external, safety_factor, fuel_tank)
        ).x[0]

        r_outer = ro_ri * r_inner

        # Convergence check
        error                                          = fuel_tank.outer_diameter / 2 - r_outer
        rel_error                                      = error / (fuel_tank.outer_diameter / 2)
        fuel_tank.fuel.volume_properties.net_volume    = V_guess
        V_guess                                       += alpha * rel_error
        iteration                                     += 1

    # Store results
    fuel_tank.inner_diameter                 = 2 * r_inner
    fuel_tank.volume_properties.net_volume   = V_total
    fuel_tank.inner_length                   = L_inner
    fuel_tank.wall_thickness                 = (fuel_tank.outer_diameter - fuel_tank.inner_diameter) / 2   
    
    if fuel_tank.xz_plane_symmetric:
        fuel_tank.volume_properties.net_volume      *= 2
        fuel_tank.fuel.volume_properties.net_volume *= 2
    
    fuel_tank.fuel.mass_properties.mass =  fuel_tank.fuel.volume_properties.net_volume *  fuel_tank.fuel.density
    
    V_material = fuel_tank.volume_properties.gross_volume - fuel_tank.volume_properties.net_volume
    
    fuel_tank.mass_properties.structural_mass = V_material * fuel_tank.material.density  # Structural Mass of the tank

    return


def tank_width(ro_ri, P_internal, P_external, safety_factor, fuel_tank):
    """
    Compute the von Mises stress difference for a candidate tank wall thickness.  

    Parameters
    ----------
    ro_ri : float
        Outer-to-inner radius ratio of the tank.  
    P_internal : float
        Design internal pressure [Pa].  
    P_external : float
        Design external pressure [Pa].  
    safety_factor : float
        Structural factor of safety.  
    fuel_tank : Fuel_Tank
        Fuel tank object with material properties.  

    Returns
    -------
    stress_diff : float
        Absolute difference between actual von Mises stress and 
        allowable yield stress (scaled by safety factor).  

    Notes
    -----
    * Uses Lame’s constants to compute radial, hoop, and axial stresses.  
    * Von Mises criterion reduces these stresses to an equivalent stress.  
    """
    # Lame’s constants
    A = (P_internal - P_external * ro_ri**2) / (ro_ri**2 - 1)
    B = (P_internal - P_external) * ro_ri**2 / (ro_ri**2 - 1)

    # Principal stresses
    sigma_theta = A + B    # hoop stress
    sigma_r     = A - B    # radial stress
    sigma_z     = A        # axial stress

    # Von Mises equivalent stress
    sigma_vm = np.sqrt(((sigma_theta - sigma_r)**2 + 
                        (sigma_r - sigma_z)**2 + 
                        (sigma_z - sigma_theta)**2) / 2)

    return np.abs(sigma_vm - fuel_tank.material.yield_tensile_strength / safety_factor)
