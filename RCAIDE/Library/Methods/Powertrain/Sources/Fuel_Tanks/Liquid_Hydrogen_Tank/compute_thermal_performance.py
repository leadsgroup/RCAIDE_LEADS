# RCAIDE/Library/Components/Powertrain/Energy/Sources/Fuel_Tanks/Liquid_Hydrogen_Tank/compute_thermal_performance.py
# 
# Created: Aug 2025, S. Shekar
#
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from copy import deepcopy
import RCAIDE
from RCAIDE.Framework.Core import Units

# Python imports
import numpy as np
from scipy.optimize import minimize

# ----------------------------------------------------------------------------------------------------------------------
#  Thermal Solver
# ----------------------------------------------------------------------------------------------------------------------    
def compute_thermal_performance(fuel_tank):
    """
    Compute insulation thickness and thermal performance of a liquid hydrogen tank.

    Parameters
    ----------
    fuel_tank : Fuel_Tank
        Fuel tank object with geometry, material, and design attributes.

    Returns
    -------
    None
        Updates the following attributes of the fuel_tank object in place:
            - wall_thickness : float
                Optimized insulation thickness [m].  
            - insulation_wall_conductive_heat_transfer : float
                Net conduction heat transfer [W].  
            - (prints insulation thickness in inches and mass in lbs).  

    Notes
    -----
    * Uses iterative optimization to determine insulation thickness that satisfies
      acceptable heat leak constraints.  
    * Considers convection, radiation, and conduction heat transfer for both 
      cylindrical walls and spherical end caps.  

    **Major Assumptions**
        * Steady-state heat transfer.  
        * Gray-body radiation with emissivity of 0.03.  
        * Uniform wall thickness and isotropic materials.  

    See Also
    --------
    insulation_width : Helper function for insulation optimization.  
    heat_transfer_wrap : Heat transfer balance evaluation.  
    """
    PI_Q = 1.5  # heat flow multiplier for thermal sizing

    # Get atmospheric conditions at design altitude
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(fuel_tank.design_altitude,
                                           fuel_tank.design_isa_deviation)
    Ta = atmo_data.temperature

    # Tank geometry
    ro = fuel_tank.outer_diameter / 2
    ri = fuel_tank.inner_diameter / 2
    li = fuel_tank.inner_length 

    # Optimize insulation thickness
    t_ins = minimize(
        insulation_width,
        x0=0.01,
        bounds=[(1e-8, 1e8)],
        method='L-BFGS-B',
        tol=1e-10,
        args=(Ta, PI_Q, fuel_tank, atmo_data)
    ).x

    # Insulation geometry and mass
    a_ins = 2 * np.pi * ro * (li) + 4 * np.pi * ro**2
    v_ins = (np.pi * (ro + t_ins)**2 * (li) + (4/3) * np.pi * (ro + t_ins)**3) \
          - (np.pi * ro**2 * (li) + (4/3) * np.pi * ro**3)

    mass_ins = (v_ins * fuel_tank.insulation_material.density
               + a_ins * fuel_tank.insulation_material.specific_density)

    # Store results
    fuel_tank.insulation_thickness = t_ins[0]

    # add the insulation mass to the tank weight
    fuel_tank.mass_properties.insulation_mass =  mass_ins[0]
    if fuel_tank.xz_plane_symmetric:
        fuel_tank.mass_properties.insulation_mass *= 2

    # Recompute net tank volume based on the updated thickness
    r_in   = (fuel_tank.outer_diameter -  2 * (fuel_tank.wall_thickness+fuel_tank.insulation_thickness)) / 2
    l_in = fuel_tank.aspect_ratio * fuel_tank.inner_diameter

    fuel_tank.inner_length = l_in - fuel_tank.inner_diameter
    tank_volume_i = np.pi * ( r_in** 2) * (fuel_tank.inner_length )  +  4 / 3 * np.pi * ( r_in** 3) 
    fuel_volume = (1 - fuel_tank.ullage_volume_fraction)  * tank_volume_i
    
    if fuel_tank.xz_plane_symmetric:
        tank_volume_i *= 2 
        fuel_volume   *=2

    fuel_tank.volume_properties.net_volume      =tank_volume_i
    fuel_tank.fuel.volume_properties.net_volume =deepcopy(fuel_volume)
    fuel_tank.fuel.mass_properties.mass         =deepcopy(fuel_tank.fuel.volume_properties.net_volume *  fuel_tank.fuel.density)

    return 


def insulation_width(t_ins, Ta, PI_Q, fuel_tank, atmo_data):   
    """
    Objective function for insulation thickness optimization.

    Parameters
    ----------
    t_ins : float
        Candidate insulation thickness [m].
    Ta : float
        Ambient temperature [K].
    PI_Q : float
        Heat flow multiplier for sizing.
    fuel_tank : Fuel_Tank
        Fuel tank object.
    atmo_data : Data
        Atmospheric properties at design altitude.

    Returns
    -------
    error : float
        Difference between allowable heat leak and computed conduction load.
    """
    ri = fuel_tank.inner_diameter / 2
    li = fuel_tank.inner_length 
    Ti = fuel_tank.design_inlet_temperature
    Qo = fuel_tank.acceptable_heat_leak

    # Estimate equilibrium wall temperature
    Te = minimize(
        heat_transfer_wrap,
        x0=(Ta[0] + Ti) / 2,
        method='L-BFGS-B',
        bounds=[(Ti, Ta)],
        tol=1e-10,
        args=(t_ins, fuel_tank, atmo_data)
    ).x

    Qc_mat = fuel_tank.insulation_wall_conductive_heat_transfer

    return np.abs(PI_Q * Qc_mat / (2*np.pi*ri*(li) + 4*np.pi*ri**2) - Qo)


def heat_transfer_wrap(Te, t_ins, fuel_tank, atmo_data):
    """
    Compute net heat transfer balance at insulation outer surface.

    Parameters
    ----------
    Te : float
        Effective insulation outer surface temperature [K].
    t_ins : float
        Insulation thickness [m].
    fuel_tank : Fuel_Tank
        Fuel tank object.
    atmo_data : Data
        Atmospheric properties at design altitude.

    Returns
    -------
    error : float
        Absolute difference between external heat input and internal conduction.
    """
    # Geometry
    ro = fuel_tank.outer_diameter / 2
    ri = fuel_tank.inner_diameter / 2
    li = fuel_tank.inner_length 

    # Atmospheric properties
    p        = atmo_data.pressure          
    rho_air  = atmo_data.density             
    mu_air   = atmo_data.dynamic_viscosity
    k_air    = atmo_data.thermal_conductivity   
    Ta       = atmo_data.temperature
    g        = 9.81
    Ti       = fuel_tank.design_inlet_temperature

    # Air properties
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    Cp_air     = atmosphere.fluid_properties.compute_cp(Ta) 

    nu     = mu_air / rho_air             # kinematic viscosity
    alpha  = k_air / (rho_air * Cp_air)   # thermal diffusivity
    Pr     = nu / alpha                   # Prandtl number
    Ra     = (g / Ta) * (Ta - Te) * (2*ro + 2*t_ins)**3 / (alpha * nu)  # Rayleigh number
    
    # ---- Cylindrical section ----
    Nu_cyl = (0.60 + 0.387 * Ra**(1/6) / (1 + (0.559/Pr)**(9/16))**(8/27))**2
    h_cyl  = Nu_cyl * k_air / (2*ro + 2*t_ins)

    Qv_cyl = h_cyl * (np.pi * (2*ro + 2*t_ins) * (li)) * (Ta - Te)
    Qr_cyl = (5.67e-8) * 0.03 * (np.pi * (2*ro + 2*t_ins) * (li)) * (Ta**4 - Te**4)
    Qc_cyl = (Te - Ti) / (
        np.log(ro/ri) / (2*np.pi*(li)*fuel_tank.material.thermal_conductivity)
        + np.log((ro+t_ins)/ro) / (2*np.pi*(li)*fuel_tank.insulation_material.thermal_conductivity)
    )

    # ---- Spherical end caps ----
    Nu_sph = 2 + 0.589 * Ra**(1/4) / (1 + (0.469/Pr)**(9/16))**(4/9)
    h_sph  = Nu_sph * k_air / (2*ro + 2*t_ins)

    Qv_sph = h_sph * (np.pi * (2*ro + 2*t_ins)**2) * (Ta - Te)
    Qr_sph = (5.67e-8) * 0.03 * (np.pi * (2*ro + 2*t_ins)**2) * (Ta**4 - Te**4)
    Qc_sph = (Te - Ti) / (
        (ro - ri) / (4*np.pi*fuel_tank.material.thermal_conductivity*ri*ro)
        + t_ins / (4*np.pi*fuel_tank.insulation_material.thermal_conductivity*ro*(ro+t_ins))
    )

    # Total heat transfer
    Qv = Qv_cyl + Qv_sph
    Qr = Qr_cyl + Qr_sph
    Qc = Qc_cyl + Qc_sph

    fuel_tank.insulation_wall_conductive_heat_transfer = Qc

    return np.abs(Qv + Qr - Qc)
