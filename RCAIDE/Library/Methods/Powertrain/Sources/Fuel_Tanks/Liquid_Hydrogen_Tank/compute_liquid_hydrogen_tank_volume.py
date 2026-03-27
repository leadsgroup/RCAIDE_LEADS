# RCAIDE/Library/Components/Powertrain/Energy/Sources/Fuel_Tanks/compute_liquid_hydrogen_tank_volume.py
# 
# Created: Dec 2025, S. Shekar
#
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import Units, Data

# Python imports
from copy import deepcopy
import numpy as np
from scipy.optimize import minimize, minimize_scalar, brentq

# ----------------------------------------------------------------------------------------------------------------------
#  Structural Solver
# ----------------------------------------------------------------------------------------------------------------------    
def compute_liquid_hydrogen_tank_volume(fuel_tank,fuel_tanks):
    """
    Size a liquid hydrogen tank to meet outer-diameter constraints while satisfying
    structural and thermal limits via nested 1D root solves.

    Parameters
    ----------
    fuel_tank : Fuel_Tank
        Fuel tank object containing geometry, material, fuel, and environment data.

    Returns
    -------
    None
        Updates the fuel_tank object in place, including:
            - inner_structure.thickness : float
                Outer/inner radius ratio of the pressure shell.
            - inner_structure.outer_diameter : float
                Outer diameter of the inner vessel [m].
            - inner_structure.inner_diameter : float
                Inner diameter of the inner vessel [m].
            - inner_structure.inner_length : float
                Cylindrical inner length [m].
            - inner_structure.outer_length : float
                Cylindrical outer length [m].
            - inner_structure.material_volume : float
                Pressure shell material volume [m³].
            - insulation_thickness : float
                Required insulation thickness [m].
            - fuel.volume_properties.{gross_volume, net_volume} : float
                Sized fuel volumes [m³] (scaled if symmetric).
            - fuel.mass_properties.mass : float
                Fuel mass [kg].
            - mass_properties.structural_mass : float
                Tank structural mass [kg].

    Notes
    -----
    * Uses von Mises stress on a thick-walled cylinder with hemispherical caps.
    * Thermal sizing balances convection/radiation with conduction through insulation.
    * Outer-diameter constraint is enforced by iterating on fuel volume until geometry closes.
    * Symmetry doubles volume and material where specified.
    """
    
    fuel_tank.wall_thickness = None
    fuel_tank.volume_properties.net_volume = None

    # Constants
    safety_factor   = 1.6          # structural factor of safety
    pressure_factor = 5.0          # internal pressure multiplier for sizing
    T_inlet         = fuel_tank.design_inlet_temperature

    # Saturation and design pressures
    P_sat = fuel_tank.fuel.liquid_hydrogen_properties(T_inlet, "Pressure (MPa)") * Units.MPa
    P_internal = pressure_factor * P_sat
    P_external = fuel_tank.design_external_pressure

    PI_Q = 1.5  # heat flow multiplier for thermal sizing

    # Get atmospheric conditions at design altitude
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(fuel_tank.design_altitude,
                                           fuel_tank.design_isa_deviation)
    Ta = float(np.asarray(atmo_data.temperature).reshape(-1)[0])

    
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
        try:
            ro_ri = brentq(
                tank_width,
                1 + 1e-5,
                1.1,
                xtol=1e-6,
                args=(P_internal, P_external, safety_factor, fuel_tank)
            )
        except ValueError:
            ro_ri = minimize(
                tank_width,
                x0=(1 + 1e-2),
                bounds=[(1+1e-5,1.1)],
                method='L-BFGS-B',
                tol=1e-5,
                args=(P_internal, P_external, safety_factor, fuel_tank)
            ).x[0]
    
        r_outer = ro_ri * r_inner # This is the outer diameter of the inner vessel


        bracket = bracket_root(
            insulation_width,
            start=1e-6,
            factor=5,
            limit=1e2,
            args=(Ta, PI_Q, fuel_tank, atmo_data, r_outer, r_inner, L_inner)
        )
        if bracket:
            try:
                t_ins = brentq(
                    insulation_width,
                    *bracket,
                    xtol=1e-9,
                    args=(Ta, PI_Q, fuel_tank, atmo_data, r_outer, r_inner, L_inner)
                )
            except ValueError:
                t_ins = minimize(
                    insulation_width,
                    x0=0.01,
                    bounds=[(1e-8, 1e8)],
                    method='L-BFGS-B',
                    tol=1e-10,
                    args=(Ta, PI_Q, fuel_tank, atmo_data,r_outer,r_inner,L_inner)
                ).x[0]
        else:
            t_ins = minimize(
                insulation_width,
                x0=0.01,
                bounds=[(1e-8, 1e8)],
                method='L-BFGS-B',
                tol=1e-10,
                args=(Ta, PI_Q, fuel_tank, atmo_data,r_outer,r_inner,L_inner)
            ).x[0]

        # Convergence check
        error                                          = fuel_tank.diameters.external / 2 - (r_outer+t_ins)
        rel_error                                      = error / (fuel_tank.diameters.external / 2)
        fuel_tank.fuel.volume_properties.net_volume    = V_guess
        fuel_tank.fuel.volume_properties.gross_volume  = V_total
        fuel_tank.fuel.mass_properties.mass            = float(V_guess *  fuel_tank.fuel.density)  
        V_guess                                       += alpha * rel_error
        iteration                                     += 1

    if abs(error) > tol:
        print("[Warning] compute_liquid_hydrogen_tank_volume did not converge within the iteration limit.")

    # Store results
    fuel_tank.inner_structure                = Data()
    fuel_tank.inner_structure.thickness      = r_outer -r_inner
    fuel_tank.inner_structure.outer_diameter = 2*r_outer
    fuel_tank.inner_structure.inner_diameter = 2*r_inner
    fuel_tank.inner_structure.inner_length   = L_inner
    fuel_tank.inner_structure.outer_length   =  (2 * r_outer * fuel_tank.aspect_ratio)-2*r_outer
    fuel_tank.insulation_thickness           = t_ins 

    # Insulation geometry and mass
    a_ins = 2 * np.pi * fuel_tank.diameters.external/2 * (fuel_tank.lengths.external) + 4 * np.pi * (fuel_tank.diameters.external/2)**2
    v_ins = (np.pi * (fuel_tank.diameters.external/2)**2 * (fuel_tank.lengths.external) + (4/3) * np.pi * (fuel_tank.diameters.external/2)**3)-\
            (np.pi * (fuel_tank.inner_structure.outer_diameter/2)**2 * (fuel_tank.inner_structure.outer_length) + (4/3) * np.pi * (fuel_tank.inner_structure.outer_diameter/2)**3)
         
    mass_ins = (v_ins * fuel_tank.insulation_material.density
               + a_ins * fuel_tank.insulation_material.specific_density)

    # Material volume between inner and outer shells (cylinder + two hemispherical caps)
    L_outer = fuel_tank.inner_structure.outer_length
    V_outer = np.pi * r_outer**2 * L_outer + (4.0/3.0) * np.pi * r_outer**3
    V_inner = np.pi * r_inner**2 * L_inner + (4.0/3.0) * np.pi * r_inner**3
    V_material = V_outer - V_inner

    if fuel_tank.xz_plane_symmetric:
        fuel_tank.fuel.volume_properties.gross_volume*= 2
        fuel_tank.fuel.volume_properties.net_volume *= 2
        V_material *= 2
        mass_ins *=2
    
    if np.isnan(mass_ins):
        print(f"[WARNING] Tank '{fuel_tank.tag}' is too small and has negative fuel volume. Removing from list.")
        fuel_tanks.pop(fuel_tank.tag)
    fuel_tank.fuel.mass_properties.mass =  fuel_tank.fuel.volume_properties.net_volume *  fuel_tank.fuel.density
    fuel_tank.mass_properties.insulation_mass =  mass_ins
    fuel_tank.mass_properties.structural_mass = V_material * fuel_tank.material.density  # Structural Mass of the tank
    fuel_tank.mass_properties.mass = fuel_tank.tank_accesories_weight_factor*(fuel_tank.mass_properties.insulation_mass + fuel_tank.mass_properties.structural_mass)
    
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
        Signed difference between actual von Mises stress and 
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

    return sigma_vm - fuel_tank.material.yield_tensile_strength / safety_factor


def insulation_width(t_ins, Ta, PI_Q, fuel_tank, atmo_data,r_o,r_i,l_i):   
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
    Ti = fuel_tank.design_inlet_temperature
    Qo = fuel_tank.acceptable_heat_leak

    # Estimate equilibrium wall temperature.
    Ta = float(np.asarray(Ta).reshape(-1)[0])
    Te_lo = min(Ti, Ta)
    Te_hi = max(Ti, Ta)
    args = (t_ins, fuel_tank, atmo_data, r_o, r_i, l_i)

    if abs(Te_hi - Te_lo) < 1e-9:
        Te = Te_lo
        heat_transfer_wrap(Te, *args)
    else:
        try:
            Te = brentq(heat_transfer_wrap, Te_lo, Te_hi, xtol=1e-9, args=args)
        except ValueError:
            # Fallback when brentq does not find a sign change in [Te_lo, Te_hi].
            opt = minimize_scalar(
                lambda t: abs(heat_transfer_wrap(t, *args)),
                bounds=(Te_lo, Te_hi),
                method="bounded"
            )
            Te = float(opt.x)
            heat_transfer_wrap(Te, *args)

    Qc_mat = fuel_tank.insulation_wall_conductive_heat_transfer

    return PI_Q * Qc_mat / (2*np.pi*r_i*(l_i) + 4*np.pi*r_i**2) - Qo


def heat_transfer_wrap(Te, t_ins, fuel_tank, atmo_data,ro,ri,li):
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
        Net heat flow residual (external - internal conduction).
    """
    # Atmospheric properties
    p        = float(np.asarray(atmo_data.pressure).reshape(-1)[0])
    rho_air  = float(np.asarray(atmo_data.density).reshape(-1)[0])
    mu_air   = float(np.asarray(atmo_data.dynamic_viscosity).reshape(-1)[0])
    k_air    = float(np.asarray(atmo_data.thermal_conductivity).reshape(-1)[0])
    Ta       = float(np.asarray(atmo_data.temperature).reshape(-1)[0])
    g        = 9.81
    Ti       = fuel_tank.design_inlet_temperature

    # Air properties
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    Cp_air     = float(np.asarray(atmosphere.fluid_properties.compute_cp(Ta)).reshape(-1)[0])

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
    return float(np.asarray(Qv + Qr - Qc).reshape(-1)[0])


def bracket_root(func, start=1e-6, factor=10, limit=1e2, args=()):
    """
    Expand a bracket until a sign change is found or a limit is reached.
    """
    a = start
    fa = func(a, *args)
    b = a * factor
    fb = func(b, *args)
    while np.sign(fa) == np.sign(fb) and b < limit:
        a, fa = b, fb
        b *= factor
        fb = func(b, *args)
    if np.sign(fa) == np.sign(fb):
        return None
    return a, b
