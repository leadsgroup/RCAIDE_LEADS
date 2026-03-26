# RCAIDE/Library/Components/Powertrain/Energy/Sources/Fuel_Tanks/compute_liquid_hydrogen_tank_conformal_volume.py
# 
# Created: Feb 2026, S. Shekar
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
from scipy.optimize import minimize_scalar

# ----------------------------------------------------------------------------------------------------------------------
#  Structural Solver
# ----------------------------------------------------------------------------------------------------------------------    
def compute_liquid_hydrogen_tank_conformal_volume(fuel_tank,fuel_tanks):
    """
    Size a liquid hydrogen tank to meet outer-diameter constraints while satisfying
    structural and thermal limits via nested 1D root solves.

    Parameters
    """  

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
        V_guess = deepcopy(fuel_tank.volume_properties.gross_volume  * 0.4)
    else:
        V_guess = deepcopy(fuel_tank.volume_properties.gross_volume * 0.85)

    # Iterative solver loop
    tol       = 1e-2
    error     = 1e2
    alpha     = 1
    iteration = 0
    max_iter  = 10000
    while abs(error) > tol and iteration < max_iter:
        # Compute internal tank geometry
        V_total = V_guess / (1 - fuel_tank.ullage_volume_fraction)  
       
       # Based on the Total volume compute the internal height
        h_i =   ((V_total/fuel_tank.aspect_ratio)*(fuel_tank.average_outer_height/fuel_tank.average_outer_width))**(1/3) #inner height in m
        w_i =   fuel_tank.average_outer_width/fuel_tank.average_outer_height *h_i
        l_i =   fuel_tank.aspect_ratio * h_i
        
        
        
        # Keep thickness search strictly 1D and bounded to avoid L-BFGS-B failures.
        th_min = 1e-6
        th_max = 0.25 * min(l_i, h_i, w_i)
        thickness_result = minimize_scalar(
            tank_width,
            bounds=(th_min, th_max),
            method="bounded",
            args=(l_i, h_i, w_i, P_internal, P_external, safety_factor, fuel_tank),
            options={"xatol": 1e-8, "maxiter": 500}
        )
        if not thickness_result.success:
            raise RuntimeWarning('Optimizer did not converge')


        th = float(thickness_result.x)
        h_o =   h_i + 2*th
        l_o =   l_i + 2*th
        w_o =   w_i + 2*th
        mass_struct = ((l_o* w_o * h_o) - (h_i*l_i*w_i))* fuel_tank.inner_structure.material.density 
        t_ins, mass_ins = thermal_solver_basic_rectangular(Ta, fuel_tank,l_o,w_o,h_o)

        h_o_o =   h_o + 2*t_ins
        l_o_o =   l_o + 2*t_ins
        w_o_o =   w_o + 2*t_ins

        V_calculated = h_o_o * l_o_o * w_o_o
        
        V_cuboid =  fuel_tank.average_outer_width *  fuel_tank.average_outer_height *  fuel_tank.average_outer_length
        error  = V_cuboid - V_calculated
        rel_error  = error / (V_cuboid)
        V_guess  += alpha * rel_error 
        iteration     += 1

    if abs(error) > tol:
        print("[Warning] compute_liquid_hydrogen_tank_volume did not converge within the iteration limit.")

    if fuel_tank.xz_plane_symmetric:
        V_guess *= 2
        V_total *= 2
        mass_struct *= 2
        mass_ins *= 2

    fuel_tank.fuel.volume_properties.net_volume    = V_guess
    fuel_tank.fuel.volume_properties.gross_volume  = V_total
    
    fuel_tank.fuel.mass_properties.mass =  fuel_tank.fuel.volume_properties.net_volume *  fuel_tank.fuel.density
    fuel_tank.mass_properties.insulation_mass =  mass_ins
    fuel_tank.mass_properties.structural_mass = mass_struct 
    fuel_tank.inner_structure.thickness       = th
    fuel_tank.inner_structure.outer_length    = l_o
    fuel_tank.inner_structure.inner_length    = l_i
    fuel_tank.inner_structure.outer_width     = w_o
    fuel_tank.inner_structure.inner_width     = w_i
    fuel_tank.inner_structure.outer_height    = h_o
    fuel_tank.inner_structure.inner_height    = h_i

    fuel_tank.outer_length  = l_o_o
    fuel_tank.outer_width   = w_o_o
    fuel_tank.outer_height  = h_o_o

    fuel_tank.insulation.thickness  = t_ins
    fuel_tank.total_thickness       = t_ins + th

    fuel_tank.mass_properties.mass = fuel_tank.tank_accesories_weight_factor*(fuel_tank.mass_properties.insulation_mass + fuel_tank.mass_properties.structural_mass)
    
    return


def tank_width(th, li,hi,wi,P_internal, P_external, safety_factor, fuel_tank ):
        th = float(np.asarray(th).reshape(-1)[0])
        a = wi/hi #width-to-height ratio
        h=li
        H=wi
        P=(P_internal - P_external)
        b=1.0
        c=th/2

        I1 = (b*th**3/12) #Iyy moment of inertia component
        I2 = (b*th**3/12) #Ixx moment of inertia component
        K = (I2/I1)*a #Vessel parameter
        Sm_eq1 = (P * h) / (2 * th)
        # Sm_eq1 = (P * h) / (2 * th)*(3-((6+K*(11-a**2))/(3+5*K)))
        Sm_eq2 = (P * H) / (2 * th)
        # Sm_eq2 = (P * H) / (2 * th)

        # The bending stress at Location N, short side plate, Equation (3):
        # SbN_eq3 = ((P * c) / (24 * I1)) * (-3 * (H)**2 + 2*(h)**2 * ((3 + 5*(a)**2 * K) / (3 + 5*K)))
        SbN_eq3 = ((P * c) / (12 * I1)) * (-1.5 * (H)**2 + (h)**2 * ((1 + (a)**2 * K) / (1 + K)))
        SbN_eq3i = -SbN_eq3  # Inside is negative
        SbN_eq3o = SbN_eq3  # Outside is positive

        # The bending stress at Location Q, short side plate, Equation (4):
        # SbQ_eq4 = ((P * h**2 * c) / (12 * I1)) * ((3 + 5*a**2 * K) / (3 + 5*K))
        SbQ_eq4 = ((P * h**2 * c) / (12 * I1)) * ((1 + a**2 * K) / (1 + K))
        SbQ_eq4i = SbQ_eq4  # Inside is positive
        SbQ_eq4o = -SbQ_eq4  # Outside is negative

        # The bending stress at Location M, long side plate, Equation (5):
        # SbM_eq5 = ((P * h**2 * c) / (12 * I2)) * ((3 + K*(6 - a**2)) / (3 + 5*K))
        SbM_eq5 = ((P * h**2 * c) / (12 * I2)) * (-1.5 + (1 + a**2 * K) / (1 + K))
        SbM_eq5i = SbM_eq5  # Inside is negative
        SbM_eq5o = abs(SbM_eq5)  # Outside is positive

        # The bending stress at Location Q, long side plate, Equation (6):
        # SbQ_eq6 = (((P * h**2 * c) / (12 * I2)) * ((3 + 5*a**2 * K) / (3 + 5*K)))
        SbQ_eq6 = (((P * h**2 * c) / (12 * I2)) * ((1 + a**2 * K) / (1 + K)))
        SbQ_eq6i = SbQ_eq6  # Inside is positive
        SbQ_eq6o = -SbQ_eq6  # Outside is negative

        # Short side plate at Location N, Membrane + Bending Stress:
        # N_totali = Sm_eq1 + SbN_eq3i
        # N_totalo = Sm_eq1 + SbN_eq3o
        N_totali = Sm_eq1
        N_totalo = Sm_eq1

        # Short side plate at Location Q, Membrane + Bending Stress:
        # Qs_totali = Sm_eq1 + SbQ_eq4i
        # Qs_totalo = Sm_eq1 + SbQ_eq4o
        Qs_totali = Sm_eq1
        Qs_totalo = Sm_eq1

        # Long side plate at Location M, Membrane + Bending Stress:
        # M_totali = Sm_eq2 + SbM_eq5i
        # M_totalo = Sm_eq2 + SbM_eq5o
        M_totali = Sm_eq2
        M_totalo = Sm_eq2

        # Long side plate at Location Q, Membrane + Bending Stress:
        # Q_totali = Sm_eq2 + SbQ_eq6i
        # Q_totalo = Sm_eq2 + SbQ_eq6o
        Q_totali = Sm_eq2
        Q_totalo = Sm_eq2
        #
        # s1s = (Pi - Po)*(h)/(2*(th))*(3 - ((6+K*(11-alfa**2))/(3 + 5*K))) #membrane stress short plate
        # s1l = (Pi - Po)*(wi)/(2*(th)) #membrane stress long plate
        # sts = (Pi - Po)*(h)/(2*th)*((6+K*(11-alfa**2))/(3 + 5*K))
        # s2sm = (Pi - Po)*(li/2)/(24*I1)*(-3*hi**2 + 2*h**2*((3+5*alfa**2*K)/(3+5*K))) #middle bending stress short plate
        # s2sc = (Pi - Po)*(h**2)*(((hi/2)**2+(li/2)**2)**(1/2))/(12*I1)*((3+5*alfa**2*K)/(3+5*K)) #corner bending stress short plate
        # s2lm = (Pi - Po)*(h**2)*(li/2)/(12*I2)*((3+K*(6-alfa**2))/(3+5*K)) #middle bending stress long plate
        # s2lc = (Pi - Po)*(h**2)*(((hi/2)**2+(li/2)**2)**(1/2))/(12*I2)*((3+5*alfa**2*K)/(3+5*K)) #corner bending stress long plate

        sv = max(N_totali, N_totalo, M_totali, M_totalo, Q_totali, Q_totalo, Qs_totali, Qs_totalo) #maximum stress
        # sv = max(N_totali, N_totalo, M_totali, M_totalo)  # maximum stress
        # print("sv", sv)
        return np.abs(sv - fuel_tank.inner_structure.material.yield_tensile_strength/safety_factor) #von Mises criteria
    
def thermal_solver_basic_rectangular(Ta, fuel_tank,lo,wo,ho):
    #Reads properties, tank material (mt), insulation material (mi), tank geometry - 
    #inner length (li), inner radius (ri), outer radius (ro), H2 temperature (Ti), 
    #ambient temperature (Ta), allowable total heat leak (Qo, W), and multipliers

    Ti = fuel_tank.design_inlet_temperature
    Qo = fuel_tank.acceptable_total_heat_leak

    # Convert total allowable heat leak [W] to an equivalent heat flux [W/m^2]
    # using the tank outer area before insulation.
    area_ref = 2 * (lo * wo + lo * ho + wo * ho)
    q_flux = Qo / max(area_ref, 1e-12)

    t_ins = fuel_tank.insulation.material.thermal_conductivity * (Ta-Ti) / max(q_flux, 1e-12)
    h_o_o =   ho + 2*t_ins
    l_o_o =   lo + 2*t_ins
    w_o_o =   wo + 2*t_ins
    a_ins = 2*(l_o_o*w_o_o + l_o_o*h_o_o + w_o_o*h_o_o)
    v_ins = (l_o_o*w_o_o*h_o_o)-(lo*wo*ho)
    mass_ins = v_ins*fuel_tank.insulation.material.density + a_ins* fuel_tank.insulation.material.specific_density
    
    return t_ins, mass_ins
