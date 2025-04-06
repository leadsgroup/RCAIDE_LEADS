# RCAIDE/Library/Methods/Powertrain/Converters/Combustor/compute_combustor_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke


# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports 
# ----------------------------------------------------------------------------------------------------------------------   
import  numpy as  np 

# ---------------------------------------------------------------------------------------------------------------------- 
# compute_combustor_performance
# ----------------------------------------------------------------------------------------------------------------------    
def compute_combustor_performance(combustor, conditions):
    """
    Computes the thermodynamic performance of a combustor in a gas turbine engine.
    
    Parameters
    ----------
    combustor : RCAIDE.Library.Components.Converters.Combustor
        Combustor component with the following attributes:
            - tag : str
                Identifier for the combustor
            - working_fluid : Data
                Working fluid properties object
            - turbine_inlet_temperature : float
                Target turbine inlet temperature [K]
            - pressure_ratio : float
                Pressure ratio across the combustor (typically < 1.0 due to losses)
            - efficiency : float
                Combustion efficiency
            - area_ratio : float
                Exit to inlet area ratio
            - fuel_data : Data
                Fuel properties
                - specific_energy : float
                    Fuel specific energy [J/kg]
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - energy : Data
                Energy conditions
                - converters : dict
                    Converter energy conditions indexed by tag
    
    Returns
    -------
    None
        Results are stored in conditions.energy.converters[combustor.tag].outputs:
            - stagnation_temperature : numpy.ndarray
                Stagnation temperature at combustor exit [K]
            - stagnation_pressure : numpy.ndarray
                Stagnation pressure at combustor exit [Pa]
            - stagnation_enthalpy : numpy.ndarray
                Stagnation enthalpy at combustor exit [J/kg]
            - fuel_to_air_ratio : numpy.ndarray
                Fuel-to-air ratio
            - static_temperature : numpy.ndarray
                Static temperature at combustor exit [K]
            - static_pressure : numpy.ndarray
                Static pressure at combustor exit [Pa]
            - mach_number : numpy.ndarray
                Mach number at combustor exit
    
    Notes
    -----
    This function computes the thermodynamic properties at the combustor exit based on
    the inlet conditions, combustor characteristics, and fuel properties. It calculates
    the fuel-to-air ratio required to achieve the specified turbine inlet temperature,
    accounting for combustion efficiency and pressure losses.
    
    The computation follows these steps:
        1. Extract inlet conditions (temperature, pressure, Mach number)
        2. Compute working fluid properties (gamma, Cp)
        3. Calculate stagnation pressure at exit using pressure ratio
        4. Set exit stagnation temperature to the specified turbine inlet temperature
        5. Compute stagnation enthalpies at inlet and exit
        6. Calculate fuel-to-air ratio required to achieve the temperature rise
        7. Compute exit static conditions (temperature, pressure) based on exit Mach number
        8. Store all results in the conditions data structure
    
    **Major Assumptions**
        * Constant efficiency and pressure ratio
        * Turbine inlet temperature is controlled to a specified value
        * Mach number is preserved from inlet to exit
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University https://web.stanford.edu/~cantwell/AA283_Course_Material/
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Turbine.compute_turbine_performance
    """
    combustor_conditions    = conditions.energy.converters[combustor.tag]
    T0                      = combustor_conditions.inputs.static_temperature
    P0                      = combustor_conditions.inputs.static_pressure  
    M0                      = combustor_conditions.inputs.mach_number 
                                 
    # Unpack ram inputs         
    working_fluid           = combustor.working_fluid
 
    # Compute the working fluid properties 
    gamma  = working_fluid.compute_gamma(T0,P0) 
    Cp     = working_fluid.compute_cp(T0,P0) 
    
    # unpacking the values form inputs
    Tt_in    = combustor_conditions.inputs.stagnation_temperature
    Pt_in    = combustor_conditions.inputs.stagnation_pressure
    nondim_r = combustor_conditions.inputs.nondim_mass_ratio 
    Tt4      = combustor.turbine_inlet_temperature *  np.ones_like(Tt_in)
    pib      = combustor.pressure_ratio
    eta_b    = combustor.efficiency
    htf      = combustor.fuel_data.specific_energy 
    
    # compute stanation pressure 
    Pt_out  = pib * Pt_in 
    
    #Computing stagnation enthalpies from stagnation temperatures
    ht4     = nondim_r * Cp* Tt4 
    ht_in   = nondim_r * Cp* Tt_in
    
    # Compute the fuel to air ratio using turbine exit temperature, the fuel properties and freestream temperature
    f       = (ht4 - ht_in)/(eta_b*htf-ht4)

    # Computing the exit static and stagnation conditions
    ht_out  = Tt4 * Cp
    
    T_out     = Tt4/(1.+(gamma-1.)/2.*M0*M0)
    P_out     = Pt_out/((1.+(gamma-1.)/2.*M0*M0)**(gamma/(gamma-1.)))     
    
    # Pack results 
    combustor_conditions.outputs.stagnation_temperature  = Tt4
    combustor_conditions.outputs.stagnation_pressure     = Pt_out
    combustor_conditions.outputs.stagnation_enthalpy     = ht_out
    combustor_conditions.outputs.fuel_to_air_ratio       = f
    combustor_conditions.outputs.static_temperature      = T_out
    combustor_conditions.outputs.static_pressure         = P_out 
    combustor_conditions.outputs.mach_number             = M0 
    
    return 
