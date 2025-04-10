# RCAIDE/Library/Methods/Powertrain/Converters/Compressor/compute_compressor_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
# Imports 
# ---------------------------------------------------------------------------------------------------------------------- 
import numpy as np
from copy import  deepcopy

# ---------------------------------------------------------------------------------------------------------------------- 
# compute_compression_nozzle_performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_compressor_performance(compressor, conditions):
    """
    Computes the performance of a compressor based on its polytropic efficiency.

    Parameters
    ----------
    compressor : RCAIDE.Library.Components.Converters.Compressor
        Compressor component with the following attributes:
            - tag : str
                Identifier for the compressor
            - pressure_ratio : float
                Pressure ratio across compressor
            - polytropic_efficiency : float
                Polytropic efficiency of compression
            - working_fluid : Data
                Working fluid properties object
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - energy.converters[compressor.tag].inputs : Data
                Input conditions
                    - stagnation_temperature : numpy.ndarray
                        Inlet stagnation temperature [K]
                    - stagnation_pressure : numpy.ndarray
                        Inlet stagnation pressure [Pa]
                    - static_temperature : numpy.ndarray
                        Inlet static temperature [K]
                    - static_pressure : numpy.ndarray
                        Inlet static pressure [Pa]
                    - mach_number : numpy.ndarray
                        Inlet Mach number
            - energy.hybrid_power_split_ratio : float
                Ratio of power split for hybrid systems

    Returns
    -------
    None
        Results are stored in conditions.energy.converters[compressor.tag].outputs:
            - work_done : numpy.ndarray
                Specific work done by compressor [J/kg]
            - stagnation_temperature : numpy.ndarray
                Exit stagnation temperature [K]
            - stagnation_pressure : numpy.ndarray
                Exit stagnation pressure [Pa]
            - stagnation_enthalpy : numpy.ndarray
                Exit stagnation enthalpy [J/kg]
            - static_temperature : numpy.ndarray
                Exit static temperature [K]
            - static_pressure : numpy.ndarray
                Exit static pressure [Pa]
            - mach_number : numpy.ndarray
                Exit Mach number
            - gas_constant : numpy.ndarray
                Gas constant [J/(kg·K)]
            - gamma : numpy.ndarray
                Ratio of specific heats
            - cp : numpy.ndarray
                Specific heat at constant pressure [J/(kg·K)]

    Notes
    -----
    This function implements the thermodynamic calculations for a compressor with
    a specified pressure ratio and polytropic efficiency. The work done is adjusted
    by the hybrid power split ratio if applicable.
    
    **Major Assumptions**
        * Constant polytropic efficiency
        * Constant pressure ratio
        * Ideal gas behavior
        * Adiabatic process

    **Theory**
    The compression process follows the polytropic relation:

    .. math::
        T_{t,out}/T_{t,in} = (P_{t,out}/P_{t,in})^{(\\gamma-1)/(\\gamma \\eta_{p})}

    where :math:`\\eta_{p}` is the polytropic efficiency.

    Enthalpy is calculated using the specific heat at constant pressure and the stagnation temperature.

    .. math::
        h_{t} = C_{p} T_{t}

    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_BOOK/AA283_Aircraft_and_Rocket_Propulsion_BOOK_Brian_J_Cantwell_May_28_2024.pdf

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Compressor.append_compressor_conditions
    """          
    
    # Unpack component inputs
    PR                    = compressor.pressure_ratio
    etapold               = compressor.polytropic_efficiency
    working_fluid         = compressor.working_fluid
    compressor_conditions = conditions.energy.converters[compressor.tag]
    Tt_in                 = compressor_conditions.inputs.stagnation_temperature
    Pt_in                 = compressor_conditions.inputs.stagnation_pressure 
    T0                    = compressor_conditions.inputs.static_temperature
    P0                    = compressor_conditions.inputs.static_pressure  
    M0                    = compressor_conditions.inputs.mach_number 
     
    # Compute the working fluid properties 
    gamma  = working_fluid.compute_gamma(T0,P0) 
    Cp     = working_fluid.compute_cp(T0,P0)    
    R      = working_fluid.compute_R(T0,P0)
        
    # Compute the output properties based on the pressure ratio of the component
    ht_in     = Tt_in*Cp 
    Pt_out    = Pt_in*PR
    Tt_out    = Tt_in*(PR**((gamma-1)/(gamma*etapold)))
    ht_out    = Tt_out*Cp
    T_out     = Tt_out/(1.+(gamma-1.)/2.*M0*M0)
    P_out     = Pt_out/((1.+(gamma-1.)/2.*M0*M0)**(gamma/(gamma-1.))) 
    M_out     = np.sqrt( (((Pt_out/P_out)**((gamma-1.)/gamma))-1.) *2./(gamma-1.) )
    
    # Compute the work done by the compressor (normalized by mass flow i.e. J/(kg/s)
    work_done = ht_out - ht_in
    
    phi       =  conditions.energy.hybrid_power_split_ratio 

    # Pack results  
    compressor_conditions.outputs.work_done               = (1-phi)*work_done
    compressor_conditions.outputs.stagnation_temperature  = Tt_out
    compressor_conditions.outputs.stagnation_pressure     = Pt_out
    compressor_conditions.outputs.stagnation_enthalpy     = ht_out
    compressor_conditions.outputs.static_temperature      = T_out
    compressor_conditions.outputs.static_pressure         = P_out 
    compressor_conditions.outputs.mach_number             = M_out
    compressor_conditions.outputs.gas_constant            = R
    compressor_conditions.outputs.gamma                   = gamma 
    compressor_conditions.outputs.cp                      = Cp
    
    return 

