# RCAIDE/Library/Methods/Powertrain/Converters/Turbine/compute_turbine_performance.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke    

# ----------------------------------------------------------------------------------------------------------------------
#  compute_turbine_performance
# ----------------------------------------------------------------------------------------------------------------------     
def compute_turbine_performance(turbine, turbine_conditions, conditions):
    """
    Computes the thermodynamic properties at the exit of a turbine based on work extraction.
    
    Parameters
    ----------
    turbine : Turbine
        The turbine component with the following attributes:
            - mechanical_efficiency : float
                Mechanical efficiency of the turbine [unitless]
            - polytropic_efficiency : float
                Polytropic efficiency of the expansion process [unitless]
            - working_fluid : FluidModel
                Object containing methods to compute fluid properties
    turbine_conditions : Conditions
        Object containing turbine inlet conditions with the following attributes:
            - inputs.stagnation_temperature : numpy.ndarray
                Inlet stagnation temperature [K]
            - inputs.stagnation_pressure : numpy.ndarray
                Inlet stagnation pressure [Pa]
            - inputs.static_temperature : numpy.ndarray
                Inlet static temperature [K]
            - inputs.static_pressure : numpy.ndarray
                Inlet static pressure [Pa]
            - inputs.mach_number : numpy.ndarray
                Inlet Mach number [unitless]
            - inputs.bypass_ratio : numpy.ndarray
                Bypass ratio [unitless]
            - inputs.fuel_to_air_ratio : numpy.ndarray
                Fuel-to-air ratio [unitless]
            - inputs.compressor.work_done : numpy.ndarray
                Compressor work [J/(kg/s)]
            - inputs.fan.work_done : numpy.ndarray
                Fan work done [J/(kg/s)]
            - inputs.shaft_power_off_take.work_done : numpy.ndarray
                Shaft power off take [J/(kg/s)]
    conditions : Conditions
        Object containing freestream conditions (not directly used in this function)
    
    Returns
    -------
    None
        This function modifies the turbine_conditions.outputs object in-place with the following attributes:
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
                Exit Mach number [unitless]
            - gas_constant : numpy.ndarray
                Gas constant [J/(kg·K)]
            - pressure_ratio : numpy.ndarray
                Ratio of exit to inlet pressure [unitless]
            - temperature_ratio : numpy.ndarray
                Ratio of exit to inlet temperature [unitless]
            - gamma : numpy.ndarray
                Ratio of specific heats [unitless]
            - cp : numpy.ndarray
                Specific heat at constant pressure [J/(kg·K)]
    
    Notes
    -----
    This function computes the thermodynamic properties at the exit of a turbine
    based on the work extracted to drive compressors, fans, and other components. It uses 
    equations from the AA283 Course Notes [1] to compute the exit conditions.
    
    **Major Assumptions**
        * Constant polytropic efficiency and pressure ratio
        * Adiabatic process
    
    **Theory**
    
    The energy drop across the turbine is calculated using:
    
    .. math::
        \\Delta h_t = -\\frac{1}{1+f} \\cdot (W_{comp} + W_{shaft} + \\alpha \\cdot W_{fan}) \\cdot \\frac{1}{\\eta_{mech}}
    
    The exit temperature is calculated using:
    
    .. math::
        T_{t,out} = T_{t,in} + \\frac{\\Delta h_t}{C_p}
    
    The exit pressure is calculated using:
    
    .. math::
        P_{t,out} = P_{t,in} \\cdot \\left(\\frac{T_{t,out}}{T_{t,in}}\\right)^{\\frac{\\gamma}{\\gamma-1}} \\cdot \\frac{1}{\\eta_{poly}}
    
    References
    ----------
    [1] Cantwell, B., "AA283 Course Notes", Stanford University https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_BOOK/AA283_Aircraft_and_Rocket_Propulsion_BOOK_Brian_J_Cantwell_May_28_2024.pdf
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Turbine.append_turbine_conditions
    """
                             
    # Unpack ram inputs       
    working_fluid   = turbine.working_fluid

    # Compute the working fluid properties
    T0              = turbine_conditions.inputs.static_temperature
    P0              = turbine_conditions.inputs.static_pressure  
    M0              = turbine_conditions.inputs.mach_number   
    gamma           = working_fluid.compute_gamma(T0,P0) 
    Cp              = working_fluid.compute_cp(T0,P0) 
    R               = working_fluid.compute_R(T0,P0)    
    
    #Unpack turbine entering properties 
    eta_mech        = turbine.mechanical_efficiency
    etapolt         = turbine.polytropic_efficiency
    alpha           = turbine_conditions.inputs.bypass_ratio
    Tt_in           = turbine_conditions.inputs.stagnation_temperature
    Pt_in           = turbine_conditions.inputs.stagnation_pressure
    compressor_work = turbine_conditions.inputs.compressor.work_done
    fan_work        = turbine_conditions.inputs.fan.work_done
    f               = turbine_conditions.inputs.fuel_to_air_ratio 
    shaft_takeoff   = turbine_conditions.inputs.shaft_power_off_take.work_done 
  
    # Using the work done by the compressors/fan and the fuel to air ratio to compute the energy drop across the turbine
    deltah_ht = -1/(1+f) * (compressor_work + shaft_takeoff + alpha * fan_work) * 1/eta_mech
    
    # Compute the output stagnation quantities from the inputs and the energy drop computed above
    Tt_out    = Tt_in+deltah_ht/Cp
    ht_out    = Cp*Tt_out   
    Pt_out    = Pt_in*(Tt_out/Tt_in)**(gamma/((gamma-1)*etapolt)) 
    pi_t      = Pt_out/Pt_in
    tau_t     = Tt_out/Tt_in
    T_out     = Tt_out/(1.+(gamma-1.)/2.*M0*M0)
    P_out     = Pt_out/((1.+(gamma-1.)/2.*M0*M0)**(gamma/(gamma-1.)))         
    
    # Pack outputs of turbine 
    turbine_conditions.outputs.stagnation_pressure     = Pt_out
    turbine_conditions.outputs.stagnation_temperature  = Tt_out
    turbine_conditions.outputs.stagnation_enthalpy     = ht_out
    turbine_conditions.outputs.static_temperature      = T_out
    turbine_conditions.outputs.static_pressure         = P_out 
    turbine_conditions.outputs.mach_number             = M0 
    turbine_conditions.outputs.gas_constant            = R 
    turbine_conditions.outputs.pressure_ratio          = pi_t   
    turbine_conditions.outputs.temperature_ratio       = tau_t     
    turbine_conditions.outputs.gamma                   = gamma 
    turbine_conditions.outputs.cp                      = Cp    
    
    return