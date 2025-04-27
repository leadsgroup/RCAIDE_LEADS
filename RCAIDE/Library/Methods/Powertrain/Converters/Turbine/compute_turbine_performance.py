# RCAIDE/Library/Methods/Powertrain/Converters/Turbine/compute_turbine_performance.py
# 
# Created:  Jun 2024, M. Clarke

import numpy as  np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_turbine_performance
# ----------------------------------------------------------------------------------------------------------------------     
def compute_turbine_performance(turbine,conditions):
    """
    Computes turbine performance parameters based on input conditions and component characteristics.
    
    Parameters
    ----------
    turbine : RCAIDE.Library.Components.Powertrain.Converters.Turbine
        Turbine component with the following attributes:
            - tag : str
                Identifier for the turbine
            - working_fluid : Data
                Working fluid object with methods to compute properties
                    - compute_gamma : function
                        Computes ratio of specific heats
                    - compute_cp : function
                        Computes specific heat at constant pressure
                    - compute_R : function
                        Computes gas constant
            - mechanical_efficiency : float
                Mechanical efficiency [unitless]
            - polytropic_efficiency : float
                Polytropic efficiency [unitless]
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - energy : Data
                Energy conditions
                    - converters : dict
                        Converter energy conditions indexed by tag
                            - inputs : Data
                                Input conditions
                                    - static_temperature : array_like
                                        Static temperature [K]
                                    - static_pressure : array_like
                                        Static pressure [Pa]
                                    - mach_number : array_like
                                        Mach number [unitless]
                                    - stagnation_temperature : array_like
                                        Entering stagnation temperature [K]
                                    - stagnation_pressure : array_like
                                        Entering stagnation pressure [Pa]
                                    - bypass_ratio : array_like
                                        Bypass ratio [unitless]
                                    - fuel_to_air_ratio : array_like
                                        Fuel-to-air ratio [unitless]
                                    - compressor : Data
                                        Compressor data
                                            - work_done : array_like
                                                Compressor work [J/(kg/s)]
                                            - external_shaft_work_done : array_like
                                                Shaft power off take [J/(kg/s)]
                                    - fan : Data
                                        Fan data
                                            - work_done : array_like
                                                Fan work done [J/(kg/s)]
    
    Returns
    -------
    None
        Results are stored in conditions.energy.converters[turbine.tag].outputs:
            - stagnation_pressure : array_like
                Exiting stagnation pressure [Pa]
            - stagnation_temperature : array_like
                Exiting stagnation temperature [K]
            - stagnation_enthalpy : array_like
                Exiting stagnation enthalpy [J/kg]
            - static_temperature : array_like
                Exiting static temperature [K]
            - static_pressure : array_like
                Exiting static pressure [Pa]
            - mach_number : array_like
                Exiting Mach number [unitless]
            - gas_constant : array_like
                Gas constant [J/(kg·K)]
            - pressure_ratio : array_like
                Pressure ratio across turbine [unitless]
            - temperature_ratio : array_like
                Temperature ratio across turbine [unitless]
            - gamma : array_like
                Ratio of specific heats [unitless]
            - cp : array_like
                Specific heat at constant pressure [J/(kg·K)]
    
    Notes
    -----
    This function calculates the performance of a turbine by computing the energy
    extraction required to drive the compressor, fan, and any external power loads.
    It then determines the resulting thermodynamic properties at the turbine exit.
    
    The computation follows these steps:
        1. Extract turbine parameters and working fluid properties
        2. Compute the working fluid properties (gamma, Cp, R) at inlet conditions
        3. Calculate the energy drop across the turbine based on compressor/fan work
           and mechanical efficiency
        4. Compute the exit stagnation temperature, enthalpy, and pressure
        5. Calculate the exit static temperature and pressure
        6. Compute performance ratios (pressure ratio, temperature ratio)
        7. Store all results in the conditions data structure
    
    **Major Assumptions**
        * Constant polytropic efficiency throughout the turbine
        * Constant pressure ratio across the turbine
        * The working fluid behaves as a perfect gas
        * Mechanical losses are accounted for through a constant efficiency factor
    
    **Theory**
    The energy balance across the turbine is:
    
    .. math::
        \\Delta h_t = -\\frac{1}{1+f} \\cdot \\frac{W_{comp} + W_{ext} + \\alpha W_{fan}}{\\eta_{mech}}
    
    where:
        - :math:`\\Delta h_t` is the enthalpy drop across the turbine
        - :math:`f` is the fuel-to-air ratio
        - :math:`W_{comp}` is the compressor work
        - :math:`W_{ext}` is the external shaft work
        - :math:`W_{fan}` is the fan work
        - :math:`\\alpha` is the bypass ratio
        - :math:`\\eta_{mech}` is the mechanical efficiency
    
    The exit stagnation temperature is:
    
    .. math::
        T_{t,out} = T_{t,in} + \\frac{\\Delta h_t}{C_p}
    
    The exit stagnation pressure is calculated using the polytropic efficiency:
    
    .. math::
        P_{t,out} = P_{t,in} \\cdot \\left(\\frac{T_{t,out}}{T_{t,in}}\\right)^{\\frac{\\gamma}{(\\gamma-1)\\eta_{poly}}}
    
    References
    ----------
    [1] Mattingly, J.D., "Elements of Gas Turbine Propulsion", AIAA Education Series, 2005
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Compressor.compute_compressor_performance
    RCAIDE.Library.Methods.Powertrain.Converters.Fan.compute_fan_performance
    """              
                             
    # Unpack ram inputs       
    working_fluid      = turbine.working_fluid
    turbine_conditions = conditions.energy.converters[turbine.tag]

    # Compute the working fluid properties
    T0              = turbine_conditions.inputs.static_temperature
    P0              = turbine_conditions.inputs.static_pressure  
    M0              = turbine_conditions.inputs.mach_number

    gamma               = conditions.freestream.isentropic_expansion_factor
    Cp                  = conditions.freestream.specific_heat_at_constant_pressure
    R                   = conditions.freestream.gas_specific_constant    
    #gamma           = working_fluid.compute_gamma(T0,P0) 
    #Cp              = working_fluid.compute_cp(T0,P0) 
    #R               = working_fluid.compute_R(T0,P0)    
    
    #Unpack turbine entering properties 
    eta_mech              = turbine.mechanical_efficiency
    etapolt               = turbine.polytropic_efficiency
    alpha                 = turbine_conditions.inputs.bypass_ratio
    Tt_in                 = turbine_conditions.inputs.stagnation_temperature
    Pt_in                 = turbine_conditions.inputs.stagnation_pressure
    compressor_work       = turbine_conditions.inputs.compressor.work_done
    fan_work              = turbine_conditions.inputs.fan.work_done
    f                     = turbine_conditions.inputs.fuel_to_air_ratio  
    external_power        = turbine_conditions.inputs.compressor.external_shaft_work_done  
  
    # Using the work done by the compressors/fan and the fuel to air ratio to compute the energy drop across the turbine
    deltah_ht = -1/(1+f) * (compressor_work + external_power + alpha * fan_work) * 1/eta_mech
    
    # Compute the output stagnation quantities from the inputs and the energy drop computed above
    Tt_out    = Tt_in+deltah_ht/Cp
    ht_out    = Cp*Tt_out   
    Pt_out    = Pt_in*(Tt_out/Tt_in)**(gamma/((gamma-1)*etapolt)) 
    pi_t      = Pt_out/Pt_in
    tau_t     = Tt_out/Tt_in
    T_out     = Tt_out/(1.+(gamma-1.)/2.*M0*M0)
    P_out     = Pt_out/((1.+(gamma-1.)/2.*M0*M0)**(gamma/(gamma-1.)))   
    h_out      = T_out * Cp
    u_out     = np.sqrt(2*(ht_out-h_out))      
    
    # Pack outputs of turbine 
    turbine_conditions.outputs.stagnation_pressure     = Pt_out
    turbine_conditions.outputs.stagnation_temperature  = Tt_out
    turbine_conditions.outputs.stagnation_enthalpy     = ht_out
    turbine_conditions.outputs.static_temperature      = T_out
    turbine_conditions.outputs.static_pressure         = P_out 
    turbine_conditions.outputs.velocity                = u_out 
    turbine_conditions.outputs.mach_number             = M0 
    turbine_conditions.outputs.gas_constant            = R 
    turbine_conditions.outputs.pressure_ratio          = pi_t   
    turbine_conditions.outputs.temperature_ratio       = tau_t     
    turbine_conditions.outputs.gamma                   = gamma 
    turbine_conditions.outputs.cp                      = Cp    
    
    return