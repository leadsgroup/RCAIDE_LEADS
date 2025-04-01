# RCAIDE/Methods/Energy/Propulsors/Turbojet_Propulsor/size_core.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet_Propulsor import compute_thrust

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  size_core
# ---------------------------------------------------------------------------------------------------------------------- 
def size_core(turbojet,turbojet_conditions,conditions):
    """
    Designs a turbojet engine by computing performance properties and sizing components based on design conditions.
    
    Parameters
    ----------
    turbojet : Turbojet
        Turbojet engine object containing design parameters and components
            - design_mach_number : float
                Design point Mach number [-]
            - design_altitude : float
                Design point altitude [m]
            - design_isa_deviation : float
                ISA temperature deviation [K]
            - working_fluid : Gas
                Working fluid object for gas properties
            - Components:
                - ram : Ram
                - inlet_nozzle : Compression_Nozzle
                - low_pressure_compressor : Compressor
                - high_pressure_compressor : Compressor
                - combustor : Combustor
                - high_pressure_turbine : Turbine
                - low_pressure_turbine : Turbine
                - core_nozzle : Supersonic_Nozzle
    
    Returns
    -------
    None
        Updates turbojet object attributes in-place:
            - mass_flow_rate_design : float
                Design core mass flow rate [kg/s]
            - design_core_massflow : float
                Core mass flow at design point [kg/s]
    
    Notes
    -----
    This function performs the following steps:
      1. Computes atmospheric conditions at design point
      2. Sets up freestream conditions
      3. Links and analyzes flow through each component:
          - Ram inlet
          - Inlet nozzle
          - Low pressure compressor
          - High pressure compressor
          - Combustor
          - High pressure turbine
          - Low pressure turbine
          - Core nozzle
      4. Sizes the core based on design thrust requirements
      5. Computes static sea level performance
    
    **Major Assumptions**
        * Quasi-one-dimensional flow
        * Each component operates in steady state
        * Perfect gas behavior in non-combustion sections
        * US Standard Atmosphere 1976 model
        * Earth gravity model
        * Design point defines core sizing
    
    **Theory**
    The design process follows standard gas turbine design principles:
    
    .. math::
        \\text{Mass flow continuity: } \\dot{m}_{in} = \\dot{m}_{out}
    
        \\text{Power Balance: } W_{compressor} = W_{turbine}
    
        \\text{Core sizing: } \\dot{m}_{core} = \\frac{F_{design}}{F_{sp} a_0}
    
    where:
        - :math:`F_{design}` is the design thrust
        - :math:`F_{sp}` is the specific thrust
        - :math:`a_0` is the freestream speed of sound
    
    **Extra modules required**
        * numpy
    
    References
    ----------
    [1] Mattingly, J. D., "Elements of Gas Turbine Propulsion", McGraw-Hill, 1996
    [2] Walsh, P. P., Fletcher, P., "Gas Turbine Performance", Blackwell Science, 2004
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet_Propulsor.compute_turbojet_performance
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet_Propulsor.size_core
    RCAIDE.Library.Methods.Powertrain.Propulsors.Common.compute_static_sea_level_performance
    """             
    #unpack inputs
    a0                   = conditions.freestream.speed_of_sound
    throttle             = 1.0

    #unpack from turbojet 
    Tref                        = turbojet.reference_temperature
    Pref                        = turbojet.reference_pressure 

    total_temperature_reference = turbojet_conditions.total_temperature_reference  
    total_pressure_reference    = turbojet_conditions.total_pressure_reference 

    #compute nondimensional thrust
    turbojet_conditions.throttle = 1.0
    compute_thrust(turbojet,turbojet_conditions,conditions)

    #unpack results 
    Fsp                         = turbojet_conditions.non_dimensional_thrust

    #compute dimensional mass flow rates
    mdot_core                   = turbojet.design_thrust/(Fsp*a0*throttle)  
    mdhc                        = mdot_core/ (np.sqrt(Tref/total_temperature_reference)*(total_pressure_reference/Pref))

    #pack outputs
    turbojet.mass_flow_rate_design               = mdot_core
    turbojet.compressor_nondimensional_massflow  = mdhc

    return    
