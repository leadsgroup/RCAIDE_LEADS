# RCAIDE/Methods/Energy/Propulsors/Turboprop/size_core.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop          .compute_thrust import compute_thrust 

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  size_core
# ---------------------------------------------------------------------------------------------------------------------- 
def size_core(turboprop, conditions):
    """
    Sizes the core flow for a turboprop engine at the design condition.
    
    Parameters
    ----------
    turboprop : RCAIDE.Library.Components.Propulsors.Turboprop
        Turboprop engine component with the following attributes:
            - tag : str
                Identifier for the turboprop
            - reference_temperature : float
                Reference temperature for mass flow scaling [K]
            - reference_pressure : float
                Reference pressure for mass flow scaling [Pa]
            - design_thrust : float
                Design thrust at the design point [N]
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - energy.propulsors[turboprop.tag] : Data
                Turboprop-specific conditions
                - total_temperature_reference : numpy.ndarray
                    Reference total temperature [K]
                - total_pressure_reference : numpy.ndarray
                    Reference total pressure [Pa]
                - throttle : float
                    Throttle setting [0-1]
                - thrust_specific_fuel_consumption : numpy.ndarray
                    Thrust specific fuel consumption [kg/(N·s)]
                - non_dimensional_thrust : numpy.ndarray
                    Non-dimensional thrust
    
    Returns
    -------
    None
        Results are stored in the turboprop object:
          - TSFC : float
              Thrust specific fuel consumption [kg/(N·s)]
          - design_mass_flow_rate : float
              Core mass flow rate at design point [kg/s]
          - compressor_nondimensional_massflow : float
              Non-dimensional mass flow parameter [kg·√K/(s·Pa)]
    
    Notes
    -----
    This function determines the core mass flow rate required to produce the design thrust
    at the specified design conditions. It uses the non-dimensional thrust parameter to
    scale the mass flow appropriately.
    
    **Major Assumptions**
        * Perfect gas behavior
        * Turboprop engine with conventional architecture
        * Design point is at maximum throttle (throttle = 1.0)
    
    **Theory**
    The core mass flow rate is calculated from the design thrust and non-dimensional thrust:
    
    .. math::
        \\dot{m}_{core} = \\frac{F_{design} \\cdot \\text{throttle}}{F_{sp}}
    
    The non-dimensional mass flow parameter is then calculated:
    
    .. math::
        \\dot{m}_{hc} = \\frac{\\dot{m}_{core}}{\\sqrt{\\frac{T_{ref}}{T_{t,ref}}} \cdot \\frac{P_{t,ref}}{P_{ref}}}
    
    References
    ----------
    [1] Mattingly, J.D., "Elements of Gas Turbine Propulsion", 2nd Edition, AIAA Education Series, 2005.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.compute_thrust
    """
    turboprop_conditions = conditions.energy.propulsors[turboprop.tag]
    Tt_ref               = turboprop_conditions.total_temperature_reference  
    Pt_ref               = turboprop_conditions.total_pressure_reference  
    Tref                 = turboprop.reference_temperature
    Pref                 = turboprop.reference_pressure
    
    # Compute nondimensional thrust
    turboprop_conditions.throttle = 1.0
    compute_thrust(turboprop,conditions) 

    # Store results on turboprop data structure 
    TSFC        = turboprop_conditions.thrust_specific_fuel_consumption
    Fsp         = turboprop_conditions.non_dimensional_thrust  
    mdot_core   = turboprop.design_thrust*turboprop_conditions.throttle/(Fsp) 
    mdhc        = mdot_core/ (np.sqrt(Tref/Tt_ref)*(Pt_ref/Pref))   
    
    turboprop.TSFC                                = TSFC
    turboprop.design_mass_flow_rate               = mdot_core 
    turboprop.compressor_nondimensional_massflow  = mdhc
       
    return    
