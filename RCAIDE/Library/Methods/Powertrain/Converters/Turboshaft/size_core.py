# RCAIDE/Methods/Energy/Converters/Turboshaft/size_core.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Jun 2024, M. Guidotti  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Library.Methods.Powertrain.Converters.Turboshaft import compute_power

# Python package imports
import numpy                                                       as np

# ----------------------------------------------------------------------------------------------------------------------
#  size_core
# ----------------------------------------------------------------------------------------------------------------------
def size_core(turboshaft, conditions):
    """
    Sizes the core flow for a turboshaft engine at the design condition.
    
    Parameters
    ----------
    turboshaft : RCAIDE.Library.Components.Converters.Turboshaft
        Turboshaft engine component with the following attributes:
            - tag : str
                Identifier for the turboshaft
            - reference_temperature : float
                Reference temperature [K]
            - reference_pressure : float
                Reference pressure [Pa]
            - design_power : float
                Design power output [W]
            - compressor : Data
                Compressor component
                - mass_flow_rate : float
                    Mass flow rate through the compressor [kg/s]
            - mass_flow_rate : float
                Mass flow rate through the turboshaft [kg/s]
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - energy : Data
                Energy conditions
                - converters : dict
                    Converter energy conditions indexed by tag
                    - total_temperature_reference : float
                        Reference total temperature [K]
                    - total_pressure_reference : float
                        Reference total pressure [Pa]
                    - non_dimensional_power : float
                        Non-dimensional power
    
    Returns
    -------
    None
        Results are stored in the turboshaft object:
            - mass_flow_rate : float
                Mass flow rate through the turboshaft [kg/s]
            - compressor.mass_flow_rate : float
                Mass flow rate through the compressor [kg/s]
    
    Notes
    -----
    This function sizes the core flow of a turboshaft engine based on the design power
    requirement and the non-dimensional power computed from the engine cycle analysis.
    It calculates the dimensional mass flow rate needed to produce the specified design
    power and corrects it to the reference conditions.
    
    The computation follows these steps:
        1. Extract reference conditions and engine parameters
        2. Compute non-dimensional power using the compute_power function
        3. Calculate the dimensional mass flow rate required to produce the design power
        4. Correct the mass flow rate to the reference conditions
        5. Store the results in the turboshaft object
    
    **Major Assumptions**
        * Perfect gas behavior
        * Turboshaft engine with free power turbine
    
    **Theory**
    The dimensional mass flow rate is calculated as:
    
    .. math::
        \\dot{m}_{air} = \\frac{P_{design}}{P_{sp}}
    
    The corrected mass flow rate for the compressor is:
    
    .. math::
        \\dot{m}_{compressor} = \\frac{\\dot{m}_{air}}{\\sqrt{\\frac{T_{ref}}{T_{t,ref}}}\\frac{P_{t,ref}}{P_{ref}}}
    
    where:
      - P_{design} is the design power
      - P_{sp} is the non-dimensional power
      - T_{ref} is the reference temperature
      - T_{t,ref} is the reference total temperature
      - P_{ref} is the reference pressure
      - P_{t,ref} is the reference total pressure
      
    References
    ----------
    [1] Mattingly, J.D., "Elements of Gas Turbine Propulsion", AIAA Education Series, 2005, pp. 332-336
    [2] Stuyvenberg, L., "Helicopter Turboshafts", University of Colorado, 2015 https://www.colorado.edu/faculty/kantha/sites/default/files/attached-files/70652-116619_-_luke_stuyvenberg_-_dec_17_2015_1258_pm_-_stuyvenberg_helicopterturboshafts.pdf
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Turboshaft.compute_power
    """
    
    #unpack from turboshaft
    turboshaft_conditions                          = conditions.energy.converters[turboshaft.tag] 
    Tref                                           = turboshaft.reference_temperature
    Pref                                           = turboshaft.reference_pressure 
    total_temperature_reference                    = turboshaft_conditions.total_temperature_reference  
    total_pressure_reference                       = turboshaft_conditions.total_pressure_reference 

    #compute nondimensional power
    compute_power(turboshaft,conditions)

    #unpack results 
    Psp                                            = turboshaft_conditions.non_dimensional_power
    
    #compute dimensional mass flow rates
    mdot_air                                       = turboshaft.design_power/Psp
    mdot_compressor                                = mdot_air/ (np.sqrt(Tref/total_temperature_reference)*(total_pressure_reference/Pref))

    #pack outputs
    turboshaft.mass_flow_rate                      = mdot_air
    turboshaft.compressor.mass_flow_rate           = mdot_compressor

    return    
