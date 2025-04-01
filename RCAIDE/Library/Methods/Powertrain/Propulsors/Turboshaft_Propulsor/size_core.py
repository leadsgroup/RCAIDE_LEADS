# RCAIDE/Methods/Energy/Propulsors/Turboshaft_Propulsor/size_core.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Jun 2024, M. Guidotti & D.J. Lee

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboshaft_Propulsor import compute_power

# Python package imports
import numpy                                                       as np

# ----------------------------------------------------------------------------------------------------------------------
#  size_core
# ----------------------------------------------------------------------------------------------------------------------
def size_core(turboshaft,turboshaft_conditions,conditions):
    """
    Sizes the core flow path of a turboshaft engine at the design point condition.

    Parameters
    ----------
    turboshaft : Turboshaft
        Turboshaft engine object containing design parameters
            - reference_temperature : float
                Reference temperature [K]
            - reference_pressure : float
                Reference pressure [Pa]
            - design_power : float
                Design power output [W]
            - inputs.bypass_ratio : float
                Engine bypass ratio
            - inputs.total_temperature_reference : float
                Reference total temperature [K]
            - inputs.total_pressure_reference : float
                Reference total pressure [Pa]
            - inputs.number_of_engines : int
                Number of engines
    turboshaft_conditions : Conditions
        Turboshaft operating conditions data structure
    conditions : Conditions
        Freestream conditions data structure
            - freestream.speed_of_sound : float
                Freestream speed of sound [m/s]

    Returns
    -------
    None
        Results are stored in turboshaft object:
            - mass_flow_rate_design : float
                Design point mass flow rate [kg/s]
            - compressor.mass_flow_rate : float
                Compressor mass flow rate [kg/s]

    Notes
    -----
    This function determines the core flow path size required to achieve the
    specified design power output. The sizing process accounts for component
    performance characteristics and operating conditions.

    **Major Assumptions**
        * Perfect gas behavior
        * Core flow is sized at design point conditions
        * Component efficiencies are constant
        * Free power turbine configuration

    **Theory**

    The core sizing is based on achieving the required power output while
    maintaining appropriate flow conditions through each component. The process
    uses non-dimensional parameters to determine the required mass flow rate.

    References
    ----------
    [1] https://soaneemrana.org/onewebmedia/ELEMENTS%20OF%20GAS%20TURBINE%20PROPULTION2.pdf
    [2] https://www.colorado.edu/faculty/kantha/sites/default/files/attached-files/70652-116619_-_luke_stuyvenberg_-_dec_17_2015_1258_pm_-_stuyvenberg_helicopterturboshafts.pdf

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboshaft_Propulsor.compute_power
    """             
    
    #unpack from turboshaft
    Tref                                           = turboshaft.reference_temperature
    Pref                                           = turboshaft.reference_pressure 
    total_temperature_reference                    = turboshaft_conditions.total_temperature_reference  
    total_pressure_reference                       = turboshaft_conditions.total_pressure_reference 

    #compute nondimensional power
    compute_power(turboshaft,turboshaft_conditions,conditions)

    #unpack results 
    Psp                                            = turboshaft_conditions.non_dimensional_power
    
    #compute dimensional mass flow rates
    mdot_air                                       = turboshaft.design_power/Psp
    mdot_compressor                                = mdot_air/ (np.sqrt(Tref/total_temperature_reference)*(total_pressure_reference/Pref))

    #pack outputs
    turboshaft.mass_flow_rate_design               = mdot_air
    turboshaft.compressor.mass_flow_rate           = mdot_compressor

    return    
