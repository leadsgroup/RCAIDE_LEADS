# RCAIDE/Library/Components/Propulsors/Converters/Expansion_Nozzle.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Feb 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports   
from .Converter  import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Expansion_Nozzle.append_expansion_nozzle_conditions import append_expansion_nozzle_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  Expansion Nozzle
# ----------------------------------------------------------------------------------------------------------------------
class Expansion_Nozzle(Converter):
    """
    An expansion nozzle component model for propulsion systems that converts pressure energy into kinetic energy.

    Attributes
    ----------
    tag : str
        Identifier for the nozzle. Default is 'Nozzle'.
        
    polytropic_efficiency : float
        Efficiency of the expansion process accounting for losses. Default is 1.0.
        
    pressure_ratio : float
        Ratio of outlet to inlet pressure. Default is 1.0.

    Notes
    -----
    The Expansion_Nozzle class models a nozzle that converts pressure energy into 
    kinetic energy through an expansion process. The model:
    * Handles subsonic and choked flow conditions
    * Accounts for losses through polytropic efficiency
    * Maintains constant pressure ratio at design conditions
    * Assumes quasi-one-dimensional flow

    **Major Assumptions**
    * Pressure ratio and efficiency do not change with varying conditions
    * Flow can be subsonic or choked (but not supersonic)

    **Definitions**

    'Polytropic Efficiency'
        Measure of expansion process efficiency accounting for real gas effects

    'Pressure Ratio'
        Ratio of exit to inlet pressure
    
    'Choked Flow'
        Condition where flow reaches sonic velocity at the throat

    References
    ----------
    [1] Mattingly, J. D., & Boyer, K. M. (2016). Elements of propulsion: Gas 
        turbines and rockets, second edition Jack D. Mattingly, Keith M. Boyer. 
        American Institute of Aeronautics and Astronautics.

    See Also
    --------
    RCAIDE.Library.Components.Component
    RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle
    RCAIDE.Library.Methods.Powertrain.Converters.Expansion_Nozzle
    """
    
    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
            1. Pressure ratio and efficiency do not change with varying conditions.
            2. Subsonic or choked output.

        Source:
            https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/
        """          
        #set the defaults
        self.tag = 'Nozzle'
        self.polytropic_efficiency           = 1.0
        self.pressure_ratio                  = 1.0

    def append_operating_conditions(self,segment,energy_conditions,noise_conditions=None):
        append_expansion_nozzle_conditions(self,segment,energy_conditions)
        return                        