# RCAIDE/Library/Components/Propulsors/Converters/Compression_Nozzle.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Feb 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports   
from .Converter  import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Compression_Nozzle.append_compression_nozzle_conditions import append_compression_nozzle_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  Compression Nozzle 
# ---------------------------------------------------------------------------------------------------------------------- 
class Compression_Nozzle(Converter):
    """
    A compression nozzle component model for propulsion systems that converts flow energy into pressure.

    Attributes
    ----------
    tag : str
        Identifier for the nozzle. Default is 'Nozzle'.
        
    polytropic_efficiency : float
        Efficiency of the compression process. Default is 1.0.
        
    pressure_ratio : float
        Ratio of outlet to inlet pressure. Default is 1.0.
        
    pressure_recovery : float
        Pressure recovery factor. Default is 1.0.
        
    compressibility_effects : bool
        Flag for including compressibility effects. Default is False.
        
    compression_levels : float
        Number of compression stages. Default is 0.0.
        
    theta : float
        Nozzle angle [rad]. Default is 0.0.

    Notes
    -----
    The Compression_Nozzle class models a nozzle that increases the pressure of the flow.
    The model assumes:
    * Pressure ratio and efficiency do not change with varying conditions
    * Flow is either subsonic or choked

    **Definitions**

    'Polytropic Efficiency'
        Measure of compression process efficiency accounting for real gas effects

    'Pressure Recovery'
        Ratio of actual to ideal pressure rise
        
    'Choked Flow'
        Condition where flow reaches sonic velocity at the throat

    References
    ----------
    [1] Mattingly, J. D., & Boyer, K. M. (2016). Elements of propulsion: Gas 
        turbines and rockets, second edition Jack D. Mattingly, Keith M. Boyer. 
        American Institute of Aeronautics and Astronautics.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Compression_Nozzle
    """

    def __defaults__(self): 
        """This sets the default values for the component to function.

        Assumptions:
            1. Pressure ratio and efficiency do not change with varying conditions.
            2. Subsonic or choked output.

        Source:
            Mattingly, J. D., & Boyer, K. M. (2016). Elements of propulsion: Gas turbines and rockets, second edition Jack D. Mattingly, Keith M. Boyer. American Institute of Aeronautics and Astronautics. 
        """    
        self.tag                             = 'Nozzle'
        self.polytropic_efficiency           = 1.0
        self.pressure_ratio                  = 1.0
        self.pressure_recovery               = 1.0
        self.compressibility_effects         = False 
        self.compression_levels              = 0.0
        self.theta                           = 0.0

    def append_operating_conditions(self,segment,propulsor): 
        propulsor_conditions =  segment.state.conditions.energy[propulsor.tag]
        append_compression_nozzle_conditions(self,segment,propulsor_conditions)
        return