# RCAIDE/Library/Components/Propulsors/Converters/Compressor.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Feb 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports   
from .Converter  import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Compressor.append_compressor_conditions import append_compressor_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  Compressor 
# ---------------------------------------------------------------------------------------------------------------------- 
class Compressor(Converter):
    """
    A compressor component model for gas turbine engines.

    Attributes
    ----------
    tag : str
        Identifier for the compressor. Default is 'Compressor'.
        
    polytropic_efficiency : float
        Efficiency of the compression process accounting for losses. Default is 1.0.
        
    pressure_ratio : float
        Ratio of outlet to inlet total pressure. Default is 1.0.

    Notes
    -----
    The Compressor class models the compression process in gas turbine engines.
    It calculates the work required to increase the pressure of the working fluid
    while accounting for losses through the polytropic efficiency.

    **Definitions**

    'Polytropic Efficiency'
        A measure of compression efficiency that accounts for the continuous
        nature of the compression process and real gas effects

    'Pressure Ratio'
        Ratio of exit to inlet total pressure, a key parameter determining
        the compressor's performance and work input required

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Compressor
    """
    
    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
            None

        Source:
            None 
        """          
        #set the default values
        self.tag                             = 'Compressor'
        self.polytropic_efficiency           = 1.0
        self.pressure_ratio                  = 1.0

    def append_operating_conditions(self,segment,propulsor): 
        propulsor_conditions =  segment.state.conditions.energy[propulsor.tag]
        append_compressor_conditions(self,segment,propulsor_conditions)
        return        