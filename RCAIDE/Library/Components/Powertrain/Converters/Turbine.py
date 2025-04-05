# RCAIDE/Library/Components/Propulsors/Converters/Turbine.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports  
from .Converter  import Converter 
from RCAIDE.Library.Methods.Powertrain.Converters.Turbine.append_turbine_conditions import append_turbine_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  Turbine
# ----------------------------------------------------------------------------------------------------------------------  
class Turbine(Converter):
    """
    A turbine component model for gas turbine and turbofan engines.

    Attributes
    ----------
    tag : str
        Identifier for the turbine. Default is 'Turbine'.
        
    mechanical_efficiency : float
        Efficiency of mechanical power transmission. Default is 1.0.
        
    polytropic_efficiency : float
        Efficiency of the expansion process accounting for losses. Default is 1.0.

    Notes
    -----
    The Turbine class models the expansion and work extraction process in a 
    turbine stage. The model includes:
    
    * Work extraction calculations
    * Pressure ratio effects
    * Temperature changes
    * Efficiency losses
    * Mechanical power transmission
    * Real gas effects

    **Major Assumptions**

    * Efficiencies do not change with varying conditions

    **Definitions**

    'Mechanical Efficiency'
        Ratio of shaft power output to gas power extraction
    'Polytropic Efficiency'
        Measure of expansion process efficiency accounting for losses

    References
    ----------
    [1] Mattingly, J. D., & Boyer, K. M. (2016). Elements of propulsion: Gas 
        turbines and rockets, second edition Jack D. Mattingly, Keith M. Boyer. 
        American Institute of Aeronautics and Astronautics.

    See Also
    --------
    RCAIDE.Library.Components.Component
    RCAIDE.Library.Methods.Powertrain.Converters.Turbine.append_turbine_conditions
    """
    
    def __defaults__(self):
        """ This sets the default values for the component to function.

        Assumptions:
            None

        Source:
            None 
        """         
        #set the default values
        self.tag                               ='Turbine'
        self.mechanical_efficiency             = 1.0
        self.polytropic_efficiency             = 1.0

    def append_operating_conditions(self,segment,energy_conditions,noise_conditions=None):  
        append_turbine_conditions(self,segment,energy_conditions)
        return                            
    