# RCAIDE/Library/Components/Propulsors/Ram_Air_Turbine.py
# 
#  
# Created:  Sep. 2025, M. Guidotti 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
## RCAIDE imports
from RCAIDE.Framework.Core                  import Data 
from .Components                            import Component
 
# ----------------------------------------------------------------------
#  Ram_Air_Turbine
# ----------------------------------------------------------------------
class Ram_Air_Turbine(Component):
    """
    A Ram_Air_Turbine propulsion system model that simulates the performance of a Ram_Air_Turbine engine.
   

    Attributes
    ----------
    tag : str
        Identifier for the shaft engine. Default is 'turboshaft'. 
        
    turboshaft : Component
        Turboshaft component. Default is the Turboshaft Class.
        
    generator : Component
        Generator component. Default is DC_Generator Class.
        
    gearbox : Component
        Gearbox data structure. Default is None. 
        
    inverse_calculation : Component
        Flag that determines the how calculations are performed. Default is False    

    Notes
    -----
    The Ram_Air_Turbine class inherits from the Turboshaft class and implements
    methods for computing Ram_Air_Turbine engine performance. Unlike other gas turbine
    engines that produce thrust, a Ram_Air_Turbine engine's primary output is shaft
    power, typically used to drive a helicopter rotor or other mechanical systems. 

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Propulsors.Turboshaft 
    """ 
    def __defaults__(self):
        # setting the default values
        self.tag                       = 'Ram_Air_Turbine'
        self.gearbox                   = Data()
        self.rotor                     = None
        self.generator                 = None
