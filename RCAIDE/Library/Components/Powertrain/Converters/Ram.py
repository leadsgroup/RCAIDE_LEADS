# RCAIDE/Library/Components/Propulsors/Converters/Ram.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Feb 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports
from RCAIDE.Framework.Core import Data
from .Converter  import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Ram.append_ram_conditions import append_ram_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  Ram Component
# ----------------------------------------------------------------------------------------------------------------------
class Ram(Converter):
    """
    A ram air compression component model that simulates the compression of incoming air flow.

    Attributes
    ----------
    tag : str
        Identifier for the ram component. Default is 'Ram'.
        
    working_fluid : Data
        Container for working fluid properties and conditions. Default is empty Data().

    Notes
    -----
    The Ram class models the compression of incoming air due to the ram effect
    in high-speed flight. It calculates:

    * Total pressure rise due to flow deceleration
    * Temperature increase from compression
    * Changes in fluid properties
    * Ram recovery efficiency
    * Compressibility effects

    The model assumes:

    * Quasi-one-dimensional flow
    * Adiabatic process (no heat transfer)
    * Perfect gas behavior
    * Steady flow conditions
    * No boundary layer effects
    * No shock waves (unless explicitly modeled)

    **Theory**
    The ram compression process follows isentropic flow relations for subsonic flow
    and includes normal shock relations for supersonic flow. The total pressure 
    recovery depends on the flight Mach number and inlet geometry.

    **Definitions**
    
    'Ram Effect'
        Pressure rise due to deceleration of high-speed flow
    'Recovery Factor'
        Ratio of actual to ideal pressure rise
    'Working Fluid'
        Fluid medium undergoing compression (typically air)

    References
    ----------
    [1] Mattingly, J. D., & Boyer, K. M. (2016). Elements of propulsion: Gas 
        turbines and rockets, second edition Jack D. Mattingly, Keith M. Boyer. 
        American Institute of Aeronautics and Astronautics.

    See Also
    --------
    RCAIDE.Library.Components.Component
    RCAIDE.Library.Methods.Powertrain.Converters.Ram.append_ram_conditions
    """

    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
            None

        Source:
        [1] Mattingly, J. D., & Boyer, K. M. (2016). Elements of propulsion: Gas 
            turbines and rockets, second edition Jack D. Mattingly, Keith M. Boyer. 
            American Institute of Aeronautics and Astronautics.
        """
        #set the deafult values
        self.tag                      = 'Ram' 
        self.working_fluid            = Data()


    def append_operating_conditions(self,segment,propulsor): 
        propulsor_conditions =  segment.state.conditions.energy[propulsor.tag]
        append_ram_conditions(self,segment,propulsor_conditions)
        return                         