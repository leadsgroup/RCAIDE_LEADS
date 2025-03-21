# RCAIDE/Energy/Networks/Electric.py
# 
# Created:  Jul 2023, M. Clarke
# Modified: Sep 2024, S. Shekar
#           Jan 2025, M. Clarke
  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports 
import RCAIDE  
from RCAIDE.Framework.Mission.Common                      import Residuals
from RCAIDE.Library.Mission.Common.Unpack_Unknowns.energy import unknowns
from .Network                                             import Network              
from RCAIDE.Library.Methods.Powertrain.Systems.compute_avionics_power_draw import compute_avionics_power_draw
from RCAIDE.Library.Methods.Powertrain.Systems.compute_payload_power_draw  import compute_payload_power_draw
import numpy as np

from .Hybrid import Hybrid
# Python imports
import  numpy as  np

# ----------------------------------------------------------------------------------------------------------------------
#  Electric
# ----------------------------------------------------------------------------------------------------------------------  
class Electric(Hybrid):
    """ An electric network comprising one or more electrichemical energy source and/or fuel cells that to power electro-
        mechanical conversion machines eg. electric motors via a bus. Electronic speed controllers, thermal management
        system, avionics, and other eletric power systes paylaods are also modeled. Ducted fans, generators, rotors and
        motors etc. arranged into groups, called propulsor groups, to siginify how they are connected in the network.
        The network also takes into consideration thermal management components that are connected to a coolant line.
        Based on the propulsor, additional unknowns and residuals are added to the mission  


        Assumptions:
        The y axis rotation is used for rotating the rotor about the Y-axis for tilt rotors and tiltwings

        Source:
        None
    """  
    def __defaults__(self):
        """ This sets the default values for the network to function.

            Assumptions:
            None

            Source:
            N/A

            Inputs:
            None

            Outputs:
            None

            Properties Used:
            N/A
        """         

        self.tag                          = 'electric'
        self.hybrid_power_split_ratio     = 1.0