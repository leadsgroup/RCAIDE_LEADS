# RCAIDE/Framework/Analyses/Stability/Athena_Vortex_Lattice.py 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports   
from RCAIDE.Framework.Core                                     import Data
from RCAIDE.Framework.Analyses                                 import Process  
from .Stability                                                import Stability   
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice import *

# ----------------------------------------------------------------------------------------------------------------------
#  Athena_Vortex_Lattice
# ----------------------------------------------------------------------------------------------------------------------
class Athena_Vortex_Lattice(Stability):
    """This is a subsonic aerodynamic buildup analysis based on the vortex lattice method

     Assumptions:
     Stall effects are negligible 
 
     Source:
     N/A
 
     Inputs:
     None
 
     Outputs:
     None
 
     Properties Used:
     N/A 
    """      
    
    def __defaults__(self):
        """This sets the default values and methods for the analysis.

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
        
        self.tag                                    = 'Athena_Vortex_Lattice'  
        self.vehicle                                = Data()  
        self.process                                = Process()
        self.process.initialize                     = Process()  
                   
         

    def initialize(self, vehicle):
        pass
        return 
    
         
    def evaluate(self,state, vehicle):
        """The default evaluate function.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        results   <RCAIDE data class>

        Properties Used:
        self.settings
        self.vehicle
        """           
        return 
    
     
 