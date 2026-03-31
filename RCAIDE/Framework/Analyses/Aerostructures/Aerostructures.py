# RCAIDE/Framework/Analyses/Aerodynamics/Aerodynamics.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports   
from RCAIDE.Framework.Core     import Data
from RCAIDE.Framework.Analyses import Analysis  

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Aerodynamics
# ---------------------------------------------------------------------------------------------------------------------- 
class Aerostructures(Analysis):
    """This is the base class for aerodynamics analyses. It contains functions
    that are built into the default class.
    
    Assumptions:
    None
    
    Source:
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
        self.tag                                                         = 'aerodynamics'   
        self.settings                                                    = Data() 
        
        
        
    def evaluate(self,state, vehicle):
        """The default evaluate function.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        results   <Results class> (empty)

        Properties Used:
        N/A
        """           
        
        results = Data()
        
        return results
    
    def post_process(self):
        """The default post processing function.

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
        
        return     