# RCAIDE/Framework/Analyses/Mission/Segments/Conditions/Numerics.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Data
from .Conditions import Conditions 
from RCAIDE.Library.Methods.Utilities.Chebyshev  import chebyshev_data 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Numerics
# ----------------------------------------------------------------------------------------------------------------------

class Numerics(Conditions):
    """ Creates the data structure for the numerical solving of a mission.
    
        Assumptions:
        None
        
        Source:
        None
    """
    
    def __defaults__(self):
        """This sets the default values.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            None
    
            Outputs:
            None
    
            Properties Used:
            None
        """           
        self.tag                              = 'numerics' 
        self.number_of_control_points         = 16
        self.discretization_method            = chebyshev_data
        self.solver                           = Data()
        self.solver.type                      = "optimize" # options: "optimize", "root_finder"
        self.solver.method                    = "SLSQP"    
        self.solver.objective                 = None        # options: # None, energy , power 
        self.solver.tolerance_solution        = 1E-6     
        self.solver.converged                 = None
        self.solver.print_output              = True
        self.solver.max_evaluations           = 200
        self.solver.step_size                 = 1E-8    
        
        self.dimensionless                    = Conditions()
        self.dimensionless.control_points     = np.empty([0,0])
        self.dimensionless.differentiate      = np.empty([0,0])
        self.dimensionless.integrate          = np.empty([0,0]) 
            
        self.time                             = Conditions()
        self.time.control_points              = np.empty([0,0])
        self.time.differentiate               = np.empty([0,0])
        self.time.integrate                   = np.empty([0,0]) 
        
        
        
        