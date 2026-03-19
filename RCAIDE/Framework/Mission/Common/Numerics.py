# RCAIDE/Framework/Analyses/Mission/Segments/Conditions/Numerics.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from pyclbr import Class
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
        self.tag                                = 'numerics' 
        self.number_of_control_points           = 16
        self.discretization_method              = chebyshev_data
    
        # mission solver 
        self.mission_solver                     = Conditions()
        self.mission_solver.type                = "optimize" # options: "optimize", "root_finder"
        self.mission_solver.method              = "SLSQP"    
        self.mission_solver.objective           = "energy"   # options: # None, energy , power 
        self.mission_solver.tolerance           = 1E-6     
        self.mission_solver.converged           = None
        self.mission_solver.print_output        = True
        self.mission_solver.max_evaluations     = 200
        self.mission_solver.step_size           = 1E-8  
        self.mission_solver.verbose             = False
           
        # network solver    
        self.network_solver                     = Conditions()
        self.network_solver.type                = None 
        self.network_solver.tolerance           = 1E-6     
        self.network_solver.converged           = None
        self.network_solver.print_output        = True
        self.network_solver.max_evaluations     = 200
        self.network_solver.step_size           = 1E-8   
        self.network_solver.verbose             = False
           
        self.dimensionless                      = Conditions()
        self.dimensionless.control_points       = np.empty([0,0])
        self.dimensionless.differentiate        = np.empty([0,0])
        self.dimensionless.integrate            = np.empty([0,0]) 
               
        self.time                               = Conditions()
        self.time.control_points                = np.empty([0,0])
        self.time.differentiate                 = np.empty([0,0])
        self.time.integrate                     = np.empty([0,0])
