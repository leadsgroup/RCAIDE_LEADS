# RCAIDE/Framework/Analyses/Aerostructures/Finite_Element_Analysis.py
#  
# Created:  Mar 2026 , S.Sharma

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports   
from RCAIDE.Framework.Core                                     import Data, Units
from RCAIDE.Framework.Analyses                                 import Process 
from RCAIDE.Library.Methods.Aerostructures                       import Common
from .Aerostructures                                             import Aerostructures
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis import *   

# package imports 
import numpy as np 
import os,pickle

# ----------------------------------------------------------------------------------------------------------------------
#  Finite_Element_Analysis
# ---------------------------------------------------------------------------------------------------------------------- 
class Finite_Element_Analysis(Aerostructures):
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
        self.tag                                                    = 'Finite_Element_Analysis'  
        
        

    def initialize(self, vehicle): 
        
        # use_surrogate   = self.settings.use_surrogate   
        # # If we are using the surrogate
        # if use_surrogate == True: 
        #     #  training data
        #     if not os.path.exists(self.filename):
        #         train_VLM_surrogates(self, vehicle)
    
        #         if self.settings.store_training_data:
        #             with open(self.filename, 'wb') as file:
        #                 pickle.dump(self.training, file)
        #     else:
        #         with open(self.filename, 'rb') as file:
        #             self.training = pickle.load(file)
        #         print(r""" 
        #         [INFO] Aerodynamic training data loaded. Delete the file and rerun to regenerate. """)
        #     # build surrogate
        #     build_VLM_surrogates(self, vehicle)        
    
        # # build the evaluation process
        # compute   =  self.process.compute                  
        # if use_surrogate == True: 
        #     compute.lift.inviscid_wings  = evaluate_surrogate
        # else:
        #     compute.lift.inviscid_wings  = evaluate_no_surrogate
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
        settings = self.settings 
        results  = self.process.compute(state,settings,vehicle)
        
        return results