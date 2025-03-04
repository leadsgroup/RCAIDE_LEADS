# RCAIDE/Framework/Analyses/Propulsion/Ducted_Fan_Design_Code.py
#  
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports   
from RCAIDE.Framework.Core import  Data , Units, Container
from RCAIDE.Framework.Analyses import Analysis   

# package imports 
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
# Ducted_Fan_Design_Code
# ---------------------------------------------------------------------------------------------------------------------- 
class Ducted_Fan_Design_Code(Analysis): 
    def __defaults__(self):
        """This sets the default values and methods for the analysis.

        Assumptions:
            None

        Source:
            https://web.mit.edu/drela/Public/web/dfdc/
            http://www.esotec.org/sw/DFDC.html 
        """          
        self.tag                                          = 'dfdc'    
                                                          
        self.current_status                               = Data()        
        self.current_status.batch_index                   = 0
        self.current_status.batch_file                    = None
        self.current_status.deck_file                     = None
        self.current_status.cases                         = None      
        self.geometry                                     = None   
                                                          
        self.settings                                     = Data()
        self.settings.filenames                           = Data()
        self.settings.filenames.dfdc_bin_name             = 'dfdc'   # to call dfdc from command line. If avl is not on the system path, include absolute path to the avl binary i.e. '/your/path/to/avl'
        self.settings.filenames.run_folder                = 'dfdc_files'   
        self.settings.filenames.deck_template             = 'commands_{0:02d}.deck'  
        self.settings.filenames.results_template          = 'results_Vinf_{:.0f}_RPM_{:.0f}_Alt_{:.0f}' 
        self.settings.filenames.case                      = None
        self.settings.filenames.log_filename              = 'dfdc_log.txt'
        self.settings.filenames.err_filename              = 'dfdc_err.txt'
 
        self.settings.print_output                        = False 
                   
        # Regression Status           
        self.settings.keep_files                          = False           
        self.settings.new_regression_results              = False   
    
        self.run_cases                                    = Container() 
        
        # Conditions table, used for surrogate model training
        self.training                                     = Data()   
                  
        # Standard subsonic/transolic aircarft          
        self.training.tip_mach                            = np.array([0.6, 0.7, 0.8])   
        self.training.mach                                = np.array([0.15,0.3,0.45])  
        self.training.altitude                            = np.array([0,2500,5000,8000])*Units.feet    
                                                        
        self.training_file                                = None
                  
        # Surrogate model          
        self.surrogates                                   = Data()  
 
     
    def append_case(self,case):
        """ Adds a case to the set of run cases "
        
	Assumptions:
	    None
    
	Source:
	    None
    
	Inputs:
	    None
    
	Outputs:
	    None
    
	Properties Used:
	    N/A
	"""
        run_cases   =  self.run_cases  
        run_cases.append(case) 
        return
