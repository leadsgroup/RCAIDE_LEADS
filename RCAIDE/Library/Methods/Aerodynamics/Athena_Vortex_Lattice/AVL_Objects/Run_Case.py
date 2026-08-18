# RCAIDE/Library/Methods/Aerodynamics/Athena_Vortex_Lattice/AVL_Objects/Run_Case.py
#
# Created: Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core import  Data , DataOrdered

# ----------------------------------------------------------------------------------------------------------------------
# Run_Case
# ----------------------------------------------------------------------------------------------------------------------   
class Run_Case(Data):
    """ This data class defines the parameters for the analysis cases 
    including angle of attack and mach number 

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
    
    def __defaults__(self):
        """Defines the data structure and defaults of aerodynamics coefficients, 
        body axis derivatives and stability axis derivatives   

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

        self.index                                                    = 0		 
        self.tag                                                      = 'case'
        self.mass                                                     = 0.0
                                
        self.conditions                                               = Data()
        self.stability_and_control                                    = Data()  
        self.stability_and_control.control_surface_names              = None
        self.stability_and_control.control_surface_functions          = None
        self.stability_and_control.number_of_control_surfaces         = 0
                 
        self.conditions.freestream                                    = Data() 
        self.conditions.freestream.mach                               = 0.0       
        self.conditions.freestream.velocity                           = 0.0       
        self.conditions.freestream.density                            = 1.225    
        self.conditions.freestream.gravitational_acceleration         = 9.81        
        self.conditions.aerodynamics                                  = Data() 
        self.conditions.aerodynamics.parasite_drag                    = 0.0     
        self.conditions.aerodynamics.angles                           = Data()  
        self.conditions.aerodynamics.angles.alpha                     = None     
        self.conditions.aerodynamics.angles.beta                      = 0.0  
        self.conditions.aerodynamics.coefficients                     = Data()  
        self.conditions.aerodynamics.coefficients.lift                = Data()  
        self.conditions.aerodynamics.coefficients.lift.inviscid       = Data()
        self.conditions.aerodynamics.coefficients.lift.inviscid.total = 0
        
        self.conditions.static_stability                      = Data()  
        self.conditions.static_stability.coefficients         = Data()  
        self.conditions.static_stability.coefficients.roll    = 0
        self.conditions.static_stability.coefficients.pitch   = 0        

        self.aero_result_filename_1                           = None
        self.aero_result_filename_2                           = None
        self.aero_result_filename_3                           = None 
        self.aero_result_filename_4                           = None
        self.eigen_result_filename_1                          = None 
        self.eigen_result_filename_2                          = None 
        return
 
class Container(DataOrdered):
    """ This is a data class for the addition of a cases to the set of run cases

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
        case.index = len(self)+1
        self.append(case)

        return
    
    
# ------------------------------------------------------------
#  Handle Linking
# ------------------------------------------------------------
Run_Case.Container = Container
