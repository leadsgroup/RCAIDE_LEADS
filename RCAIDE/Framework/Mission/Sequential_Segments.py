# RCAIDE/Framework/Analyses/Mission/Sequential_Segments.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports   
import RCAIDE
from RCAIDE.Library.Mission.Common.Segments    import  sequential_segments
from RCAIDE.Library.Mission.Common.Pre_Process import  aerodynamics,stability, energy,emissions, set_residuals_and_unknowns
from RCAIDE.Framework.Core                               import Container as ContainerBase
from RCAIDE.Framework.Analyses                           import Process 
from . import Segments

# ----------------------------------------------------------------------------------------------------------------------
# ANALYSIS
# ----------------------------------------------------------------------------------------------------------------------  -Mission
class Sequential_Segments(Segments.Segment.Container):
    """ Solves each segment one at time
    
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

        self.tag = 'mission'
        
        #   Initialize   
        self.process.initialize                                = Process() 
        self.process.initialize.aero                           = aerodynamics
        self.process.initialize.stability                      = stability
        self.process.initialize.energy                         = energy
        self.process.initialize.emissions                      = emissions
        self.process.initialize.set_residuals_and_unknowns     = set_residuals_and_unknowns
 
        #   Converge 
        self.process.converge    = sequential_segments
         
        #   Iterate     
        del self.process.iterate  

        return  

                        
    def evaluate(self,state=None):
        """ This executes the entire process
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            State  [Data()]
    
            Outputs:
            State  [Data()]
    
            Properties Used:
            None
        """  
        
        if state is None:
            state = self.state
        self.process(self)
        return self     
        
    
# ----------------------------------------------------------------------
#   Container Class
# ----------------------------------------------------------------------
class Container(ContainerBase):
    """ Container for mission
    
        Assumptions:
        None
        
        Source:
        None
    """    
    
    def evaluate(self,state=None):
        """ Go through the missions, run through them, save the results
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            state   [Data()]
    
            Outputs:
            Results [Data()]
    
            Properties Used:
            None
        """
        pass

# Link container
Sequential_Segments.Container = Container