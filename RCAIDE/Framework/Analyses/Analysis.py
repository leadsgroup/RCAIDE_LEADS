# RCAIDE/Framework/Analyses/Analysis.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Data 
from RCAIDE.Framework.Core import Container as ContainerBase 

# ----------------------------------------------------------------------------------------------------------------------
# Analysis
# ----------------------------------------------------------------------------------------------------------------------  
class Analysis(Data):
    """ RCAIDE.Framework.Analyses.Analysis()
    
        The Top Level Analysis Class
        
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
        self.tag    = 'analysis'
        self.features = Data()
        self.settings = Data()
    
# ----------------------------------------------------------------------------------------------------------------------  
#  CONFIG CONTAINER
# ----------------------------------------------------------------------------------------------------------------------   
class Container(ContainerBase):
    """ RCAIDE.Framework.Analyses.Analysis.Container()
    
        The Analysis Container Class
        
            Assumptions:
            None
            
            Source:
            N/A
    """ 

    
# ----------------------------------------------------------------------------------------------------------------------   
#  LINKING
# ----------------------------------------------------------------------------------------------------------------------   
Analysis.Container = Container