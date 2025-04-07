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
        
    def initialize(self,*args,**kwarg):
        """This is used to initialize the analysis' specific algorithms.
                
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
    
    def evaluate(self,*args,**kwarg):
        """This is used to execute the analysis' specific algorithms.
                
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
        raise NotImplementedError
        return Data()
    
    def post_process(self,*args,**kwarg):
        """This is used to post_process 
                
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
    
    def __call__(self,*args,**kwarg):
        
        """This is used to set the class' call behavior to the evaluate function.
                        
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
        
        return self.evaluate(*args,**kwarg)
    
    
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
        
    def initialize(self,*args,**kwarg):
        """This is used to execute the initialize functions of the analyses
            stored in the container.
                                        
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
        for tag,analysis in self.items:
            if hasattr(analysis,'initialize'):
                analysis.initialize(*args,**kwarg) 
    
    def evaluate(self,*args,**kwarg):
        """This is used to execute the evaluate functions of the analyses
            stored in the container.
                                                
                Assumptions:
                None
                                                
                Source:
                N/A
                                                
                Inputs:
                None
                                                
                Outputs:
                Results of the Evaluate Functions
                                                
                Properties Used:
                N/A
            """
        results = Data()
        for tag,analysis in self.items(): 
            if hasattr(analysis,'evaluate'):
                result = analysis.evaluate(*args,**kwarg)
            else:
                result = analysis(*args,**kwarg)
            results[tag] = result
        return results
    
    def post_process(self,*args,**kwarg):
        """This is used to execute post processing functions 
                                                
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
        
        for tag,analysis in self.items():
            if hasattr(analysis,'post_process'):
                analysis.post_process(*args,**kwarg)
                
    
    def __call__(self,*args,**kwarg): 
        """This is used to set the class' call behavior to the evaluate functions.
                                                        
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
        
        return self.evaluate(*args,**kwarg)

    
# ----------------------------------------------------------------------------------------------------------------------   
#  LINKING
# ----------------------------------------------------------------------------------------------------------------------   
Analysis.Container = Container