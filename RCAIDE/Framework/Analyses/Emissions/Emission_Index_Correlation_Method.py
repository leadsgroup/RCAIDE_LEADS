# RCAIDE/Framework/Analyses/Emissions/Emission_Index_Correlation_Method.py
#  
# Created:  Jul 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Analyses    import Process 
from RCAIDE.Library.Methods.Emissions.Emission_Index_Empirical_Method import *  
from .Emissions                   import Emissions 
  

# ----------------------------------------------------------------------------------------------------------------------
#  Correlation_Buildup
# ---------------------------------------------------------------------------------------------------------------------- 
class Emission_Index_Correlation_Method(Emissions): 
    """
    Analysis class for computing aircraft emissions using empirical correlation methods.

    Attributes
    ----------
    process : Process
        Container for computational processes
        - compute : Process
            Computation process container
            - emissions : function
                Function handle for emissions calculation

    Notes
    -----
    This class implements an emissions analysis method based on empirical correlations
    and pre-defined emission indices.

    The analysis follows these steps:
    1. Initialization of computation processes
    2. Assignment of empirical correlation functions
    3. Evaluation of emissions for given mission segments

    **Major Assumptions**
    * Emission indices are constant for each fuel type
    * Linear scaling with fuel flow rate
    * Well-mixed combustion products
    * Steady-state operation

    **Definitions**

    'Emission Index (EI)'
        Mass of pollutant emitted per unit mass of fuel burned [g/kg_fuel]

    'Process'
        Container for computational methods and their settings

    See Also
    --------
    RCAIDE.Framework.Analyses.Emissions : Parent emissions analysis class
    RCAIDE.Library.Methods.Emissions.Emission_Index_Empirical_Method : Implementation methods

    References
    ----------
    [1] Norman, P. D., et al. (2003). Development of the technical basis for a new emissions 
        parameter covering the whole AIRcraft operation: NEPAIR (ENV.C.1/ETU/2000/0094). 
        Final Technical Report.
    [2] Dopelheuer, A., & Lecht, M. (1999). Influence of engine performance on emission 
        characteristics. RTO MP-14, paper 20.
    """    
    
    def __defaults__(self): 
        """
        This sets the default values and methods for the analysis.
    
        Assumptions:
        None
    
        Source:
        None 
        """ 

        # build the evaluation process
        compute                         = Process()  
        compute.emissions               = None  
        self.process                    = Process()
        self.process.compute            = compute 
                
        return
            
    def initialize(self):   
        """
        This function computes the emissions of different species from the Emission Indices 
        available in literature.
    
        Assumptions:
        None
    
        Source:
        None 
        """         
        
        compute   =  self.process.compute     
        compute.emissions  = evaluate_correlation_emissions_indices
        return 


    def evaluate(self,state,vehicle):
        """
        The default evaluate function.

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