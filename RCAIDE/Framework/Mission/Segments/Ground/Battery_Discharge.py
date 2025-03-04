# RCAIDE/Framework/Analyses/Mission/Segments/Ground/Battery_Disharge.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Mission.Segments.Evaluate        import Evaluate    
from RCAIDE.Framework.Core                             import Units
from RCAIDE.Library.Mission.Segments                   import Ground  
from RCAIDE.Library.Methods.skip                       import skip 

# ----------------------------------------------------------------------------------------------------------------------
#  SEGMENT
# ----------------------------------------------------------------------------------------------------------------------
class Battery_Discharge(Evaluate): 

    # ------------------------------------------------------------------
    #   Data Defaults
    # ------------------------------------------------------------------  

    def __defaults__(self):  

        """ This sets the default solver flow. Anything in here can be modified after initializing a segment.
    
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
        
        # --------------------------------------------------------------
        #   User Inputs
        # --------------------------------------------------------------
        self.altitude               = None
        self.time                   = 1.0 * Units.seconds 
        self.cooling_time           = 0.0 * Units.seconds
        self.overcharge_contingency = 1.10 
        self.true_course            = 0.0 * Units.degrees 

        # -------------------------------------------------------------------------------------------------------------- 
        #  Mission specific processes 
        # --------------------------------------------------------------------------------------------------------------       
        initialize                         = self.process.initialize 
        initialize.conditions              = Ground.Battery_Charge_Discharge.initialize_conditions 
        iterate                            = self.process.iterate 
        iterate.unknowns.mission           = skip
        iterate.conditions.aerodynamics    = skip
        iterate.conditions.stability       = skip  
        post_process                       = self.process.post_process  
        post_process.noise                 = skip
        post_process.emissions             = skip
        
        return
