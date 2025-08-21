# RCAIDE/Framework/Analyses/Mission/Segments/Vertical_Flight/Hover.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports 
from RCAIDE.Framework.Core                        import Units 
from RCAIDE.Framework.Mission.Segments.Evaluate   import Evaluate 
from RCAIDE.Library.Mission                       import Common,Segments
from RCAIDE.Library.Methods.skip                  import skip 

# ----------------------------------------------------------------------------------------------------------------------
#  Hover
# ---------------------------------------------------------------------------------------------------------------------- 
class Hover(Evaluate):
    """ A stationary hover for VTOL aircraft. No aerodynamic drag and lift are used, since there is no velocity.
    
        Assumptions:
        None
        
        Source:
        None
    """     
    
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
        
        # -------------------------------------------------------------------------------------------------------------- 
        #   User Inputs
        # -------------------------------------------------------------------------------------------------------------- 
        
        self.altitude           = None
        self.time               = 1.0 * Units.seconds
        self.true_course        = 0.0 * Units.degrees            
             
        # -------------------------------------------------------------------------------------------------------------- 
        #  Mission specific processes 
        # --------------------------------------------------------------------------------------------------------------    
        initialize                         = self.process.initialize
        iterate                            = self.process.iterate 
        initialize.conditions              = Segments.Vertical_Flight.Hover.initialize_conditions
        iterate.conditions.aerodynamics    = skip
        iterate.residuals.flight_dynamics  = Common.Residuals.flight_dynamics
        return

