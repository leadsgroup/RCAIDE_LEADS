# RCAIDE/Framework/Analyses/Mission/Segments/Ground/Test_Stand.py
# 
# 
# Created:  Apr 2025, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Framework.Mission.Segments.Evaluate       import Evaluate
from RCAIDE.Framework.Core                            import Units, Data 
from RCAIDE.Library.Mission.Segments                  import Ground  
from RCAIDE.Library.Mission.Common                    import Residuals , Unpack_Unknowns, Update

# ----------------------------------------------------------------------------------------------------------------------
#  Test_Stand
# ----------------------------------------------------------------------------------------------------------------------
class Test_Stand(Evaluate):
    """ Segment for simulating an aircraft on a test stand. Useful for estimating LTO emissions at different throttle settings 
    """     

    def __defaults__(self):
        """ This sets the default solver flow. Anything in here can be modified after initializing a segment. 
        """         

        # -------------------------------------------------------------------------------------------------------------- 
        #   User Inputs
        # -------------------------------------------------------------------------------------------------------------- 
 
        self.velocity             = None 
        self.throttle             = 1.0
        self.altitude             = 0.0
        self.time                 = 1.0
        self.true_course          = 0.0 * Units.degrees
 
        ones_row                           = self.state.ones_row 
        self.state.unknowns['test_stand']  =  0* ones_row(1)  
        self.state.residuals['test_stand'] =  0* ones_row(1)         
 
        # -------------------------------------------------------------------------------------------------------------- 
        #  Mission specific processes 
        # --------------------------------------------------------------------------------------------------------------       
        initialize                         = self.process.initialize 
        initialize.conditions              = Ground.Test_Stand.initialize_conditions  
        iterate                            = self.process.iterate 
        iterate.unknowns.mission           = skip
        iterate.conditions.aerodynamics    = skip
        iterate.conditions.stability       = skip  
        post_process                       = self.process.post_process  
        post_process.noise                 = skip 
        return


