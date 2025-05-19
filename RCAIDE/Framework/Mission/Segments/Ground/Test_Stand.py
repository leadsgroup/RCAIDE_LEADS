# RCAIDE/Framework/Analyses/Mission/Segments/Ground/Test_Stand.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Framework.Mission.Segments.Evaluate       import Evaluate
from RCAIDE.Framework.Core                            import Units, Data 
from RCAIDE.Library.Mission.Segments                  import Ground  
from RCAIDE.Library.Mission.Common                    import Residuals , Unpack_Unknowns, Update
from RCAIDE.Library.Methods.skip                      import skip 

# ----------------------------------------------------------------------------------------------------------------------
#  Takeoff
# ----------------------------------------------------------------------------------------------------------------------

class Test_Stand(Evaluate):
    """  
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
 
        self.velocity             = 150 * Units.knots 
        self.throttle             = 1.0
        self.altitude             = 0.0
        self.time                 = 0.0
        self.true_course          = 0.0 * Units.degrees 

        # -------------------------------------------------------------------------------------------------------------- 
        #  Mission Unknowns and Residuals
        # -------------------------------------------------------------------------------------------------------------- 
        ones_row                                  = self.state.ones_row
        self.state.residuals.final_velocity_error = ones_row(1) * 0  
        self.state.unknowns.ground_velocity       = ones_row(1) * 0   

        # -------------------------------------------------------------------------------------------------------------- 
        #  Mission specific processes 
        # --------------------------------------------------------------------------------------------------------------  
        #initialize                         = self.process.initialize
        #initialize.conditions              = Ground.Test_Stand.initialize_conditions
        #iterate                            = self.process.iterate   
        #iterate.conditions.forces_ground   = Update.ground_forces
        #iterate.unknowns.mission           = Unpack_Unknowns.ground
        #iterate.residuals.flight_dynamics  = Residuals.flight_dynamics
        

        initialize                         = self.process.initialize  
        initialize.conditions              = Ground.Test_Stand.initialize_conditions  
        converge                           = self.process.converge 
        converge.solver                    = skip 
        iterate                            = self.process.iterate 
        iterate.unknowns.mission           = skip
        iterate.conditions.aerodynamics    = skip
        iterate.conditions.stability       = skip  
        post_process                       = self.process.post_process  
        post_process.noise                 = skip
        post_process.emissions             = skip        
        
        return


