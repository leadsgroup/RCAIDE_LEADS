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
        self.true_course          = 0.0 * Units.degrees 

        ## -------------------------------------------------------------------------------------------------------------- 
        ##  Mission Unknowns and Residuals
        ## -------------------------------------------------------------------------------------------------------------- 
        #ones_row_m1                               = self.state.ones_row_m1
        #self.state.residuals.final_velocity_error = 0.0
        #self.state.residuals.force_x              = ones_row_m1(1) * 0.0    
        #self.state.unknowns.elapsed_time          = 30.                        
        #self.state.unknowns.ground_velocity       = ones_row_m1(1) * 0  

        ## -------------------------------------------------------------------------------------------------------------- 
        ##  Mission Conditions 
        ## --------------------------------------------------------------------------------------------------------------          
        #ones_row = self.state.ones_row   

        ## -------------------------------------------------------------------------------------------------------------- 
        ##  Mission specific processes 
        ## --------------------------------------------------------------------------------------------------------------  
        #initialize                         = self.process.initialize
        #initialize.conditions              = Ground.Takeoff.initialize_conditions
        #iterate                            = self.process.iterate   
        #iterate.conditions.forces_ground   = Update.ground_forces
        #iterate.unknowns.mission           = Unpack_Unknowns.ground
        #iterate.residuals.flight_dynamics  = Residuals.flight_dynamics
        
        return


