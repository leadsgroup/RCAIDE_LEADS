# RCAIDE/Framework/Analyses/Mission/Segments/Climb/Constant_Acceleration_Constant_Pitchrate_Constant_Angle.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Framework.Core                            import Units 
from RCAIDE.Framework.Mission.Segments.Evaluate       import Evaluate
from RCAIDE.Library.Mission                           import Common,Segments

# ----------------------------------------------------------------------------------------------------------------------
#  Constant_Acceleration_Constant_Pitchrate_Constant_Angle
# ----------------------------------------------------------------------------------------------------------------------
class Constant_Acceleration_Constant_Pitchrate_Constant_Angle(Evaluate):
    """ Vehicle accelerates at a constant rate between two airspeeds.
    
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
        
        # --------------------------------------------------------------
        #   User Inputs
        # --------------------------------------------------------------
        self.altitude_start         = None
        self.altitude_end           = None
        self.air_speed_start        = None
        self.climb_angle            = 0.0 * Units['rad'] 
        self.acceleration           = 1.  * Units['m/s/s'] 
        self.pitch_initial          = None
        self.pitch_final            = 0.0 * Units['rad']
        self.true_course            = 0.0 * Units.degrees  
        
        # -------------------------------------------------------------------------------------------------------------- 
        #  Mission specific processes 
        # --------------------------------------------------------------------------------------------------------------  
        initialize                         = self.process.initialize 
        initialize.conditions              = Segments.Climb.Constant_Acceleration_Constant_Pitchrate_Constant_Angle.initialize_conditions  
        iterate                            = self.process.iterate  
        iterate.unknowns.mission           = Common.Unpack_Unknowns.orientation
        iterate.unknowns.controls          = Common.Unpack_Unknowns.control_surfaces
        iterate.residuals.flight_dynamics  = Common.Residuals.flight_dynamics
        
        return

