# RCAIDE/Framework/Analyses/Mission/Segments/Climb/Constant_Speed_Constant_Angle.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Framework.Core                                     import Units 
from RCAIDE.Framework.Mission.Segments.Evaluate       import Evaluate
from RCAIDE.Library.Mission                          import Common,Segments

# ----------------------------------------------------------------------------------------------------------------------
#  Constant_Speed_Constant_Angle
# ---------------------------------------------------------------------------------------------------------------------- 
class Constant_Speed_Constant_Angle(Evaluate):
    """ A basic constant true airspeed climb at a constant angle. Usually used for certification to ensure a vehicle
        can clear and obstacle.
    
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
        self.altitude_start    = None # Optional
        self.altitude_end      = 10. * Units.km
        self.climb_angle       = 3.  * Units.deg
        self.air_speed         = 100 * Units.m / Units.s
        self.true_course       = 0.0 * Units.degrees       
                
        # -------------------------------------------------------------------------------------------------------------- 
        #  Mission specific processes 
        # --------------------------------------------------------------------------------------------------------------   
        initialize                         = self.process.initialize  
        initialize.differentials_altitude  = Common.Initialize.differentials_altitude
        initialize.conditions              = Segments.Climb.Constant_Speed_Constant_Angle.initialize_conditions  
        iterate                            = self.process.iterate
        iterate.residuals.mission.flight_dynamics  = Common.Residuals.flight_dynamics
        iterate.unknowns.mission.mission           = Common.Unpack_Unknowns.orientation   
    
        return
       