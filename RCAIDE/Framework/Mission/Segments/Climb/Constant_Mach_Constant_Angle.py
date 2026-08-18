# RCAIDE/Framework/Analyses/Mission/Segments/Climb/Constant_Mach_Constant_Angle.py
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
# Constant_Mach_Constant_Angle
# ---------------------------------------------------------------------------------------------------------------------- 
class Constant_Mach_Constant_Angle(Evaluate):
    """ Climb at a constant mach number and at a constant angle.
        This segment takes longer to solve than most because it has extra unknowns and residuals
    
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
        self.mach_number       = None
        self.true_course       = 0.0 * Units.degrees 

        # -------------------------------------------------------------------------------------------------------------- 
        #  Unique Mission Unknowns and Residuals
        # --------------------------------------------------------------------------------------------------------------  
        self.assigned_control_variables.altitude.active = True
        
        # -------------------------------------------------------------------------------------------------------------- 
        #  Mission specific processes 
        # --------------------------------------------------------------------------------------------------------------   
        initialize                                 = self.process.initialize  
        initialize.differentials_altitude          = Common.Initialize.differentials_altitude
        initialize.conditions                      = Segments.Climb.Constant_Mach_Constant_Angle.initialize_conditions  
        iterate                                    = self.process.iterate  
        iterate.conditions.differentials           = Segments.Climb.Constant_Mach_Constant_Angle.update_differentials 
        iterate.unknowns.mission.mission           = Common.Unpack_Unknowns.orientation
        iterate.unknowns.kinematics                = Segments.Climb.Constant_Mach_Constant_Angle.initialize_conditions
        iterate.residuals.mission.altitude         = Segments.Climb.Constant_Mach_Constant_Angle.residual_altitude
          
        return

