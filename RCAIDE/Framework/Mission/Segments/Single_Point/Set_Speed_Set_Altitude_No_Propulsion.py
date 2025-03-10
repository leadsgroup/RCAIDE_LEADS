# RCAIDE/Framework/Analyses/Mission/Segments/Single_Point/Set_Speed_Set_Altitude_No_Propulsion.py
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
from RCAIDE.Library.Methods.skip                      import skip 
 
# Package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Set_Speed_Set_Altitude_No_Propulsion
# ---------------------------------------------------------------------------------------------------------------------- 
class Set_Speed_Set_Altitude_No_Propulsion(Evaluate):
    """ This is a segment that is solved using a single point. A snapshot in time.
        We fix the speed and altitude. Throttle is solved from those.
    
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
        self.altitude                                = None
        self.air_speed                               = 10. * Units['km/hr']
        self.distance                                = 10. * Units.km
        self.linear_acceleration_x                   = 0.
        self.linear_acceleration_y                   = 0.  
        self.linear_acceleration_z                   = 0. # note that down is positive
        self.roll_rate                               = 0.
        self.pitch_rate                              = 0.  
        self.yaw_rate                                = 0.  
        self.state.numerics.number_of_control_points = 1   
         
        # -------------------------------------------------------------------------------------------------------------- 
        #  Mission specific processes 
        # --------------------------------------------------------------------------------------------------------------     
        initialize                               = self.process.initialize 
        initialize.expand_state                  = skip
        initialize.differentials                 = skip
        initialize.conditions                    = Segments.Single_Point.Set_Speed_Set_Altitude_No_Propulsion.initialize_conditions 
        iterate                                  = self.process.iterate 
        iterate.initials.energy                  = skip
        iterate.unknowns.controls                = Common.Unpack_Unknowns.control_surfaces
        iterate.unknowns.mission                 = Common.Unpack_Unknowns.orientation  
        iterate.conditions.planet_position       = skip    
        iterate.conditions.acceleration          = skip
        iterate.conditions.angular_acceleration  = skip 
        iterate.conditions.weights               = skip
        iterate.residuals.flight_dynamics        = Common.Residuals.flight_dynamics
        post_process                             = self.process.post_process 
        post_process.inertial_position           = skip     
        
        return

