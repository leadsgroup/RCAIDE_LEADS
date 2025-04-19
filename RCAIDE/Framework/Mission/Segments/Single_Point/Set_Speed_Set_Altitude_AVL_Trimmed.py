# RCAIDE/Framework/Analyses/Mission/Segments/Single_Point/Set_Speed_Set_Altitude_AVL_Trimmed.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
from RCAIDE.Framework.Core                           import Units  
from RCAIDE.Framework.Mission.Segments               import Segment
from RCAIDE.Framework.Mission.Common.Results         import Results
from RCAIDE.Library.Mission                          import Common , Segments
from RCAIDE.Framework.Analyses                       import Process   
from RCAIDE.Library.Mission                          import Common,Segments
from RCAIDE.Library.Methods.skip                     import skip 

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Set_Speed_Set_Altitude
# ---------------------------------------------------------------------------------------------------------------------- 
class Set_Speed_Set_Altitude_AVL_Trimmed(Segment):
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

        
        # --------------------------------------------------------------
        #   State
        # --------------------------------------------------------------
        
        # conditions
        self.temperature_deviation                   = 0.0
        self.sideslip_angle                          = 0.0 
        self.angle_of_attack                         = None
        self.trim_lift_coefficient                   = None        
        self.bank_angle                              = 0.0 
        self.linear_acceleration_x                   = 0.
        self.linear_acceleration_y                   = 0.  
        self.linear_acceleration_z                   = 0. # note that down is positive
        self.roll_rate                               = 0.
        self.pitch_rate                              = 0.  
        self.yaw_rate                                = 0.  
        self.state.numerics.number_of_control_points = 1     
        self.state.conditions.update(Results())
        
        # ---------------------------------------------------------------
        # Define Flight Controls and Residuals 
        # ---------------------------------------------------------------     
        self.flight_dynamics_and_controls()    
        
        # --------------------------------------------------------------
        #   Initialize - before iteration
        # -------------------------------------------------------------- 
        initialize                         = self.process.initialize 
        initialize.expand_state            = skip 
        initialize.differentials           = Common.Initialize.differentials_dimensionless 
        initialize.conditions              = Segments.Single_Point.Set_Speed_Set_Altitude_AVL_Trimmed.initialize_conditions  
        
        # --------------------------------------------------------------          
        #   Iterate  
        # -------------------------------------------------------------- 
        iterate                            = self.process.iterate 
        iterate.initials                   = Process()
        iterate.initials.time              = Common.Initialize.time
        iterate.initials.weights           = Common.Initialize.weights
        iterate.initials.energy            = skip
        iterate.initials.inertial_position = Common.Initialize.inertial_position
        iterate.initials.planet_position   = Common.Initialize.planet_position
        
        
        # Unpack Unknowns
        iterate.unknowns                   = Process()
        iterate.unknowns.mission           = Common.Unpack_Unknowns.orientation  
        
        # Update Conditions
        iterate.conditions = Process()
        iterate.conditions.differentials         = Common.Update.differentials_time
        iterate.conditions.orientations          = Common.Update.orientations
        iterate.conditions.acceleration          = skip 
        iterate.conditions.angular_acceleration  = skip 
        iterate.conditions.altitude              = Common.Update.altitude
        iterate.conditions.atmosphere            = Common.Update.atmosphere
        iterate.conditions.gravity               = Common.Update.gravity
        iterate.conditions.freestream            = Common.Update.freestream 
        iterate.conditions.thrust                = Common.Update.thrust
        iterate.conditions.aerodynamics          = Common.Update.aerodynamics
        iterate.conditions.stability             = Common.Update.stability
        iterate.conditions.weights               = skip # Common.Update.weights
        iterate.conditions.forces                = Common.Update.forces
        iterate.conditions.moments               = Common.Update.moments
        iterate.conditions.planet_position       = skip

        # Solve Residuals 
        #iterate.unknowns.controls                = Common.Unpack_Unknowns.control_surfaces
        #iterate.unknowns.mission                 = Common.Unpack_Unknowns.orientation  
        iterate.residuals.flight_dynamics        = Common.Residuals.flight_dynamics

        # --------------------------------------------------------------  
        #  Post Process   
        # -------------------------------------------------------------- 
        post_process                    = self.process.post_process   
        post_process.inertial_position  = skip
        post_process.energy             = skip
        post_process.noise              = Common.Update.noise
        post_process.emissions          = skip 
                
        return

