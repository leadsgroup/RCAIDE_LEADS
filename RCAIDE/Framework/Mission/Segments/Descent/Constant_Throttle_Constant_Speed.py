# RCAIDE/Framework/Analyses/Mission/Segments/Descent/Constant_Throttle_Constant_Speed.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Framework.Core                       import Units 
from RCAIDE.Framework.Mission.Segments.Evaluate  import Evaluate 
from RCAIDE.Library.Mission                      import Common,Segments
from RCAIDE.Framework.Analyses                   import Process  

# ----------------------------------------------------------------------------------------------------------------------
# Constant_Throttle_Constant_Speed
# ---------------------------------------------------------------------------------------------------------------------- 
class Constant_Throttle_Constant_Speed(Evaluate):
    """ Descent at a constant throttle setting and true airspeed. This segment may not always converge as the vehicle 
        could be deficient in thrust.
    
        Assumptions:
        You set a reasonable throttle setting that can provide enough thrust.
        
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
        self.throttle          = 0.5
        self.air_speed         = None
        self.true_course       = 0.0 * Units.degrees       

        # -------------------------------------------------------------------------------------------------------------- 
        #  Mission specific processes 
        # --------------------------------------------------------------------------------------------------------------      
        initialize                         = self.process.initialize   
        initialize.conditions              = Segments.Descent.Constant_Throttle_Constant_Speed.initialize_conditions 
        
        iterate                            = self.process.iterate 
         
        # Update Conditions
        iterate.conditions = Process()
        iterate.conditions.velocities                 = Segments.Descent.Constant_Throttle_Constant_Speed.update_velocity_vector_from_wind_angle 
        iterate.conditions.angles                     = Segments.Descent.Constant_Throttle_Constant_Speed.unpack_body_angle  
        iterate.conditions.differentials_altitude     = Segments.Descent.Constant_Throttle_Constant_Speed.update_differentials_altitude  
        iterate.conditions.differentials              = Common.Update.differentials_time 
        iterate.conditions.orientations               = Common.Update.orientations   
        iterate.conditions.acceleration               = Common.Update.acceleration          
        iterate.conditions.atmosphere                 = Common.Update.atmosphere
        iterate.conditions.gravity                    = Common.Update.gravity
        iterate.conditions.freestream                 = Common.Update.freestream 
        iterate.conditions.energy                     = Common.Update.thrust
        iterate.conditions.aerodynamics               = Common.Update.aerodynamics
        iterate.conditions.stability                  = Common.Update.stability
        iterate.conditions.weights                    = Common.Update.weights
        iterate.conditions.forces                     = Common.Update.forces
        iterate.conditions.moments                    = Common.Update.moments
        iterate.conditions.planet_position            = Common.Update.planet_position
        iterate.residuals.flight_dynamics             = Common.Residuals.flight_dynamics 
        
        return

