# RCAIDE/Framework/Analyses/Mission/Segments/Cruise/Curved_Constant_Radius_Constant_Speed_Constant_Altitude.py
# 
# 
# Created:  September 2024, A. Molloy, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
from RCAIDE.Framework.Mission.Segments.Evaluate   import Evaluate 
from RCAIDE.Framework.Core                        import Units   
from RCAIDE.Library.Mission                       import Common,Segments 
from RCAIDE.Library.Methods.skip                  import skip 

# ----------------------------------------------------------------------------------------------------------------------
#  Curved_Constant_Radius_Constant_Speed_Constant_Altitude
# ----------------------------------------------------------------------------------------------------------------------  

class Curved_Constant_Radius_Constant_Speed_Constant_Altitude(Evaluate):
    """ Curved path with fixed true airspeed and altitude and a set sector arc with a constant radius.
       
        Assumptions:
        Constant radius
        Constant speed
        Constant altitude
        
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
        # User Inputs
        # -------------------------------------------------------------------------------------------------------------- 
        self.altitude          = None
        self.air_speed         = None 
        self.turn_radius       = None
        self.turn_angle        = 0.0 * Units.degrees # + indicated right hand turn, negative indicates left-hand turn defaults to straight flight
        self.true_course       = 0.0 * Units.degrees 

        # -------------------------------------------------------------------------------------------------------------- 
        #  Mission specific processes 
        # -------------------------------------------------------------------------------------------------------------- 
        initialize                         = self.process.initialize  
        initialize.conditions              = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude.initialize_conditions  
        iterate                            = self.process.iterate     
        iterate.conditions.acceleration    = skip 
        iterate.unknowns.mission           = Common.Unpack_Unknowns.orientation
        iterate.unknowns.controls          = Common.Unpack_Unknowns.control_surfaces   
        iterate.residuals.flight_dynamics  = Common.Residuals.flight_dynamics 
        post_process                       = self.process.post_process 
        post_process.inertial_position     = Common.Update.curvilinear_inertial_horizontal_position
 
        return

