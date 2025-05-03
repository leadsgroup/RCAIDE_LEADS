# RCAIDE/Framework/Analyses/Mission/Segments/Ground/Takeoff.py
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

# ----------------------------------------------------------------------------------------------------------------------
#  Takeoff
# ----------------------------------------------------------------------------------------------------------------------

class Takeoff(Evaluate):
    """ Segment for takeoff. Integrates equations of motion
        including rolling friction.
        
        Assumptions:
        Notes Regarding Friction Coefficients
        Dry asphalt or concrete: .04 brakes off, .4 brakes on
        Wet asphalt or concrete: .05 brakes off, .225 brakes on
        Icy asphalt or concrete: .02 brakes off, .08 brakes on
        Hard turf:               .05 brakes off, .4 brakes on
        Firm dirt:               .04 brakes off, .3 brakes on
        Soft turf:               .07 brakes off, .2 brakes on
        Wet grass:               .08 brakes off, .2 brakes on
        
        Source: General Aviation Aircraft Design: Applied Methods and Procedures,
        by Snorri Gudmundsson, copyright 2014, published by Elsevier, Waltham,
        MA, USA [p.938]
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

        self.ground_incline       = 0.0 
        self.velocity_start       = None
        self.velocity_end         = 150 * Units.knots
        self.friction_coefficient = 0.04
        self.throttle             = 1.0
        self.altitude             = 0.0
        self.true_course          = 0.0 * Units.degrees 

        # -------------------------------------------------------------------------------------------------------------- 
        #  Mission Unknowns and Residuals
        # -------------------------------------------------------------------------------------------------------------- 
        ones_row_m1                               = self.state.ones_row_m1
        self.state.residuals.final_velocity_error = 0.0
        self.state.residuals.force_x              = ones_row_m1(1) * 0.0    
        self.state.unknowns.elapsed_time          = 30.                        
        self.state.unknowns.ground_velocity       = ones_row_m1(1) * 0  

        # -------------------------------------------------------------------------------------------------------------- 
        #  Mission Conditions 
        # --------------------------------------------------------------------------------------------------------------          
        ones_row = self.state.ones_row  
        self.state.conditions.ground                              = Data()
        self.state.conditions.ground.incline                      = ones_row(1) * 0.0
        self.state.conditions.ground.friction_coefficient         = ones_row(1) * 0.0
        self.state.conditions.frames.inertial.ground_force_vector = ones_row(3) * 0.0  

        # -------------------------------------------------------------------------------------------------------------- 
        #  Mission specific processes 
        # --------------------------------------------------------------------------------------------------------------  
        initialize                         = self.process.initialize
        initialize.conditions              = Ground.Takeoff.initialize_conditions
        iterate                            = self.process.iterate   
        iterate.conditions.forces_ground   = Update.ground_forces
        iterate.unknowns.mission           = Unpack_Unknowns.ground
        iterate.residuals.flight_dynamics  = Residuals.flight_dynamics
        
        return


