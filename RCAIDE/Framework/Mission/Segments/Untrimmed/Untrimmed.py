# RCAIDE/Framework/Analyses/Mission/Segments/Untrimmed/Untrimmed.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Framework.Mission.Segments         import Segment
from RCAIDE.Framework.Mission.Common.Results   import Results
from RCAIDE.Library.Mission                    import Common ,Solver,  Segments
from RCAIDE.Framework.Analyses                 import Process   
from RCAIDE.Library.Methods.skip               import skip 

# ----------------------------------------------------------------------------------------------------------------------
#  Untrimmed
# ---------------------------------------------------------------------------------------------------------------------- 

class Untrimmed(Segment):
    """ Base process class used to analyze a vehicle in each flight segment  
    
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
    
    def __defaults__(self):
        """This sets the default values.
    
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
        self.temperature_deviation                            = 0.0
        self.sideslip_angle                                   = 0.0 
        self.angle_of_attack                                  = 1.0 *  Units.degree
        self.bank_angle                                       = 0.0 
        self.linear_acceleration_x                            = 0.
        self.linear_acceleration_y                            = 0.  
        self.linear_acceleration_z                            = 0. # note that down is positive
        self.roll_rate                                        = 0.
        self.pitch_rate                                       = 0. 
        self.initial_battery_conditions                       = Data()
        self.initial_battery_conditions.cell_temperature      = None 
        self.initial_battery_conditions.state_of_charge       = None
        self.initial_battery_conditions.charge_throughput     = None
        self.initial_battery_conditions.increment_battery_age = False 
        self.yaw_rate                                         = 0.  
        self.state.numerics.number_of_control_points          = 2     
        self.trim_lift_coefficient                            = None
        self.state.conditions.update(Results())
        
        # ---------------------------------------------------------------
        # Define Flight Controls and Residuals 
        # ---------------------------------------------------------------     
        self.flight_dynamics_and_controls()    
        
        # --------------------------------------------------------------
        #   Initialize - before iteration
        # -------------------------------------------------------------- 
        initialize                         = self.process.initialize 
        initialize.expand_state            = Solver.expand_state
        initialize.differentials           = Common.Initialize.differentials_dimensionless 
        initialize.conditions              = Segments.Untrimmed.Untrimmed.initialize_conditions  
        
        # --------------------------------------------------------------          
        #   Iterate  
        # -------------------------------------------------------------- 
        iterate                            = self.process.iterate 
        iterate.initials                   = Process()
        iterate.initials.time              = Common.Initialize.time
        iterate.initials.weights           = Common.Initialize.weights 
        iterate.initials.energy            = Common.Initialize.energy
        iterate.initials.inertial_position = Common.Initialize.inertial_position
        iterate.initials.planet_position   = Common.Initialize.planet_position
        
        
        # Unpack Unknowns
        iterate.unknowns.mission.mission                   = Process()
        
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
        iterate.conditions.weights               = Common.Update.weights
        iterate.conditions.forces                = Common.Update.forces
        iterate.conditions.moments               = Common.Update.moments
        iterate.conditions.planet_position       = skip

        # Solve Residuals 
        iterate.unknowns.mission.controls                = Common.Unpack_Unknowns.control_surfaces
        iterate.unknowns.mission.mission                 = Common.Unpack_Unknowns.orientation  
        iterate.residuals.mission.flight_dynamics        = Common.Residuals.flight_dynamics

        # --------------------------------------------------------------  
        #  Post Process   
        # -------------------------------------------------------------- 
        post_process                    = self.process.post_process   
        post_process.inertial_position  = skip
        post_process.energy             = skip
        post_process.aeroacoustics      = Common.Update.aeroacoustics
        post_process.emissions          = skip
        
        return 