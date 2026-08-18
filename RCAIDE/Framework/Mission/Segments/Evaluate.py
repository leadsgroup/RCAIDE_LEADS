# RCAIDE/Framework/Analyses/Mission/Segment/Evaluate.py
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
from RCAIDE.Library.Mission                    import Common , Solver 
from RCAIDE.Framework.Analyses                 import Process  

# ----------------------------------------------------------------------------------------------------------------------
#  ANALYSES
# ---------------------------------------------------------------------------------------------------------------------- 
class Evaluate(Segment):
    """ Base process class used to analyze a vehicle in each flight segment.

    Attributes
    ----------
    hybrid_power_split_ratio : float or None
        Fraction of propulsive shaft power provided by the electrical motor
        (phi). 0 = all fuel combustion, 1 = all electric. Required for Hybrid
        networks; auto-set for Fuel (0), Electric (1), and Fuel_Cell (1)
        networks if left as None.

    battery_fuel_cell_power_split_ratio : float or None
        Fraction of electrical bus power supplied by batteries vs fuel cells
        (psi). 1 = all battery, 0 = all fuel cell. Required for Hybrid
        networks; auto-set for Fuel (0), Electric (1), and Fuel_Cell (0)
        networks if left as None.
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
        self.temperature_deviation                             = 0.0
        self.sideslip_angle                                    = 0.0
        self.crosswind_speed                                   = 0.0
        self.angle_of_attack                                   = 1.0 *  Units.degree
        self.bank_angle                                        = 0.0
        self.hybrid_power_split_ratio                          = None
        self.battery_fuel_cell_power_split_ratio               = None
        self.initial_battery_conditions                        = Data()
        self.initial_battery_conditions.state_of_charge        = 1.0
        self.initial_battery_conditions.cell_temperature       = None
        self.initial_battery_conditions.charge_throughput      = None
        self.initial_battery_conditions.increment_battery_age  = False
        self.lift_coefficient                                  = None
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
        initialize.conditions              = None 

        # --------------------------------------------------------------         
        #   Converge 
        # -------------------------------------------------------------- 
        converge                           = self.process.converge 
        converge.solver                    = Solver.converge     

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
        iterate.unknowns                         = Process()
        iterate.unknowns.mission                 = Process()
        iterate.unknowns.mission.controls        = Common.Unpack_Unknowns.control_surfaces
        iterate.unknowns.mission.mission         = Common.Unpack_Unknowns.orientation
        
        # Update Conditions
        iterate.conditions = Process()
        iterate.conditions.differentials         = Common.Update.differentials_time
        iterate.conditions.orientations          = Common.Update.orientations
        iterate.conditions.acceleration          = Common.Update.acceleration
        iterate.conditions.angular_acceleration  = Common.Update.angular_acceleration
        iterate.conditions.altitude              = Common.Update.altitude
        iterate.conditions.atmosphere            = Common.Update.atmosphere
        iterate.conditions.gravity               = Common.Update.gravity
        iterate.conditions.freestream            = Common.Update.freestream
        iterate.conditions.network               = Common.Update.network
        iterate.conditions.thrust                = Common.Update.thrust
        iterate.conditions.aerodynamics          = Common.Update.aerodynamics
        iterate.conditions.aerostructures        = Common.Update.aerostructures
        iterate.conditions.weights               = Common.Update.weights
        iterate.conditions.stability             = Common.Update.stability
        iterate.conditions.forces                = Common.Update.forces
        iterate.conditions.moments               = Common.Update.moments
        iterate.conditions.planet_position       = Common.Update.planet_position

        # Solve Residuals
        iterate.residuals                  = Process()
        iterate.residuals.mission          = Process()
        iterate.residuals.network          = Process()
        iterate.residuals.flight_dynamics  = Common.Residuals.flight_dynamics

        # --------------------------------------------------------------  
        #  Post Process   
        # -------------------------------------------------------------- 
        post_process                    = self.process.post_process   
        post_process.inertial_position  = Common.Update.linear_inertial_horizontal_position
        post_process.energy             = Common.Update.energy 
        post_process.aeroacoustics      = Common.Update.aeroacoustics
        post_process.emissions          = Common.Update.emissions
        
        return

