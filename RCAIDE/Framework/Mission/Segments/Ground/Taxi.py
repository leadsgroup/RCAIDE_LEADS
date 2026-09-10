# RCAIDE/Framework/Analyses/Mission/Segments/Ground/Taxi.py
#
#
# Created: Aug 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from RCAIDE.Framework.Mission.Segments.Evaluate   import Evaluate
from RCAIDE.Framework.Core                        import Units, Data
from RCAIDE.Library.Mission.Segments              import Ground
from RCAIDE.Library.Mission.Common                import Update

# ----------------------------------------------------------------------------------------------------------------------
#  Taxi
# ----------------------------------------------------------------------------------------------------------------------

class Taxi(Evaluate):
    """ Fixed ground speed and altitude over a set distance, with rolling
        friction -- the ground-roll equivalent of Constant_Speed_Constant_Altitude.
    """

    def __defaults__(self):
        """ This sets the default solver flow. Anything in here can be modified after initializing a segment.
        """

        # --------------------------------------------------------------------------------------------------------------
        #   User Inputs
        # --------------------------------------------------------------------------------------------------------------
        self.altitude             = 0.0
        self.air_speed            = 20.0 * Units.knots
        self.distance             = 1.0 * Units.nmi
        self.ground_incline       = 0.0
        self.friction_coefficient = 0.04
        self.true_course          = 0.0 * Units.degrees

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
        initialize                        = self.process.initialize
        initialize.conditions             = Ground.Taxi.initialize_conditions
        iterate                           = self.process.iterate
        iterate.conditions.forces_ground  = Update.ground_forces

        return
