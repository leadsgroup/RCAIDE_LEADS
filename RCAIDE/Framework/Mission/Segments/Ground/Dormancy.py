# RCAIDE/Framework/Mission/Segments/Ground/Dormancy.py
#
#
# Created: Aug 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Mission.Segments.Evaluate        import Evaluate
from RCAIDE.Framework.Core                              import Units
from RCAIDE.Library.Mission.Segments                    import Ground
from RCAIDE.Library.Methods.skip                        import skip

# ----------------------------------------------------------------------------------------------------------------------
#  SEGMENT
# ----------------------------------------------------------------------------------------------------------------------
class Dormancy(Evaluate):
    """ A stationary ground-hold segment of prescribed duration (gate dormancy,
    turnaround, overnight parking). No flight dynamics; energy-network sources
    still evolve over the held duration via the normal network/energy update.
    """

    # ------------------------------------------------------------------
    #   Data Defaults
    # ------------------------------------------------------------------

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
        #   User Inputs
        # --------------------------------------------------------------
        self.altitude          = 0.0
        self.time              = 1.0 * Units.seconds
        self.true_course       = 0.0 * Units.degrees
        self.ground_operations = True

        # --------------------------------------------------------------------------------------------------------------
        #  Mission specific processes
        # --------------------------------------------------------------------------------------------------------------
        initialize                       = self.process.initialize
        initialize.conditions            = Ground.Dormancy.initialize_conditions
        iterate                          = self.process.iterate
        iterate.unknowns.mission.mission = skip
        iterate.conditions.aerodynamics  = skip
        iterate.conditions.stability     = skip
        post_process                     = self.process.post_process
        post_process.aeroacoustics       = skip
        post_process.emissions           = skip

        return
