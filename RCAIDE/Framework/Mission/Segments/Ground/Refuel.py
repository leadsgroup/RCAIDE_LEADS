# RCAIDE/Framework/Mission/Segments/Ground/Refuel.py
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
class Refuel(Evaluate):
    """ A stationary ground refueling segment of prescribed duration. Each
    Cryogenic_Tank fills at ``nominal_fill_rate`` until its fuel_mass reaches
    ``refuel_target_fill_fraction`` of design_full_liquid_mass, at which point
    the fill cuts off (mirroring Battery_Recharge's cutoff_SOC) -- it does not
    just run a fixed rate for a fixed time and hope it lands on target, since
    ongoing boil-off during the fill would otherwise make it fall short. No
    flight dynamics.
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
        self.altitude                     = 0.0
        self.time                         = 30.0 * Units.minutes
        self.refuel_target_fill_fraction  = 1.0
        self.nominal_fill_rate            = None  # kg/s; None -> sized automatically to comfortably finish within self.time
        self.true_course                  = 0.0 * Units.degrees
        self.ground_operations            = True

        # --------------------------------------------------------------------------------------------------------------
        #  Mission specific processes
        # --------------------------------------------------------------------------------------------------------------
        initialize                       = self.process.initialize
        initialize.conditions            = Ground.Refuel.initialize_conditions
        iterate                          = self.process.iterate
        iterate.unknowns.mission.mission = skip
        iterate.conditions.aerodynamics  = skip
        iterate.conditions.stability     = skip
        post_process                     = self.process.post_process
        post_process.aeroacoustics       = skip
        post_process.emissions           = skip

        return
