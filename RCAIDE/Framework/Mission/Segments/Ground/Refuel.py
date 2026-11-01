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
    """ A stationary ground refueling segment of prescribed duration. Each fuel tank fills until it reaches
    its target, then the fill cuts off for the rest of the segment. No flight dynamics.

    Attributes
    ----------
    refuel_target_fill_fraction : float or None
        Target fill as a fraction of the tank's full liquid mass. None refuels to the tank's mission fuel load,
        which the mass_properties Pre_Process limits to MTOW.
    nominal_fill_rate : float or None
        Fill rate [kg/s]. None sizes it to finish within ``time``.
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
        self.refuel_target_fill_fraction  = None
        self.nominal_fill_rate            = None
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
