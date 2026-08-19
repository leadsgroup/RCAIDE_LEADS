 # RCAIDE/Framework/Analyses/Aeroacoustics/Semi_Empirical.py
#
# Created:  Jul 2023, M. Clarke
# Modified: Aug 2026, P. Siripun

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.evaluate_aeroacoustics import evaluate_aeroacoustics
from RCAIDE.Framework.Core                                                      import Data
from .Aeroacoustics import Aeroacoustics

# ----------------------------------------------------------------------------------------------------------------------
#  Semi_Empirical
# ----------------------------------------------------------------------------------------------------------------------
class Semi_Empirical(Aeroacoustics):
    """
    This analysis calls RCAIDE's semi-empirical noise models for individual airframe and
    turbofan engine components and sums the results into the total sound pressure level at a
    grid of ground receptors under the flight path.

    Assumptions
    -----------
    * Flat, level ground at the mission's local horizontal-position origin (matches
      segment.state.conditions.frames.inertial.position_vector).
    * Only receptors within settings.noise_receptor_search_radius of the aircraft's current
      ground position are evaluated at each control point.
    * Jet noise assumes no reverse-thrust/idle-approach ground operation (compute_jet_noise's
      velocity-reduction flag is always 0) -- there is no tracked reverse-thrust state in
      conditions to detect that automatically.

    References
    ----------
    [1] Guo, Yueping. "A Semi-Empirical Model for Aircraft Landing Gear Noise Prediction." AIAA 2006-2627.
    [2] Guo, Yueping. "Aircraft Flap Side Edge Noise Modeling and Prediction" (2012)
    [3] Guo, Yueping. "Aircraft Slat Noise Modeling and Prediction" (2010)
    [4] SAE-AIR-5662: Method for Predicting Lateral Attenuation of Airplane Noise
    [5] Enhanced Fan Noise Modeling for Turbofan Engines (NASA)
    [6] Enhanced Core Noise Modeling for Turbofan Engines (NASA)
    """

    def __defaults__(self):
        self.tag                                    = "Semi_Empirical"
        self.settings.noise_hemisphere_radius       = 50
        self.settings.noise_receptor_search_radius  = 30000  # [m] receptors farther than this are skipped per control point

        # empirical model calibration constants, exposed here so they can be tuned/overridden
        # per vehicle without editing the noise model source
        self.settings.landing_gear_noise_parameters = Data(
            Low  = Data(beta=4.5e-8, St0=1.0, sigma=4.0, mu=2.5, q=2.6, h=0.2, A=3.53, B=0.62),
            Mid  = Data(beta=1.5e-8, St0=0.3, sigma=3.0, mu=1.5, q=4.2, h=0.6, A=0.42, B=0.18),
            High = Data(beta=3.2e-5, St0=0.1, sigma=2.0, mu=1.1, q=4.2, h=1.0, A=0.08, B=0.10),
        )
        self.settings.flap_noise_parameters = Data(
            A0=3e-5, mu0=0.7693, mu1=1.0, mu2=0.292, alpha_0=0.008, sigma_f=0.436332,
        )
        self.settings.slat_noise_parameters = Data(
            amplitude=1e-5, St_peak=2.0,
        )
        return

    def evaluate_aeroacoustics(self, segment, vehicle):
        evaluate_aeroacoustics(segment, self.settings, vehicle)
        return
