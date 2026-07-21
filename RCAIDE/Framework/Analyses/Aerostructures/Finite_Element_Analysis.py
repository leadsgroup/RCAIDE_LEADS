# RCAIDE/Framework/Analyses/Aerostructures/Finite_Element_Analysis.py
#
# Created:  Mar 2026, S. Sharma
# Modified: Jul 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core                                         import Data
from RCAIDE.Framework.Analyses                                     import Process
from .Aerostructures                                               import Aerostructures
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis import (
    evaluate_surrogate,
    evaluate_no_surrogate,
    build_FEA_surrogates,
)

# ----------------------------------------------------------------------------------------------------------------------
#  Finite_Element_Analysis
# ----------------------------------------------------------------------------------------------------------------------
class Finite_Element_Analysis(Aerostructures):
    """Aerostructural analysis using Euler-Bernoulli FEA on the wing wingbox.

    In surrogate mode (default) the analysis trains once over an AoA × Mach grid
    at initialisation and then evaluates via a RegularGridInterpolator during
    mission analysis.  In direct mode (use_surrogate = False) VLM + FEA are run
    at every mission control point.
    """

    def __defaults__(self):
        self.tag = 'Finite_Element_Analysis'

        # Process
        self.process                      = Process()
        self.process.compute              = Process()
        self.process.compute.structural   = None   # wired in initialize()

        # --- Settings (inherited: discretization=400, load_factor=1.0) ---
        self.settings.use_surrogate       = True

        # --- Training data (populated by copy_structural_training from VLM) ---
        self.training                     = Data()
        self.training.Mach                = None
        self.training.subsonic            = None
        self.training.supersonic          = None
        self.training.transonic           = None

        # --- Surrogate containers ---
        self.surrogates                   = Data()

        # --- Mach blending thresholds used by evaluate_surrogate ---
        self.hsub_min                     = 0.85
        self.hsub_max                     = 0.95
        self.hsup_min                     = 1.05
        self.hsup_max                     = 1.15

    # ------------------------------------------------------------------------------------------------------------------
    def initialize(self, vehicle, aerodynamics=None):
        """Build the FEA surrogate.

        Args:
            vehicle      : RCAIDE vehicle
            aerodynamics : (optional) already-initialised VLM analysis whose training
                           object already contains structural deflection/twist data
                           (populated when VLM was trained with aerostructural_analyses
                           passed to train_VLM_surrogates).  When provided, the
                           structural data is copied directly — no second VLM run.
        """
        use_surrogate = self.settings.use_surrogate

        if use_surrogate:
            if aerodynamics is not None and self.has_structural_training_data(aerodynamics):
                # Reuse the structural data that train_VLM_surrogates already computed
                self.copy_structural_training(aerodynamics)
                print("\n Aerostructural surrogate data copied from VLM training.")

            else:
                raise RuntimeError(
                    "FEA surrogate requires structural training data. "
                    "Call vlm.initialize(vehicle, fea) before fea.initialize(vehicle, vlm)."
                )

            build_FEA_surrogates(self, vehicle)
            self.process.compute.structural = evaluate_surrogate
        else:
            self.process.compute.structural = evaluate_no_surrogate
        return

    def has_structural_training_data(self, aerodynamics):
        """Return True if the VLM training object already contains FEA deflection data."""
        sub = aerodynamics.training.subsonic
        return (sub is not None
                and hasattr(sub, 'deflection_w')
                and len(sub.deflection_w) > 0)

    def copy_structural_training(self, aerodynamics):
        """Copy structural fields from VLM training into self.training."""
        src_sub = aerodynamics.training.subsonic
        dst_sub                      = Data()
        dst_sub.Mach                 = src_sub.Mach
        dst_sub.deflection_u         = src_sub.deflection_u
        dst_sub.deflection_v         = src_sub.deflection_v
        dst_sub.deflection_w         = src_sub.deflection_w
        dst_sub.elastic_twist        = src_sub.elastic_twist
        self.training.angle_of_attack = aerodynamics.training.angle_of_attack
        self.training.Mach            = aerodynamics.training.Mach
        self.training.subsonic        = dst_sub
        self.training.supersonic      = None
        self.training.transonic       = None

    # ------------------------------------------------------------------------------------------------------------------
    def evaluate(self, state, vehicle):
        settings = self.settings
        results  = self.process.compute(state, settings, vehicle)
        return results
