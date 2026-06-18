# RCAIDE/Framework/Analyses/Weights/Weights.py 
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
import importlib

from RCAIDE.Framework.Core     import Data
from RCAIDE.Framework.Analyses import Analysis  

# ----------------------------------------------------------------------
#  Analysis
# ---------------------------------------------------------------------- 
class Weights(Analysis):
    """ This is a class that call the functions that computes the weight of 
    an aircraft depending on its configration
    
    Assumptions:
        None

    Source:
        N/A

    Inputs:
        None
        
    Outputs:
        None

    Properties Used:
         N/A
    """
    def __defaults__(self):
        """Set default values and settings for the weights analysis.

        Notes
        -----
        **Settings**

        ``run_weights_analysis`` (default ``True``)
            Perform a full OEW buildup every call. Takeoff weight is always
            recomputed as ``OEW + payload + fuel``. Set to ``False`` only when
            you want to bypass the buildup entirely and rely on values already
            on the vehicle (e.g. a hand-calculated fixed weight).

        ``overwrite_operating_empty_weight`` (default ``True``)
            Replace ``mass_properties.operating_empty`` with the value derived
            from the weight breakdown after each evaluation.

        ``iterate_mtow`` (default ``False``)
            Iterate MTOW to satisfy the MTOW capacity fraction constraint.
            Requires ``settings.mtow_capacity_fraction`` to be set (0–1).
            Significantly increases computational cost; leave ``False``
            for most optimizations.

        ``run_center_of_gravity_analysis`` (default ``False``)
            Compute and store the vehicle CG after the weight buildup.

        ``run_moments_of_inertia_analysis`` (default ``False``)
            Compute and store the vehicle MOI tensor after the weight buildup.

        ``write_mass_properties`` (default ``False``)
            Write a weight breakdown Excel report alongside the script output.

        ``weight_correction_factors``
            Multiplicative scale factors applied to individual weight components
            after the buildup (e.g. 0.9 applies a 10 % weight reduction).

        ``weight_correction_additions``
            Additive corrections applied to individual weight components after
            the buildup.
        """
        self.tag                                                    = 'weights'
        self.method                                                 = None
        self.aircraft_type                                          = None
        self.propulsion_architecture                                = None
        self.print_weight_analysis_report                           = True
        self.settings                                               = Data()
        self.settings.overwrite_operating_empty_weight              = True
        self.settings.run_weights_analysis                          = True
        self.settings.run_center_of_gravity_analysis                = False
        self.settings.run_moments_of_inertia_analysis               = False
        self.settings.update_max_fuel_mass                          = False
        self.settings.update_fuel_mass                              = False
        self.settings.write_mass_properties                         = False
        self.settings.iterate_mtow                                  = False
        self.settings.mtow_capacity_fraction                        = 0.0
    
        self.settings.weight_correction_factors                     = Data()
        self.settings.weight_correction_factors.empty               = Data()
        self.settings.weight_correction_factors.empty.propulsion    = Data()
        self.settings.weight_correction_factors.empty.structural    = Data()
        self.settings.weight_correction_factors.empty.systems       = Data()

        self.settings.weight_correction_additions                   = Data()
        self.settings.weight_correction_additions.empty             = Data()
        self.settings.weight_correction_additions.empty.propulsion  = Data()
        self.settings.weight_correction_additions.empty.structural  = Data()
        self.settings.weight_correction_additions.empty.systems     = Data()
        self.settings.weight_correction_additions.operational_items = Data()
        
    def evaluate(self, vehicle):
        """Evaluate the weight analysis.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        results 
        """
        #unpack
        compute_module = importlib.import_module(f"RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.{self.propulsion_architecture}.{self.aircraft_type}.{self.method}.compute_operating_empty_weight")

        compute_operating_empty_weight = getattr(compute_module, "compute_operating_empty_weight")
        
        # Call the function
        results = compute_operating_empty_weight(vehicle, self.settings) 
        vehicle.mass_properties.weight_breakdown = results 
        return results        