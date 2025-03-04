# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
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
        """This sets the default values and methods for the weights analysis.

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
        self.tag           = 'weights' 
        self.method        = None
        self.vehicle       = None
        self.aircraft_type = None
        self.propulsion_architecture = None
        self.settings      = Data()
        
    def evaluate(self):
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
        vehicle = self.vehicle 


        
        try:
            compute_module = importlib.import_module(f"RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.{self.propulsion_architecture}.{self.aircraft_type}.{self.method}.compute_operating_empty_weight")
        except:
            raise Exception('Aircraft Type or Weight Buildup Method do not exist!')
        compute_operating_empty_weight = getattr(compute_module, "compute_operating_empty_weight")
        # Call the function
        results = compute_operating_empty_weight(vehicle, self.settings)


        # storing weigth breakdown into vehicle
        vehicle.weight_breakdown = results

        # updating empty weight
        vehicle.mass_properties.operating_empty = results.empty.total

        # done!
        return results        