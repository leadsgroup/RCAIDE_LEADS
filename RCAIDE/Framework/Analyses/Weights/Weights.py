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
        self.tag                                     = 'weights' 
        self.method                                  = None
        self.vehicle                                 = None
        self.aircraft_type                           = None
        self.propulsion_architecture                 = None 
        self.settings                                = Data()
        self.settings.update_mass_properties         = True
        self.settings.update_center_of_gravity       = True
        self.settings.update_moment_of_inertia       = True
        self.print_weight_analysis_report            = True
        
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
        
        if self.aircraft_type ==  None:
            raise Exception('Specify Aircraft Type. Current options are: "Transport", "BWB", "General_Aviation" and "VTOL" ')
        
        try:
            compute_module = importlib.import_module(f"RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.{self.propulsion_architecture}.{self.aircraft_type}.{self.method}.compute_operating_empty_weight")
            if self.print_weight_analysis_report:
                print("\nPerforming Weights Analysis")
                print("--------------------------------------------------------")
                print("Propulsion Architecture:", self.propulsion_architecture)
                print("Aircraft Type          :", self.aircraft_type)
                print("Method                 :", self.method)
                
                if  self.settings.update_mass_properties:
                    print("Aircraft operating empty weight will be overwritten")
                if  self.settings.update_center_of_gravity:
                    print("Aircraft center of gravity location will be overwritten")
                if  self.settings.update_moment_of_inertia:
                    print("Aircraft moment of intertia tensor will be overwritten")  
        except:
            raise Exception('Aircraft Type or Weight Buildup Method do not exist!')
        compute_operating_empty_weight = getattr(compute_module, "compute_operating_empty_weight")
        
        # Call the function
        results = compute_operating_empty_weight(vehicle, self.settings)

        # storing weigth breakdown into vehicle
        vehicle.weight_breakdown = results

        # updating empty weight
        if self.settings.update_mass_properties: 
            vehicle.mass_properties.operating_empty = results.empty.total

        # done!
        return results        