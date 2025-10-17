# RCAIDE/Library/Components/Powertrain/Modulators/DC_to_DC_Converter.py
#  
# Created:  Sep. 2025, M. Guidotti 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
from RCAIDE.Library.Components import Component 
from RCAIDE.Library.Methods.Powertrain.Modulators.DC_to_DC_Converter.append_dcdc_conditions   import append_dcdc_conditions 
from RCAIDE.Library.Methods.Powertrain.Modulators.DC_to_DC_Converter.compute_dcdc_performance import compute_dcdc_performance
 
# ----------------------------------------------------------------------------------------------------------------------
#  DC_to_DC_Converter Class
# ---------------------------------------------------------------------------------------------------------------------- 
class DC_to_DC_Converter(Component):
    """
    Class for modeling DC_to_DC_Converters in electric propulsion systems
    
    Attributes
    ----------
    tag : str
        Identifier for the DC_to_DC_Converter (default: 'DC_to_DC_Converter')
        
    efficiency : float
        Power conversion efficiency of the DC_to_DC_Converter (default: 0.0)

    Notes
    -----
    The DC_to_DC_Converter

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Motor
        Electric motor components controlled by the DC_to_DC_Converter
    """
    
    def __defaults__(self):
        """
        Sets default values for DC_to_DC_Converter attributes
        
        Notes
        -----
        Initializes the DC_to_DC_Converter with a default tag and zero efficiency. The efficiency
        should be set to an appropriate value based on the specific DC_to_DC_Converter being modeled.
        """         

        self.tag                   = 'DC_to_DC_converter'  
        self.bus_voltage           = None
        self.efficiency            = 1.0 
        self.assigned_distributors = []

    def append_operating_conditions(self,segment): 
        """
        Append DC_to_DC_Converter operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing state conditions
        propulsor : Component
            Propulsor component associated with this DC_to_DC_Converter
            
        Notes
        -----
        Updates the segment conditions with DC_to_DC_Converter-specific parameters including
        power throughput and losses.
        """ 
        append_dcdc_conditions(self,segment)
        return 
    
    def compute_performance(self,state):

        P_mech,P_elec,stored_results_flag,stored_modulator_tag =  compute_dcdc_performance(self,state)
        return P_mech,P_elec,stored_results_flag,stored_modulator_tag