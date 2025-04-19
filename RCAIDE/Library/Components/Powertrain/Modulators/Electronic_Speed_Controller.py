# RCAIDE/Library/Components/Powertrain/Modulators/Electronic_Speed_Controller.py
#  
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
from RCAIDE.Library.Components import Component 
from RCAIDE.Library.Methods.Powertrain.Modulators.Electronic_Speed_Controller.append_esc_conditions   import append_esc_conditions 
 
# ----------------------------------------------------------------------------------------------------------------------
#  Electronic Speed Controller Class
# ---------------------------------------------------------------------------------------------------------------------- 
class Electronic_Speed_Controller(Component):
    """
    Class for modeling electronic speed controllers in electric propulsion systems
    
    Attributes
    ----------
    tag : str
        Identifier for the ESC (default: 'electronic_speed_controller')
        
    efficiency : float
        Power conversion efficiency of the ESC (default: 0.0)

    Notes
    -----
    The Electronic Speed Controller (ESC) regulates power delivery to electric motors,
    controlling motor speed and torque. The efficiency attribute accounts for power
    losses during voltage and current modulation.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Motor
        Electric motor components controlled by the ESC
    """
    
    def __defaults__(self):
        """
        Sets default values for ESC attributes
        
        Notes
        -----
        Initializes the ESC with a default tag and zero efficiency. The efficiency
        should be set to an appropriate value based on the specific ESC being modeled.
        """         

        self.tag              = 'electronic_speed_controller'  
        self.efficiency       = 0.0 

    def append_operating_conditions(self,segment,propulsor): 
        """
        Append ESC operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing state conditions
        propulsor : Component
            Propulsor component associated with this ESC
            
        Notes
        -----
        Updates the segment conditions with ESC-specific parameters including
        power throughput and losses.
        """
        propulsor_conditions =  segment.state.conditions.energy[propulsor.tag]
        append_esc_conditions(self,segment,propulsor_conditions)
        return 