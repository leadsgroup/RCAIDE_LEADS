# RCAIDE/Energy/Peripherals/Payload.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from RCAIDE.Library.Components import Component  
from RCAIDE.Library.Methods.Powertrain.Systems.append_payload_conditions import append_payload_conditions
 
# ----------------------------------------------------------------------------------------------------------------------
#  Avionics
# ----------------------------------------------------------------------------------------------------------------------              
class Payload(Component):
    """
    A payload component model for representing mission cargo and equipment.

    Attributes
    ----------
    tag : str
        Identifier for the payload. Default is 'payload'.
        
    power_draw : float
        Power consumption of the payload. Default is 0.0.

    Notes
    -----
    The Payload class models mission-specific cargo and equipment, including:

    * Mass properties
    * Power requirements
    * Location in vehicle
    * Energy consumption
    * Operating conditions

    The model can represent:

    * Scientific instruments
    * Cargo containers
    * Passenger accommodations
    * Mission-specific equipment
    * Sensor packages
    * Communication systems

    **Major Assumptions**
    
    * Constant power draw during operation
    * Fixed mass and volume
    * Rigid mounting to vehicle
    * No thermal effects on vehicle
    * No aerodynamic effects
    * Steady-state operation

    See Also
    --------
    RCAIDE.Library.Components.Component
    RCAIDE.Library.Methods.Powertrain.Systems.append_payload_conditions
    """          
    def __defaults__(self):
        """This sets the default power draw.

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
        self.tag        = 'payload' 
        self.power_draw = 0.0
         
    def append_operating_conditions(self,segment,bus): 
        append_payload_conditions(self,segment,bus)
        return 