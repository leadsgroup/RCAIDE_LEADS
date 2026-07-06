# RCAIDE/Library/Components/Powertrain/Systems/Flight_Controls.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
from RCAIDE.Library.Methods.Powertrain.Systems.append_systems_conditions import append_systems_conditions
from RCAIDE.Library.Methods.Powertrain.Systems.compute_flight_controls_power_draw import compute_flight_controls_power_draw
 
# ----------------------------------------------------------------------------------------------------------------------
#  Flight_Controls
# ----------------------------------------------------------------------------------------------------------------------            
class Flight_Controls(Systems):
    """
    A class representing aircraft flight_controls systems and their power requirements.

    Attributes
    ----------
    power_draw : float
        Power consumption of the flight_controls system, defaults to 0.0
        
    tag : str
        Unique identifier for the flight_controls system, defaults to 'flight_controls'

    Notes
    -----
    The flight_controls class models the electrical power requirements of aircraft 
    electronics and instruments. This includes:
        * Flight management systems
        * Navigation equipment
        * Communication systems
        * Display systems
    
    **Major Assumptions**
        * Constant power draw during operation
        * Instantaneous power availability
        * No thermal management considerations
    
    **Definitions**

    'Power Draw'
        The electrical power required by the flight_controls system during operation
        
    'Bus'
        The electrical distribution system supplying power to the flight_controls

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Systems.Systems
        Base system class
    """        
    def __defaults__(self):
        """
        Sets default values for the flight_controls system attributes.
        """                  
        self.tag        = 'Flight_Controls' 

    def compute_performance(self, vehicle,state,bus):
        compute_flight_controls_power_draw(self, vehicle,state,bus)
        return