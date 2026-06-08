# RCAIDE/Library/Components/Powertrain/Systems/Avionics.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Systems import Systems
from RCAIDE.Library.Methods.Powertrain.Systems.append_systems_conditions import append_systems_conditions 
 
# ----------------------------------------------------------------------------------------------------------------------
#  Avionics
# ----------------------------------------------------------------------------------------------------------------------            
class Avionics(Systems):
    """
    A class representing aircraft avionics systems and their power requirements.

    Attributes
    ----------
    power_draw : float
        Power consumption of the avionics system, defaults to 0.0
        
    tag : str
        Unique identifier for the avionics system, defaults to 'Avionics'

    Notes
    -----
    The avionics class models the electrical power requirements of aircraft 
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
        The electrical power required by the avionics system during operation
        
    'Bus'
        The electrical distribution system supplying power to the avionics

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Systems.Systems
        Base system class
    """        
    def __defaults__(self):
        """
        Sets default values for the avionics system attributes.
        """                  
        self.tag        = 'Avionics'
        self.power_draw = 0

    def append_operating_conditions(self, segment, bus): 
        """
        Adds operating conditions for the avionics system to a mission segment.

        Parameters
        ----------
        segment : Data
            Mission segment to which conditions are being added
        bus : Data
            Electrical bus supplying power to the avionics
        """
        append_systems_conditions(self, segment, bus)
        return 