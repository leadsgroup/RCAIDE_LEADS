# RCAIDE/Library/Compoments/Powertrain/Systems/Avionics.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from RCAIDE.Library.Components import Component
from RCAIDE.Library.Methods.Powertrain.Systems.append_avionics_conditions import append_avionics_conditions
 
# ----------------------------------------------------------------------------------------------------------------------
#  Avionics
# ----------------------------------------------------------------------------------------------------------------------            
class Avionics(Component):
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
    RCAIDE.Library.Components.Powertrain.Systems.System
        Base system class
    """        
    def __defaults__(self):
        """
        Sets default values for the avionics system attributes.
        """                 
        self.power_draw = 0.0
        self.tag        = 'Avionics'

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
        append_avionics_conditions(self, segment, bus)
        return
            
    def power(self):
        """
        Calculates the power draw from the avionics system.

        Returns
        -------
        float
            Power draw in Watts

        Notes
        -----
        Sets both the input power and returns the power draw value for use
        in energy calculations.
        """                 
        self.inputs.power = self.power_draw
        return self.power_draw