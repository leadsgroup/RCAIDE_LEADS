# RCAIDE/Library/Components/Powertrain/Distributors/Fuel_Line.py 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
from RCAIDE.Library.Components                                import Component
from RCAIDE.Library.Components.Component                      import Container    
from RCAIDE.Library.Methods.Powertrain.Distributors.Fuel_Line import *

# ----------------------------------------------------------------------------------------------------------------------
#  Fuel Line
# ---------------------------------------------------------------------------------------------------------------------- 
class Fuel_Line(Component):
    """
    Class for managing fuel distribution between aircraft fuel system components
    
    Attributes
    ----------
    tag : str
        Identifier for the fuel line (default: 'fuel_line')
        
    fuel_tanks : Container
        Collection of fuel tanks connected to this line
        
    assigned_propulsors : list
        List of propulsion systems supplied by this fuel line
        
    active : bool
        Flag indicating if the fuel line is operational (default: True)
        
    efficiency : float
        Fuel transfer efficiency (default: 1.0)

    Notes
    -----
    The fuel line manages fuel distribution between tanks and engines, handling
    fuel transfer and flow control. It supports multiple fuel tanks and propulsors
    in various aircraft configurations.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks
        Fuel tank components
    RCAIDE.Library.Components.Powertrain.Propulsors
        Aircraft propulsion system components
    """ 
    
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """          
        self.tag                           = 'fuel_line'  
        self.fuel_tanks                    = Container() 
        self.assigned_propulsors           = [] 
        self.assigned_converters           = []
        self.converters                    = Container()       
        self.active                        = True 
        self.efficiency                    = 1.0
        

    def append_operating_conditions(self, segment):
        """
        Append operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing operating conditions
        """
        append_fuel_line_conditions(self, segment)
        return

        
    def append_segment_conditions(self, segment):
        """
        Append segment-specific conditions to the bus
        
        Parameters
        ----------
        conditions : Data
            Container for segment conditions
        segment : Segment
            Flight segment data
        """
        append_fuel_line_segment_conditions(self, segment)
        return    
        