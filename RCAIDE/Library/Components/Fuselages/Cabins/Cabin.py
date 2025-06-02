# RCAIDE/Components/Fuselages/Cabins/Cabin.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                import Data ,  Units
from RCAIDE.Library.Components.Component  import Container
from RCAIDE.Library.Components            import Component
from RCAIDE.Library.Components.Fuselages.Cabins.Classes import First, Economy

# ---------------------------------------------------------------------------------------------------------------------- 
#  Fuselage
# ---------------------------------------------------------------------------------------------------------------------- 
class Cabin(Component): 
    """
    Aircraft cabin component for modeling passenger compartment configurations and layouts.

    Attributes
    ----------
    tag : str
        Identifier for the cabin component
    type_A_door_length : float
        Length of Type A passenger doors
    galley_lavatory_length : float
        Combined length allocated for galley and lavatory facilities in meters
    emergency_exit_seat_pitch : float
        Required seat pitch spacing near emergency exits in meters
    length : float
        Total cabin length in meters
    wide_body : bool
        Flag indicating if this is a wide-body aircraft cabin configuration
    tail : Data
        Tail section geometric properties
            - fineness_ratio : float
                Ratio of tail length to maximum diameter
    nose : Data
        Nose section geometric properties
            - fineness_ratio : float
                Ratio of nose length to maximum diameter
    classes : Container
        Collection of cabin class configurations (First, Business, Economy)

    Notes
    -----
    The Cabin class serves as the primary container for aircraft passenger 
    compartment modeling. It defines the overall cabin geometry, door 
    configurations, and safety requirements while providing a framework 
    for organizing different passenger class sections.

    Default door and facility dimensions are based on commercial aviation 
    standards. The wide_body flag affects cabin layout calculations and 
    passenger capacity modeling.

    **Definitions**

    'Type A Door'
        Large passenger door meeting regulatory requirements for emergency 
        evacuation, typically 42+ inches wide. Refer to https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC25-17A.pdf
        for additional information on exit door types and dimensions.
    'Fineness Ratio'
        Ratio of length to maximum diameter, affecting aerodynamic properties
    """
    
    def __defaults__(self):
        """
        Sets default values for all fuselage attributes.
        """      
        
        self.tag                                = 'cabin'  
        self.type_A_door_length                 = 36 *  Units.inches
        self.galley_lavatory_length             = 32 *  Units.inches  
        self.emergency_exit_seat_pitch          = 36 *  Units.inches
        self.length                             = 0
        self.wide_body                          = False 
        self.tail                               = Data()
        self.tail.fineness_ratio                = 0 
        self.nose                               = Data() 
        self.nose.fineness_ratio                = 0
        self.classes                            = Container()
        
    def append_cabin_class(self,cabin_class): 

        # Assert database type
        if not (isinstance(cabin_class,RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy) or  \
                isinstance(cabin_class,RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business) or  \
                 isinstance(cabin_class,RCAIDE.Library.Components.Fuselages.Cabins.Classes.First)):
            raise Exception('input component must be of type Cabin_Class')

        # Store data
        self.classes.append(cabin_class)

        return
     