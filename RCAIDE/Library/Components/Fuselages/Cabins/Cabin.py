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

# ---------------------------------------------------------------------------------------------------------------------- 
#  Fuselage
# ---------------------------------------------------------------------------------------------------------------------- 
class Cabin(Component): 
    
    def __defaults__(self):
        """
        Sets default values for all fuselage attributes.
        """      
        
        self.tag                                = 'cabin'
        self.number_of_passengers               = 1.0
        self.type_A_door_length                 = 36 *  Units.inches
        self.galley_lavatory_length             = 32 *  Units.inches  
        self.emergency_exit_seat_pitch          = 36 *  Units.inches
        self.layout_of_passenger_accommodations = Data()
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
     