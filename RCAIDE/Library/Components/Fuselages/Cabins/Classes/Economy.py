# RCAIDE/Components/Fuselages/Cabins/Cabin_Class.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                import  Units 
from RCAIDE.Library.Components            import Component
from RCAIDE.Library.Components.Component  import Container 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Economy
# ---------------------------------------------------------------------------------------------------------------------- 
class Economy(Component): 
    
    def __defaults__(self):
        """
        Sets default values for economy class portion of the cabin.
        """      
        
        self.tag                                 = 'economy_class'
        self.number_of_passengers                = 0 
        self.number_of_seats_abrest              = 0
        self.number_of_rows                      = 0 
        self.number_of_seats                     = 0 
        self.seat_width                          = 18 *  Units.inches
        self.seat_arm_rest_width                 = 2 *  Units.inches
        self.seat_length                         = 18 *  Units.inches
        self.seat_pitch                          = 32 *  Units.inches
        self.aisle_width                         = 15  *  Units.inches          
        self.galley_lavatory_percent_x_locations = []      
        self.exits                               = Container()
        self.y_offset_distance                   = 0

    def append_exit(self, cabin_exit):
        """
        Adds a pair of emergency exits to the cabin class.

        Parameters
        ----------
        cabin_exit : Exit
            Exit pair to be added, e.g. RCAIDE.Library.Components.Fuselages.Cabins.Exits.Type_A(), with a
            unique tag such as 'front_exit' or 'overwing_exit'
        """
        # Assert database type
        if not isinstance(cabin_exit, RCAIDE.Library.Components.Fuselages.Cabins.Exits.Exit):
            raise Exception('input component must be of type Exit')

        # Store data
        self.exits.append(cabin_exit)

        return
