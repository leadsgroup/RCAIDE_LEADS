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
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia import compute_cuboid_moment_of_inertia
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Fuselage
# ---------------------------------------------------------------------------------------------------------------------- 
class Cabin(Component): 
    
    def __defaults__(self):
        """
        Sets default values for all fuselage attributes.
        """      
        
        self.tag                       = 'cabin' 
        self.number_of_seats           = 0 
        self.filled_seats_arrangement  = 'random' # ['random','ascending','descending']
        self.type_A_door_length        = 36 *  Units.inches
        self.galley_lavatory_length    = 32 *  Units.inches  
        self.emergency_exit_seat_pitch = 36 *  Units.inches 
        self.offset_x                  = 0
        self.length                    = 0
        self.width                     = 0
        self.height                    = 0
        self.wide_body                 = False 
        self.tail                      = Data()
        self.tail.fineness_ratio       = 0 
        self.nose                      = Data() 
        self.nose.fineness_ratio       = 0
        self.classes                   = Container()
        
    def append_cabin_class(self,cabin_class): 

        # Assert database type
        if not (isinstance(cabin_class,RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy) or  \
                isinstance(cabin_class,RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business) or  \
                 isinstance(cabin_class,RCAIDE.Library.Components.Fuselages.Cabins.Classes.First)):
            raise Exception('input component must be of type Cabin_Class')

        # Store data
        self.classes.append(cabin_class)

        return
     

    def compute_moment_of_inertia(self, center_of_gravity=[[0, 0, 0]], fuel_flag=False): 
        """
        Computes the moment of inertia tensor for the cabin.

        Parameters
        ---------- 
        center_of_gravity : list, optional
            Reference point coordinates, defaults to [[0, 0, 0]]
        
        Returns
        -------
        ndarray
            3x3 moment of inertia tensor
        """
        if self.height == 0:
            self.height = self.width
        I = compute_cuboid_moment_of_inertia(self.origin, self.mass_properties.mass,self.length,self.width,self.height, inner_length = 0, width_inner = 0, height_inner = 0, center_of_gravity = np.array([[0,0,0]]))  
        return I           