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
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_cabin_moment_of_inertia import compute_cabin_moment_of_inertia
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity.compute_cabin_center_of_gravity import compute_cabin_center_of_gravity 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Fuselage
# ---------------------------------------------------------------------------------------------------------------------- 
class Cabin(Component): 
    
    def __defaults__(self):
        """
        Sets default values for all fuselage attributes.
        """      
        
        self.tag                       = 'cabin'
        self.number_of_passengers      = 0 
        self.number_of_seats           = 0 
        self.filled_seats_arrangement  = 'random' # ['random','ascending','descending']
        self.type_A_door_length        = 36 *  Units.inches
        self.galley_lavatory_length    = 32 *  Units.inches  
        self.emergency_exit_seat_pitch = 36 *  Units.inches  
        self.length                    = 0
        self.width                     = 0
        self.height                    = 0
        self.wide_body                 = False
        self.wall_thickness            = 0 *  Units.inches
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
      

    def compute_moments_of_inertia(self,vehicle,center_of_gravity=[[0, 0, 0]]): 
        """
        Computes the moment of inertia tensor for the cabin.

        Parameters
        ----------
        center_of_gravity : list, optional
            Reference point coordinates for moment calculation, defaults to [[0, 0, 0]] 

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.moments_of_inertia.compute_fuselage_moment_of_inertia
            Implementation of the moment of inertia calculation
        """
        
        for fuselage in vehicle.fuselages:
            for cabin in fuselage.cabins:
                if cabin.tag == self.tag: 
                    _ , _ = compute_cabin_moment_of_inertia(self,fuselage,center_of_gravity) 
        for wing in vehicle.wings:
            if type(wing) == RCAIDE.Library.Components.Wings.Blended_Wing_Body:
                for cabin in wing.cabins:
                    if cabin.tag == self.tag: 
                        _ , _ = compute_cabin_moment_of_inertia(self,wing,center_of_gravity) 
        return

    def compute_center_of_gravity(self,vehicle): 
        """
        Computes the center of gravity for the cabin.

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.center_of_gravity.compute_fuselage_center_of_gravity
            Implementation of the center of gravity calculation
        """
        
        for fuselage in  vehicle.fuselages:
            for cabin in fuselage.cabins:
                if cabin.tag == self.tag and fuselage.layout_of_passenger_accommodations != None:
                    _  = compute_cabin_center_of_gravity(self, fuselage) 
            
        for wing in vehicle.wings:
            if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
                for cabin in wing.cabins:
                    if cabin.tag == self.tag and wing.layout_of_passenger_accommodations != None: 
                        _  = compute_cabin_center_of_gravity(self, wing)                     
        return           
    
    