# RCAIDE/Compoments/Fuselages/Cabins/Cabin_Class.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
from RCAIDE.Framework.Core                import  Units 
from RCAIDE.Library.Components            import Component 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Cabin_Class
# ---------------------------------------------------------------------------------------------------------------------- 
class Business(Component): 
    
    def __defaults__(self):
        """
        Sets default values for all fuselage attributes.
        """      
        
        self.tag                                 = 'buisness_class' 
        self.number_of_seats_abrest              = 0
        self.aile_width                          = 0
        self.number_of_rows                      = 0
        self.seat_width                          = 0
        self.seat_arm_rest_width                 = 0
        self.seat_length                         = 0
        self.seat_pitch                          = 0
        self.aile_width                          = 18  *  Units.inches          
        self.galley_lavatory_percent_x_locations = []      
        self.emergency_exit_percent_x_locations  = [] 
        self.type_A_exit_percent_x_locations     = []
        self.tail_length                         = 0
        self.tail_taper                          = 0
        self.nose_length                         = 0
        self.nose_taper                          = 0
               
     