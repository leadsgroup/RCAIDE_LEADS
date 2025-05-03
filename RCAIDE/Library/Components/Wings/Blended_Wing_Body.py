# RCAIDE/Components/Wings/Blended_Wing_Body.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import Data, Container
from .Main_Wing import Main_Wing

# python imports 
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Blended_Wing_Body
# ----------------------------------------------------------------------------------------------------------------------  
class Blended_Wing_Body(Main_Wing):
    """
    A blended wing body (BWB) design that smoothly integrates the wing and fuselage 
    into a single lifting body configuration.

    Attributes
    ----------
    tag : str
        Unique identifier for the BWB fuselage component, defaults to 'bwb_fuselage'
    
    aft_centerbody_area : float
        Cross-sectional area of the aft centerbody section in square meters
        
    aft_centerbody_taper : float
        Taper ratio of the aft centerbody section, defined as the ratio of tip 
        to root chord lengths
        
    cabin_area : float
        Total available cabin floor area in square meters

    Notes
    -----
    The blended wing body design offers several advantages over conventional tube-and-wing
    configurations:
    
    * Reduced wetted area leading to lower skin friction drag
    * Improved lift-to-drag ratio due to the lifting body design
    * Potential for increased internal volume and better weight distribution

    **Definitions**

    'Centerbody'
        The central section of the BWB that houses the passenger cabin and cargo hold
        
    'Aft Centerbody'
        The rear section of the centerbody that transitions into the outer wing sections

    See Also
    --------
    RCAIDE.Library.Components.Fuselages.Fuselage
        Base fuselage class that provides common functionality
    """
    
    def __defaults__(self):
        """
        Sets the default values for the BWB fuselage component attributes.

        Notes
        -----
        This method initializes all required attributes with default values. Users should 
        modify these values based on their specific design requirements after instantiation.
        """      
          
        self.tag                                    = 'blended_wing_body'  
                 
        self.areas.center_body                      = 0.0        
        self.areas.aft_centerbody                   = 0.0
        self.aft_centerbody_taper                   = 0.0
        self.cabin_area                             = 0.0
        self.cabin_offset                           = 0.0
          
        self.number_of_passengers                   = 0.0  
        self.layout_of_passenger_accommodations     = None 
 
        self.effective_diameter                     = 0.0
        self.width                                  = 0.0  
        
        self.heights                                = Data() 
        self.heights.maximum                        = 0.0
        self.heights.at_quarter_length              = 0.0
        self.heights.at_three_quarters_length       = 0.0
        self.heights.at_wing_root_quarter_chord     = 0.0
        self.heights.at_vertical_root_quarter_chord = 0.0 
        
        self.lengths                                = Data()     
        self.lengths.nose                           = 0.0
        self.lengths.tail                           = 0.0
        self.lengths.total                          = 0.0 

        self.fineness                               = Data() 
        self.fineness.nose                          = 0.0 
        self.fineness.tail                          = 0.0
        
        self.x_rotation                             = 0.0
        self.y_rotation                             = 0.0
        self.z_rotation                             = 0.0    
        self.cabins                                 = Container() 

    
    def append_cabin(self,cabin):
        """
        Adds a new segment to the fuselage's segment container.

        Parameters
        ----------
        segment : Data
            Fuselage segment to be added
        """

        # Assert database type
        if not isinstance(cabin,RCAIDE.Library.Components.Fuselages.Cabins.Cabin):
            raise Exception('input component must be of type Cabin')

        # Store data
        self.cabins.append(cabin)

        return      