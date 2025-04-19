# RCAIDE/Compoments/Fuselages/Tube_Fuselage.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports     
from .Fuselage import Fuselage
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_fuselage_moment_of_inertia import  compute_fuselage_moment_of_inertia
 
# ---------------------------------------------------------------------------------------------------------------------- 
#  Tube_Fuselage
# ----------------------------------------------------------------------------------------------------------------------  
class Tube_Fuselage(Fuselage):
    """
    A conventional cylindrical fuselage design commonly used in commercial and general 
    aviation aircraft.

    Attributes
    ----------
    tag : str
        Unique identifier for the fuselage component, defaults to 'tube_fuselage'

    Notes
    -----
    The tube fuselage is the most common configuration in modern aircraft design.
    
    **Major Assumptions**
    
    * Cross-sections are primarily circular or near-circular
    * Smooth transitions between different diameter sections
    
    **Definitions**

    'Tube Fuselage'
        A fuselage design with predominantly circular cross-sections, typically
        consisting of a constant-diameter center section with tapered nose and
        tail sections

    See Also
    --------
    RCAIDE.Library.Components.Fuselages.Fuselage
        Base class providing core fuselage functionality
    RCAIDE.Library.Components.Fuselages.Blended_Wing_Body_Fuselage
        Alternative fuselage design for comparison
    """
    
    def __defaults__(self):
        """
        Sets default values for the tube fuselage attributes.
        """      
        self.tag = 'tube_fuselage'
        
    def compute_moment_of_inertia(self, center_of_gravity): 
        """
        Computes the moment of inertia tensor for the tube fuselage.

        Parameters
        ----------
        center_of_gravity : list
            Reference point coordinates [x, y, z] for moment calculation

        Returns
        -------
        I : ndarray
            3x3 moment of inertia tensor in kg*m^2

        See Also
        --------
        RCAIDE.Library.Methods.Weights.Moment_of_Inertia.compute_fuselage_moment_of_inertia
            Implementation of the moment of inertia calculation
        """
        I = compute_fuselage_moment_of_inertia(self,center_of_gravity) 
        return I        
  