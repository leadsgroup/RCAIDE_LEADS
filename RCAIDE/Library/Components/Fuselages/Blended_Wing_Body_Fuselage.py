# RCAIDE/Components/Fuselages/Blended_Wing_Body_Fuselage.py
# 
# Created:  Mar 2024, M. Clarke 
# Modified: Mar 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports    
import RCAIDE
from RCAIDE.Library.Components.Wings      import Main_Wing
from RCAIDE.Library.Components.Component  import Container


# python imports 
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Blended_Wing_Body_Fuselage
# ----------------------------------------------------------------------------------------------------------------------  
class Blended_Wing_Body_Fuselage(Main_Wing):
    """

    """
        
    def __init__(self):
        """
        Sets default values for all fuselage attributes.
        """
        super().__init__()  # Initialize Main_Wing properties
        self.tag = 'blended_wing_fuselage'
        
        # Use composition: create an instance of Fuselage
        self.fuselage = RCAIDE.Library.Components.Fuselages.Fuselage()
        
        
    def append_segment(self,segment):
        """
        Adds a new segment to the fuselage's segment container.

        Parameters
        ----------
        segment : DataHow
            Fuselage segment to be added
        """

        # Assert database type
        if not isinstance(segment,RCAIDE.Library.Components.Wings.Segments.Segment):
            raise Exception('input component must be of type Segment')

        # Store data
        self.segments.append(segment)

        return
    
    

    def compute_moment_of_inertia(self, center_of_gravity=[[0, 0, 0]]): 
        """
        Computes the moment of inertia tensor for the fuselage.

        Parameters
        ----------
        center_of_gravity : list, optional
            Reference point coordinates for moment calculation, defaults to [[0, 0, 0]]

        Returns
        -------
        I : ndarray
            3x3 moment of inertia tensor in kg*m^2

        See Also
        --------
        RCAIDE.Library.Methods.Weights.Moment_of_Inertia.compute_fuselage_moment_of_inertia
            Implementation of the moment of inertia calculation
        """
        I = RCAIDE.Library.Methods.Weights.Moment_of_Inertia.compute_fuselage_moment_of_inertia(self,center_of_gravity) 
        return I    