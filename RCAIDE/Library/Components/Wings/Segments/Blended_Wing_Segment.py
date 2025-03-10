# RCAIDE/Library/Compoments/Wings/Segment.py
# 
# Created:

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
# import RCAIDE
# from RCAIDE.Framework.Core     import Data, Container
# from RCAIDE.Library.Components import Component   
from .Segment import Segment
# ---------------------------------------------------------------------------------------------------------------------- 
#  Segment
# ----------------------------------------------------------------------------------------------------------------------   
class Blended_Wing_Segment(Segment):

    def __defaults__(self):
        """
        Sets default values for the wing segment attributes.
        """         
        self.tag                     = 'blended_wing_segment'