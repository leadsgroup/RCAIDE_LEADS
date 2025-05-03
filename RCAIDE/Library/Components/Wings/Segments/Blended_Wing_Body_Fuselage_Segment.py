# RCAIDE/Library/Compoments/Wings/Segments/Blended_Wing_Body_Fuselage_Segment.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
from RCAIDE.Library.Components.Wings.Segments import Segment

# ---------------------------------------------------------------------------------------------------------------------- 
#  Segment
# ----------------------------------------------------------------------------------------------------------------------   
class Blended_Wing_Body_Fuselage_Segment(Segment):
    '''
    ''' 
    def __defaults__(self):
        self.tag                           = 'bwb_fuselage_segment' 
        self.percent_chord_cabin_start      = 0.0   
        self.percent_aft_centerbody_start   = 0.0
        

