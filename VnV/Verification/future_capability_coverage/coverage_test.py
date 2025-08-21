# AVL_test.py
# 
# Created:  Dec 2023, M. Clarke 

""" setup file for segment test regression with a Boeing 737"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units ,  Data
from RCAIDE.Library.Plots             import *       

# python imports 
import numpy as np
import pylab as plt 
import sys
import os
 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    
 
    # materials 
    material            = RCAIDE.Library.Attributes.Materials.Acrylic()    
    material            = RCAIDE.Library.Attributes.Materials.Magnesium()  
    material            = RCAIDE.Library.Attributes.Materials.Titanium()   
    material            = RCAIDE.Library.Attributes.Materials.Aluminum()    
    material            = RCAIDE.Library.Attributes.Materials.Aluminum_Alloy()  
    material            = RCAIDE.Library.Attributes.Materials.Perfluoroalkoxy()  
    material            = RCAIDE.Library.Attributes.Materials.Polyetherimide()  
    material            = RCAIDE.Library.Attributes.Materials.Polyimide()  
    material            = RCAIDE.Library.Attributes.Materials.Polytetrafluoroethylene()
    material            = RCAIDE.Library.Attributes.Materials.CrossLinked_Polyethylene()
    material            = RCAIDE.Library.Attributes.Materials.Stainless_Steel_304()
    material            = RCAIDE.Library.Attributes.Materials.Aerogel()
    material            = RCAIDE.Library.Attributes.Materials.Polyurethane_Foam()
    material            = RCAIDE.Library.Attributes.Materials.Vacuum_Multilayer_Insulation()
    

      
    # gases 
    working_fluid                       = RCAIDE.Library.Attributes.Gases.CO2()        
    working_fluid                       = RCAIDE.Library.Attributes.Gases.Steam()      
   
    # cryogens
    cryogens =  RCAIDE.Library.Attributes.Cryogens.Cryogen()  
    cryogens =  RCAIDE.Library.Attributes.Cryogens.Liquid_Hydrogen()  
    
    # propellants 
    propellant  = RCAIDE.Library.Attributes.Propellants.Aviation_Gasoline() 
    propellant  = RCAIDE.Library.Attributes.Propellants.Ethane() 
    propellant  = RCAIDE.Library.Attributes.Propellants.Ethanol() 
    propellant  = RCAIDE.Library.Attributes.Propellants.Propanol() 
    propellant  = RCAIDE.Library.Attributes.Propellants.Methane() 
    propellant  = RCAIDE.Library.Attributes.Propellants.Propane()
    propellant  = RCAIDE.Library.Attributes.Propellants.Gaseous_Hydrogen()
    propellant  = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    propellant  = RCAIDE.Library.Attributes.Propellants.Alcohol_Mixture()
    propellant  = RCAIDE.Library.Attributes.Propellants.Alkane_Mixture()
    propellant  = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()
    propellant  = RCAIDE.Library.Attributes.Propellants.Butanol()
    propellant  = RCAIDE.Library.Attributes.Propellants.Liquid_Petroleum_Gas()
    propellant  = RCAIDE.Library.Attributes.Propellants.Jet_A1()    
    propellant  = RCAIDE.Library.Attributes.Propellants.JP7()  
    propellant  = RCAIDE.Library.Attributes.Propellants.Rocket_LH2()  
    propellant  = RCAIDE.Library.Attributes.Propellants.Rocket_RP1()
    
    # networks
    network =  RCAIDE.Framework.Networks.Hydrogen()
    
    # booms
    boom      = RCAIDE.Library.Components.Booms.Boom()
    segment_1 = RCAIDE.Library.Components.Booms.Segments.Circle_Segment()
    boom.append_segment(segment_1)
    segment_2 = RCAIDE.Library.Components.Booms.Segments.Ellipse_Segment()
    boom.append_segment(segment_2)
    segment_3 = RCAIDE.Library.Components.Booms.Segments.Rounded_Rectangle_Segment()
    boom.append_segment(segment_3)
    segment_4 = RCAIDE.Library.Components.Booms.Segments.Super_Ellipse_Segment()
    boom.append_segment(segment_4)
    segment_5 = RCAIDE.Library.Components.Booms.Segments.Segment()
    boom.append_segment(segment_5) 

    # nacelles
    nacelle      = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    segment_1 = RCAIDE.Library.Components.Nacelles.Segments.Circle_Segment()
    nacelle.append_segment(segment_1)
    segment_2 = RCAIDE.Library.Components.Nacelles.Segments.Ellipse_Segment()
    nacelle.append_segment(segment_2)
    segment_3 = RCAIDE.Library.Components.Nacelles.Segments.Rounded_Rectangle_Segment()
    nacelle.append_segment(segment_3)
    segment_4 = RCAIDE.Library.Components.Nacelles.Segments.Super_Ellipse_Segment()
    nacelle.append_segment(segment_4)
    segment_5 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nacelle.append_segment(segment_5)
    
    return
    
    
if __name__ == '__main__': 
    main()    
