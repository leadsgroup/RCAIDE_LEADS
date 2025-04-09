# RCAIDE/Library/Attributes/Propellants/__init__.py
# 

"""
This module provides propellant definitions for RCAIDE simulations. It includes classes 
for various propellant types categorized as follows:

**Aviation Fuels**
    - Aviation_Gasoline : Aviation gasoline properties
    - Jet_A : Commercial aviation kerosene
    - Jet_A1 : International spec aviation kerosene
    - JP7 : Advanced high thermal stability jet fuel

**Rocket Propellants**
    - Rocket_LH2 : Rocket-grade liquid hydrogen
    - Rocket_RP1 : Rocket-grade kerosene

**Cryogenic Fuels**
    - Liquid_Hydrogen : LH2 for aviation applications
    - Liquid_Natural_Gas : LNG fuel properties
    - Gaseous_Hydrogen : GH2 properties

**Hydrocarbon Fuels**
    - Methane : CH4 properties
    - Propane : C3H8 properties
    - Ethane : C2H6 properties
    - Alkane_Mixture : Customizable alkane blend properties

**Alcohols**
    - Ethanol : C2H5OH properties
    - Butanol : C4H9OH properties
    - Propanol : C3H7OH properties
    - Alcohol_Mixture : Customizable alcohol blend properties

**Base Classes**
    - Propellant : Base class defining standard propellant interfaces

See Also
--------
RCAIDE.Library.Methods.Propulsors : Propulsion analysis tools
RCAIDE.Library.Components.Propulsors : Propulsion system components
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .Propellant           import Propellant
from .Aviation_Gasoline    import Aviation_Gasoline
from .Jet_A                import Jet_A
from .Jet_A1               import Jet_A1 
from .Gaseous_Hydrogen     import Gaseous_Hydrogen
from .Liquid_Natural_Gas   import Liquid_Natural_Gas
from .Liquid_Petroleum_Gas import Liquid_Petroleum_Gas
from .JP7                  import JP7
from .Methane              import Methane
from .Propane              import Propane
from .Ethane               import Ethane
from .Ethanol              import Ethanol
from .Liquid_Hydrogen      import Liquid_Hydrogen
from .Rocket_LH2           import Rocket_LH2
from .Rocket_RP1           import Rocket_RP1
from .Butanol		   import Butanol
from .Propanol		   import Propanol
from .Alkane_Mixture	   import Alkane_Mixture
from .Alcohol_Mixture	   import Alcohol_Mixture