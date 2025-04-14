# RCAIDE/Library/Attributes/Materials/__init__.py
# 

"""
This module provides material handling capabilities for RCAIDE. It includes classes for managing different types of materials 
and their physical properties, with implementations for various structural and aerospace materials including:
    - Metals (Aluminum, Steel, Nickel, Titanium, Magnesium)
    - Composites (Unidirectional and Bidirectional Carbon Fiber, Carbon Fiber Honeycomb)
    - Polymers (Epoxy, Polyetherimide, Acrylic)
    - Surface Treatments (Paint)

The Solid class serves as the base class for all material implementations.

See Also
--------
RCAIDE.Library.Attributes.Gases : Related module for gas properties
RCAIDE.Library.Attributes.Liquids : Related module for liquid properties
RCAIDE.Library.Attributes.Cryogens : Related module for cryogenic material properties
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .Solid                       import Solid
from .Acrylic                     import Acrylic
from .Aluminum                    import Aluminum
from .Aluminum_Alloy              import Aluminum_Alloy
from .Bidirectional_Carbon_Fiber  import Bidirectional_Carbon_Fiber
from .CrossLinked_Polyethylene    import CrossLinked_Polyethylene
from .Copper                      import Copper
from .Epoxy                       import Epoxy 
from .Nickel                      import Nickel
from .Magnesium                   import Magnesium
from .Carbon_Fiber_Honeycomb      import Carbon_Fiber_Honeycomb
from .Paint                       import Paint
from .Polyetherimide              import Polyetherimide
from .Perfluoroalkoxy             import Perfluoroalkoxy
from .Polytetrafluoroethylene     import Polytetrafluoroethylene
from .Polyimide                   import Polyimide
from .Steel                       import Steel
from .Titanium                    import Titanium
from .Unidirectional_Carbon_Fiber import Unidirectional_Carbon_Fiber