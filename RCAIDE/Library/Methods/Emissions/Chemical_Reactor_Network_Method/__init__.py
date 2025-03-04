# RCAIDE/Library/Methods/Emissions/Chemical_Reactor_Network_Method/__init__.py

# Created: June 2024, M. Clarke, M. Guidotti 

"""
Chemical Reactor Network (CRN) Method 

This module provides functionality for building, training and evaluating surrogate models as well 
as for direct computation of emission indices using Chemical Reactor Networks.

Notes
-----
The Chemical Reactor Network Method package contains tools for:

- Training CRN emission index surrogate models
- Building CRN emission index surrogate models
- Evaluating chemical reactions using Cantera
- Computing emission indices using CRN models

See Also
--------
RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method.train_CRN_EI_surrogates
RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method.build_CRN_EI_surrogates
RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method.evaluate_CRN_emission_indices
RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method.evaluate_cantera
RCAIDE.Library.Components.Powertrain.Converters.Combustor
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
from .build_CRN_EI_surrogates       import build_CRN_EI_surrogates
from .train_CRN_EI_surrogates       import train_CRN_EI_surrogates
from .evaluate_cantera              import evaluate_cantera
from .evaluate_CRN_emission_indices import *  