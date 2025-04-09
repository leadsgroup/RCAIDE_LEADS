# RCAIDE/Library/Methods/Powertrain/Converters/Combustor/__init__.py

"""
Methods for modeling and analyzing combustors in propulsion systems.

This module provides functions for computing the thermodynamic performance of combustors
in gas turbine engines and other propulsion systems. It includes methods for calculating
combustion properties, fuel consumption, and temperature rise across the combustor, as well
as functions for initializing combustor conditions during mission analysis.

The combustor is a critical component in gas turbine engines where fuel is mixed with
compressed air and burned to produce high-temperature gases that drive the turbine.

See Also
--------
RCAIDE.Library.Methods.Powertrain.Converters.Turbine
RCAIDE.Library.Methods.Powertrain.Converters.Compressor
RCAIDE.Library.Methods.Powertrain.Converters.Compression_Nozzle
""" 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
        
from .compute_combustor_performance         import compute_combustor_performance  
from .append_combustor_conditions           import append_combustor_conditions 

