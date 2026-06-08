# RCAIDE/__init__.py
# 
"""
=============================================
RCAIDE (:mod:`RCAIDE`)
=============================================

.. currentmodule:: RCAIDE

RCAIDE (Rapid Conceptual Aircraft Integrated Design Environment) - A framework for aircraft design and analysis

Sub-folders
============================================

.. autosummary::
   :toctree: generated/

   Framework    -- Core framework functionality and utilities
   Library      -- Components, methods, and analysis tools

Functions
==========================================

.. autosummary::
   :toctree: generated/

   load         -- Load saved RCAIDE data
   save         -- Save RCAIDE data

Classes
-------
.. autosummary::
   :toctree: generated/
   
   Vehicle      -- Base vehicle class for all analyses

Notes
-----
RCAIDE is designed to provide a flexible and extensible environment for aircraft
design and analysis. It uses a modular approach where vehicles can be built up
from components and analyzed through various mission profiles.
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from . import Framework
from . import Library
from . import Input_Output

from .Vehicle        import Vehicle
from .Input_Output   import save, load, export, import_data, save_results, load_results

# backward-compatibility aliases
from .Input_Output.export      import export as export_rcaide_data
from .Input_Output.import_data import import_data as import_rcaide_data

import os
from warnings import simplefilter
simplefilter('ignore')

def get_version():
    """Read the version from the VERSION file."""
    # Get the directory of the current file
    current_dir = os.path.dirname(__file__)
    # Construct the full path to the VERSION file
    version_file_path = os.path.join(current_dir, 'VERSION')
    
    if os.path.exists(version_file_path):
        with open(version_file_path, 'r') as version_file:
            return version_file.read().strip()
    return '0.0.0' # fallback version

__version__ = get_version()