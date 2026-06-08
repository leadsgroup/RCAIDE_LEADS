# RCAIDE/Input_Output/__init__.py
#
# Unified IO submodule.
#
# Public API
# ----------
# save(obj, filename)              -- pickle RCAIDE Data objects
# load(filename)                   -- unpickle RCAIDE Data objects
# export(vehicle, configs, ...)    -- JSON export with type annotations
# import_data(filename)            -- JSON import, restores typed objects
# save_results(mission, filename)  -- write mission conditions to HDF5
# load_results(filename)           -- read mission conditions from HDF5

from .save         import save
from .load         import load
from .export       import export
from .import_data  import import_data
from .save_results import save_results
from .load_results import load_results
