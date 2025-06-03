# RCAIDE/Library/Methods/Aerodynamics/Athena_Vortex_Lattice/__init__.py
"""
Athena Vortex Lattice (AVL) interface methods for aerodynamic analysis and surrogate modeling.

This module provides a comprehensive interface to the AVL aerodynamic analysis 
software, including geometry translation, analysis execution, result processing, 
and surrogate model generation. The interface handles automatic file management, 
data translation between RCAIDE and AVL formats, and training of surrogate models 
for efficient aerodynamic predictions. Note, that is that all modules require AVL 
to be installed and accessible.

Additional information on AVL, including download instructions, can be found at: 
https://web.mit.edu/drela/Public/web/avl/

See Also
--------
RCAIDE.Library.Methods.Aerodynamics : Parent aerodynamics methods module
"""

from .evaluate_AVL              import  *
from .build_AVL_surrogates      import build_AVL_surrogates
from .train_AVL_surrogates      import train_AVL_surrogates 
from .create_avl_datastructures import translate_avl_wing, translate_avl_body , populate_wing_sections, populate_body_sections
from .purge_files               import purge_files
from .read_results              import read_results
from .run_AVL_analysis          import run_AVL_analysis
from .translate_data            import translate_conditions_to_cases, translate_results_to_conditions
from .write_geometry            import write_geometry
from .write_mass_file           import write_mass_file
from .write_input_deck          import write_input_deck
from .write_run_cases           import write_run_cases
from .write_avl_airfoil_file    import write_avl_airfoil_file 

from . import AVL_Objects
