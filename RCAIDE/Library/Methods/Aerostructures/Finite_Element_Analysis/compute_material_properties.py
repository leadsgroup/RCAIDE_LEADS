# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/compute_material_properties.py
# 
# Created: Mar 2026, M. Clarke, S. Sharma  

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
# Import Supporting Functions
import RCAIDE 

# Python Imports
import numpy as np 

# ----------------------------------------------------------------------
#  compute_material_properties
# ----------------------------------------------------------------------
def compute_material_properties(mat_name):
    """
    Returns material properties based on name.
    """
    if mat_name == "Al7075_T6":
        E = 71.7e9       # Young's Modulus (Pa)
        Nu = 0.33        # Poisson's Ratio
        Rho = 2810       # Density (kg/m^3)
        Yield = 503e6    # Yield Strength (Pa)
        
    elif mat_name == "CFRP_uCRM":
        E = 43.7e9       # Equivalent quasi-isotropic laminate modulus
        Nu = 0.30        
        Rho = 1395       # Derived smeared density to hit 806.39 kg target
        Yield = 600e6
    
    else:
        # Default to Aluminum if unknown
        E = 70e9
        Nu = 0.33
        Rho = 2700
        Yield = 250e6
        
    G = E / (2 * (1 + Nu))
    return E, G, Rho, Yield, Nu