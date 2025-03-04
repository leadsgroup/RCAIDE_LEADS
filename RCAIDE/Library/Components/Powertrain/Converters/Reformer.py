# RCAIDE/Compoments/Propulsors/Converters/Reformer.py
# 
# 
# Created:  Jan 2025, M. Clarke, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core              import Data
from .Converter  import Converter
import numpy as np
import scipy as sp

# ---------------------------------------------------------------------------------------------------------------------- 
#  Nacalle
# ----------------------------------------------------------------------------------------------------------------------  
class Reformer(Converter):
    """
    Reformer Component Class

    This class models a fuel reformer that converts Jet-A fuel into hydrogen-rich reformate gas. 
    It inherits from the base Converter class and implements reformer-specific attributes and methods.

    Attributes
    ----------
    tag : str
        Identifier for the reformer component, defaults to 'reformer'

    x_H : float
        Mass fraction of hydrogen content in Jet-A [-]
    x_C : float 
        Mass fraction of carbon content in Jet-A [-]
    y_H2 : float
        Mole fraction of hydrogen content in reformate [mol]
    y_CO : float
        Mole fraction of carbon monoxide content in reformate [mol]
    rho_F : float
        Density of Jet-A [g/cm^3]
    rho_S : float 
        Density of water [g/cm^3]
    rho_A : float
        Density of air [g/cm^3]
    MW_F : float
        Average molecular weight of Jet-A [g/g-mol]
    MW_S : float
        Average molecular weight of steam [g/g-mol]
    MW_C : float
        Average molecular weight of carbon [g/g-mol]
    MW_H2 : float
        Average molecular weight of hydrogen [g/g-mol]
    A_F_st_Jet_A : float
        Stoichiometric air-to-fuel mass ratio [lb_Air/lb_Jet_A]
    theta : float
        Contact time [sec^-1]
    LHV_F : float
        Lower heating value of Jet-A [kJ/g-mol]
    LHV_H2 : float
        Lower heating value of Hydrogen [kJ/g-mol]
    LHV_CO : float
        Lower heating value of Carbon Monoxide [kJ/g-mol]
    V_cat : float
        Catalyst bed volume [cm^3]
    eta : float
        Reformer efficiency [-]

    Notes
    -----
    The reformer model includes parameters for:
    - Fuel composition and properties
    - Reformate composition
    - Reformer geometry and performance characteristics
    - Thermodynamic properties of reactants/products

    """
    
    def __defaults__(self):
        """ 
        """      
        
        self.tag          = 'reformer'  

        # Jet-A parameters
        self.x_H          = 0.1348   # [-]               mass fraction of hydrogen content in Jet-A
        self.x_C          = 0.8637   # [-]               mass fraction of carbon content in Jet-A
        
        # Reformate parameters
        self.y_H2         = 0.9      # [mol]             mole fraction of hydrogen content in reformate
        self.y_CO         = 0.3      # [mol]             mole fraxtion of carbon monoxide content in reformate
    
        # Reformer parameters
        self.rho_F        = 0.813    # [g/cm**3]         Density of Jet-A
        self.rho_S        = 1        # [g/cm**3]         Density of water
        self.rho_A        = 0.001293 # [g/cm**3]         Density of air
        self.MW_F         = 160      # [g/g-mol]         Average molecular weight of Jet-A    
        self.MW_S         = 18.01    # [g/g-mol]         Average molecular weight of steam
        self.MW_C         = 12.01    # [g/g-mol]         Average molecular weight of carbon
        self.MW_H2        = 2.016    # [g/g-mol]         Average molecular weight of hydrogen
        self.A_F_st_Jet_A = 14.62    # [lb_Air/lb_Jet_A] Stoichiometric air-to-fuel mass ratio 
        self.theta        = 0.074    # [sec**-1]         Contact time
        self.LHV_F        = 43.435   # [kJ/g-mol]        Lower heating value of Jet-A
        self.LHV_H2       = 240.2    # [kJ/g-mol]        Lower heating value of Hydrogen
        self.LHV_CO       = 283.1    # [kJ/g-mol]        Lower heating value of Carbon Monoxide
        self.V_cat        = 9.653    # [cm**3]           Catalyst bed volume
        self.eta          = 0.9