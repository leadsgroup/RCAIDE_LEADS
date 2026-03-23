# RCAIDE/Library/Attributes/Solids/Vacuum_Multilayer_Insulation.py
# 
# Created: Aug 2025, S. Shekar
#
#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from .Solid import Solid 

#-------------------------------------------------------------------------------
# Vacuum Cellular Multilayer Insulation (VCMLI)
#------------------------------------------------------------------------------- 
class Vacuum_Cellular_Multilayer_Insulation(Solid): 
    """ 
    A class representing Vacuum Cellular Multilayer Insulation (VCMLI) material 
    properties, used extensively in aerospace cryogenic and thermal 
    protection systems.  

    Attributes
    ----------
    density : float
        Material density in kg/m³ (65).  
    thermal_conductivity : float
        Heat conduction coefficient in W/(m·K) (0.005).  
    specific_heat_capacity : float
        Specific heat at constant pressure in J/(kg·K). Typically ~1000 J/(kg·K), 
        but not explicitly set here.  

    Notes
    -----
    Vacuum Cellular Multilayer Insulation (VCMLI) is a highly effective cryogenic 
    insulation material, consisting of alternating reflective and spacer layers 
    maintained under Vacuum Cellular. It provides extremely low thermal conductivity, 
    making it the material of choice for liquid hydrogen storage tanks, space 
    vehicle insulation, and other cryogenic systems.  

    Unlike metals or structural solids, VCMLI is not intended for carrying 
    significant mechanical loads. Its primary function is to reduce radiative 
    and conductive heat transfer.  

    **Definitions**

    'Multilayer Insulation (MLI)'
        A thermal insulation system consisting of multiple thin reflective layers 
        separated by low-conductivity spacers and maintained under Vacuum Cellular.  

    'Thermal Conductivity'
        The property of a material to conduct heat, measured in watts per 
        meter-kelvin.  
    """

    def __defaults__(self):
        """
        Sets material properties at instantiation. 

        Parameters
        ----------
        None

        Returns
        -------
        None

        Notes
        -----
        * Uses typical values for Vacuum Cellular multilayer insulation in cryogenic 
          applications.  
        * Mechanical strength properties are not included because VCMLI is 
          not used as a structural material.  
        """
        self.density                    = 65 * Units['kg/(m**3)']        
        self.thermal_conductivity       = 0.005
        self.specific_density           = 1.1
