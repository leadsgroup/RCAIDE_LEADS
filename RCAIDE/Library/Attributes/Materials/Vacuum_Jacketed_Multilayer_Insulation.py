# RCAIDE/Library/Attributes/Solids/Vacuum_Gap_Multilayer_Insulation.py
# 
# Created: Aug 2025, S. Shekar
#
#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from .Solid import Solid 

#-------------------------------------------------------------------------------
# Vacuum Jacketed Multilayer Insulation (VJMLI)
#------------------------------------------------------------------------------- 
class Vacuum_Jacketed_Multilayer_Insulation(Solid): 
    """ 
    A class representing Vacuum Jacketed Multilayer Insulation (VGMLI) material 
    properties. VGMLI is an advanced cryogenic insulation technology that 
    combines vacuum gaps with multilayer reflective barriers to minimize 
    conductive and radiative heat transfer.  

    Attributes
    ----------
    density : float
        Effective bulk density in kg/m³ (80).  
    thermal_conductivity : float
        Effective heat conduction coefficient in W/(m·K) (0.0003).  
    specific_density : float
        Relative density compared to water (1.1).  
    specific_heat_capacity : float
        Specific heat at constant pressure in J/(kg·K). Not explicitly set here.  

    Notes
    -----
    Vacuum Jacketed Multilayer Insulation (VGMLI) is widely used in aerospace for 
    liquid hydrogen storage, cryogenic tanks, and space vehicle insulation.  
    By integrating vacuum layers with multilayer insulation, VGMLI achieves 
    extremely low thermal conductivity, making it one of the most effective 
    cryogenic insulators.  

    Unlike structural materials, VGMLI is not intended to bear significant 
    mechanical loads; its primary purpose is thermal protection.  

    **Definitions**

    'Vacuum Jacketed Multilayer Insulation (VGMLI)'
        A thermal insulation system that integrates vacuum gaps with 
        multilayer reflective insulation to reduce conductive and 
        radiative heat transfer.  

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
        * Uses typical representative values for VGMLI in cryogenic 
          hydrogen storage applications.  
        * Mechanical properties are not defined since VGMLI is not 
          a structural material.  
        """
        self.density              = 80 * Units['kg/(m**3)']        
        self.thermal_conductivity = 0.0003
        self.specific_density     = 1.1
