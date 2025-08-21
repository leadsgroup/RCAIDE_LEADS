# RCAIDE/Library/Attributes/Solids/Polyurethane_Foam.py
# 
# Created: Aug 2025, S. Shekar
#
#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from .Solid import Solid 

#-------------------------------------------------------------------------------
# Polyurethane Foam
#------------------------------------------------------------------------------- 
class Polyurethane_Foam(Solid): 
    """ 
    A class representing polyurethane foam material properties, primarily used 
    as lightweight insulation and core filler in aerospace and engineering 
    applications.  

    Attributes
    ----------
    density : float
        Material density in kg/m³ (32).  
    thermal_conductivity : float
        Heat conduction coefficient in W/(m·K) (0.022).  
    specific_heat_capacity : float
        Specific heat at constant pressure in J/(kg·K). Typical values for 
        rigid polyurethane foams are ~1400 J/(kg·K) (not explicitly set here).  
    compressive_strength : float
        Compressive stress limit in Pa. Usually ranges from 200–400 kPa for 
        rigid polyurethane foams (not set by default).  
    tensile_strength : float
        Tensile stress limit in Pa. Typically low for foams (not set by default).  
    minimum_gage_thickness : float
        Minimum manufacturable thickness in meters (default 0.0).  

    Notes
    -----
    Polyurethane foam is commonly used in thermal protection systems, cryogenic 
    insulation, sandwich panel cores, and vibration damping components. Its 
    very low density and thermal conductivity make it ideal for aerospace 
    insulation, but its structural strength is limited compared to metals and 
    composites.  

    **Definitions**

    'Thermal Conductivity'
        Measure of a material’s ability to conduct heat, expressed in watts per 
        meter-kelvin.  

    'Foam Density'
        Mass per unit volume of the foam, a key factor determining thermal and 
        mechanical performance.  

    References
    ----------
    [1] Ashby, M. F., & Jones, D. R. H. (2012). *Engineering Materials 2: 
        An Introduction to Microstructures, Processing and Design*. 
        Elsevier.  
    [2] NASA Glenn Research Center. "Foam Insulation for Aerospace Applications."  
        https://www.nasa.gov/centers/glenn/technology/Foam.html
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
        * Uses typical values for rigid polyurethane foam of density ~32 kg/m³.  
        * Mechanical strength values are not explicitly defined here but can 
          be added for structural analysis.  
        """
        self.density                    = 32 * Units['kg/(m**3)']        
        self.thermal_conductivity       = 0.022
