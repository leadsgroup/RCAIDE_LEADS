# RCAIDE/Library/Components/Powertrain/Energy/Sources/Fuel_Tanks/LPG_Tank.py
# 
# Created: Oct 2025, M. Guidotti
#
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from .Non_Integral_Tank  import Non_Integral_Tank 
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank.compute_non_integral_tank_volume  import *

# ----------------------------------------------------------------------------------------------------------------------
#  LPG_Tank
# ---------------------------------------------------------------------------------------------------------------------    
class LPG_Tank(Non_Integral_Tank):
    """
   
    """

    def __defaults__(self):
        """
        Set default values for LPG tank attributes.  

        Parameters
        ----------
        None  

        Returns
        -------
        None  
        """
        self.tag                      = 'LPG_Tank'
        self.material                 = None
        self.insulation_material      = None
        self.design_inlet_temperature = 293.15
        self.design_altitiude         = 0
        self.acceptable_heat_leak     = 293.15
        self.design_altitude          = 30000 * Units.ft
        self.design_isa_deviation     = 0
        self.ullage_volume_fraction   = 0.07
        self.design_external_pressure = 0 