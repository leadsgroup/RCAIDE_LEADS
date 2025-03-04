# RCAIDE/Library/Attributes/Solids/Aluminum_Alloy.py
# 

# Created: Jan 2025 M. Clarke

#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from .Solid import Solid 

#-------------------------------------------------------------------------------
# Aluminum 
#------------------------------------------------------------------------------- 
class Aluminum_Alloy(Solid): 
    """ Physical constants specific to 6061-T6 Aluminum
    """

    def __defaults__(self):
        """Sets material properties at instantiation. 

        Assumptions:
            None
    
        Source:
            Cao W, Zhao C, Wang Y, et al. Thermal modeling of full-size-scale cylindrical battery pack cooled
            by channeled liquid flow[J]. International journal of heat and mass transfer, 2019, 138: 1178-1187. 
        """

        self.density                    = 2700. * Units['kg/(m**3)']  
        self.thermal_conductivity       = 202.4
        self.specific_heat_capacity     = 871 
        self.ultimate_tensile_strength  = 310e6 * Units.Pa
        self.ultimate_shear_strength    = 206e6 * Units.Pa
        self.ultimate_bearing_strength  = 607e6 * Units.Pa
        self.yield_tensile_strength     = 276e6 * Units.Pa
        self.yield_shear_strength       = 206e6 * Units.Pa
        self.yield_bearing_strength     = 386e6 * Units.Pa
        self.minimum_gage_thickness     = 0.0   * Units.m
        self.minimum_gage_thickness     = 1.5e-3   * Units.m
        self.minimum_width              = 25.4e-3  * Units.m        
