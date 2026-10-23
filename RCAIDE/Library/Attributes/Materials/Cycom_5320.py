# RCAIDE/Library/Attributes/Solids/Cycom5320.py
# 
# Created: Aug 2025, S. Shekar
#
#-------------------------------------------------------------------------------
# Imports
#-------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from .Solid import Solid 

#-------------------------------------------------------------------------------
# Cycom 5320
#------------------------------------------------------------------------------- 
class Cycom_5320(Solid):
    """
    A class representing a CYCOM 5320-1/IM7 unidirectional-tape carbon-fiber
    composite laminate, used here as a candidate cryogenic tank structural
    material.

    Attributes
    ----------
    density : float
        Laminate density in kg/m³ (1590), estimated by rule-of-mixtures from
        the neat resin's 1.31 g/cc cured density and Hexcel IM7 fiber
        (~1.78 g/cc) at the datasheet's ~60% nominal fiber volume fraction --
        not the neat resin density alone.
    thermal_conductivity : float
        Heat conduction coefficient in W/(m·K) (0.2).
    yield_tensile_strength : float
        Stress at which the material begins to fail, in Pa (1310e6),
        taken as the datasheet's balanced 0 deg/90 deg laminate tensile
        strength (186-203 ksi across test conditions), the physically
        appropriate layup assumption for the biaxial (hoop + axial)
        stress state of an isotropic thick-walled pressure vessel.

    Notes
    -----
    CYCOM 5320-1 is a toughened epoxy prepreg resin system; properties here
    reflect the IM7-reinforced unidirectional-tape laminate (Cytec/Solvay
    document ASM-9124-EN, Rev. October 2015), not the neat resin alone.

    **Definitions**

    'Yield Strength'
        The stress at which a material begins to plastically deform.

    'Ultimate Strength'
        The maximum stress that a material can withstand before fracture.

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
        * CYCOM 5320-1/IM7 unidirectional tape, Cytec/Solvay datasheet
          ASM-9124-EN (Rev. October 2015): 1.31 g/cc cured resin density,
          ~60% nominal fiber volume, 0 deg/90 deg laminate tensile strength
          186-203 ksi. Laminate density rule-of-mixtures estimate using
          Hexcel IM7 fiber density ~1.78 g/cc.
        """
        self.density                    = 1590 * Units['kg/(m**3)']
        self.thermal_conductivity       = 0.2
        self.yield_tensile_strength     = 1310e6 * Units.Pa
