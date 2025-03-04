# RCAIDE/Library/Components/Propulsors/Converters/Shaft_Power_Offtake.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Feb 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports   
from .Converter  import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Offtake_Shaft.append_offtake_shaft_conditions import append_offtake_shaft_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
# Shaft_Power_Offtake
# ---------------------------------------------------------------------------------------------------------------------- 
class Offtake_Shaft(Converter):
    """
    A power offtake shaft component model for extracting power from rotating machinery.

    Attributes
    ----------
    power_draw : float
        Amount of power extracted from the shaft [W]. Default is 0.0.
    reference_temperature : float
        Reference temperature for performance calculations [K]. Default is 288.15.
    reference_pressure : float
        Reference pressure for performance calculations [Pa]. Default is 1.01325e5.

    Notes
    -----
    The Offtake_Shaft class models power extraction from rotating shafts in 
    propulsion systems. It is typically used to simulate:
    * Accessory power extraction
    * Generator drives
    * Hydraulic pump drives
    * Air conditioning system drives
    * Other mechanical power requirements

    The model accounts for:
    * Power extraction effects on main shaft
    * Temperature effects on performance
    * Pressure effects on performance
    * Impact on engine thermodynamic cycle

    **Definitions**

    'Power Draw'
        Amount of mechanical power extracted from the main shaft
    'Reference Conditions'
        Standard day conditions used for performance normalization

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Offtake_Shaft
    """
    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
            None

        Source:
            None 
        """          
        self.power_draw            = 0.0
        self.reference_temperature = 288.15
        self.reference_pressure    = 1.01325 * 10 ** 5 

    def append_operating_conditions(self,segment,propulsor): 
        propulsor_conditions =  segment.state.conditions.energy[propulsor.tag]
        append_offtake_shaft_conditions(self,segment,propulsor_conditions)
        return                         