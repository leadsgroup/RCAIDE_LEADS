# RCAIDE/Methods/Powertrain/Sources/Batteries/Ragone/find_specific_power.py
# 
# 
# Created:  Jul 2023, M. Clarke 
 
# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ---------------------------------------------------------------------------------------------------------------------- -Ragone
def find_specific_power(battery, specific_energy):
    """
    Determines specific power from a Ragone curve correlation.
    
    Parameters
    ----------
    battery : Battery
        The battery component with the following attributes:
            - cell.ragone.const_1 : float
                Coefficient in the Ragone curve equation [W/kg]
            - cell.ragone.const_2 : float
                Exponent coefficient in the Ragone curve equation [kg/J]
    specific_energy : float
        Specific energy value for which to calculate the specific power [J/kg]
    
    Returns
    -------
    None
        This function modifies the battery object in-place, setting the following attributes:
            - specific_power : float
                Calculated specific power [W/kg]
            - specific_energy : float
                Input specific energy value [J/kg]
    
    Notes
    -----
    This function calculates the specific power of a battery based on its specific energy
    using a Ragone curve correlation. The Ragone curve describes the trade-off between
    specific energy and specific power in energy storage devices.
    
    The correlation used is:
    
    .. math::
        P_{specific} = C_1 \\cdot 10^{C_2 \\cdot E_{specific}}
    
    where:
      - :math:`P_{specific}` is the specific power [W/kg]
      - :math:`E_{specific}` is the specific energy [J/kg]
      - :math:`C_1` and :math:`C_2` are empirical constants
    
    **Major Assumptions**
        * The Ragone curve can be accurately represented by the exponential equation
        * The correlation is valid across the entire range of specific energy values
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Sources.Battery_Modules.Common.find_specific_energy
    """
    
    const_1                 = battery.cell.ragone.const_1
    const_2                 = battery.cell.ragone.const_2
    specific_power          = const_1*10.**(const_2*specific_energy)
    battery.specific_power  = specific_power
    battery.specific_energy = specific_energy