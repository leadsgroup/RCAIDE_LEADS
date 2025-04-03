# RCAIDE/Methods/Powertrain/Sources/Batteries/Common/find_total_mass_gain.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ---------------------------------------------------------------------------------------------------------------------- 
def find_total_mass_gain(battery):
    """
    Calculates the total mass of air that the battery accumulates when discharged fully.
    
    Parameters
    ----------
    battery : Battery
        The battery component with the following attributes:
            - maximum_energy : float
                Maximum energy capacity of the battery [J]
            - mass_gain_factor : float
                Mass of air gained per unit of energy [kg/J]
    
    Returns
    -------
    mgain : float
        Total mass gain when the battery is fully discharged [kg]
    
    Notes
    -----
    This function calculates the total mass of air that a battery would accumulate
    when discharged from full capacity to zero. This is relevant for metal-air batteries
    (such as aluminum-air or zinc-air) where oxygen from the air is consumed during
    the discharge process, resulting in a mass increase.
    
    The calculation is based on a linear relationship between energy output and
    air mass gain, using the mass_gain_factor property of the battery.
    
    **Major Assumptions**
        * Linear relationship between energy output and mass gain
        * Complete discharge from maximum energy to zero
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Sources.Battery_Modules.Aluminum_Air.compute_al_air_cell_performance
    """
    mgain = battery.maximum_energy * battery.mass_gain_factor
    
    return mgain