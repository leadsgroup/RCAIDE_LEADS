# RCAIDE/Methods/Powertrain/Sources/Batteries/Common/find_mass_gain_rate.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ---------------------------------------------------------------------------------------------------------------------- -Common
def find_mass_gain_rate(battery, power):
    """
    Calculates the mass gain rate of the battery from the ambient air.
    
    Parameters
    ----------
    battery : Battery
        The battery component with the following attributes:
            - mass_gain_factor : float
                Mass of air gained per unit of energy [kg/J]
    power : float or numpy.ndarray
        Power being drawn from the battery [W]
    
    Returns
    -------
    mdot : float or numpy.ndarray
        Mass gain rate [kg/s]
    
    Notes
    -----
    This function calculates the rate at which a battery gains mass from the ambient air
    during discharge. This is relevant for metal-air batteries (such as aluminum-air or 
    zinc-air) where oxygen from the air is consumed during the discharge process, 
    resulting in a mass increase.
    
    The calculation is based on a linear relationship between power output and
    mass gain rate, using the mass_gain_factor property of the battery.
    
    **Major Assumptions**
        * Earth atmospheric composition
        * Linear relationship between power output and mass gain rate
    """
    #weight gain of battery (positive means mass gain)
    mdot = -(power) * (battery.mass_gain_factor)  
                
    return mdot