# RCAIDE/Methods/Powertrain/Sources/Batteries/Aluminum_Air/compute_al_air_cell_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  compute_al_air_cell_performance
# ----------------------------------------------------------------------------------------------------------------------  
def find_aluminum_mass(battery, energy):
    """
    Calculates the aluminum mass required for a given energy output from an aluminum-air battery.
    
    Parameters
    ----------
    battery : AluminumAirBattery
        The aluminum-air battery component with the following attributes:
            - cell.aluminum_mass_factor : float
                Mass of aluminum required per unit of energy [kg/J]
    energy : float
        Energy to be produced by the battery [J]
    
    Returns
    -------
    aluminum_mass : float
        Mass of aluminum required [kg]
    
    Notes
    -----
    This function calculates the mass of aluminum that would be consumed to produce
    the specified amount of energy in an aluminum-air battery. The calculation is based
    on a linear relationship between energy output and aluminum consumption.
    
    See Also
    --------
    find_water_mass : Function to calculate water mass gain
    """
    aluminum_mass = energy*battery.cell.aluminum_mass_factor
    return aluminum_mass 

def find_water_mass(battery, energy):
    """
    Calculates the water mass gained during operation of an aluminum-air battery for a given energy output.
    
    Parameters
    ----------
    battery : AluminumAirBattery
        The aluminum-air battery component with the following attributes:
            - cell.water_mass_gain_factor : float
                Mass of water gained per unit of energy [kg/J]
    energy : float
        Energy produced by the battery [J]
    
    Returns
    -------
    water_mass : float
        Mass of water gained [kg]
    
    Notes
    -----
    This function calculates the mass of water that would be produced as a byproduct
    during the operation of an aluminum-air battery when generating the specified
    amount of energy. The calculation is based on a linear relationship between
    energy output and water production.
    
    In aluminum-air batteries, the electrochemical reaction produces aluminum hydroxide,
    which contains water molecules, resulting in a net gain of water mass in the system.
    
    See Also
    --------
    find_aluminum_mass : Function to calculate aluminum mass consumption
    """
    water_mass = energy*battery.cell.water_mass_gain_factor
    return water_mass
