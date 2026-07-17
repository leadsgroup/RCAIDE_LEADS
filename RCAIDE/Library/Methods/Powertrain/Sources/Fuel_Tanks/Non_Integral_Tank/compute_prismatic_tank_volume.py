# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/Non_Integral_Tank/compute_prismatic_tank_volume.py
# 
# 
# Created: Aug 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_non_dimensional_moment_of_inertia import compute_cuboid_non_dimensional_moi

# ----------------------------------------------------------------------------------------------------------------------
#  Methods to compute volume of non integrak tanks
# ----------------------------------------------------------------------------------------------------------------------  
def compute_prismatic_tank_volume(fuel_tank):
    """
    Computes the volume of a Prismatic non-integral fuel tanks.
 
    Parameters
    ----------
    fuel_tank : Fuel_Tank
        Fuel tank object containing tank specifications
            - fuel : Fuel
                Fuel properties including density 
            - length : float
                Length of the tank (computed) 
            - width : float
                Width of the tank (computed) 
            - height : float
                Height of the tank (computed)  

    Returns
    -------
    volume : float
        Internal volume of the non-integral fuel tank 
    """
    
    l = fuel_tank.lengths.external
    w = fuel_tank.widths.external
    h = fuel_tank.heights.external
    t = fuel_tank.wall_thickness
     
    # determine inner dimensions of the fuel tank
    inner_length = l - 2 * t 
    inner_width  = w - 2 * t  
    inner_height = h - 2 * t            

    # compute net volume and gross volume
    fuel_tank.volume_properties.net_volume            = inner_length * inner_width *  inner_height
    fuel_tank.volume_properties.gross_volume          = l * w * h
    fuel_tank.fuel.mass_properties.center_of_gravity  =  [[fuel_tank.lengths.external /2, 0, 0]]
    fuel_tank.mass_properties.center_of_gravity       =  [[fuel_tank.lengths.external /2, 0, 0]]
    fuel_tank.fuel.origin                             = fuel_tank.origin

    fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor = compute_cuboid_non_dimensional_moi(inner_length, inner_width, inner_height)

    return
