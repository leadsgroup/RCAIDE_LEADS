# RCAIDE/Library/Methods/Stability/Common/update_center_of_gravity.py
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports   
from RCAIDE.Framework.Core     import Data   
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity     import compute_vehicle_center_of_gravity

# package imports
import numpy   as np
from copy      import  deepcopy 

# ----------------------------------------------------------------------------------------------------------------------
#  update_center_of_gravity
# ---------------------------------------------------------------------------------------------------------------------- 
def update_center_of_gravity(vehicle, conditions):
    """
    Updates the vehicle center of gravity accounting for fuel consumption during flight.

    Parameters
    ----------
    vehicle : RCAIDE.Library.Components.Vehicle
        Vehicle object containing all components and fuel systems
            - networks : list
                List of propulsion networks
            - fuel_lines : list
                List of fuel lines within each network
            - fuel_tanks : list
                List of fuel tanks within each fuel line
                    - fuel : RCAIDE.Library.Components.Energy.Storages.Fuel
                        Fuel component with mass properties
                            - mass_properties : Data
                                - mass : float
                                    Current fuel mass [kg]
                                - center_of_gravity : list
                                    Fuel tank center of gravity [m]
                            - origin : list
                                Fuel tank origin location [m]
    conditions : RCAIDE.Framework.Core.Data
        Flight conditions and state data
            - aerodynamics : Data
                - angles : Data
                    - alpha : float
                        Angle of attack [radians]
            - energy : Data
                - fuel_lines : dict
                    Dictionary of fuel line results
                        - fuel_tanks : dict
                            Dictionary of fuel tank results
                                - fuel_mass : float
                                    Current fuel mass [kg]

    Returns
    -------
    CG : numpy.ndarray
        Updated center of gravity coordinates [m]
        Shape: (3, N) where N is the number of flight conditions

    Notes
    -----
    This function calculates the updated center of gravity of the vehicle by
    accounting for fuel consumption during flight. It computes the mass and
    moment contributions of remaining fuel and combines them with the dry
    vehicle mass and moment to determine the new center of gravity location.
    
    **Major Assumptions**
        * Fuel tank locations and orientations remain constant
        * No fuel sloshing or redistribution effects
        * Vehicle structure mass remains constant
        * Fuel density is constant
    
    **Theory**

    The center of gravity is calculated using the principle of moments:
    :math:`\\vec{r}_{CG} = \\frac{\\sum m_i \\vec{r}_i}{\\sum m_i}`

    where :math:`m_i` is the mass of component :math:`i` and :math:`\\vec{r}_i` is its position.

    **Definitions**

    'Center of Gravity'
        Point where the total mass of the vehicle can be considered to act.
    """
    # --------------------------------------------------------------------------
    # unpack 
    # --------------------------------------------------------------------------
    AoA           = conditions.aerodynamics.angles.alpha  

    # --------------------------------------------------------------------------       
    # update center of gravity 
    # --------------------------------------------------------------------------
    # create aircraft copy without fuel mass 
    vehicle_no_fuel =  deepcopy(vehicle)
    for network in vehicle_no_fuel.networks: 
        for fuel_line in network.fuel_lines:   
            for fuel_tank in fuel_line.fuel_tanks: 
                fuel_tank.fuel.mass_properties.mass = 0.0
    
    # run c.g. function to get total mass  and moment without updating C.G.
    _ , Mom_0, Mass_0 = compute_vehicle_center_of_gravity(vehicle_no_fuel, update_center_of_gravity= False) 
    
    # determine original fuel mass and moment and remove it from total mass and moment
    Mom_fuel = np.array([[0.0, 0.0, 0.0]])
    M_fuel   = np.zeros_like(AoA)
    for network in vehicle.networks: 
        for fuel_line in network.fuel_lines:  
            fuel_line_results   = conditions.energy.fuel_lines[fuel_line.tag]
            for fuel_tank in fuel_line.fuel_tanks: 
                m_fuel        = fuel_line_results.fuel_tanks[fuel_tank.tag].fuel_mass[0] 
                global_cg_loc = np.array(fuel_tank.fuel.mass_properties.center_of_gravity) + np.array(fuel_tank.fuel.origin)  
                M_fuel        += m_fuel
                Mom_fuel      += np.multiply(m_fuel, global_cg_loc)               
    
    # recompute mass and moment of fuel
    Mom_aircraft = Mom_0 + Mom_fuel
    M_aircraft   = Mass_0 + M_fuel
        
    # compute updated C.G. 
    CG = np.atleast_2d(Mom_aircraft[:, 0]).T / M_aircraft
    
    return CG