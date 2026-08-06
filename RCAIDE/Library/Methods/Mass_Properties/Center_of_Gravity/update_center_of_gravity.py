# RCAIDE/Library/Methods/Stability/Common/update_center_of_gravity.py
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports     

# package imports
import numpy   as np 

# ----------------------------------------------------------------------------------------------------------------------
#  update_center_of_gravity
# ---------------------------------------------------------------------------------------------------------------------- 
def update_center_of_gravity(state, vehicle):
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
            - weights      : Data 

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
    
    # unpack
    conditions     = state.conditions
    N              = state.numerics.number_of_control_points
    
    # --------------------------------------------------------------------------       
    # update center of gravity
    # -------------------------------------------------------------------------- 
    Mom_tot   = np.zeros((N,3))
    Mass_tot  = np.zeros((N,1))
    for item in conditions.weights.components.mass.keys():
        mass = conditions.weights.components.mass[item]
        CG   = conditions.weights.components.global_center_of_gravity[item]
        sym  = conditions.weights.components.symmetry_flag[item] 
        Mass_tot += conditions.weights.components.mass[item]
        Mom_tot  += CG*sym * mass
    
    state.conditions.weights.vehicle.global_center_of_gravity =  Mom_tot / Mass_tot
    
    return 