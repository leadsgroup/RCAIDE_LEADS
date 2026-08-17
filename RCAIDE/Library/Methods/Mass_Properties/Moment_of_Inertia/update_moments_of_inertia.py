# RCAIDE/Library/Methods/Stability/Common/update_moments_of_inertia.py
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports     
import RCAIDE
# package imports
import numpy   as np 

# ----------------------------------------------------------------------------------------------------------------------
#  update_moments_of_inertia
# ---------------------------------------------------------------------------------------------------------------------- 
def update_moments_of_inertia(state,vehicle):
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
            - weights : Data 

    Returns
    -------
    CG : numpy.ndarray
        Updated moment of intertia tensor variables
        Shape: (1, N) where N is the number of flight conditions

    Notes
    -----
    This function calculates the moment of inertia of the vehicle by
    accounting for fuel consumption during flight.  
    
    **Major Assumptions**
        * Fuel tank locations and orientations remain constant
        * No fuel sloshing or redistribution effects
        * Vehicle structure mass remains constant
        * Fuel density is constant 
 
    """
    
    # unoack
    conditions     = state.conditions
    N              = state.numerics.number_of_control_points
    
    for network in vehicle.networks:
        for source in  network.sources: 
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank): 
                update_fuel_tank_moment_of_inertia(source,state) 
            
    # --------------------------------------------------------------------------     
    # update aircraft MOI
    # --------------------------------------------------------------------------
    MOI_Ixx   = np.zeros((N,1))
    MOI_Ixy   = np.zeros((N,1))
    MOI_Ixz   = np.zeros((N,1))
    MOI_Iyx   = np.zeros((N,1))
    MOI_Iyy   = np.zeros((N,1))
    MOI_Iyz   = np.zeros((N,1))
    MOI_Izx   = np.zeros((N,1))
    MOI_Izy   = np.zeros((N,1))
    MOI_Izz   = np.zeros((N,1))
    for item in conditions.weights.components.mass.keys():
        MOI_Ixx  += conditions.weights.components.moments_of_inertia_Ixx[item]
        MOI_Ixy  += conditions.weights.components.moments_of_inertia_Ixy[item]
        MOI_Ixz  += conditions.weights.components.moments_of_inertia_Ixz[item]
        MOI_Iyx  += conditions.weights.components.moments_of_inertia_Iyx[item]
        MOI_Iyy  += conditions.weights.components.moments_of_inertia_Iyy[item]
        MOI_Iyz  += conditions.weights.components.moments_of_inertia_Iyz[item]
        MOI_Izx  += conditions.weights.components.moments_of_inertia_Izx[item]
        MOI_Izy  += conditions.weights.components.moments_of_inertia_Izy[item]
        MOI_Izz  += conditions.weights.components.moments_of_inertia_Izz[item]
         
    conditions.weights.vehicle.moments_of_inertia_Ixx  = MOI_Ixx 
    conditions.weights.vehicle.moments_of_inertia_Ixy  = MOI_Ixy 
    conditions.weights.vehicle.moments_of_inertia_Ixz  = MOI_Ixz 
    conditions.weights.vehicle.moments_of_inertia_Iyx  = MOI_Iyx 
    conditions.weights.vehicle.moments_of_inertia_Iyy  = MOI_Iyy 
    conditions.weights.vehicle.moments_of_inertia_Iyz  = MOI_Iyz 
    conditions.weights.vehicle.moments_of_inertia_Izx  = MOI_Izx 
    conditions.weights.vehicle.moments_of_inertia_Izy  = MOI_Izy 
    conditions.weights.vehicle.moments_of_inertia_Izz  = MOI_Izz 
    
    return 
     
def update_fuel_tank_moment_of_inertia(fuel_tank,state): 
    conditions        = state.conditions
    N                 = state.numerics.number_of_control_points
    center_of_gravity = conditions.weights.vehicle.global_center_of_gravity   
    fuel_tag          = fuel_tank.fuel.tag
    
    # mass of fuel 
    M_fuel            = conditions.weights.components.mass[fuel_tag]
    
    # local non-dimensional tensor 
    MOI_fuel_non_dim  = fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor
    
    # local dimensional tensor 
    I_fuel_local      = M_fuel[:,:, None]  * np.array(MOI_fuel_non_dim)[None,:,:]
    
    # moment arm 
    s       = np.array(center_of_gravity) - conditions.weights.components.global_center_of_gravity[fuel_tag]
    term_1  = np.repeat(np.repeat(np.vecdot(s, s)[:,None, None],3, axis=2), 3, axis=1)
    term_2 = np.repeat(np.array(np.identity(3))[None,:,:],N, axis=0)
    term_3  = np.multiply.outer(s, s)[:, :, 0, :]
    
    # parallel axis moment 
    I_fuel_par        = M_fuel[:,:, None] *  (term_1* term_2 - term_3 )     
    
    # total moment of inertia 
    I_fuel            = I_fuel_local  + I_fuel_par     
    
    # update data structures  
    conditions.weights.components.moments_of_inertia_Ixx[fuel_tag][:,0] = I_fuel[:,0,0]
    conditions.weights.components.moments_of_inertia_Ixy[fuel_tag][:,0] = I_fuel[:,0,1]
    conditions.weights.components.moments_of_inertia_Ixz[fuel_tag][:,0] = I_fuel[:,0,2]
    conditions.weights.components.moments_of_inertia_Iyx[fuel_tag][:,0] = I_fuel[:,1,0]
    conditions.weights.components.moments_of_inertia_Iyy[fuel_tag][:,0] = I_fuel[:,1,1]
    conditions.weights.components.moments_of_inertia_Iyz[fuel_tag][:,0] = I_fuel[:,1,2]
    conditions.weights.components.moments_of_inertia_Izx[fuel_tag][:,0] = I_fuel[:,2,0]
    conditions.weights.components.moments_of_inertia_Izy[fuel_tag][:,0] = I_fuel[:,2,1]
    conditions.weights.components.moments_of_inertia_Izz[fuel_tag][:,0] = I_fuel[:,2,2]
    
    return 