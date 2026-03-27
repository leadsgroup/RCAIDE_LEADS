# RCAIDE/Library/Methods/Mass_Properties/Moment_of_Inertia/compute_rounded_end_cylinder_moment_of_inertia.py 
# 
# Created:  Aug. 2025, M. Clarke  
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# rcaide imports
import RCAIDE

# package imports 
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Rounded-End Cylinder Moment of Inertia
# ----------------------------------------------------------------------------------------------------------------------   
def compute_rounded_end_cylinder_moment_of_inertia(component,outer_length,outer_radius,inner_length,inner_radius,
                                                   center_of_gravity = np.array([[0,0,0]]), fuel_tank=False):  
    """
    Computes the moment of inertia tensor for a hollow rounded-end cylinder.

    Parameters
    ----------
    origin : numpy.ndarray
        Origin coordinates of the component [m]
    mass : float
        Total mass of the cylinder [kg]
    outer_length : float
        Length of the outer cylinder [m]
    outer_radius : float
        Radius of the outer cylinder [m]
    inner_length : float, optional
        Length of the inner cylinder (for hollow cylinder) [m], default is 0
    inner_radius : float, optional
        Radius of the inner cylinder (for hollow cylinder) [m], default is 0
    center_of_gravity : numpy.ndarray, optional
        Global center of gravity coordinates [m], default is [0,0,0]

    Returns
    -------
    I_global : numpy.ndarray
        Moment of inertia tensor in global coordinate system [kg⋅m²]
        Shape: (3, 3) matrix
    mass : float
        Mass of the cylinder [kg]

    Notes
    -----
    This function computes the moment of inertia tensor for a hollow rounded-end
    cylinder using the parallel axis theorem to transform from local to global
    coordinate systems. The cylinder consists of a cylindrical section with
    hemispherical end caps.
    
    **Major Assumptions**
        * Cylinder has constant density throughout
        * Cylinder axis is aligned with the x-axis
        * Rounded ends are perfect hemispheres
        * Parallel axis theorem is valid for the transformation
        * Point mass approximation for zero radius/length cases
    
    **Theory**

    The volume of a rounded-end cylinder is:
    :math:`V = V_{cylinder} + V_{hemispheres} = \\pi r^2 L + \\frac{4}{3}\\pi r^3`

    For a hollow cylinder:
    :math:`V_{total} = V_{outer} - V_{inner}`

    The density is calculated as:
    :math:`\\rho = \\frac{m}{V}`

    The parallel axis theorem transforms the moment of inertia:
    :math:`I_{global} = I_{local} + m(\\|\\vec{s}\\|^2 \\mathbf{I} - \\vec{s} \\vec{s}^T)`

    where :math:`\\vec{s}` is the vector from the component origin to the global center of gravity.
    """

    # ----------------------------------------------------------------------------------------------------------------------
    # unpack 
    # ---------------------------------------------------------------------------------------------------------------------- 
    mass           = component.mass_properties.mass
        
    # ----------------------------------------------------------------------------------------------------------------------    
    # Setup
    # ----------------------------------------------------------------------------------------------------------------------           
    I =  np.zeros((3, 3))
    
    # ----------------------------------------------------------------------------------------------------------------------    
    # Moment of inertia in local system 
    # ----------------------------------------------------------------------------------------------------------------------
    # volume of cylindrical part 
    volume_cyl = (np.pi * (outer_radius ** 2 )* outer_length ) -  (np.pi * (inner_radius ** 2) * inner_length )
   
    # volume of sperical end caps 
    volume_sph =   ( 4 / 3 * np.pi * outer_radius ** 3) -  ( 4 / 3 * np.pi * inner_radius ** 3)  
   
    # total volume 
    volume     = volume_cyl + volume_sph
    
    # use volume to determine mass split (assume constant density)
    mass_cyl = mass * (volume_cyl / volume  )
    mass_sph = mass * (volume_sph / volume  )
        
    # MOI about cylindrical axis
    d = outer_length / 2 + (3/8)*outer_radius # distance from centroid to base of hemisphere
    I_cylinder_cylin_axis      = mass_cyl *  ( (outer_radius**2 + inner_radius**2)/4 + (outer_length**2)/12 )
    I_sperical_caps_cylin_axis = 2/5 * mass_sph  *  (outer_radius**5 - inner_radius**5)/ (outer_radius**3 - inner_radius**3)
    I_tot_cylin_axis           = I_cylinder_cylin_axis + I_sperical_caps_cylin_axis + mass_sph*(d**2)

    # MOI about longitudinal axis (passing through the center of the circle)
    I_cylinder_long_axis       = 0.5 * mass_cyl * (outer_radius**2 + inner_radius**2)
    I_sperical_caps_long_axis  = 2/5 * mass_sph *  (outer_radius**5 - inner_radius**5)/ (outer_radius**3 - inner_radius**3)
    I_tot_long_axis            = I_cylinder_long_axis + I_sperical_caps_long_axis    

    # depending on orientation of cylindrical tank     
    if isinstance(component, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank):
        if component.bwb_aft_tank:
            I[0][0] = I_tot_long_axis
            I[1][1] = I_tot_cylin_axis
            I[2][2] = I_tot_cylin_axis
        else:
            I[0][0] = I_tot_cylin_axis
            I[1][1] = I_tot_long_axis
            I[2][2] = I_tot_cylin_axis
    else: 
        I[0][0] =  I_tot_long_axis
        I[1][1] =  I_tot_cylin_axis
        I[2][2] =  I_tot_cylin_axis      

    # Store moment of inertia tensor on component 
    component.mass_properties.moments_of_inertia.tensor                 = I   
    component.mass_properties.moments_of_inertia.non_dimensional_tensor = I / mass  
    
    if fuel_tank == True:
        # unpack fuel 
        fuel      = component.fuel
        
        # compute MOI of fuel assume inner walls of tank is boundary of fuel
        _,_ = compute_rounded_end_cylinder_moment_of_inertia(fuel,inner_length, inner_radius,inner_length=0,inner_radius=0,center_of_gravity = center_of_gravity,fuel_tank=False)        
        
    return I,  mass