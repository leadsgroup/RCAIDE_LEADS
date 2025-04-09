# RCAIDE/Library/Methods/Powertrain/Propulsors/Common/
# 
# Created:  Jan 2025, M. Clarke  


# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE   
from RCAIDE.Framework.Mission.Common import Results, Residuals
from RCAIDE.Library.Mission.Common.Update.orientations import orientations

# Python package imports
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Operating Test Conditions Set-up
# ---------------------------------------------------------------------------------------------------------------------- 
def setup_operating_conditions(component, altitude=0, velocity_range=np.array([10]), angle_of_attack=0):
    """
    Sets up operating conditions for single component analysis.
    
    Parameters
    ----------
    component : RCAIDE.Library.Components.Component
        Component to set up operating conditions for
            - working_fluid : Data
                Working fluid properties object (will be set by this function)
    altitude : float, optional
        Altitude for analysis [m]
        Default: 0 (sea level)
    velocity_range : numpy.ndarray, optional
        Array of velocities to analyze [m/s]
        Default: np.array([10])
    angle_of_attack : float, optional
        Angle of attack for analysis [deg]Default: 0
    
    Returns
    -------
    state : RCAIDE.Framework.Mission.Common.State
        State object containing:
            - conditions : Data
                Flight conditions
                    - freestream : Data
                        Freestream properties
                            - altitude : numpy.ndarray
                                Altitude [m]
                            - mach_number : numpy.ndarray
                                Mach number
                            - pressure : numpy.ndarray
                                Atmospheric pressure [Pa]
                            - temperature : numpy.ndarray
                                Atmospheric temperature [K]
                            - density : numpy.ndarray
                                Air density [kg/m³]
                            - dynamic_viscosity : numpy.ndarray
                                Air dynamic viscosity [kg/(m·s)]
                            - gravity : numpy.ndarray
                                Gravitational acceleration [m/s²]
                            - isentropic_expansion_factor : numpy.ndarray
                                Ratio of specific heats (gamma)
                            - Cp : numpy.ndarray
                                Specific heat at constant pressure [J/(kg·K)]
                            - R : numpy.ndarray
                                Gas constant [J/(kg·K)]
                            - speed_of_sound : numpy.ndarray
                                Speed of sound [m/s]
                            - velocity : numpy.ndarray
                                Freestream velocity [m/s]
                    - frames : Data
                        Reference frames
                            - body : Data
                                Body-fixed frame
                                    - inertial_rotations : numpy.ndarray
                                        Rotation angles [rad]
                            - inertial : Data
                                Inertial frame
                                    - velocity_vector : numpy.ndarray
                                        Velocity vector [m/s]
    
    Notes
    -----
    This function creates a standardized set of operating conditions for component analysis.
    It sets up atmospheric conditions based on the US Standard Atmosphere 1976 model and
    initializes all necessary parameters for component performance evaluation.
    
    The function:
        1. Sets up Earth as the planet and air as the working fluid
        2. Computes atmospheric properties at the specified altitude
        3. Creates a conditions data structure with all necessary parameters
        4. Sets up reference frames and orientations
        5. Appends component-specific operating conditions
    
    **Major Assumptions**
        * US Standard Atmosphere 1976
        * Earth gravity
        * Air as working fluid
    
    See Also
    --------
    RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976
    RCAIDE.Library.Mission.Common.Update.orientations
    """
    
    planet                                            = RCAIDE.Library.Attributes.Planets.Earth()
    working_fluid                                     = RCAIDE.Library.Attributes.Gases.Air() 
    
    # append working fluid properties 
    component.working_fluid                           = working_fluid     
    
    atmosphere_sls                                    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data                                         = atmosphere_sls.compute_values(altitude,0.0) 
    p                                                 = atmo_data.pressure          
    T                                                 = atmo_data.temperature       
    rho                                               = atmo_data.density          
    a                                                 = atmo_data.speed_of_sound    
    mu                                                = atmo_data.dynamic_viscosity 
                                                      
    conditions                                        = Results() 
    conditions.freestream.altitude                    = np.atleast_2d(altitude)
    conditions.freestream.mach_number                 = np.atleast_2d(velocity_range/a)
    conditions.freestream.pressure                    = np.atleast_2d(p)
    conditions.freestream.temperature                 = np.atleast_2d(T)
    conditions.freestream.density                     = np.atleast_2d(rho)
    conditions.freestream.dynamic_viscosity           = np.atleast_2d(mu)
    conditions.freestream.gravity                     = np.atleast_2d(planet.sea_level_gravity)
    conditions.freestream.isentropic_expansion_factor = np.atleast_2d(working_fluid.compute_gamma(T,p))
    conditions.freestream.Cp                          = np.atleast_2d(working_fluid.compute_cp(T,p))
    conditions.freestream.R                           = np.atleast_2d(working_fluid.gas_specific_constant)
    conditions.freestream.speed_of_sound              = np.atleast_2d(a)

    num_ctrl_pts      = len(velocity_range)    
    conditions._size  = num_ctrl_pts
    conditions.expand_rows(num_ctrl_pts)
     
    conditions.freestream.velocity                    = np.atleast_2d(velocity_range) 
    conditions.frames.body.inertial_rotations[:, 1]   = angle_of_attack
    conditions.frames.inertial.velocity_vector[:, 0]  = np.atleast_2d(velocity_range)

    # setup conditions   
    segment                                          = RCAIDE.Framework.Mission.Segments.Segment()  
    segment.state.conditions                         = conditions    
    orientations(segment) 
    segment.state.residuals.network                  = Residuals()
    
    # append component-specific operating conditions 
    component.append_operating_conditions(segment,segment.state.conditions.energy,segment.state.conditions.noise)    
    segment.state.conditions.expand_rows(num_ctrl_pts)              
    return segment.state
 
    
    