# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/compute_rotor_performance.py
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports
import RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Actuator_Disc_Theory.Actuator_Disk_performance as Actuator_Disk_performance
import RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Blade_Element_Momentum_Theory_Helmholtz_Wake.BEMT_Helmholtz_performance as BEMT_Helmholtz_performance
 
# ---------------------------------------------------------------------------------------------------------------------- 
#  Generalized Rotor Class
# ----------------------------------------------------------------------------------------------------------------------  
def compute_rotor_performance(rotor, conditions):
    """
    Analyzes a general rotor given geometry and operating conditions.
    
    Parameters
    ----------
    rotor : RCAIDE.Library.Components.Powertrain.Converters.Rotor
        Rotor component with the following attributes:
            - fidelity : str
                Analysis fidelity level ('Actuator_Disk_Theory' or 'Blade_Element_Momentum_Theory_Helmholtz_Wake')
            - tag : str
                Identifier for the rotor
            - number_of_blades : int
                Number of blades on the rotor
            - tip_radius : float
                Tip radius of the rotor [m]
            - hub_radius : float
                Hub radius of the rotor [m]
            - twist_distribution : array_like
                Blade twist distribution [radians]
            - chord_distribution : array_like
                Blade chord distribution [m]
            - orientation_euler_angles : list
                Orientation of the rotor [rad, rad, rad]
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - freestream : Data
                Freestream properties
                - density : array_like
                    Air density [kg/m³]
                - dynamic_viscosity : array_like
                    Dynamic viscosity [kg/(m·s)]
                - speed_of_sound : array_like
                    Speed of sound [m/s]
                - temperature : array_like
                    Temperature [K]
            - frames : Data
                Reference frames
                - body : Data
                    Body frame
                    - transform_to_inertial : array_like
                        Rotation matrix from body to inertial frame
                - inertial : Data
                    Inertial frame
                    - velocity_vector : array_like
                        Velocity vector in inertial frame [m/s]
            - energy : Data
                Energy conditions
                - converters : dict
                    Converter energy conditions indexed by tag
                    - throttle : array_like
                        Throttle setting [0-1]
    
    Returns
    -------
    None
        Results are stored in conditions.energy.converters[rotor.tag] with attributes
        depending on the fidelity level used. See the documentation for the specific
        analysis method for details on the outputs.
    
    Notes
    -----
    This function serves as a dispatcher that calls the appropriate rotor analysis method
    based on the specified fidelity level. It supports two fidelity levels:
    
        1. Actuator_Disk_Theory: A simplified model that treats the rotor as an actuator
           disk, suitable for preliminary design and analysis.
        
        2. Blade_Element_Momentum_Theory_Helmholtz_Wake: A higher-fidelity model that
           combines blade element theory with a Helmholtz wake model, providing more
           accurate predictions of rotor performance.
    
    The function simply checks the fidelity level and calls the corresponding analysis
    function, passing the rotor and conditions as arguments.
    
    **Major Assumptions**
        * The assumptions depend on the specific analysis method used
        * See the documentation for Actuator_Disk_performance or BEMT_Helmholtz_performance
          for details on the assumptions made by each method
    
    References
    ----------
    [1] Drela, M. "Qprop Formulation", MIT AeroAstro, June 2006 http://web.mit.edu/drela/Public/web/qprop/qprop_theory.pdf
    
    [2] Leishman, Gordon J. Principles of helicopter aerodynamics Cambridge university press, 2006.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Actuator_Disc_Theory.Actuator_Disk_performance
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Blade_Element_Momentum_Theory_Helmholtz_Wake.BEMT_Helmholtz_performance
    """
    
    if rotor.fidelity == 'Blade_Element_Momentum_Theory_Helmholtz_Wake': 

        BEMT_Helmholtz_performance(rotor,conditions)
                      
    elif rotor.fidelity == 'Actuator_Disk_Theory': 

        Actuator_Disk_performance(rotor,conditions)
     
    return