# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/append_rotor_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_rotor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_rotor_conditions(rotor, segment, energy_conditions, noise_conditions): 
    """
    Initializes and appends rotor conditions to the energy and noise conditions dictionaries.
    
    Parameters
    ----------
    rotor : Rotor
        The rotor component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the rotor is operating.
    energy_conditions : dict
        Dictionary containing energy-related conditions for all propulsion components.
    noise_conditions : dict
        Dictionary containing noise-related conditions for all propulsion components.
    
    Returns
    -------
    None
        This function modifies the energy_conditions and noise_conditions dictionaries in-place.
    
    Notes
    -----
    This function creates empty Conditions objects for the rotor's energy and noise
    characteristics within the respective dictionaries. These conditions will be populated 
    during the mission analysis process.
    
    The energy conditions include various performance metrics such as:
    - Orientation and thrust vector angle
    - Blade pitch command
    - Torque, thrust, and throttle settings
    - RPM and angular velocity
    - Disc and power loading
    - Tip Mach number
    - Efficiency and figure of merit
    - Power coefficient
    
    All values are initialized as zero or one arrays (as appropriate) with the same
    length as the segment's state vector.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.compute_rotor_performance
    """
    ones_row    = segment.state.ones_row 
    energy_conditions[rotor.tag]                               = Conditions()   
    energy_conditions[rotor.tag].orientation                   = 0. * ones_row(3) 
    energy_conditions[rotor.tag].commanded_thrust_vector_angle = 0. * ones_row(1) 
    energy_conditions[rotor.tag].blade_pitch_command           = ones_row(1) * rotor.blade_pitch_command
    energy_conditions[rotor.tag].torque                        = 0. * ones_row(1)
    energy_conditions[rotor.tag].throttle                      = ones_row(1)
    energy_conditions[rotor.tag].thrust                        = 0. * ones_row(1)
    energy_conditions[rotor.tag].rpm                           = 0. * ones_row(1)
    energy_conditions[rotor.tag].omega                         = 0. * ones_row(1)
    energy_conditions[rotor.tag].disc_loading                  = 0. * ones_row(1)                 
    energy_conditions[rotor.tag].power_loading                 = 0. * ones_row(1)
    energy_conditions[rotor.tag].tip_mach                      = 0. * ones_row(1)
    energy_conditions[rotor.tag].efficiency                    = 0. * ones_row(1)
    energy_conditions[rotor.tag].figure_of_merit               = 0. * ones_row(1)
    energy_conditions[rotor.tag].power_coefficient             = 0. * ones_row(1) 
    noise_conditions[rotor.tag]                                = Conditions() 
    return 
