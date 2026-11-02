# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/append_rotor_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# package imports
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_rotor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_rotor_conditions(rotor, segment): 
    """
    Initializes and appends rotor conditions to the energy and noise conditions dictionaries.
    
    Parameters
    ----------
    rotor : Rotor
        The rotor component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the rotor is operating.
    
    Returns
    -------
    None
        This function modifies the segment.state.conditions.energy and segment.state.conditions.aeroacoustics dictionaries in-place.
    
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
    segment.state.conditions.energy.converters[rotor.tag]                               = Conditions()   
    segment.state.conditions.energy.converters[rotor.tag].orientation                   = 0. * ones_row(3) 
    segment.state.conditions.energy.converters[rotor.tag].design_flag                   = False 
    segment.state.conditions.energy.converters[rotor.tag].commanded_thrust_vector_angle = 0. * ones_row(1) 
    segment.state.conditions.energy.converters[rotor.tag].blade_pitch_command           = ones_row(1) * rotor.blade_pitch_command 
    segment.state.conditions.energy.converters[rotor.tag].torque                        = 0. * ones_row(1)
    segment.state.conditions.energy.converters[rotor.tag].throttle                      = ones_row(1)
    segment.state.conditions.energy.converters[rotor.tag].thrust                        = 0. * ones_row(3)
    segment.state.conditions.energy.converters[rotor.tag].rpm                           = 0. * ones_row(1)
    segment.state.conditions.energy.converters[rotor.tag].omega                         = 0. * ones_row(1)
    segment.state.conditions.energy.converters[rotor.tag].disc_loading                  = 0. * ones_row(1)                 
    segment.state.conditions.energy.converters[rotor.tag].power_loading                 = 0. * ones_row(1)
    segment.state.conditions.energy.converters[rotor.tag].tip_mach                      = 0. * ones_row(1)
    segment.state.conditions.energy.converters[rotor.tag].efficiency                    = 0. * ones_row(1)
    segment.state.conditions.energy.converters[rotor.tag].figure_of_merit               = 0. * ones_row(1)
    segment.state.conditions.energy.converters[rotor.tag].power_coefficient             = 0. * ones_row(1) 

    # outputs of the rotor performance model, by shape: per point, per point and blade station (Nr), per point,
    # station and azimuth (Na); azimuthal_distribution and the station counts describe the disc, not the flight.
    # Only a rotor that can run its model has them: BEMT needs blade stations, so a rotor without blade geometry
    # (e.g. a turboprop's lumped-efficiency propeller) is never evaluated by it.
    rotor_conditions = segment.state.conditions.energy.converters[rotor.tag]
    rows             = ones_row(1).shape[0]
    bemt             = rotor.fidelity == 'Blade_Element_Momentum_Theory_Helmholtz_Wake'
    if not bemt or np.ndim(rotor.chord_distribution) == 1:
        per_point = ['power', 'advance_ratio', 'speed_of_sound', 'density', 'thrust_per_blade', 'torque_per_blade',
                     'thrust_coefficient', 'torque_coefficient']
        if bemt:
            Nr, Na    = len(rotor.chord_distribution), rotor.number_azimuthal_stations
            per_point = per_point + ['rotor_drag', 'rotor_drag_coefficient']
            for key in ['blade_tangential_induced_velocity', 'blade_axial_induced_velocity', 'blade_reynolds_number',
                        'blade_effective_angle_of_attack', 'blade_tangential_velocity', 'blade_axial_velocity',
                        'blade_velocity', 'blade_Mach_number', 'drag_coefficient', 'lift_coefficient', 'blade_dT_dr',
                        'blade_thrust_distribution', 'blade_dQ_dr', 'blade_torque_distribution',
                        'converged_inflow_ratio', 'blade_H_distribution']:
                rotor_conditions[key] = np.zeros((rows, Nr))
            for key in ['disc_radial_distribution', 'disc_reynolds_number', 'disc_effective_angle_of_attack',
                        'disc_tangential_induced_velocity', 'disc_axial_induced_velocity', 'disc_tangential_velocity',
                        'disc_axial_velocity', 'disc_velocity', 'disc_Mach_number', 'disc_circulation', 'disc_dT_dr',
                        'disc_thrust_distribution', 'disc_azimuthal_distribution', 'disc_dQ_dr',
                        'disc_torque_distribution']:
                rotor_conditions[key] = np.zeros((rows, Nr, Na))
            rotor_conditions.azimuthal_distribution    = np.zeros(Na)
            rotor_conditions.number_radial_stations    = Nr
            rotor_conditions.number_azimuthal_stations = Na
        for key in per_point:
            rotor_conditions[key] = 0. * ones_row(1)
        rotor_conditions.velocity = 0. * ones_row(3)

    segment.state.conditions.aeroacoustics.converters[rotor.tag]                                = Conditions() 
    return 
