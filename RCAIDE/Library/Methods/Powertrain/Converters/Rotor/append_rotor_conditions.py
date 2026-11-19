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
    segment.state.conditions.energy.converters[rotor.tag].throttle                      = ones_row(1)
    segment.state.conditions.energy.converters[rotor.tag].omega                         = 0. * ones_row(1)

    # outputs of any rotor performance method; a field the selected method does not compute stays NaN
    rotor_conditions = segment.state.conditions.energy.converters[rotor.tag]
    rows             = ones_row(1).shape[0]
    for key, shape in rotor_conditions_schema(rotor).items():
        rotor_conditions[key] = np.full((rows,) + shape, np.nan)
    rotor_conditions.azimuthal_distribution    = np.full(rotor.number_azimuthal_stations, np.nan)
    rotor_conditions.number_radial_stations    = blade_stations(rotor)
    rotor_conditions.number_azimuthal_stations = rotor.number_azimuthal_stations

    segment.state.conditions.aeroacoustics.converters[rotor.tag]                                = Conditions() 
    return


def rotor_conditions_schema(rotor):
    """
    Output fields of a rotor's conditions and their shape per control point, the same for every performance method.

    Parameters
    ----------
    rotor : RCAIDE.Library.Components.Powertrain.Converters.Rotor
        Rotor with chord_distribution and number_azimuthal_stations

    Returns
    -------
    schema : dict
        field -> shape per control point: (1,) scalars, (3,) vectors, (Nr,) blade stations and (Nr, Na) disc
        stations, with Nr blade stations (0 for a rotor without blade geometry) and Na azimuthal stations
    """
    Nr, Na = blade_stations(rotor), rotor.number_azimuthal_stations
    schema = {key: (1,) for key in
              ['torque', 'power', 'rpm', 'tip_mach', 'efficiency', 'figure_of_merit', 'advance_ratio',
               'speed_of_sound', 'density', 'disc_loading', 'power_loading', 'thrust_per_blade', 'torque_per_blade',
               'thrust_coefficient', 'torque_coefficient', 'power_coefficient', 'rotor_drag',
               'rotor_drag_coefficient']}
    schema.update({key: (3,) for key in ['thrust', 'velocity']})
    schema.update({key: (Nr,) for key in
                   ['blade_tangential_induced_velocity', 'blade_axial_induced_velocity', 'blade_reynolds_number',
                    'blade_effective_angle_of_attack', 'blade_tangential_velocity', 'blade_axial_velocity',
                    'blade_velocity', 'blade_Mach_number', 'drag_coefficient', 'lift_coefficient', 'blade_dT_dr',
                    'blade_thrust_distribution', 'blade_dQ_dr', 'blade_torque_distribution',
                    'converged_inflow_ratio', 'blade_H_distribution']})
    schema.update({key: (Nr, Na) for key in
                   ['disc_radial_distribution', 'disc_azimuthal_distribution', 'disc_reynolds_number',
                    'disc_effective_angle_of_attack', 'disc_tangential_induced_velocity',
                    'disc_axial_induced_velocity', 'disc_tangential_velocity', 'disc_axial_velocity', 'disc_velocity',
                    'disc_Mach_number', 'disc_circulation', 'disc_dT_dr', 'disc_thrust_distribution', 'disc_dQ_dr',
                    'disc_torque_distribution']})
    return schema


def blade_stations(rotor):
    """Number of radial blade stations, 0 for a rotor without blade geometry."""
    return np.size(rotor.chord_distribution) if np.ndim(rotor.chord_distribution) == 1 else 0
