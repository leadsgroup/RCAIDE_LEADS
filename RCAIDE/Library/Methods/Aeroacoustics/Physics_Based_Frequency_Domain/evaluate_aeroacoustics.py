# RCAIDE/Library/Methods/Aeroacoustics/Physics_Based_Frequency_Domain/evaluate_aeroacoustics.py
#
# Created:   Jul 2023, M. Clarke
# Modified:  Oct 2024, A. Molloy

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE Imports
import RCAIDE
from RCAIDE.Library.Methods.Aeroacoustics.Common.decibel_arithmetic                                  import SPL_arithmetic
from RCAIDE.Library.Methods.Aeroacoustics.Common.generate_hemisphere_microphone_locations            import generate_hemisphere_microphone_locations
from RCAIDE.Library.Methods.Aeroacoustics.Physics_Based_Frequency_Domain.Rotor.compute_rotor_noise   import compute_rotor_noise

# package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  evaluate_aeroacoustics
# ----------------------------------------------------------------------------------------------------------------------
def evaluate_aeroacoustics(segment, settings, vehicle):
    """ Computes the sound pressure level on a hemisphere of ground microphones around each
    rotor source and sums the results into the total aircraft noise.

    Assumptions:
    Ground microphone angles start in front of the aircraft (0 deg) and sweep in a lateral
    direction to the starboard wing and around to the tail (180 deg)

    Source:
    N/A

    Inputs:
    settings.
        center_frequencies  - 1/3 octave band frequencies   [unitless]

    Outputs:
    None

    Properties Used:
    N/A
    """

    # unpack
    conditions           = segment.state.conditions
    dim_cf               = len(settings.center_frequencies )
    ctrl_pts             = int(segment.state.numerics.number_of_control_points)
    microphone_locations = generate_hemisphere_microphone_locations(settings)
    N_hemisphere_mics    = len(microphone_locations)

    # create empty arrays for results
    total_SPL_dBA          = np.ones((ctrl_pts,N_hemisphere_mics))*1E-16
    total_SPL_spectra      = np.ones((ctrl_pts,N_hemisphere_mics,dim_cf))*1E-16

    # iterate through sources and iteratively add rotor noise
    rotor_tag = None
    i = 0
    for network in vehicle.networks:
        for propulsor in network.propulsors:
            if propulsor.active == True:
                for sub_tag , sub_item in  propulsor.items():
                    if isinstance(sub_item, RCAIDE.Library.Components.Powertrain.Converters.Rotor):
                        rotor_tag         = compute_rotor_noise(microphone_locations,sub_item,segment,settings, rotor_index = i, previous_rotor_tag= rotor_tag, identical_propulsors=propulsor.identical_propulsors)
                        total_SPL_dBA     = SPL_arithmetic(np.concatenate((total_SPL_dBA[:,None,:],conditions.aeroacoustics.converters[sub_item.tag].SPL_dBA[:,None,:]),axis =1),sum_axis=1)
                        total_SPL_spectra = SPL_arithmetic(np.concatenate((total_SPL_spectra[:,None,:,:],conditions.aeroacoustics.converters[sub_item.tag].SPL_1_3_spectrum[:,None,:,:]),axis =1),sum_axis=1)
                        i += 1

    conditions.aeroacoustics.SPL_dBA              = (total_SPL_dBA) *  (1 - settings.noise_reduction_factors.SPL_dbA)
    conditions.aeroacoustics.SPL_1_3_spectrum_dBA = (total_SPL_spectra) * (1 - settings.noise_reduction_factors.SPL_dbA)
    return
