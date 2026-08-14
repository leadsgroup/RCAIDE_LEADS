# RCAIDE/Framework/Analyses/Aeroacoustics/Semi_Empirical.py
#
# Created:  Jul 2023, M. Clarke
# Modified: Aug 2026, P. Siripun

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Aeroacoustics.Common.decibel_arithmetic                       import SPL_arithmetic
from RCAIDE.Library.Methods.Aeroacoustics.Common.generate_zero_elevation_microphone_locations import generate_zero_elevation_microphone_locations
from RCAIDE.Library.Methods.Aeroacoustics.Common.atmospheric_attenuation                  import atmospheric_attenuation
from RCAIDE.Library.Methods.Aeroacoustics.Common.compute_lateral_attenuation              import compute_lateral_attenuation
from RCAIDE.Library.Methods.Aeroacoustics.Metrics.A_weighting_metric                      import A_weighting_metric
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Airframe.landing_gear_noise_model import compute_landing_gear_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Airframe.flap_noise_model        import flap_noise_model
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Airframe.slat_noise_model        import slat_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.compute_fan_noise import compute_fan_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.compute_core_noise import compute_core_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.compute_jet_noise import compute_jet_noise
from RCAIDE.Framework.Core                                                                import Data
from .Aeroacoustics import Aeroacoustics

# Python Imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Semi_Empirical
# ----------------------------------------------------------------------------------------------------------------------
class Semi_Empirical(Aeroacoustics):
    """
    This analysis calls RCAIDE's semi-empirical noise models for individual airframe and
    turbofan engine components and sums the results into the total sound pressure level at a
    grid of ground receptors under the flight path.

    Assumptions
    -----------
    * Flat, level ground at the mission's local horizontal-position origin (matches
      segment.state.conditions.frames.inertial.position_vector).
    * Only receptors within settings.noise_receptor_search_radius of the aircraft's current
      ground position are evaluated at each control point.
    * Jet noise assumes no reverse-thrust/idle-approach ground operation (compute_jet_noise's
      velocity-reduction flag is always 0) -- there is no tracked reverse-thrust state in
      conditions to detect that automatically.

    References
    ----------
    [1] Guo, Yueping. "A Semi-Empirical Model for Aircraft Landing Gear Noise Prediction." AIAA 2006-2627.
    [2] Guo, Yueping. "Aircraft Flap Side Edge Noise Modeling and Prediction" (2012)
    [3] Guo, Yueping. "Aircraft Slat Noise Modeling and Prediction" (2010)
    [4] SAE-AIR-5662: Method for Predicting Lateral Attenuation of Airplane Noise
    [5] Enhanced Fan Noise Modeling for Turbofan Engines (NASA)
    [6] Enhanced Core Noise Modeling for Turbofan Engines (NASA)
    """

    def __defaults__(self):
        self.tag                                    = "Semi_Empirical"
        self.settings.noise_hemisphere_radius       = 50
        self.settings.noise_receptor_search_radius  = 2000  # [m] receptors farther than this are skipped per control point

        # empirical model calibration constants, exposed here so they can be tuned/overridden
        # per vehicle without editing the noise model source
        self.settings.landing_gear_noise_parameters = Data(
            Low  = Data(beta=4.5e-8, St0=1.0, sigma=4.0, mu=2.5, q=2.6, h=0.2, A=3.53, B=0.62),
            Mid  = Data(beta=1.5e-8, St0=0.3, sigma=3.0, mu=1.5, q=4.2, h=0.6, A=0.42, B=0.18),
            High = Data(beta=3.2e-5, St0=0.1, sigma=2.0, mu=1.1, q=4.2, h=1.0, A=0.08, B=0.10),
        )
        self.settings.flap_noise_parameters = Data(
            A0=3e-5, mu0=0.7693, mu1=1.0, mu2=0.292, alpha_0=0.008, sigma_f=0.436332,
        )
        self.settings.slat_noise_parameters = Data(
            amplitude=1e-5, St_peak=2.0,
        )
        return

    def evaluate_aeroacoustics(self, segment, vehicle):
        # unpack
        settings   = self.settings
        conditions = segment.state.conditions
        dim_cf     = len(settings.center_frequencies)
        ctrl_pts   = int(segment.state.numerics.number_of_control_points)
        frequency  = settings.center_frequencies[5:]

        # ground receptor grid, in the same local flat-earth frame as
        # conditions.frames.inertial.position_vector [meters]
        receptor_locations = generate_zero_elevation_microphone_locations(settings)
        search_radius      = settings.noise_receptor_search_radius

        # create empty arrays for results
        total_SPL_dBA     = np.ones((ctrl_pts, len(receptor_locations))) * 1E-16
        total_SPL_spectra = np.ones((ctrl_pts, len(receptor_locations), dim_cf)) * 1E-16

        landing_gears = [gear for gear in vehicle.landing_gears if gear.gear_extended]
        control_surfaces = [(wing, cs) for wing in vehicle.wings for cs in wing.control_surfaces if cs.deflection != 0]

        has_turbofans = False
        for network in vehicle.networks:
            for propulsor in network.propulsors:
                if propulsor.active and isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan):
                    has_turbofans = True

        if not (landing_gears or control_surfaces or has_turbofans):
            conditions.aeroacoustics.hemisphere_SPL_dBA              = total_SPL_dBA * (1 - settings.noise_reduction_factors.SPL_dbA)
            conditions.aeroacoustics.hemisphere_SPL_1_3_spectrum_dBA = total_SPL_spectra * (1 - settings.noise_reduction_factors.SPL_dbA)
            return

        aircraft_position = conditions.frames.inertial.position_vector    # [m], z negative-up
        aircraft_velocity = conditions.frames.inertial.velocity_vector    # [m/s]

        # --- geometry pass: cheap, per control point, pure numpy. Receptor selection is
        # inherently ragged (a different number of nearby receptors at each control point as
        # the aircraft moves), so this part can't be flattened into one dense array op. The
        # expensive part -- the noise model evaluations -- is batched below instead.
        cpt_list, receptor_list, R_list, theta_list, l_seg_list, d_seg_list = [], [], [], [], [], []

        for cpt in range(ctrl_pts):
            ac_pos        = aircraft_position[cpt]
            heading_speed = np.linalg.norm(aircraft_velocity[cpt])
            heading       = aircraft_velocity[cpt] / heading_speed if heading_speed > 0 else np.array([1.0, 0.0, 0.0])

            # vector from the aircraft to every receptor (receptor z = 0, i.e. ground level)
            relative_position = receptor_locations - ac_pos
            R = np.linalg.norm(relative_position, axis=1)

            nearby = np.where(R <= search_radius)[0]
            if len(nearby) == 0:
                continue

            R_nearby = R[nearby]
            rel_unit = relative_position[nearby] / R_nearby[:, None]

            # polar emission angle from the nose (0) to the tail (180), matching the
            # noise models' convention
            theta = np.arccos(np.clip(rel_unit @ heading, -1.0, 1.0))

            # sideline distance / AGL altitude split, for lateral attenuation
            along_track = relative_position[nearby, 0:2] @ heading[0:2]
            l_seg       = np.sqrt(np.maximum(R_nearby**2 - along_track**2 - relative_position[nearby, 2]**2, 0.0))
            d_seg       = np.full_like(l_seg, -ac_pos[2])

            cpt_list.append(np.full(len(nearby), cpt))
            receptor_list.append(nearby)
            R_list.append(R_nearby)
            theta_list.append(theta)
            l_seg_list.append(l_seg)
            d_seg_list.append(d_seg)

        if not cpt_list:
            # no control point had any receptor in range
            conditions.aeroacoustics.hemisphere_SPL_dBA              = total_SPL_dBA * (1 - settings.noise_reduction_factors.SPL_dbA)
            conditions.aeroacoustics.hemisphere_SPL_1_3_spectrum_dBA = total_SPL_spectra * (1 - settings.noise_reduction_factors.SPL_dbA)
            return

        cpt_arr      = np.concatenate(cpt_list)
        receptor_arr = np.concatenate(receptor_list)
        R_arr        = np.concatenate(R_list)
        theta_arr    = np.concatenate(theta_list)
        l_seg_arr    = np.concatenate(l_seg_list)
        d_seg_arr    = np.concatenate(d_seg_list)

        R_val     = R_arr.reshape(-1, 1)
        theta_col = theta_arr.reshape(-1, 1)

        # microphone locations in the aircraft body frame (x forward, y lateral), used by
        # compute_jet_noise: matches the R_val/theta_col convention exactly, since
        # atan2(R sin(theta), R cos(theta)) recovers theta for theta in [0, pi]
        mic_locations = np.stack([R_arr * np.cos(theta_arr), R_arr * np.sin(theta_arr), np.zeros_like(R_arr)], axis=1)

        # --- noise evaluation pass: one batched call per component across every control
        # point x receptor pair, instead of one call per control point ---
        component_spectra = []

        for gear in landing_gears:
            spl = compute_landing_gear_noise(R_val, theta_col, gear, vehicle, cpt_arr, frequency, segment, self).Total
            component_spectra.append(spl)

        for wing, control_surface in control_surfaces:
            if type(control_surface) is RCAIDE.Library.Components.Wings.Control_Surfaces.Flap:
                spl = flap_noise_model(R_val, theta_col, control_surface, wing, cpt_arr, frequency, segment, self)
                component_spectra.append(spl)
            elif type(control_surface) is RCAIDE.Library.Components.Wings.Control_Surfaces.Slat:
                spl = slat_noise(R_val, theta_col, control_surface, wing, cpt_arr, segment, frequency, self)
                component_spectra.append(spl)

        for network in vehicle.networks:
            for propulsor in network.propulsors:
                if not (propulsor.active and (isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) or isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet))):
                    continue 
                pressure_ratio = propulsor.fan.pressure_ratio * propulsor.low_pressure_compressor.pressure_ratio * propulsor.high_pressure_compressor.pressure_ratio
                
                if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan): 
                    fan_noise = compute_fan_noise(R_val, theta_col, propulsor, None, cpt_arr, segment, frequency)
                    component_spectra.append(fan_noise.SPL_1_3_spectrum[0])

                core_noise = compute_core_noise(R_val, theta_col, propulsor, pressure_ratio, cpt_arr, segment, frequency)
                component_spectra.append(core_noise.SPL_1_3_spectrum[0])

                jet_noise = compute_jet_noise(mic_locations, propulsor, cpt_arr, segment, frequency, 0)
                component_spectra.append(jet_noise.SPL_1_3_spectrum[0])

        total_spectrum = SPL_arithmetic(np.array(component_spectra), sum_axis=0)

        att_dB     = atmospheric_attenuation(R_arr, frequency)
        LADJ_dB, _ = compute_lateral_attenuation(l_seg_arr, d_seg_arr)
        attenuated = total_spectrum - att_dB - LADJ_dB[:, None]

        total_SPL_spectra[cpt_arr, receptor_arr, 5:] = attenuated
        total_SPL_dBA[cpt_arr, receptor_arr]         = SPL_arithmetic(A_weighting_metric(attenuated, frequency), sum_axis=1)

        conditions.aeroacoustics.hemisphere_SPL_dBA              = total_SPL_dBA * (1 - settings.noise_reduction_factors.SPL_dbA)
        conditions.aeroacoustics.hemisphere_SPL_1_3_spectrum_dBA = total_SPL_spectra * (1 - settings.noise_reduction_factors.SPL_dbA)
        return
