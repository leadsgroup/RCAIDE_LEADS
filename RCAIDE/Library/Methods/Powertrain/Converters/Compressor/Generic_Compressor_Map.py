# RCAIDE/Library/Methods/Powertrain/Converters/Compressor/Generic_Compressor_Map.py
#
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Framework.Core import Data

# Python package imports
import numpy as np

# Physical floor on extrapolated efficiency -- a real (or windmilling) compressor
# stage doesn't have zero or negative efficiency, so this bounds the linear
# extrapolation used far outside the tabulated 60-110% corrected-speed range.
MINIMUM_ADIABATIC_EFFICIENCY = 0.30

# ----------------------------------------------------------------------------------------------------------------------
#  interp_with_linear_extrapolation
# ----------------------------------------------------------------------------------------------------------------------
def interp_with_linear_extrapolation(x, xp, fp):
    """
    Like `numpy.interp`, but linearly extrapolates past the ends of the table
    using the slope of the nearest segment, instead of clamping to the
    boundary value. Needed for `Generic_Compressor_Map`: clamping would turn
    the table's own tabulated range (60-110% corrected speed) into a hard
    wall, exactly the kind of artificial limit this map exists to remove.
    Linear extrapolation is a genuine approximation (the real droop is
    unlikely to stay linear far below 60%), not a precise prediction -- but it
    is a strictly better approximation than assuming the map goes flat at its
    lowest tabulated point, which is what clamping would otherwise imply.

    Parameters
    ----------
    x : float or numpy.ndarray
        Query point(s).
    xp : numpy.ndarray
        Table x-values, strictly increasing.
    fp : numpy.ndarray
        Table y-values, same length as `xp`.

    Returns
    -------
    numpy.ndarray
        Interpolated (or linearly extrapolated) values at `x`.
    """
    x = np.atleast_1d(np.asarray(x, dtype=float))
    result = np.interp(x, xp, fp)

    below = x < xp[0]
    if np.any(below):
        slope = (fp[1] - fp[0]) / (xp[1] - xp[0])
        result = np.where(below, fp[0] + slope * (x - xp[0]), result)

    above = x > xp[-1]
    if np.any(above):
        slope = (fp[-1] - fp[-2]) / (xp[-1] - xp[-2])
        result = np.where(above, fp[-1] + slope * (x - xp[-1]), result)

    return result

# ----------------------------------------------------------------------------------------------------------------------
# Generic_Compressor_Map
# ----------------------------------------------------------------------------------------------------------------------
class Generic_Compressor_Map(Data):
    """
    Generic, non-engine-specific compressor performance characteristic, scaled
    to a compressor's own design point. Provides pressure ratio and adiabatic
    efficiency as a function of corrected shaft speed, closing the off-design
    matching equations with the physics a constant-efficiency assumption is
    missing: a real compressor's efficiency droops away from its design
    corrected speed, most sharply below about 60% of design.

    This model is deliberately not a full two-dimensional compressor map
    (pressure ratio and efficiency as functions of *both* corrected mass flow
    and corrected speed, with a surge line). It captures only the WELL-MATCHED
    OPERATING LINE of a generic compressor -- i.e. how pressure ratio,
    corrected mass flow, and efficiency move together as speed changes for a
    correctly matched engine. That is sufficient to replace a constant
    adiabatic efficiency in a throttle/altitude/Mach off-design sweep (which
    moves the operating point along the operating line), but not for
    surge-margin or inlet-distortion analysis, which need the full map.

    Notes
    -----
    Data source: read by eye from Mattingly, Heiser, and Pratt, "Aircraft
    Engine Design", 2nd ed., Fig. 5.5 ("Typical compressor map") -- a generic,
    representative compressor map presented in the text, not measurement data
    from any specific real engine (SAFAM or otherwise). The tabulated values
    below are an approximate hand-reading of that published figure's plotted
    operating line and efficiency contours, not an automated digitization, and
    should be understood as capturing the qualitative droop behavior a
    constant-efficiency model misses, not as a precisely accurate prediction
    for any particular real compressor. Fig. 5.7/5.8 of the same reference
    show turbine efficiency staying comparatively flat over a wide corrected-
    speed range, and the text explicitly endorses a constant adiabatic
    turbine efficiency for preliminary design -- so no equivalent turbine map
    is provided here; the existing constant-efficiency turbine treatment
    already matches that guidance.

    **Major Assumptions**
        * The compressor operates on its own well-matched operating line (no
          independent throttle/backpressure degree of freedom away from that
          line).
        * Design point corresponds to 100% corrected speed, 100% corrected
          mass flow, 100% design pressure ratio, peak efficiency.
        * Values are linearly interpolated (and, outside the tabulated range,
          linearly extrapolated) between the table rows below.

    References
    ----------
    [1] Mattingly, J. D., Heiser, W. H., and Pratt, D. T., "Aircraft Engine
        Design", 2nd ed., AIAA Education Series, 2002, Fig. 5.5, p. 165, and
        Sec. 5.3.5 ("Component Matching"), pp. 168-169 (efficiency "remains
        essentially constant... in the 70-100% engine speed range", with "a
        significant reduction... when engine speed... drops below 60% of
        design").

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Compressor.compute_compressor_performance
    """

    def __defaults__(self):
        """This sets the default values for the generic map data.

        Assumptions:
            None

        Source:
            Mattingly, Heiser, and Pratt, "Aircraft Engine Design", 2nd ed., Fig. 5.5.
        """
        # Hand-read from Fig. 5.5's plotted operating line and efficiency contours,
        # in percent of the compressor's own design-point values.
        self.percent_corrected_speed         = np.array([60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0, 95.0, 100.0, 105.0, 110.0])
        self.percent_corrected_mass_flow     = np.array([40.0, 48.0, 55.0, 61.0, 68.0, 75.0, 83.0, 91.0, 100.0, 105.0, 109.0])
        self.percent_design_pressure_ratio   = np.array([22.0, 28.0, 35.0, 43.0, 52.0, 63.0, 75.0, 87.0, 100.0, 115.0, 130.0])
        self.adiabatic_efficiency            = np.array([0.700, 0.745, 0.780, 0.805, 0.825, 0.840, 0.855, 0.865, 0.870, 0.855, 0.825])

    def scale_to_design_point(self, design_pressure_ratio, design_adiabatic_efficiency=None):
        """
        Returns a copy of this generic map with pressure ratio (and, if given,
        efficiency) rescaled so its 100%-corrected-speed row matches a specific
        compressor's actual design point.

        Parameters
        ----------
        design_pressure_ratio : float
            The compressor's actual design-point pressure ratio. The generic
            map's percent-of-design pressure ratio column is scaled by this
            value so `query` returns absolute pressure ratio directly.
        design_adiabatic_efficiency : float, optional
            The compressor's actual design-point adiabatic efficiency. If
            given, the generic map's efficiency row is rescaled multiplicatively
            so it peaks at this value at 100% corrected speed, rather than the
            generic map's own peak of 0.87 -- the *shape* of the droop is kept
            generic, but its absolute level is anchored to what's actually
            known about this compressor.

        Returns
        -------
        Generic_Compressor_Map
            A new map instance scaled to the given design point.
        """
        scaled = Generic_Compressor_Map()
        scaled.percent_corrected_speed       = self.percent_corrected_speed.copy()
        scaled.percent_corrected_mass_flow   = self.percent_corrected_mass_flow.copy()
        scaled.percent_design_pressure_ratio = self.percent_design_pressure_ratio.copy()
        scaled.adiabatic_efficiency          = self.adiabatic_efficiency.copy()
        scaled.design_pressure_ratio         = design_pressure_ratio

        if design_adiabatic_efficiency is not None:
            peak_generic_efficiency   = self.adiabatic_efficiency.max()
            scaled.adiabatic_efficiency = self.adiabatic_efficiency * (design_adiabatic_efficiency / peak_generic_efficiency)

        return scaled

    def query(self, percent_corrected_speed):
        """
        Interpolates pressure ratio, corrected mass flow, and adiabatic
        efficiency at a given percent of design corrected speed.

        Parameters
        ----------
        percent_corrected_speed : float or numpy.ndarray
            Corrected shaft speed as a percentage of the design-point corrected
            speed.

        Returns
        -------
        pressure_ratio : float or numpy.ndarray
            Pressure ratio at the queried speed. Absolute if `scale_to_design_point`
            has set `design_pressure_ratio`; otherwise a percentage of design.
        percent_corrected_mass_flow : float or numpy.ndarray
            Corrected mass flow, as a percentage of the design-point corrected
            mass flow.
        adiabatic_efficiency : float or numpy.ndarray
            Adiabatic efficiency at the queried speed.
        """
        percent_corrected_speed = np.atleast_1d(percent_corrected_speed).astype(float)

        # Pressure ratio is extrapolated in LOG space, not linearly: pressure ratio's
        # rapid, accelerating rise with speed means a linear extrapolation of the
        # lowest-tabulated-segment slope overshoots badly outside the table --
        # verified directly (goes negative below ~40% corrected speed for this map,
        # see validate_generic_compressor_map.py). Log-space extrapolation stays
        # positive by construction and decays smoothly toward zero instead.
        log_percent_pressure_ratio = interp_with_linear_extrapolation(
            percent_corrected_speed, self.percent_corrected_speed, np.log(self.percent_design_pressure_ratio))
        percent_pressure_ratio = np.exp(log_percent_pressure_ratio)

        percent_mass_flow = interp_with_linear_extrapolation(percent_corrected_speed, self.percent_corrected_speed,
                                                               self.percent_corrected_mass_flow)
        percent_mass_flow = np.maximum(percent_mass_flow, 0.0)

        # Efficiency is floored, not just extrapolated: a real (or even a windmilling)
        # compressor stage doesn't have zero or negative efficiency, so a floor here
        # is a physical bound, not an arbitrary clip.
        efficiency = interp_with_linear_extrapolation(percent_corrected_speed, self.percent_corrected_speed,
                                                        self.adiabatic_efficiency)
        efficiency = np.maximum(efficiency, MINIMUM_ADIABATIC_EFFICIENCY)

        design_pressure_ratio = self.get('design_pressure_ratio', None)
        if design_pressure_ratio is not None:
            pressure_ratio = percent_pressure_ratio / 100.0 * design_pressure_ratio
            # A compressor cannot have a pressure ratio below 1 -- that would mean it
            # expands rather than compresses the flow. Log-space extrapolation keeps
            # pressure ratio positive but does not by itself keep it above 1 far below
            # the tabulated range; this is an additional physical bound, not tuning.
            pressure_ratio = np.maximum(pressure_ratio, 1.0)
        else:
            pressure_ratio = percent_pressure_ratio

        return pressure_ratio, percent_mass_flow, efficiency

    def query_by_temperature_ratio(self, temperature_ratio_target, ratio_of_specific_heats,
                                    speed_bracket_percent=(1.0, 150.0)):
        """
        Inverse lookup: given a target adiabatic temperature ratio (i.e. the
        value a compressor power balance says the stage must produce), finds
        the corrected speed, pressure ratio, and efficiency on this map that
        together produce it.

        This is how the off-design matching solver uses the map: the existing
        (map-free) power-balance equations already determine what temperature
        ratio a compressor stage must deliver -- that part of the physics
        doesn't change. What changes is how the corresponding pressure ratio
        is obtained: instead of assuming a single constant efficiency at every
        operating point, find the corrected speed on this map whose OWN
        temperature ratio (computed from ITS pressure ratio and efficiency at
        that speed) matches the target, and read off that speed's pressure
        ratio and efficiency instead.

        Implemented as a bounded root-find (`scipy.optimize.brentq`) directly
        against `query()`, not a separately-built inverse table -- an earlier
        version built its own (temperature_ratio -> speed) table and
        extrapolated that independently, which does not round-trip exactly
        with `query()`'s own (log-space, floored) extrapolation outside the
        tabulated range: the two extrapolations diverge exactly where this
        function matters most (deep off-design). Solving against `query()`
        directly guarantees self-consistency by construction: whatever speed
        is returned, `query()` at that speed reproduces the target temperature
        ratio to the root-finder's tolerance, in the tabulated range and the
        extrapolated range alike.

        Parameters
        ----------
        temperature_ratio_target : float
            The adiabatic temperature ratio (Tt_out/Tt_in) the compressor
            power balance requires.
        ratio_of_specific_heats : float
            Ratio of specific heats (gamma) of the working fluid.
        speed_bracket_percent : tuple of float, optional
            (low, high) bound on corrected speed searched, as a percentage of
            design. Widen this if a legitimately valid target temperature
            ratio falls outside the default bracket.

        Returns
        -------
        percent_corrected_speed : float
            Corrected speed, as a percentage of design, at which this map's
            own temperature ratio matches the target.
        pressure_ratio : float
            This map's pressure ratio at that corrected speed.
        adiabatic_efficiency : float
            This map's adiabatic efficiency at that corrected speed.
        """
        from scipy.optimize import brentq

        gamma = ratio_of_specific_heats

        def temperature_ratio_residual(percent_speed):
            pressure_ratio, mass_flow, efficiency = self.query(percent_speed)
            temperature_ratio = 1 + (pressure_ratio[0] ** ((gamma - 1) / gamma) - 1) / efficiency[0]
            return temperature_ratio - temperature_ratio_target

        speed_low, speed_high = speed_bracket_percent
        residual_low  = temperature_ratio_residual(speed_low)
        residual_high = temperature_ratio_residual(speed_high)
        if residual_low > 0 or residual_high < 0:
            raise ValueError(
                f"Generic_Compressor_Map: target temperature ratio {temperature_ratio_target:.4f} is "
                f"outside what {speed_low:.0f}-{speed_high:.0f}% corrected speed can produce on this map "
                f"(range {residual_low + temperature_ratio_target:.4f} to {residual_high + temperature_ratio_target:.4f})"
            )

        percent_corrected_speed = brentq(temperature_ratio_residual, speed_low, speed_high, xtol=1e-6)
        pressure_ratio, mass_flow, efficiency = self.query(percent_corrected_speed)

        return percent_corrected_speed, pressure_ratio[0], efficiency[0]
