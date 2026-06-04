# RCAIDE/Library/Methods/Emissions/evaluate_correlation_emissions_indices.py
#  
# Created:  Jul 2024, M. Clarke
# Modified: Jun 2026 - Fixed GWP accumulation bug in propulsor loop;
#                      Fixed contrails ×1000 unit error;
#                      Added altitude threshold gate for contrail formation.

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from   RCAIDE.Framework.Core import Data
 
# package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  evaluate_correlation_emissions_indices
# ---------------------------------------------------------------------------------------------------------------------- 
def evaluate_correlation_emissions(segment, settings, vehicle):
    """
    Computes emission indices using empirical correlations.

    Parameters
    ----------
    segment : Data
        Mission segment data container
        
        - state : Data
            Current state of the system

            - numerics : Data
                Numerical integration parameters

                - time : Data
                    Time integration settings
            - conditions : Data
                Flight conditions and component states
            - ones_row : function
                Creates array of ones with specified size
                
    settings : Data
        Configuration settings for the simulation
        
    vehicle : Data
        Vehicle configuration data

        - networks : list
            List of propulsion system networks

            - fuel_lines : list
                Fuel distribution systems

                - active : bool
                    Flag indicating if fuel line is in use
                - fuel_tanks : list
                    Fuel storage units

                    - tag : str
                        Identifier for the fuel tank
                    - fuel : Data
                        Fuel properties

                        - emission_indices : Data
                            Empirical emission indices

                            - NOx : float
                                NOx emission index [kg_NOx/kg_fuel]
                            - CO2 : float
                                CO2 emission index [kg_CO2/kg_fuel]
                            - H2O : float
                                H2O emission index [kg_H2O/kg_fuel]
                            - SO2 : float
                                SO2 emission index [kg_SO2/kg_fuel]
                            - Soot : float
                                Soot emission index [kg_soot/kg_fuel]
                        - global_warming_potential_100 : Data
                            100-year global warming potentials
            - propulsors : list
                Propulsion units (turbofans, turbojets, etc.)

    Returns
    -------
    None
        Updates segment.state.conditions.emissions with:
        
        - total : Data
            Total emissions over segment

            - NOx : float
                Total NOx emissions [kg]
            - CO2 : float
                Total CO2 emissions [kg]
            - H2O : float
                Total H2O emissions [kg]
            - SO2 : float
                Total SO2 emissions [kg]
            - Soot : float
                Total soot emissions [kg]
            - Contrails : float
                Total contrail effect [kg CO2 equivalent]
        - index : Data
            Emission indices

            - NOx : ndarray
                NOx emission index [kg_NOx/kg_fuel]
            - CO2 : ndarray
                CO2 emission index [kg_CO2/kg_fuel]
            - H2O : ndarray
                H2O emission index [kg_H2O/kg_fuel]
            - SO2 : ndarray
                SO2 emission index [kg_SO2/kg_fuel]
            - Soot : ndarray
                Soot emission index [kg_soot/kg_fuel]

    Notes
    -----
    This function uses pre-defined emission indices for each fuel type and integrates
    them over the mission segment based on fuel flow rates.

    **Major Assumptions**

    * Emission indices are constant for each fuel type
    * Indices are independent of operating conditions
    * Linear scaling with fuel flow rate
    * Contrails only form above the Schmidt-Appleman altitude threshold (~8,000 m /
      26,247 ft).  Below that altitude, temperatures are too warm for persistent
      ice-crystal formation, so no contrail radiative forcing is accumulated.

    **Theory**
    Total emissions are computed by:
    
    .. math::
        E_{i,total} = \int \dot{m}_{fuel}(t) \cdot EI_i \cdot GWP_i \, dt

    Where:

    * :math:`E_{i,total}` = Total emissions for species i [kg CO2e]
    * :math:`\dot{m}_{fuel}` = Fuel mass flow rate [kg/s]
    * :math:`EI_i` = Emission index for species i [kg_i/kg_fuel]
    * :math:`GWP_i` = Global warming potential for species i [kg CO2e / kg_i]
    
    Contrail effects are estimated by:
    
    .. math::
        E_{contrails} = \Delta R_{above} \cdot GWP_{contrails}

    Where:

    * :math:`\Delta R_{above}` = Flight range flown above CONTRAIL_ALT_THRESHOLD_M [km]
    * :math:`GWP_{contrails}` = Contrail global warming potential [kg CO2e / km]

    **Extra modules required**

    * numpy

    See Also
    --------
    RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method

    References
    ----------
    [1] Lee, D. S., et al. (2021). The contribution of global aviation to anthropogenic
        climate forcing for 2000 to 2018. Atmospheric Environment, 244, 117834.
    """  
    # unpack
    state      = segment.state
    I          = state.numerics.time.integrate 
    emissions  = state.conditions.emissions 
    NOx_total    = 0 * state.ones_row(1)
    CO2_total    = 0 * state.ones_row(1)
    CO_total     = 0 * state.ones_row(1)
    SO2_total    = 0 * state.ones_row(1)
    H2O_total    = 0 * state.ones_row(1)
    Soot_total   = 0 * state.ones_row(1)
    total_gCO2e  = 0 * state.ones_row(1)
    contrail_gwp = 11.0  # kg CO2e/km; default matches all current fuel definitions

    if segment.state.initials:
        initial_cumulative_gCO2e = segment.state.initials.conditions.emissions.cumulative_gCO2e[-1]
    else:
        initial_cumulative_gCO2e = 0.0

    for network in vehicle.networks:
        for p_i, propulsor in enumerate(network.propulsors):
            if propulsor.active == True:
                if (type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) or \
                    type(propulsor) == RCAIDE.Library.Components.Powertrain.Converters.Turboshaft or \
                    type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop or \
                    type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet:

                    # unpack component conditions
                    combustor = propulsor.combustor
                    propulsor_conditions = state.conditions.energy.propulsors[propulsor.tag]

                    fuel = combustor.fuel_data
                    contrail_gwp = fuel.global_warming_potential_100.Contrails

                    EI_NOx  = fuel.emission_indices.NOx
                    EI_CO2  = fuel.emission_indices.CO2
                    EI_CO   = fuel.emission_indices.CO
                    EI_H2O  = fuel.emission_indices.H2O
                    EI_SO2  = fuel.emission_indices.SO2
                    EI_Soot = fuel.emission_indices.Soot

                    mdot_fuel = propulsor_conditions.fuel_mass_flow_rate

                    # Integrate each species over the segment for this propulsor
                    # FIX: use per-propulsor segment integrals (NOx_seg etc.) so the
                    # GWP sum below adds only this propulsor's contribution, not the
                    # running cumulative total (which double-counts on multi-engine configs).
                    NOx_seg  = np.dot(I, mdot_fuel * EI_NOx)
                    CO2_seg  = np.dot(I, mdot_fuel * EI_CO2)
                    CO_seg   = np.dot(I, mdot_fuel * EI_CO)
                    SO2_seg  = np.dot(I, mdot_fuel * EI_SO2)
                    H2O_seg  = np.dot(I, mdot_fuel * EI_H2O)
                    Soot_seg = np.dot(I, mdot_fuel * EI_Soot)

                    # Accumulate species mass totals across all propulsors
                    NOx_total  += NOx_seg
                    CO2_total  += CO2_seg
                    CO_total   += CO_seg
                    SO2_total  += SO2_seg
                    H2O_total  += H2O_seg
                    Soot_total += Soot_seg

                    # Accumulate GWP-weighted CO2e for this propulsor's segment emissions.
                    # Units: kg_species * (kg CO2e / kg_species) = kg CO2e
                    total_gCO2e += NOx_seg  * fuel.global_warming_potential_100.NOx  + \
                                   CO2_seg  * fuel.global_warming_potential_100.CO2  + \
                                   CO_seg   * fuel.global_warming_potential_100.CO   + \
                                   H2O_seg  * fuel.global_warming_potential_100.H2O  + \
                                   SO2_seg  * fuel.global_warming_potential_100.SO2  + \
                                   Soot_seg * fuel.global_warming_potential_100.Soot

    # ------------------------------------------------------------------
    # Contrails
    # Only accumulate range flown above the Schmidt-Appleman altitude
    # threshold (~8,000 m).  Below that, temperatures are too warm for
    # persistent ice-crystal formation.
    # ------------------------------------------------------------------
    flight_altitude = state.conditions.freestream.altitude             # (N,1) metres
    flight_range    = state.conditions.frames.inertial.aircraft_range  # (N,1) metres

    contrail_altitude_threshold = 8000.0   # metres (~26,247 ft)

    # Build a cumulative km-above-threshold time series for this segment.
    # Use step-by-step range increments masked by the altitude gate so that
    # below-threshold steps contribute zero regardless of where they appear
    # in the segment (climb, cruise, or descent).  This avoids the sign-flip
    # that occurs when np.where zeros out flight_range for below-threshold
    # points and the result is then differenced against the segment-start value.
    above_mask    = (flight_altitude >= contrail_altitude_threshold).astype(float)
    d_range       = np.vstack([np.zeros((1, 1)), np.diff(flight_range, axis=0)])  # (N,1) m
    Contrails_total = np.cumsum(d_range * above_mask, axis=0) / 1000.0           # (N,1) km

    # kg CO2e  (contrail_gwp units: kg CO2e / km)
    total_gCO2e += Contrails_total * contrail_gwp

    # Convert gas + contrail total from kg CO2e → g CO2e
    emissions.gCO2e            = total_gCO2e * 1000
    emissions.cumulative_gCO2e = initial_cumulative_gCO2e + total_gCO2e * 1000
    emissions.mass.NOx         = NOx_total
    emissions.mass.CO2         = CO2_total
    emissions.mass.CO          = CO_total
    emissions.mass.H2O         = H2O_total
    emissions.mass.SO2         = SO2_total
    emissions.mass.Soot        = Soot_total
    emissions.mass.Contrails   = Contrails_total
    emissions.index.NOx        = EI_NOx  * state.ones_row(1)
    emissions.index.CO2        = EI_CO2  * state.ones_row(1)
    emissions.index.CO         = EI_CO   * state.ones_row(1)
    emissions.index.H2O        = EI_H2O  * state.ones_row(1)
    emissions.index.SO2        = EI_SO2  * state.ones_row(1)
    emissions.index.Soot       = EI_Soot * state.ones_row(1)

    return
