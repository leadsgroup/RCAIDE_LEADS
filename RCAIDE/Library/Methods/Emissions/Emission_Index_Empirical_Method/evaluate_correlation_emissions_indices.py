# RCAIDE/Library/Methods/Emissions/evaluate_correlation_emissions_indices.py
#  
# Created:  Jul 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from    RCAIDE.Framework.Core import Data 
 
# package imports
import  numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  evaluate_correlation_emissions_indices
# ---------------------------------------------------------------------------------------------------------------------- 
def evaluate_correlation_emissions_indices(segment,settings,vehicle):
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

    **Theory**
    Total emissions are computed by:
    
    .. math::
        E_{i,total} = \int \dot{m}_{fuel}(t) \cdot EI_i \cdot GWP_i \, dt

    Where:

    * :math:`E_{i,total}` = Total emissions for species i
    * :math:`\dot{m}_{fuel}` = Fuel mass flow rate
    * :math:`EI_i` = Emission index for species i
    * :math:`GWP_i` = Global warming potential for species i
    
    Contrail effects are estimated by:
    
    .. math::
        E_{contrails} = \Delta R \cdot GWP_{contrails}

    Where:

    * :math:`\Delta R` = Flight range [km]
    * :math:`GWP_{contrails}` = Contrail global warming potential

    **Extra modules required**

    * numpy

    See Also
    --------
    RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method

    References
    ----------
    [1] Lee, D. S., et al. (2021). The contribution of global aviation to anthropogenic climate forcing for 2000 to 2018. Atmospheric Environment, 244, 117834.
    """  
    # unpack
    state      = segment.state
    I          = state.numerics.time.integrate
    NOx_total  = 0 * state.ones_row(1)  
    CO2_total  = 0 * state.ones_row(1) 
    SO2_total  = 0 * state.ones_row(1) 
    H2O_total  = 0 * state.ones_row(1) 
    Soot_total = 0 * state.ones_row(1) 

    for network in vehicle.networks:  
        for fuel_line in network.fuel_lines:
            if fuel_line.active: 
                for fuel_tank in fuel_line.fuel_tanks:
                    mdot = 0. * state.ones_row(1)    
                    fuel =  fuel_tank.fuel 
             
                    EI_NOx  = fuel.emission_indices.NOx
                    EI_CO2  = fuel.emission_indices.CO2 
                    EI_H2O  = fuel.emission_indices.H2O
                    EI_SO2  = fuel.emission_indices.SO2
                    EI_Soot = fuel.emission_indices.Soot  
                    
                    mdot = segment.conditions.energy[fuel_line.tag][fuel_tank.tag].mass_flow_rate
                     
                    # Integrate them over the entire segment
                    NOx_total  += np.dot(I,mdot*EI_NOx)
                    CO2_total  += np.dot(I,mdot*EI_CO2)
                    SO2_total  += np.dot(I,mdot*EI_SO2)
                    H2O_total  += np.dot(I,mdot*EI_H2O) 
                    Soot_total += np.dot(I,mdot*EI_Soot)
                                     
         
    flight_range    =  state.conditions.frames.inertial.aircraft_range 
    Contrails_total =  (flight_range -   flight_range[0]) /1000 * fuel.global_warming_potential_100.Contrails

    emissions                 = Data()
    emissions.total           = Data()
    emissions.index           = Data() 
    emissions.total.NOx       = NOx_total   * fuel.global_warming_potential_100.NOx 
    emissions.total.CO2       = CO2_total   * fuel.global_warming_potential_100.CO2
    emissions.total.H2O       = H2O_total   * fuel.global_warming_potential_100.H2O  
    emissions.total.SO2       = SO2_total   * fuel.global_warming_potential_100.SO2  
    emissions.total.Soot      = Soot_total  * fuel.global_warming_potential_100.Soot 
    emissions.total.Contrails = Contrails_total   
    emissions.index.NOx       = EI_NOx   * state.ones_row(1)
    emissions.index.CO2       = EI_CO2   * state.ones_row(1)
    emissions.index.H2O       = EI_H2O   * state.ones_row(1)
    emissions.index.SO2       = EI_SO2   * state.ones_row(1)
    emissions.index.Soot      = EI_Soot  * state.ones_row(1)
    
    state.conditions.emissions = emissions
    return   

