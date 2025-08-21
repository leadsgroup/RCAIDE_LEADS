# RCAIDE/Methods/Aerodynamics/Common/cooling_drag.py
# 
# 
# Created:  May 2024, S S. Shekar 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
from RCAIDE.Framework.Core import Data

# python
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  cooling_drag
# ----------------------------------------------------------------------------------------------------------------------   
def cooling_drag(state,settings,geometry):
    """
    Computes cooling drag coefficient based on heat exchanger operation and air flow through cooling ducts.

    Parameters
    ----------
    state : Data
        Flight conditions and energy state containing:
            - conditions.freestream.density : float
                Freestream air density [kg/m³]
            - conditions.freestream.velocity : float
                Freestream velocity [m/s]
            - conditions.freestream.pressure : float
                Freestream static pressure [Pa]
            - conditions.energy.coolant_lines : dict
                Dictionary of coolant line results indexed by coolant line tag
                    - coolant_line_tag : dict
                        Dictionary of heat exchanger results indexed by heat exchanger tag
                            - heat_exchanger_tag : Data
                                Heat exchanger operation results containing:
                                    - air_mass_flow_rate : float
                                        Mass flow rate of air through heat exchanger [kg/s]
                                    - pressure_diff_air : float
                                        Pressure differential across heat exchanger [Pa]
    settings : dict
        Analysis settings and parameters
    geometry : Data
        Vehicle geometry containing:
            - reference_area : float
                Reference area for drag coefficient calculation [m²]
            - networks : list
                List of propulsion networks containing coolant lines
                    - coolant_lines : list
                        List of coolant line objects with heat exchangers
                            - tag : str
                                Unique identifier for the coolant line
                            - heat_exchangers : list
                                List of heat exchanger objects containing:
                                    - tag : str
                                        Unique identifier for the heat exchanger
                                    - atmospheric_air_inlet_to_outlet_area_ratio : float
                                        Ratio of inlet to outlet area for atmospheric air [unitless]
                                    - duct_losses : float
                                        Duct efficiency factor accounting for losses [unitless]
                                    - minimum_air_speed : float
                                        Minimum air speed required for heat exchanger operation [m/s]

    Returns
    -------
    None
        Results are stored in state.conditions.aerodynamics.coefficients.drag.cooling.total

    Notes
    -----
    This function calculates the drag penalty associated with cooling system operation, including
    the momentum deficit caused by air flow through heat exchangers and the pressure forces
    on duct surfaces. The calculation accounts for variable inlet areas based on required
    cooling flow rates.
    
    **Major Assumptions**
        * Density across the duct is equal to freestream density
        * Inlet area varies based on required cooling flow rate
        * Duct losses are characterized by a single efficiency factor
        * Heat exchanger operation ceases below minimum air speed
    
    **Theory**

    The cooling drag is calculated from momentum and pressure forces:

    :math:`F_{cooling} = \\dot{m}_{air}(V_{exit} \\eta_{duct} - V_{\\infty}) + A_{outlet} \\eta_{duct}(P_{exit} - P_{\\infty})`

    where:
        - :math:`\\dot{m}_{air}` is the air mass flow rate through the heat exchanger [kg/s]
        - :math:`V_{exit}` is the exit velocity [m/s]
        - :math:`V_{\\infty}` is the freestream velocity [m/s]
        - :math:`\\eta_{duct}` is the duct efficiency factor
        - :math:`A_{outlet}` is the outlet area [m²]
        - :math:`P_{exit}` is the exit pressure [Pa]
        - :math:`P_{\\infty}` is the freestream pressure [Pa]

    The inlet area is determined from mass flow requirements:

    :math:`A_{inlet} = \\frac{\\dot{m}_{air}}{\\rho_{\\infty} V_{\\infty}}`

    The exit velocity and pressure are calculated as:

    :math:`V_{exit} = \\frac{\\dot{m}_{air}}{\\rho_{\\infty} A_{outlet}}`

    :math:`P_{exit} = \\frac{1}{2}\\rho_{\\infty}[(1+\\eta_e)V_{exit}^2 - V_{\\infty}^2] + P_{\\infty} - \\Delta P_{HEX}`

    where :math:`\\eta_e = 0.5(1 - A_{outlet}/A_{inlet})` is the expansion efficiency.

    The cooling drag coefficient is:

    :math:`C_{D,cooling} = \\frac{F_{cooling}}{\\frac{1}{2}\\rho_{\\infty} V_{\\infty}^2 S_{ref}}`
    
    **Definitions**

    'Cooling Drag'
        Additional drag caused by the momentum deficit and pressure forces associated with air flow through cooling systems.
    
    'Duct Losses'
        Efficiency factor accounting for frictional and form losses in the cooling air ductwork.

    References
    ----------
    [1] Brelje, B., Jasa, J., Martins, J., & Gray, J. (2019). "Development of a conceptual-level thermal management system design capability in OpenConcept."
    """
    
    # Unpack Inputs 
    conditions                 = state.conditions
    density                    = conditions.freestream.density
    velocity                   = conditions.freestream.velocity
    pressure                   = conditions.freestream.pressure
    reference_area             = geometry.reference_area
    
    # Create an empty array for cooling drag coefficient
    cd_cooling  = np.zeros_like(density)
    
    for network in geometry.networks:
        for coolant_line in  network.coolant_lines:
            for tag, item in  coolant_line.items():
                if tag == 'heat_exchangers':
                    for heat_exchanger in  item:
                        # unpack
                        HEX_results                = conditions.energy.coolant_lines[coolant_line.tag][heat_exchanger.tag]
                        mass_flow_hex              = HEX_results.air_mass_flow_rate 
                        hex_pressure_diff          = HEX_results.pressure_diff_air
                        
                        # Compute the Inlet area of the Ram
                        inlet_area          = mass_flow_hex/(density*velocity)
                        outlet_area         = inlet_area*heat_exchanger.atmospheric_air_inlet_to_outlet_area_ratio
                        eta_e               = 0.5*(1-outlet_area/inlet_area)
    
                        # Compute Exit Parameters
                        exit_velocity       = mass_flow_hex/(density*outlet_area)
                        exit_pressure       = 0.5*density*((1+eta_e)*exit_velocity**2-velocity)+pressure-hex_pressure_diff
    
                        # Compute Drag due to cooling. 
                        F_cooling_drag     = mass_flow_hex*(exit_velocity*heat_exchanger.duct_losses -velocity) + outlet_area*heat_exchanger.duct_losses *(exit_pressure-pressure) 
                        cd_cooling         = F_cooling_drag/(0.5 * density * (velocity**2) * reference_area)
                        
                        # Check if fan operation is active
                        cd_cooling[state.conditions.freestream.velocity<heat_exchanger.minimum_air_speed] =  0 
    
    # dump to results
    conditions.aerodynamics.coefficients.drag.cooling.total=  cd_cooling  

    return 