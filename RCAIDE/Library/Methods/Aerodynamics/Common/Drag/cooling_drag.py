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
        Computes the cooling drag based on the results of the Heat Exchanger operation

            Inputs:
                state.
                    conditions.freestream.
                                        density                            [kg/m^3]
                                        velocity                           [Knots]        
                                        pressure                           [Pascal]
                    energy.coolant_line.heat_exchanger.
                                                    pressure_differential  [Pascal] 
                                                    mass_flow_hex          [kg/s]
                    analyses.aerodynamics.vehicle.reference_area           [m^2]

            

            Outputs:
                cooling_drag                                               [Unitless]
    
            Assumptions:
            The density across the duct is equal to the freestream density of air. 
            Inlet area is varied based of required inflow required

            Source:
            Brelje, Benjamin & Jasa, John & Martins, Joaquim & Gray, Justin. (2019).
            Development of a conceptual-level thermal management system design capability in OpenConcept. 

            Properties Used:
            None

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
                        HEX_results                = conditions.energy[coolant_line.tag][heat_exchanger.tag]
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