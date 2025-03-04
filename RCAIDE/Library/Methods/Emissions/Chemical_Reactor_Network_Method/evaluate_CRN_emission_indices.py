# RCAIDE/Library/Methods/Emissions/Chemical_Reactor_Network_Method/evaluate_CRN_emission_indices.py
#  
# Created: Jul 2024, M. Clarke, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import  RCAIDE
from    RCAIDE.Framework.Core import Data
from    RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method.evaluate_cantera import evaluate_cantera 
 
# package imports
import  numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  evaluate_correlation_emissions_indices
# ---------------------------------------------------------------------------------------------------------------------- 
def evaluate_CRN_emission_indices_no_surrogate(segment,settings,vehicle):

    """
    Computes emission indices directly using Chemical Reactor Network without surrogate models.

    Parameters
    ----------
    segment : Data
        Mission segment data container
        
        - state : Data
            Current state 

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
            - propulsors : list
                Propulsion units 
    Returns
    -------
        Updates segment.state.conditions.emissions with:
        
        total : Data
            Total emissions over segment

            - CO2 : float
                Total CO2 emissions [kg]
            - H2O : float
                Total H2O emissions [kg]
            - NOx : float
                Total NOx emissions [kg]
        index : Data
            Emission indices

            - CO2 : ndarray
                CO2 emission index [kg_CO2/kg_fuel]
            - CO : ndarray
                CO emission index [kg_CO/kg_fuel]
            - H2O : ndarray
                H2O emission index [kg_H2O/kg_fuel]
            - NO : ndarray
                NO emission index [kg_NO/kg_fuel]
            - NO2 : ndarray
                NO2 emission index [kg_NO2/kg_fuel]

    Notes
    -----
    Computes emissions by directly evaluating the chemical kinetics at each time step
    using Cantera. 

    **Extra modules required**

    * numpy
    * Cantera (through evaluate_cantera function)

    See Also
    --------
    RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method.evaluate_cantera 

    References
    ----------
    [1] Goodwin, D. G., Speth, R. L., Moffat, H. K., & Weber, B. W. (2023). Cantera: An object-oriented software toolkit for chemical kinetics, thermodynamics, and transport processes (Version 3.0.0) [Computer software]. https://www.cantera.org
    """
  
    # unpack
    state     = segment.state
    I         = state.numerics.time.integrate
    
    CO2_total = 0 * state.ones_row(1)  
    CO_total  = 0 * state.ones_row(1) 
    H2O_total = 0 * state.ones_row(1) 
    NO_total  = 0 * state.ones_row(1) 
    NO2_total = 0 * state.ones_row(1) 

    for network in vehicle.networks:  
        for fuel_line in network.fuel_lines:
            if fuel_line.active:  
                for p_i ,  propulsor in enumerate(network.propulsors):
                    if propulsor.active == True: 
                        if (type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) or \
                            type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turboshaft or \
                            type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop or \
                            type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet:    
                        
                            combustor = propulsor.combustor
                        
                            # unpack component conditions
                            n_cp                 = state.numerics.number_of_control_points 
                            propulsor_conditions = state.conditions.energy[propulsor.tag] 
                            combustor_conditions = propulsor_conditions[combustor.tag]  

                            
                            T    = combustor_conditions.inputs.stagnation_temperature
                            P    = combustor_conditions.inputs.stagnation_pressure 
                            mdot = propulsor_conditions.core_mass_flow_rate 
                            FAR  = combustor_conditions.outputs.fuel_to_air_ratio 

                            EI_CO2_comb   = 0 * state.ones_row(1)   
                            EI_CO_comb    = 0 * state.ones_row(1)  
                            EI_H2O_comb   = 0 * state.ones_row(1)  
                            EI_NO_comb    = 0 * state.ones_row(1)  
                            EI_NO2_comb   = 0 * state.ones_row(1)                              
                            if network.identical_propulsors == True and p_i != 0:
                                EI_CO2_comb = EI_CO2_prev
                                EI_CO_comb  = EI_CO_prev
                                EI_H2O_comb = EI_H2O_prev
                                EI_NO_comb  = EI_NO_prev
                                EI_NO2_comb = EI_NO2_prev 
                                
                            else:     
                                for t_idx in range(n_cp):
                                    # Call cantera 
                                    results = evaluate_cantera(combustor,T[t_idx,0],P[t_idx,0],mdot[t_idx,0],FAR[t_idx,0])
                                    
                                    EI_CO2_comb[t_idx,0] = results.EI_CO2
                                    EI_CO_comb[t_idx,0]  = results.EI_CO 
                                    EI_H2O_comb[t_idx,0] = results.EI_H2O
                                    EI_NO_comb[t_idx,0]  = results.EI_NO 
                                    EI_NO2_comb[t_idx,0] = results.EI_NO2
                                    
                                    EI_CO2_prev = EI_CO2_comb 
                                    EI_CO_prev  =  EI_CO_comb  
                                    EI_H2O_prev = EI_H2O_comb 
                                    EI_NO_prev  =  EI_NO_comb  
                                    EI_NO2_prev = EI_NO2_comb 
                                
                            CO2_total  += np.dot(I,mdot*EI_CO2_comb)
                            CO_total   += np.dot(I,mdot *EI_CO_comb )
                            H2O_total  += np.dot(I,mdot*EI_H2O_comb)
                            NO_total   += np.dot(I,mdot *EI_NO_comb ) 
                            NO2_total  += np.dot(I,mdot *EI_NO2_comb)

    emissions                 = Data()
    emissions.total           = Data()
    emissions.index           = Data() 
    emissions.total.CO2       = CO2_total  * combustor.fuel_data.global_warming_potential_100.CO2 
    emissions.total.H2O       = H2O_total  * combustor.fuel_data.global_warming_potential_100.H2O  
    emissions.total.NOx       = (NO_total + NO2_total) * combustor.fuel_data.global_warming_potential_100.NOx 
    emissions.index.CO2       = EI_CO2_comb
    emissions.index.CO        = EI_CO_comb 
    emissions.index.H2O       = EI_H2O_comb
    emissions.index.NO        = EI_NO_comb 
    emissions.index.NO2       = EI_NO2_comb 
    
    state.conditions.emissions =  emissions
    return    
    

def evaluate_CRN_emission_indices_surrogate(segment,settings,vehicle): 

    """
    Computes emission indices using pre-trained Chemical Reactor Network surrogate models.

    Parameters
    ----------
    segment : Data
        Mission segment data container
        
        - state : Data
            Current state of the system

            - numerics : Data
                Numerical integration parameters
            - conditions : Data
                Flight conditions and component states
        - analyses : Data
            Analysis settings and models

            - emissions : Data
                Emissions analysis settings

                - surrogates : Data
                    Trained surrogate models for each species
                
    settings : Data
        Configuration settings for the simulation
        
    vehicle : Data
        Vehicle configuration data

        - networks : list
            List of propulsion system networks

            - propulsors : list
                Propulsion units 

    Returns
    -------
    Updates segment.state.conditions.emissions with:
        
    total : Data
        Total emissions over segment

        - CO2 : float
            Total CO2 emissions [kg]
        - H2O : float
            Total H2O emissions [kg]
        - NOx : float
            Total NOx emissions [kg]
    index : Data
        Emission indices

        - CO2 : ndarray
            CO2 emission index [kg_CO2/kg_fuel]
        - CO : ndarray
            CO emission index [kg_CO/kg_fuel]
        - H2O : ndarray
            H2O emission index [kg_H2O/kg_fuel]
        - NO : ndarray
            NO emission index [kg_NO/kg_fuel]
        - NO2 : ndarray
            NO2 emission index [kg_NO2/kg_fuel]

    Notes
    -----
    Uses pre-trained surrogate models to estimate emission indices.

    **Extra modules required**

    * numpy

    **Major Assumptions**

    * Operating conditions fall within the training data range
    * Linear interpolation can be employed between training points

    See Also
    --------
    RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method.train_CRN_EI_surrogates
    RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method.build_CRN_EI_surrogates
    RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method.evaluate_cantera 

    References
    ----------
    [1] Goodwin, D. G., Speth, R. L., Moffat, H. K., & Weber, B. W. (2023). Cantera: An object-oriented software toolkit for chemical kinetics, thermodynamics, and transport processes (Version 3.0.0) [Computer software]. https://www.cantera.org
    """
  
    I          = segment.state.numerics.time.integrate
    surrogates = segment.analyses.emissions.surrogates
    
    CO2_total = 0 * segment.state.ones_row(1)  
    CO_total  = 0 * segment.state.ones_row(1) 
    H2O_total = 0 * segment.state.ones_row(1) 
    NO_total  = 0 * segment.state.ones_row(1) 
    NO2_total = 0 * segment.state.ones_row(1) 

    for network in vehicle.networks:    
        for propulsor in network.propulsors:
            if propulsor.active == True:
                if (type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) or \
                    type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop or \
                    type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turboshaft or \
                    type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet:    
                
                    combustor = propulsor.combustor
                
                    # unpack component conditions
                    propulsor_conditions = segment.state.conditions.energy[propulsor.tag] 
                    combustor_conditions = propulsor_conditions[combustor.tag]  

                    T    = combustor_conditions.inputs.stagnation_temperature
                    P    = combustor_conditions.inputs.stagnation_pressure 
                    mdot = propulsor_conditions.core_mass_flow_rate 
                    FAR  = combustor_conditions.outputs.fuel_to_air_ratio 
                    
                    pts = np.hstack((T,P,mdot,FAR )) 

                    EI_CO2_comb  = np.atleast_2d(surrogates.EI_CO2(pts)).T
                    EI_CO_comb   = np.atleast_2d(surrogates.EI_CO(pts)).T 
                    EI_H2O_comb  = np.atleast_2d(surrogates.EI_H2O(pts)).T 
                    EI_NO_comb   = np.atleast_2d(surrogates.EI_NO(pts)).T 
                    EI_NO2_comb  = np.atleast_2d(surrogates.EI_NO2(pts)).T        
                          
                    CO2_total += np.dot(I,mdot*EI_CO2_comb)
                    CO_total  += np.dot(I,mdot *EI_CO_comb )
                    H2O_total += np.dot(I,mdot*EI_H2O_comb)
                    NO_total  += np.dot(I,mdot *EI_NO_comb ) 
                    NO2_total += np.dot(I,mdot *EI_NO2_comb)

    emissions                 = Data()
    emissions.total           = Data()
    emissions.index           = Data() 
    emissions.total.CO2       = CO2_total * combustor.fuel_data.global_warming_potential_100.CO2 
    emissions.total.H2O       = H2O_total * combustor.fuel_data.global_warming_potential_100.H2O  
    emissions.total.NOx       = (NO_total + NO2_total) * combustor.fuel_data.global_warming_potential_100.NOx 
    emissions.index.CO2       = EI_CO2_comb
    emissions.index.CO        = EI_CO_comb 
    emissions.index.H2O       = EI_H2O_comb
    emissions.index.NO        = EI_NO_comb 
    emissions.index.NO2       = EI_NO2_comb 
 
    segment.state.conditions.emissions =  emissions
    return   

     