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
    NOx_total  = 0 * state.ones_row(1) 
    Soot_total = 0 * state.ones_row(1) 

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

                            EI_CO2_comb    = 0 * state.ones_row(1)   
                            EI_CO_comb     = 0 * state.ones_row(1)  
                            EI_H2O_comb    = 0 * state.ones_row(1)  
                            EI_NOx_comb    = 0 * state.ones_row(1)  
                            EI_soot_comb   = 0 * state.ones_row(1)   

                            combustor_PZ_phi           = np.zeros((n_cp,combustor.N_PZ))  # [-] 
                            combustor_PZ_T             = np.zeros((n_cp,combustor.N_PZ)) # [K] 
                            combustor_PZ_f_psr         = np.zeros((n_cp,combustor.N_PZ)) # [-] 
                            combustor_PZ_EI_CO2        = np.zeros((n_cp,combustor.N_PZ)) # [kg/kg_fuel]
                            combustor_PZ_EI_CO         = np.zeros((n_cp,combustor.N_PZ)) # [kg/kg_fuel]
                            combustor_PZ_EI_H2O        = np.zeros((n_cp,combustor.N_PZ)) # [kg/kg_fuel]
                            combustor_PZ_EI_NOx        = np.zeros((n_cp,combustor.N_PZ)) # [kg/kg_fuel]
                            combustor_PZ_EI_soot       = np.zeros((n_cp,combustor.N_PZ)) # [kg/kg_fuel]
                            combustor_SZ_sm_z          = np.zeros((n_cp,combustor.N_SZ)) # [-] 
                            combustor_SZ_sm_phi        = np.zeros((n_cp,combustor.N_SZ)) # [-] 
                            combustor_SZ_sm_T          = np.zeros((n_cp,combustor.N_SZ)) # [K] 
                            combustor_SZ_sm_EI_CO2     = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]
                            combustor_SZ_sm_EI_CO      = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]
                            combustor_SZ_sm_EI_H2O     = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]
                            combustor_SZ_sm_EI_NOx     = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]
                            combustor_SZ_sm_EI_soot    = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]
                            combustor_SZ_fm_z          = np.zeros((n_cp,combustor.N_SZ)) # [-] 
                            combustor_SZ_fm_phi        = np.zeros((n_cp,combustor.N_SZ)) # [-] 
                            combustor_SZ_fm_T          = np.zeros((n_cp,combustor.N_SZ)) # [K] 
                            combustor_SZ_fm_EI_CO2     = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]
                            combustor_SZ_fm_EI_CO      = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]
                            combustor_SZ_fm_EI_H2O     = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]
                            combustor_SZ_fm_EI_NOx     = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]
                            combustor_SZ_fm_EI_soot    = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]
                            combustor_SZ_joint_z       = np.zeros((n_cp,combustor.N_SZ)) # [-] 
                            combustor_SZ_joint_phi     = np.zeros((n_cp,combustor.N_SZ)) # [-] 
                            combustor_SZ_joint_T       = np.zeros((n_cp,combustor.N_SZ)) # [K] 
                            combustor_SZ_joint_EI_CO2  = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]
                            combustor_SZ_joint_EI_CO   = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]
                            combustor_SZ_joint_EI_H2O  = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]
                            combustor_SZ_joint_EI_NOx  = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]
                            combustor_SZ_joint_EI_soot = np.zeros((n_cp,combustor.N_SZ)) # [kg/kg_fuel]         

                            if network.identical_propulsors == True and p_i != 0:
                                EI_CO2_comb = EI_CO2_prev
                                EI_CO_comb  = EI_CO_prev
                                EI_H2O_comb = EI_H2O_prev
                                EI_NOx_comb  = EI_NOx_prev
                                EI_soot_comb = EI_soot_prev 

                                combustor_PZ_phi           = combustor_PZ_phi_prev          
                                combustor_PZ_T             = combustor_PZ_T_prev            
                                combustor_PZ_f_psr         = combustor_PZ_f_psr_prev        
                                combustor_PZ_EI_CO2        = combustor_PZ_EI_CO2_prev       
                                combustor_PZ_EI_CO         = combustor_PZ_EI_CO_prev        
                                combustor_PZ_EI_H2O        = combustor_PZ_EI_H2O_prev       
                                combustor_PZ_EI_NOx        = combustor_PZ_EI_NOx_prev       
                                combustor_PZ_EI_soot       = combustor_PZ_EI_soot_prev      
                                combustor_SZ_sm_z          = combustor_SZ_sm_z_prev         
                                combustor_SZ_sm_phi        = combustor_SZ_sm_phi_prev       
                                combustor_SZ_sm_T          = combustor_SZ_sm_T_prev         
                                combustor_SZ_sm_EI_CO2     = combustor_SZ_sm_EI_CO2_prev    
                                combustor_SZ_sm_EI_CO      = combustor_SZ_sm_EI_CO_prev     
                                combustor_SZ_sm_EI_H2O     = combustor_SZ_sm_EI_H2O_prev    
                                combustor_SZ_sm_EI_NOx     = combustor_SZ_sm_EI_NOx_prev    
                                combustor_SZ_sm_EI_soot    = combustor_SZ_sm_EI_soot_prev   
                                combustor_SZ_fm_z          = combustor_SZ_fm_z_prev         
                                combustor_SZ_fm_phi        = combustor_SZ_fm_phi_prev       
                                combustor_SZ_fm_T          = combustor_SZ_fm_T_prev         
                                combustor_SZ_fm_EI_CO2     = combustor_SZ_fm_EI_CO2_prev    
                                combustor_SZ_fm_EI_CO      = combustor_SZ_fm_EI_CO_prev     
                                combustor_SZ_fm_EI_H2O     = combustor_SZ_fm_EI_H2O_prev    
                                combustor_SZ_fm_EI_NOx     = combustor_SZ_fm_EI_NOx_prev    
                                combustor_SZ_fm_EI_soot    = combustor_SZ_fm_EI_soot_prev     
                                combustor_SZ_joint_z       = combustor_SZ_joint_z_prev      
                                combustor_SZ_joint_phi     = combustor_SZ_joint_phi_prev    
                                combustor_SZ_joint_T       = combustor_SZ_joint_T_prev      
                                combustor_SZ_joint_EI_CO2  = combustor_SZ_joint_EI_CO2_prev 
                                combustor_SZ_joint_EI_CO   = combustor_SZ_joint_EI_CO_prev  
                                combustor_SZ_joint_EI_H2O  = combustor_SZ_joint_EI_H2O_prev 
                                combustor_SZ_joint_EI_NOx  = combustor_SZ_joint_EI_NOx_prev 
                                combustor_SZ_joint_EI_soot = combustor_SZ_joint_EI_soot_prev
                                
                            else:     
                                for t_idx in range(n_cp):
                                    # Call cantera 
                                    results = evaluate_cantera(combustor,T[t_idx,0],P[t_idx,0],mdot[t_idx,0],FAR[t_idx,0])
                                    
                                    EI_CO2_comb[t_idx,0]                = results.EI_CO2
                                    EI_CO_comb[t_idx,0]                 = results.EI_CO 
                                    EI_H2O_comb[t_idx,0]                = results.EI_H2O
                                    EI_NOx_comb[t_idx,0]                = results.EI_NOx 
                                    EI_soot_comb[t_idx,0]               = results.EI_soot
                                    combustor_PZ_phi[t_idx,:]           = results.PZ_phi
                                    combustor_PZ_T[t_idx,:]             = results.PZ_T
                                    combustor_PZ_f_psr[t_idx,:]         = results.PZ_f_psr
                                    combustor_PZ_EI_CO2[t_idx,:]        = results.PZ_EI_CO2
                                    combustor_PZ_EI_CO[t_idx,:]         = results.PZ_EI_CO
                                    combustor_PZ_EI_H2O[t_idx,:]        = results.PZ_EI_H2O
                                    combustor_PZ_EI_NOx[t_idx,:]        = results.PZ_EI_NOx
                                    combustor_PZ_EI_soot[t_idx,:]       = results.PZ_EI_soot
                                    combustor_SZ_sm_z[t_idx,:]          = results.SZ_sm_z
                                    combustor_SZ_sm_phi[t_idx,:]        = results.SZ_sm_phi
                                    combustor_SZ_sm_T[t_idx,:]          = results.SZ_sm_T
                                    combustor_SZ_sm_EI_CO2[t_idx,:]     = results.SZ_sm_EI_CO2
                                    combustor_SZ_sm_EI_CO[t_idx,:]      = results.SZ_sm_EI_CO
                                    combustor_SZ_sm_EI_H2O[t_idx,:]     = results.SZ_sm_EI_H2O
                                    combustor_SZ_sm_EI_NOx[t_idx,:]     = results.SZ_sm_EI_NOx
                                    combustor_SZ_sm_EI_soot[t_idx,:]    = results.SZ_sm_EI_soot
                                    combustor_SZ_fm_z[t_idx,:]          = results.SZ_fm_z
                                    combustor_SZ_fm_phi[t_idx,:]        = results.SZ_fm_phi
                                    combustor_SZ_fm_T[t_idx,:]          = results.SZ_fm_T
                                    combustor_SZ_fm_EI_CO2[t_idx,:]     = results.SZ_fm_EI_CO2          
                                    combustor_SZ_fm_EI_CO[t_idx,:]      = results.SZ_fm_EI_CO     
                                    combustor_SZ_fm_EI_H2O[t_idx,:]     = results.SZ_fm_EI_H2O
                                    combustor_SZ_fm_EI_NOx[t_idx,:]     = results.SZ_fm_EI_NOx
                                    combustor_SZ_fm_EI_soot[t_idx,:]    = results.SZ_fm_EI_soot
                                    combustor_SZ_joint_z[t_idx,:]       = results.SZ_joint_z
                                    combustor_SZ_joint_phi[t_idx,:]     = results.SZ_joint_phi
                                    combustor_SZ_joint_T[t_idx,:]       = results.SZ_joint_T
                                    combustor_SZ_joint_EI_CO2[t_idx,:]  = results.SZ_joint_EI_CO2
                                    combustor_SZ_joint_EI_CO[t_idx,:]   = results.SZ_joint_EI_CO  
                                    combustor_SZ_joint_EI_H2O[t_idx,:]  = results.SZ_joint_EI_H2O 
                                    combustor_SZ_joint_EI_NOx[t_idx,:]  = results.SZ_joint_EI_NOx 
                                    combustor_SZ_joint_EI_soot[t_idx,:] = results.SZ_joint_EI_soot
                                    
                                    EI_CO2_prev                     = EI_CO2_comb 
                                    EI_CO_prev                      = EI_CO_comb  
                                    EI_H2O_prev                     = EI_H2O_comb 
                                    EI_NOx_prev                     = EI_NOx_comb  
                                    EI_soot_prev                    = EI_soot_comb
                                    combustor_PZ_phi_prev           = combustor_PZ_phi          
                                    combustor_PZ_T_prev             = combustor_PZ_T            
                                    combustor_PZ_f_psr_prev         = combustor_PZ_f_psr        
                                    combustor_PZ_EI_CO2_prev        = combustor_PZ_EI_CO2       
                                    combustor_PZ_EI_CO_prev         = combustor_PZ_EI_CO        
                                    combustor_PZ_EI_H2O_prev        = combustor_PZ_EI_H2O       
                                    combustor_PZ_EI_NOx_prev        = combustor_PZ_EI_NOx       
                                    combustor_PZ_EI_soot_prev       = combustor_PZ_EI_soot      
                                    combustor_SZ_sm_z_prev          = combustor_SZ_sm_z         
                                    combustor_SZ_sm_phi_prev        = combustor_SZ_sm_phi       
                                    combustor_SZ_sm_T_prev          = combustor_SZ_sm_T         
                                    combustor_SZ_sm_EI_CO2_prev     = combustor_SZ_sm_EI_CO2    
                                    combustor_SZ_sm_EI_CO_prev      = combustor_SZ_sm_EI_CO     
                                    combustor_SZ_sm_EI_H2O_prev     = combustor_SZ_sm_EI_H2O    
                                    combustor_SZ_sm_EI_NOx_prev     = combustor_SZ_sm_EI_NOx    
                                    combustor_SZ_sm_EI_soot_prev    = combustor_SZ_sm_EI_soot   
                                    combustor_SZ_fm_z_prev          = combustor_SZ_fm_z         
                                    combustor_SZ_fm_phi_prev        = combustor_SZ_fm_phi       
                                    combustor_SZ_fm_T_prev          = combustor_SZ_fm_T         
                                    combustor_SZ_fm_EI_CO2_prev     = combustor_SZ_fm_EI_CO2    
                                    combustor_SZ_fm_EI_CO_prev      = combustor_SZ_fm_EI_CO     
                                    combustor_SZ_fm_EI_H2O_prev     = combustor_SZ_fm_EI_H2O    
                                    combustor_SZ_fm_EI_NOx_prev     = combustor_SZ_fm_EI_NOx     
                                    combustor_SZ_fm_EI_soot_prev    = combustor_SZ_fm_EI_soot     
                                    combustor_SZ_joint_z_prev       = combustor_SZ_joint_z      
                                    combustor_SZ_joint_phi_prev     = combustor_SZ_joint_phi    
                                    combustor_SZ_joint_T_prev       = combustor_SZ_joint_T      
                                    combustor_SZ_joint_EI_CO2_prev  = combustor_SZ_joint_EI_CO2 
                                    combustor_SZ_joint_EI_CO_prev   = combustor_SZ_joint_EI_CO  
                                    combustor_SZ_joint_EI_H2O_prev  = combustor_SZ_joint_EI_H2O 
                                    combustor_SZ_joint_EI_NOx_prev  = combustor_SZ_joint_EI_NOx 
                                    combustor_SZ_joint_EI_soot_prev = combustor_SZ_joint_EI_soot
                                
                            CO2_total  += np.dot(I,mdot*EI_CO2_comb)
                            CO_total   += np.dot(I,mdot *EI_CO_comb )
                            H2O_total  += np.dot(I,mdot*EI_H2O_comb)
                            NOx_total   += np.dot(I,mdot *EI_NOx_comb ) 
                            Soot_total  += np.dot(I,mdot *EI_soot_comb)

    emissions                        = Data()
    emissions.total                  = Data()
    emissions.index                  = Data() 
    emissions.total.CO2              = CO2_total  * combustor.fuel_data.global_warming_potential_100.CO2 
    emissions.total.H2O              = H2O_total  * combustor.fuel_data.global_warming_potential_100.H2O  
    emissions.total.NOx              = NOx_total * combustor.fuel_data.global_warming_potential_100.NOx 
    emissions.total.Soot             = Soot_total * combustor.fuel_data.global_warming_potential_100.Soot
    emissions.index.CO2              = EI_CO2_comb
    emissions.index.CO               = EI_CO_comb 
    emissions.index.H2O              = EI_H2O_comb
    emissions.index.NOx              = EI_NOx_comb 
    emissions.index.Soot             = EI_soot_comb 
    emissions.index.PZ_phi           = combustor_PZ_phi
    emissions.index.PZ_T             = combustor_PZ_T
    emissions.index.PZ_f_psr         = combustor_PZ_f_psr
    emissions.index.PZ_EI_CO2        = combustor_PZ_EI_CO2
    emissions.index.PZ_EI_CO         = combustor_PZ_EI_CO
    emissions.index.PZ_EI_H2O        = combustor_PZ_EI_H2O
    emissions.index.PZ_EI_NOx        = combustor_PZ_EI_NOx
    emissions.index.PZ_EI_soot       = combustor_PZ_EI_soot
    emissions.index.SZ_sm_z          = combustor_SZ_sm_z
    emissions.index.SZ_sm_phi        = combustor_SZ_sm_phi
    emissions.index.SZ_sm_T          = combustor_SZ_sm_T   
    emissions.index.SZ_sm_EI_CO2     = combustor_SZ_sm_EI_CO2
    emissions.index.SZ_sm_EI_CO      = combustor_SZ_sm_EI_CO
    emissions.index.SZ_sm_EI_H2O     = combustor_SZ_sm_EI_H2O
    emissions.index.SZ_sm_EI_NOx     = combustor_SZ_sm_EI_NOx
    emissions.index.SZ_sm_EI_soot    = combustor_SZ_sm_EI_soot
    emissions.index.SZ_fm_z          = combustor_SZ_fm_z
    emissions.index.SZ_fm_phi        = combustor_SZ_fm_phi
    emissions.index.SZ_fm_T          = combustor_SZ_fm_T
    emissions.index.SZ_fm_EI_CO2     = combustor_SZ_fm_EI_CO2
    emissions.index.SZ_fm_EI_CO      = combustor_SZ_fm_EI_CO
    emissions.index.SZ_fm_EI_H2O     = combustor_SZ_fm_EI_H2O
    emissions.index.SZ_fm_EI_NOx     = combustor_SZ_fm_EI_NOx
    emissions.index.SZ_fm_EI_soot    = combustor_SZ_fm_EI_soot
    emissions.index.SZ_joint_z       = combustor_SZ_joint_z
    emissions.index.SZ_joint_phi     = combustor_SZ_joint_phi
    emissions.index.SZ_joint_T       = combustor_SZ_joint_T
    emissions.index.SZ_joint_EI_CO2  = combustor_SZ_joint_EI_CO2
    emissions.index.SZ_joint_EI_CO   = combustor_SZ_joint_EI_CO
    emissions.index.SZ_joint_EI_H2O  = combustor_SZ_joint_EI_H2O
    emissions.index.SZ_joint_EI_NOx  = combustor_SZ_joint_EI_NOx
    emissions.index.SZ_joint_EI_soot = combustor_SZ_joint_EI_soot
        
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
    NOx_total  = 0 * segment.state.ones_row(1) 
    Soot_total = 0 * segment.state.ones_row(1) 

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
                    
                    pts = np.hstack((T,P,mdot,FAR)) 

                    EI_CO2_comb  = np.atleast_2d(surrogates.EI_CO2(pts)).T
                    EI_CO_comb   = np.atleast_2d(surrogates.EI_CO(pts)).T 
                    EI_H2O_comb  = np.atleast_2d(surrogates.EI_H2O(pts)).T 
                    EI_NOx_comb  = np.atleast_2d(surrogates.EI_NOx(pts)).T 
                    EI_soot_comb = np.atleast_2d(surrogates.EI_soot(pts)).T        
                          
                    CO2_total += np.dot(I,mdot*EI_CO2_comb)
                    CO_total  += np.dot(I,mdot *EI_CO_comb )
                    H2O_total += np.dot(I,mdot*EI_H2O_comb)
                    NOx_total  += np.dot(I,mdot *EI_NOx_comb ) 
                    Soot_total += np.dot(I,mdot *EI_soot_comb)

    emissions                 = Data()
    emissions.total           = Data()
    emissions.index           = Data() 
    emissions.total.CO2       = CO2_total * combustor.fuel_data.global_warming_potential_100.CO2 
    emissions.total.H2O       = H2O_total * combustor.fuel_data.global_warming_potential_100.H2O  
    emissions.total.NOx       = NOx_total * combustor.fuel_data.global_warming_potential_100.NOx 
    emissions.total.Soot      = Soot_total * combustor.fuel_data.global_warming_potential_100.Soot
    emissions.index.CO2       = EI_CO2_comb
    emissions.index.CO        = EI_CO_comb 
    emissions.index.H2O       = EI_H2O_comb
    emissions.index.NOx       = EI_NOx_comb 
    emissions.index.Soot      = EI_soot_comb 
 
    segment.state.conditions.emissions =  emissions
    return   

     