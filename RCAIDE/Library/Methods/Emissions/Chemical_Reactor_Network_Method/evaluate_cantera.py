# RCAIDE/Library/Methods/Emissions/Chemical_Reactor_Network_Method/evaluate_cantera.py

# Created: June 2024, M. Clarke, M. Guidotti 
# Updated: Mar 2025, M. Guidotti
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from   RCAIDE.Framework.Core import Data  
from   RCAIDE.Library.Components.Powertrain.Converters.Combustor import Combustor
from   RCAIDE.Library.Attributes.Propellants.Jet_A1 import Jet_A1
from   RCAIDE.Library.Attributes.Gases.Air import Air
import pandas                as pd
import numpy                 as np
import importlib
import os

# ----------------------------------------------------------------------------------------------------------------------
#  evaluate_cantera
# ----------------------------------------------------------------------------------------------------------------------   
def evaluate_cantera(combustor,T,P,mdot_air,FAR,mdot_air_TakeOff, FAR_TakeOff): 

    """
    Evaluates emission indices using Chemical Reactor Network (CRN) built in Cantera.
    
    Parameters
    ----------
    combustor : Data
        Combustor configuration data
        
        - f_air_PZ : float
            Fraction of total air entering Primary Zone [-]
        - FAR_st : float
            Stoichiometric Fuel to Air ratio [-]
        - N_comb : int
            Number of can-annular combustors [-]
        - N_PZ : int
            Number of PSR (Perfectly Stirred Reactors) [-]
        - A_PZ : float
            Primary Zone cross-sectional area [m^2]
        - L_PZ : float
            Primary Zone length [m]
        - N_SZ : int
            Number of dilution air inlets [-]
        - A_SZ : float
            Secondary Zone cross-sectional area [m^2]
        - L_SZ : float
            Secondary Zone length [m]
        - phi_SZ : float
            Equivalence Ratio in the Secondary Zone [-]
        - S_PZ : float
            Mixing parameter for Primary Zone [-]
        - F_SC : float
            Fuel scaler [-]
        - number_of_assigned_PSR_1st_mixers : int
            Number of PSR assigned to first row of mixers [-]
        - number_of_assigned_PSR_2nd_mixers : int
            Number of PSR assigned to second row of mixers [-]
        - fuel_data : Data
            Fuel chemical properties and kinetics data

    T : float
        Stagnation Temperature entering combustors [K]
    P : float
        Stagnation Pressure entering combustors [Pa]
    mdot : float
        Air mass flow enetring the combustor [kg/s]
    FAR : float
        Fuel-to-Air ratio [-]

    Returns
    -------
    results : Data
        Container for emission indices
        
        - EI_CO2 : float
            CO2 emission index [kg_CO2/kg_fuel]
        - EI_CO : float
            CO emission index [kg_CO/kg_fuel]
        - EI_H2O : float
            H2O emission index [kg_H2O/kg_fuel]
        - EI_NO : float
            NO emission index [kg_NO/kg_fuel]
        - EI_NO2 : float
            NO2 emission index [kg_NO2/kg_fuel]

    Notes
    -----
    This model estimates emission indices using a Chemical Reactor Network built with Cantera.
    The network consists of Perfectly Stirred Reactors (PSRs) in the Primary Zone and 
    Plug Flow Reactors (PFRs) in the Secondary Zone.

    **Extra modules required**

    * Cantera
    * pandas
    * numpy

    **Major Assumptions**

    * PSRs represent the Primary Zone
    * Air is added between different PFRs in Secondary Zone
    * Primary Zone has a normal Equivalence Ratio distribution

    **Theory**
    The model uses a network of reactors to simulate combustion:
    
    1. Primary Zone: Multiple PSRs with varying equivalence ratios
    2. Mixing Zones: Ideal mixing between reactor outputs
    3. Secondary Zone: PFRs with dilution air addition

    See Also
    --------
    RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method.evaluate_CRN_emission_indices
    RCAIDE.Library.Components.Powertrain.Converters.Combustor

    References
    ----------
    [1] Goodwin, D. G., Speth, R. L., Moffat, H. K., & Weber, B. W. (2023). Cantera: An object-oriented software toolkit for chemical kinetics, thermodynamics, and transport processes (Version 3.0.0) [Computer software]. https://www.cantera.org
    """   

    # ------------------------------------------------------------------------------              
    # ------------------------------ Combustor Inputs ------------------------------              
    # ------------------------------------------------------------------------------              
    
    # User Inputs
    combustor                                         = Combustor()    # [-] Combustor object
    combustor.air                                     = Air()          # [-] Air object
    combustor.fuel                                    = Jet_A1()       # [-] Fuel object

    rcaide_root                                       = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
    mechanism_path                                    = os.path.join(rcaide_root, 'RCAIDE', 'RCAIDE', 'Library', 'Methods', 'Emissions', 'Chemical_Reactor_Network_Method', 'Data', 'POLIMI_PRF_PAH_RFUELS_HT_1412.yaml')
    soot_model_path                                   = os.path.join(rcaide_root, 'RESEARCH', '06_Aircraft_Electrification_Trade_Study', 'Combustor', 'Soot_Modeling.py')
    gas                                               = mechanism_path
    spec                                              = importlib.util.spec_from_file_location("Soot_Modeling", soot_model_path)
    soot_model                                        = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(soot_model)

    try: 
        import cantera as ct 
        combustor_results = compute_combustor_performance(combustor, T, P, mdot_air, FAR, mdot_air_TakeOff, FAR_TakeOff, gas, soot_model) # [-]       Run combustor function
        
        results                 = Data()                                            # [-]       Create results data structure
        results.EI_CO2          = combustor_results['SZ']['final']['EI']['CO2']     # [-]       Assign CO2 Emission Index to the results data structure
        results.EI_CO           = combustor_results['SZ']['final']['EI']['CO']      # [-]       Assign CO Emission Index to the results data structure
        results.EI_H2O          = combustor_results['SZ']['final']['EI']['H2O']     # [-]       Assign H2O Emission Index to the results data structure
        results.EI_NOx          = combustor_results['SZ']['final']['EI']['NOx']     # [-]       Assign NO Emission Index to the results data structure
        results.EI_soot         = combustor_results['SZ']['final']['EI']['soot']    # [-]       Assign NO2 Emission Index to the results data structure   
 
    except ImportError:
        print('cantera required: run pip install cantera') 
        results                 = Data()                                               
        results.EI_CO2          = 0                               
        results.EI_CO           = 0                               
        results.EI_H2O          = 0                               
        results.EI_NOx          = 0                                                    
        results.EI_soot         = 0                                                                           
     
    return results

# ----------------------------------------------------------------------
#  RQL Burner Model
# ----------------------------------------------------------------------

def compute_combustor_performance(combustor, Temp_air, Pres_air, mdot_air_tot, FAR, mdot_air_TakeOff, FAR_TakeOff, gas, soot_model):

    mdot_fuel_TakeOff = FAR_TakeOff * mdot_air_TakeOff                 # [kg/s] Fuel mass flow rate at Take Off
    mdot_fuel_tot     = mdot_air_tot * FAR                             # [kg/s] Fuel mass flow rate 
    mdot_air          = mdot_air_tot / combustor.number_of_combustors  # [kg/s] Air mass flow rate per combustor
    mdot_fuel         = mdot_fuel_tot / combustor.number_of_combustors # [kg/s] Fuel mass flow rate per combustor

    f_air_PZ          = mdot_fuel_TakeOff * combustor.F_SC / (combustor.design_equivalence_ratio_PZ * mdot_air_TakeOff * combustor.fuel.stoichiometric_fuel_air_ratio) # [-] Air mass flow rate fraction in Primary Zone
    phi_sign          = (mdot_fuel * combustor.F_SC) / (mdot_air * f_air_PZ * combustor.fuel.stoichiometric_fuel_air_ratio) # [-] Mean equivalence ratio in the Primary Zone
    sigma_phi         = phi_sign * combustor.S_PZ                      # [-] Standard deviation of the Equivalence Ratio in the Primary Zone
    A_PZ              = np.pi * (combustor.diameter / 2) ** 2          # [m^2] Cross-sectional area of the Primary Zone
    V_PZ              = A_PZ * combustor.L_PZ                          # [m^3] Volume of the Primary Zone
    V_PZ_PSR          = V_PZ / combustor.N_PZ                          # [m^3] Volume of each PSR
    mdot_air_PZ       = f_air_PZ * mdot_air                            # [kg/s] Air mass flow rate in the Primary Zone
    phi_PSR           = np.linspace(phi_sign - 2 * sigma_phi, phi_sign + 2 * sigma_phi, combustor.N_PZ) # [-] Equivalence ratio in each PSR 
    Delta_phi         = np.abs(phi_PSR[0] - phi_PSR[1])                # [-] Equivalence ratio step in the Primary Zone

    fuel              = ct.Solution(gas)                               # [-] Fuel object
    fuel.TPX          = combustor.fuel.temperature, combustor.fuel.pressure, combustor.fuel.fuel_surrogate_S1                   # [K, Pa, -] Temperauture, Pressure and Mole fraction composition of fuel
    fuel_reservoir    = ct.Reservoir(fuel)                             # [-] Fuel reservoir
    air               = ct.Solution(gas)                               # [-] Air object
    air.TPX           = Temp_air, Pres_air, combustor.air.air_surrogate                      # [K, Pa, -] Temperauture, Pressure and Mole fraction composition of air
    air_reservoir     = ct.Reservoir(air)                              # [-] Air reservoir
    fuel_hot          = ct.Solution(gas)                               # [-] Fuel hot state
    fuel_hot.TPX      = Temp_air, Pres_air, combustor.fuel.fuel_surrogate_S1                     # [K, Pa, -] Temperauture, Pressure and Mole fraction composition of hot fuel
    delta_h           = np.abs(fuel.h - fuel_hot.h)                    # [J/kg] Fuel specific enthalpy difference

    PZ_Structures     = {"PSRs": {}, "MFC_AirToPSR": {}, "MFC_FuelToPSR": {}, "PSR_Networks": {}, "MFC_PSRToMixer": {}} # [-] Primary Zone structures
    mdot_PZ, f_PSR_data, mass_psr_list = [], [], [] # [-] Arrays to store results
    mixer             = ct.ConstPressureReactor(air)                   # [-] Mixer object

    phi_diff          = phi_PSR - phi_sign                             # [-] Equivalence ratio difference
    f_PSR_data        = (1 / (np.sqrt(2 * np.pi) * sigma_phi)) * np.exp(-(phi_diff ** 2) / (2 * sigma_phi ** 2)) * Delta_phi # [-] Fraction of mass flow entering each reactor
    f_PSR_data       /= np.sum(f_PSR_data)                             # [-] Normalizes mass flow rate fraction in each PSR
    
    combustor_results = {
                        'PZ': {
                            'psr': {
                                'phi': [],
                                'T': [],
                                'f_psr':[],
                                'EI': {
                                    'CO2': [],
                                    'CO': [],
                                    'H2O': [],
                                    'NOx': [],
                                    'soot': []
                                }
                            },
                            'final': {
                                'phi': None,
                                'T': None,
                                'EI': {
                                    'CO2': None,
                                    'CO': None,
                                    'H2O': None,
                                    'NOx': None,
                                    'soot': None
                                }
                            }
                        },
                        'SZ': {
                            'sm': {
                                'phi': [],
                                'T': [],
                                'z': [],
                                'EI': {
                                    'CO2': [],
                                    'CO': [],
                                    'H2O': [],
                                    'NOx': [],
                                    'soot': []
                                }
                            },
                            'fm': {
                                'phi': [],
                                'T': [],
                                'z': [],
                                'EI': {
                                    'CO2': [],
                                    'CO': [],
                                    'H2O': [],
                                    'NOx': [],
                                    'soot': []
                                }
                            },
                            'joint': {
                                'phi': [],
                                'T': [],
                                'z': [],
                                'EI': {
                                    'CO2': [],
                                    'CO': [],
                                    'H2O': [],
                                    'NOx': [],
                                    'soot': []
                                }
                            },
                            'final': {
                                'phi': None,
                                'T': None,
                                'EI': {
                                    'CO2': None,
                                    'CO': None,
                                    'H2O': None,
                                    'NOx': None,
                                    'soot': None
                                }
                            }
                        }
                    }

    # ------------------------------------------------------------------
    #  Primary Zone (PZ)
    # ------------------------------------------------------------------

    for i in range(combustor.N_PZ):

        f_PSR_PZ_i      = f_PSR_data[i]                                # [-] Fuel mass flow rate fraction in the PSR
        mdot_air_PZ_i   = f_PSR_PZ_i * (mdot_air_PZ + mdot_fuel) / (1 + phi_PSR[i] * combustor.fuel.stoichiometric_fuel_air_ratio) # [kg/s] Air mass flow rate in the PSR
        mdot_fuel_PZ_i  = mdot_air_PZ_i * phi_PSR[i] * combustor.fuel.stoichiometric_fuel_air_ratio # [kg/s] Fuel mass flow rate in the PSR
        mdot_total_PZ_i = mdot_air_PZ_i + mdot_fuel_PZ_i               # [kg/s] Total mass flow rate in the PSR
        mdot_PZ.append(mdot_total_PZ_i)                                # [-] Store total mass flow rate in the PSR

        h_mix_PZ_i      = (1 / (mdot_air_PZ_i + mdot_fuel_PZ_i)) * (mdot_air_PZ_i * air.h + mdot_fuel_PZ_i * fuel_hot.h - mdot_fuel_PZ_i * (combustor.fuel.heat_of_vaporization + delta_h)) # [J/kg] Mixture specific enthalpy
        psr_gas_PZ_i    = ct.Solution(gas)                             # [-] PSR gas object
        psr_gas_PZ_i.set_equivalence_ratio(phi_PSR[i], combustor.fuel.fuel_surrogate_S1, combustor.air.air_surrogate)  # [-] Set equivalence ratio, fuel, and air mole fractions
        psr_gas_PZ_i.HP = h_mix_PZ_i, Pres_air                         # [J/kg, Pa] Set enthalpy and pressure
        psr_gas_PZ_i.equilibrate('HP')                                 # [-] Equilibrate the gas at constant enthalpy and pressure
        psr_PZ_i        = ct.ConstPressureReactor(psr_gas_PZ_i, name=f'PSR_{i+1}') # [-] PSR object
        psr_PZ_i.volume = V_PZ_PSR                                     # [m^3] PSR volume
        PZ_Structures["PSRs"][f'PSR_{i+1}'] = psr_PZ_i                 # [-] Store PSR object

        mfc_air_PZ_i       = ct.MassFlowController(air_reservoir, psr_PZ_i, name=f'AirToPSR_{i+1}', mdot=mdot_air_PZ_i) # [-] Air mass flow controller
        PZ_Structures["MFC_AirToPSR"][f'AirToPSR_{i+1}'] = mfc_air_PZ_i # [-] Store air mass flow controller
        mfc_fuel_PZ_i      = ct.MassFlowController(fuel_reservoir, psr_PZ_i, name=f'FuelToPSR_{i+1}', mdot=mdot_fuel_PZ_i) # [-] Fuel mass flow controller
        PZ_Structures["MFC_FuelToPSR"][f'FuelToPSR_{i+1}'] = mfc_fuel_PZ_i # [-] Store fuel mass flow controller

        psr_network_PZ_i   = ct.ReactorNet([psr_PZ_i])                 # [-] PSR network setup
        rho_PZ_i           = psr_gas_PZ_i.density                      # [kg/m^3] Gas density in the PSR
        t_res_PZ_i         = V_PZ_PSR * rho_PZ_i / mdot_total_PZ_i     # [s] Residence time in the PSR
        mass_psr_list.append(t_res_PZ_i * mdot_total_PZ_i)             # [-] Mass of PSR
        psr_network_PZ_i.advance(t_res_PZ_i)                           # [-] Advance the PSR network

        mfc_out_PZ_i       = ct.MassFlowController(psr_PZ_i, mixer, name=f'PSRToMixer_{i+1}', mdot = mdot_total_PZ_i) # [-] PSR to mixer mass flow controller
        PZ_Structures["MFC_PSRToMixer"][f'PSRToMixer_{i+1}'] = mfc_out_PZ_i # [-] Store PSR to mixer mass flow controller
        
        total_moles_PZ_i   = (rho_PZ_i * V_PZ_PSR) / psr_gas_PZ_i.mean_molecular_weight # [kmol] ???
        
        PAH_conc_PZ_i      = [psr_gas_PZ_i['C12H8'].X[0]*total_moles_PZ_i/(V_PZ_PSR), 
                              psr_gas_PZ_i['BIPHENYL'].X[0]*total_moles_PZ_i/(V_PZ_PSR),
                              psr_gas_PZ_i['FLUORENE'].X[0]*total_moles_PZ_i/(V_PZ_PSR),
                              psr_gas_PZ_i['C14H10'].X[0]*total_moles_PZ_i/(V_PZ_PSR),
                              psr_gas_PZ_i['C16H10'].X[0]*total_moles_PZ_i/(V_PZ_PSR)] # [kmol/m^3] ???
        
        C2H2_conc_PZ_i     = psr_gas_PZ_i['C2H2'].X[0]*total_moles_PZ_i/(V_PZ_PSR) # [kmol/m^3] ???
        OH_PZ_i            = psr_gas_PZ_i['OH'].X[0]*total_moles_PZ_i/(V_PZ_PSR) # [kmol/m^3] ???
        O2_PZ_i            = psr_gas_PZ_i['O2'].X[0]*total_moles_PZ_i/(V_PZ_PSR) # [kmol/m^3] ???
        O_PZ_i             = psr_gas_PZ_i['O'].X[0]*total_moles_PZ_i/(V_PZ_PSR) # [kmol/m^3] ???

        dM_dt_outflow = (-mdot_total_PZ_i / (rho_PZ_i * V_PZ_PSR)) * combustor.fuel.M_mech # [-] ???
        dM_dt_mech = soot_model.total_mech_mass(combustor.fuel.nuc_fac, combustor.fuel.sg_fac, combustor.fuel.ox_fac, combustor.fuel.L, combustor.fuel.PAH_species, combustor.fuel.radii, combustor.fuel.mu_matrix, psr_gas_PZ_i.T, combustor.fuel.n_C_matrix, PAH_conc_PZ_i, C2H2_conc_PZ_i, OH_PZ_i, O2_PZ_i, O_PZ_i) + dM_dt_outflow # [kg/m^3/s] ???
        mdot_soot_PZ_i = dM_dt_mech * V_PZ_PSR                         # [kg/s] Soot mass flow rate
        
        EI = calculate_emission_indices(psr_PZ_i, mdot_total_PZ_i, mdot_fuel_PZ_i, mdot_soot_PZ_i) # [-] Emission indices computation

        combustor_results['PZ']['psr']['phi'].append(phi_PSR[i])                 # [-] Store Equivalence ratio
        combustor_results['PZ']['psr']['T'].append(psr_gas_PZ_i.T)               # [K] Store Temperature
        combustor_results['PZ']['psr']['f_psr'].append(f_PSR_PZ_i)               # [-] Store mass flow rate fraction
        combustor_results['PZ']['psr']['EI']['NOx'].append(EI['NOx'])            # [-] Store NOx emission index
        combustor_results['PZ']['psr']['EI']['CO2'].append(EI['CO2'])            # [-] Store CO2 emission index
        combustor_results['PZ']['psr']['EI']['CO'].append(EI['CO'])              # [-] Store CO emission index
        combustor_results['PZ']['psr']['EI']['H2O'].append(EI['H2O'])            # [-] Store H2O emission index
        combustor_results['PZ']['psr']['EI']['soot'].append(EI['soot'])          # [-] Store soot emission index

    # ----------------------------------------------------------------------
    #  Initial Mixing
    # ----------------------------------------------------------------------
    
    mdot_tot_PZ       = sum(mdot_PZ)                                   # [kg/s] Total mass flow rate of the PZ
    mixture_list      = []                                             # [-] Mixture list
    for i in range(combustor.N_PZ):
        psr_output    = PZ_Structures["PSRs"][f'PSR_{i+1}'].thermo     # [-] PSR output
        mixture       = ct.Quantity(psr_output, constant='HP')         # [-] Mixture Quantity setup
        mixture.TPX   = psr_output.T, psr_output.P, psr_output.X       # [K, Pa, -] Temperauture, Pressure and Mole fraction composition of the ixture
        mixture.moles = mass_psr_list[i] / psr_output.mean_molecular_weight # [-] Mixture moles
        mixture_list.append(mixture)                                   # [-] Store mixture moles
    mixture_sum       = mixture_list[0]                                # [-] Define Mixture sum
    for mixture in mixture_list[1:]: 
        mixture_sum  += mixture                                        # [-] Add all into a Mixture sum
 
    rho_PZ = mixture_sum.density                                       # [kg/m**3] Mixture density

    total_moles_PZ = (rho_PZ * V_PZ) / mixture_sum.mean_molecular_weight # [kmol] Mixture moles
        
    PAH_conc_PZ    = [mixture_sum.phase['C12H8'].X[0]*total_moles_PZ/(V_PZ), 
                      mixture_sum.phase['BIPHENYL'].X[0]*total_moles_PZ/(V_PZ),
                      mixture_sum.phase['FLUORENE'].X[0]*total_moles_PZ/(V_PZ),
                      mixture_sum.phase['C14H10'].X[0]*total_moles_PZ/(V_PZ),
                      mixture_sum.phase['C16H10'].X[0]*total_moles_PZ/(V_PZ)] # [kmol/m^3] ???
    
    C2H2_conc_PZ   = mixture_sum.phase['C2H2'].X[0]*total_moles_PZ/(V_PZ) # [kmol/m^3] ???
    OH_PZ        = mixture_sum.phase['OH'].X[0]*total_moles_PZ/(V_PZ)  # [kmol/m^3] ???
    O2_PZ        = mixture_sum.phase['O2'].X[0]*total_moles_PZ/(V_PZ)  # [kmol/m^3] ???
    O_PZ         = mixture_sum.phase['O'].X[0]*total_moles_PZ/(V_PZ)   # [kmol/m^3] ???
    
    dM_dt_outflow_PZ = (-mdot_tot_PZ / (rho_PZ * V_PZ)) * combustor.fuel.M_mech       # [-] ???
    dM_dt_mech_PZ = soot_model.total_mech_mass(combustor.fuel.nuc_fac, combustor.fuel.sg_fac, combustor.fuel.ox_fac, combustor.fuel.L, combustor.fuel.PAH_species, combustor.fuel.radii, combustor.fuel.mu_matrix, mixture_sum.T, combustor.fuel.n_C_matrix, PAH_conc_PZ, C2H2_conc_PZ, OH_PZ, O2_PZ, O_PZ) + dM_dt_outflow_PZ # [kg/m^3/s] ???
    mdot_soot_PZ = dM_dt_mech_PZ * V_PZ                                # [kg/s] Soot mass flow rate
    
    EI_mixer_initial = calculate_emission_indices(mixture_sum, mdot_tot_PZ, mdot_fuel, mdot_soot_PZ) # [-] Emission indices computation

    combustor_results['PZ']['final']['phi'] = mixture_sum.equivalence_ratio()    # [-] Store Equivalence ratio
    combustor_results['PZ']['final']['T'] = mixture_sum.T                        # [K] Store Temperature
    combustor_results['PZ']['final']['EI']['NOx'] = EI_mixer_initial['NOx']      # [-] Store NOx emission index
    combustor_results['PZ']['final']['EI']['CO2'] = EI_mixer_initial['CO2']      # [-] Store CO2 emission index
    combustor_results['PZ']['final']['EI']['CO'] = EI_mixer_initial['CO']        # [-] Store CO emission index
    combustor_results['PZ']['final']['EI']['H2O'] = EI_mixer_initial['H2O']      # [-] Store H2O emission index
    combustor_results['PZ']['final']['EI']['soot'] = EI_mixer_initial['soot']    # [-] Store soot emission index

    # ----------------------------------------------------------------------
    #  Secondary Zone
    # ----------------------------------------------------------------------

    combustor.L_SZ                      = combustor.length - combustor.L_PZ # [m] Secondary Zone length
    A_SZ                                = np.pi * (combustor.diameter / 2) ** 2 # [m^2] Cross-sectional area of Secondary Zone
    f_air_SA                            = mdot_fuel_TakeOff / (combustor.design_equivalence_ratio_SZ * combustor.fuel.stoichiometric_fuel_air_ratio * mdot_air_TakeOff) # [-] Secondary air mass flow fraction
    f_air_DA                            = 1 - f_air_PZ - f_air_SA      # [-] Dilution air mass flow fraction
    f_FM                                = 1 - combustor.f_SM           # [-] Fast mode fraction
    beta_SA_FM                          = (f_air_SA * f_FM * mdot_air) / (combustor.l_SA_FM * combustor.L_SZ) # [kg/s/m] Secondary air mass flow rate per unit length in fast mode
    beta_SA_SM                          = (f_air_SA * combustor.f_SM * mdot_air) / (combustor.l_SA_SM * combustor.L_SZ) # [kg/s/m] Secondary air mass flow rate per unit length in slow mode
    beta_DA                             = (f_air_DA * mdot_air) / ((combustor.l_DA_end - combustor.l_DA_start) * combustor.L_SZ) # [kg/s/m] Dilution air mass flow rate per unit length
    mdot_total_sm                       = combustor.f_SM * mdot_tot_PZ # [kg/s] Initial total mass flow rate in slow mode
    mdot_total_fm                       = f_FM * mdot_tot_PZ           # [kg/s] Initial total mass flow rate in fast mode
    dz                                  = combustor.L_SZ / combustor.N_SZ # [m] Discretization step size
    z_positions                         = np.linspace(0, combustor.L_SZ, combustor.N_SZ + 1) # [m] Axial position array

    # Slow Mode 
    mixed_gas_sm                        = ct.Solution(gas)             # [-] Slow mode gas object
    mixed_gas_sm.TPX                    = mixture_sum.T, mixture_sum.P, mixture_sum.X # [K, Pa, -] Initial state from PZ mixture
    reactor_sm                          = ct.ConstPressureReactor(mixed_gas_sm) # [-] Slow mode reactor
    sim_sm                              = ct.ReactorNet([reactor_sm])  # [-] Slow mode reactor network

    for z_sm in z_positions[1:int(combustor.joint_mixing_fraction * combustor.N_SZ) + 1]:
        z_frac_sm                       = z_sm / combustor.L_SZ        # [-] Fractional position in SZ
        mdot_air_added_sm               = (beta_SA_SM * dz if z_frac_sm <= combustor.l_SA_SM else
                                           beta_DA * dz if combustor.l_DA_start <= z_frac_sm <= combustor.l_DA_end else 0.0) # [kg/s] Air mass flow rate added
        previous_mdot_total_sm          = mdot_total_sm                # [kg/s] Previous total mass flow rate
        mdot_total_sm                  += mdot_air_added_sm            # [kg/s] Update total mass flow rate
        residence_time                  = dz * A_SZ * mixed_gas_sm.density / mdot_total_sm # [s] Residence time

        air_qty                         = ct.Quantity(air)             # [-] Air quantity object
        air_qty.mass                    = mdot_air_added_sm * residence_time # [kg] Mass of air added over residence time
        mix_qty                         = ct.Quantity(mixed_gas_sm)    # [-] Mixture quantity object
        mix_qty.mass                    = previous_mdot_total_sm * residence_time # [kg] Mass of existing mixture over residence time
        mixture_sm                      = mix_qty + air_qty            # [-] Combined mixture

        mixed_gas_sm.TP                 = mixture_sm.T, mixture_sm.P   # [K, Pa] Update temperature and pressure
        mixed_gas_sm.Y                  = mixture_sm.Y                 # [-] Update composition

        reactor_sm                      = ct.ConstPressureReactor(mixed_gas_sm) # [-] Slow mode reactor
        sim_sm                          = ct.ReactorNet([reactor_sm])  # [-] Reactor setup
        sim_sm.advance(residence_time)                                 # [-] Advance simulation

        rho_SZ_sm_i                     = mixed_gas_sm.density         # [kg/m^3] Density in slow mode
        V_SZ_sm_i                       = dz * A_SZ                    # [m^3] Volume of SZ segment
        total_moles_SZ_sm_i             = (rho_SZ_sm_i * V_SZ_sm_i) / mixed_gas_sm.mean_molecular_weight # [kmol] Total moles in segment
        PAH_conc_SZ_sm_i                = np.array([mixed_gas_sm[s].X[0] * total_moles_SZ_sm_i / V_SZ_sm_i for s in ['C12H8', 'BIPHENYL', 'FLUORENE', 'C14H10', 'C16H10']]) # [kmol/m^3] PAH concentrations
        C2H2_conc_SZ_sm_i               = mixed_gas_sm['C2H2'].X[0] * total_moles_SZ_sm_i / V_SZ_sm_i # [kmol/m^3] Acetylene concentration
        OH_SZ_sm_i                      = mixed_gas_sm['OH'].X[0] * total_moles_SZ_sm_i / V_SZ_sm_i # [kmol/m^3] OH concentration
        O2_SZ_sm_i                      = mixed_gas_sm['O2'].X[0] * total_moles_SZ_sm_i / V_SZ_sm_i # [kmol/m^3] O2 concentration
        O_SZ_sm_i                       = mixed_gas_sm['O'].X[0] * total_moles_SZ_sm_i / V_SZ_sm_i # [kmol/m^3] O concentration

        dM_dt_outflow_SZ_sm_i           = (-mdot_total_sm / (rho_SZ_sm_i * V_SZ_sm_i)) * combustor.fuel.M_mech # [kg/m^3/s] Soot mass outflow rate
        dM_dt_mech_SZ_sm_i              = soot_model.total_mech_mass(combustor.fuel.nuc_fac, combustor.fuel.sg_fac, combustor.fuel.ox_fac, combustor.fuel.L, combustor.fuel.PAH_species, combustor.fuel.radii, combustor.fuel.mu_matrix, mixed_gas_sm.T, combustor.fuel.n_C_matrix, PAH_conc_SZ_sm_i, C2H2_conc_SZ_sm_i, OH_SZ_sm_i, O2_SZ_sm_i, O_SZ_sm_i) + dM_dt_outflow_SZ_sm_i # [kg/m^3/s] Soot mass change rate
        mdot_soot_SZ_sm_i               = dM_dt_mech_SZ_sm_i * V_SZ_sm_i # [kg/s] Soot mass flow rate

        EI_sm                           = calculate_emission_indices(mixed_gas_sm, mdot_total_sm, combustor.f_SM * mdot_fuel, mdot_soot_SZ_sm_i) # [-] Emission indices    

        combustor_results['SZ']['sm']['phi'].append(mixed_gas_sm.equivalence_ratio()) # [-] Store equivalence ratio
        combustor_results['SZ']['sm']['T'].append(mixed_gas_sm.T)                # [K] Store temperature 
        combustor_results['SZ']['sm']['z'].append(z_frac_sm * 100)               # [%] Store position percentage 
        combustor_results['SZ']['sm']['EI']['CO2'].append(EI_sm['CO2'])          # [g/kg_fuel] Store CO2 emission index 
        combustor_results['SZ']['sm']['EI']['NOx'].append(EI_sm['NOx'])          # [g/kg_fuel] Store NOx emission index 
        combustor_results['SZ']['sm']['EI']['CO'].append(EI_sm['CO'])            # [g/kg_fuel] Store CO emission index 
        combustor_results['SZ']['sm']['EI']['H2O'].append(EI_sm['H2O'])          # [g/kg_fuel] Store H2O emission index
        combustor_results['SZ']['sm']['EI']['soot'].append(EI_sm['soot'])        # [g/kg_fuel] Store soot emission index

    # Fast Mode
    mixed_gas_fm                        = ct.Solution(gas)             # [-] Fast mode gas object
    mixed_gas_fm.TPX                    = mixture_sum.T, mixture_sum.P, mixture_sum.X # [K, Pa, -] Initial state from PZ mixture
    reactor_fm                          = ct.ConstPressureReactor(mixed_gas_fm) # [-] Fast mode reactor
    sim_fm                              = ct.ReactorNet([reactor_fm])  # [-] Fast mode reactor network

    for z_fm in z_positions[1:int(combustor.joint_mixing_fraction * combustor.N_SZ) + 1]:
        z_frac_fm                       = z_fm / combustor.L_SZ        # [-] Fractional position in SZ
        mdot_air_added_fm               = (beta_SA_FM * dz if z_frac_fm <= combustor.l_SA_FM else
                                           beta_DA * dz if combustor.l_DA_start <= z_frac_fm <= combustor.l_DA_end else 0.0) # [kg/s] Air mass flow rate added
        previous_mdot_total_fm          = mdot_total_fm                # [kg/s] Previous total mass flow rate
        mdot_total_fm                  += mdot_air_added_fm            # [kg/s] Update total mass flow rate
        residence_time                  = dz * A_SZ * mixed_gas_fm.density / mdot_total_fm # [s] Residence time

        air_qty                         = ct.Quantity(air)             # [-] Air quantity object
        air_qty.mass                    = mdot_air_added_fm * residence_time # [kg] Mass of air added over residence time
        mix_qty                         = ct.Quantity(mixed_gas_fm)    # [-] Mixture quantity object
        mix_qty.mass                    = previous_mdot_total_fm * residence_time # [kg] Mass of existing mixture over residence time
        mixture_fm                      = mix_qty + air_qty            # [-] Combined mixture

        mixed_gas_fm.TP                 = mixture_fm.T, mixture_fm.P   # [K, Pa] Update temperature and pressure
        mixed_gas_fm.Y                  = mixture_fm.Y                 # [-] Update composition

        reactor_fm                      = ct.ConstPressureReactor(mixed_gas_fm) # [-] Slow mode reactor
        sim_fm                          = ct.ReactorNet([reactor_fm])  # [-] Reactor setup
        sim_fm.advance(residence_time)                                 # [-] Advance simulation

        rho_SZ_fm_i                     = mixed_gas_fm.density         # [kg/m^3] Density in fast mode
        V_SZ_fm_i                       = dz * A_SZ                    # [m^3] Volume of SZ segment
        total_moles_SZ_fm_i             = (rho_SZ_fm_i * V_SZ_fm_i) / mixed_gas_fm.mean_molecular_weight # [kmol] Total moles in segment
        PAH_conc_SZ_fm_i                = np.array([mixed_gas_fm[s].X[0] * total_moles_SZ_fm_i / V_SZ_fm_i for s in ['C12H8', 'BIPHENYL', 'FLUORENE', 'C14H10', 'C16H10']]) # [kmol/m^3] PAH concentrations
        C2H2_conc_SZ_fm_i               = mixed_gas_fm['C2H2'].X[0] * total_moles_SZ_fm_i / V_SZ_fm_i # [kmol/m^3] Acetylene concentration
        OH_SZ_fm_i                      = mixed_gas_fm['OH'].X[0] * total_moles_SZ_fm_i / V_SZ_fm_i # [kmol/m^3] OH concentration
        O2_SZ_fm_i                      = mixed_gas_fm['O2'].X[0] * total_moles_SZ_fm_i / V_SZ_fm_i # [kmol/m^3] O2 concentration
        O_SZ_fm_i                       = mixed_gas_fm['O'].X[0] * total_moles_SZ_fm_i / V_SZ_fm_i # [kmol/m^3] O concentration

        dM_dt_outflow_SZ_fm_i           = (-mdot_total_fm / (rho_SZ_fm_i * V_SZ_fm_i)) * combustor.fuel.M_mech # [kg/m^3/s] Soot mass outflow rate
        dM_dt_mech_SZ_fm_i              = soot_model.total_mech_mass(combustor.fuel.nuc_fac, combustor.fuel.sg_fac, combustor.fuel.ox_fac, combustor.fuel.L, combustor.fuel.PAH_species, combustor.fuel.radii, combustor.fuel.mu_matrix, mixed_gas_fm.T, combustor.fuel.n_C_matrix, PAH_conc_SZ_fm_i, C2H2_conc_SZ_fm_i, OH_SZ_fm_i, O2_SZ_fm_i, O_SZ_fm_i) + dM_dt_outflow_SZ_fm_i # [kg/m^3/s] Soot mass change rate
        mdot_soot_SZ_fm_i               = dM_dt_mech_SZ_fm_i * V_SZ_fm_i # [kg/s] Soot mass flow rate

        EI_fm                           = calculate_emission_indices(mixed_gas_fm, mdot_total_fm, f_FM * mdot_fuel, mdot_soot_SZ_fm_i) # [-] Emission indices
        
        combustor_results['SZ']['fm']['phi'].append(mixed_gas_fm.equivalence_ratio()) # [-] Store equivalence ratio
        combustor_results['SZ']['fm']['T'].append(mixed_gas_fm.T)                # [K] Store temperature 
        combustor_results['SZ']['fm']['z'].append(z_frac_fm * 100)               # [%] Store position percentage 
        combustor_results['SZ']['fm']['EI']['CO2'].append(EI_fm['CO2'])          # [g/kg_fuel] Store CO2 emission index 
        combustor_results['SZ']['fm']['EI']['NOx'].append(EI_fm['NOx'])          # [g/kg_fuel] Store NOx emission index 
        combustor_results['SZ']['fm']['EI']['CO'].append(EI_fm['CO'])            # [g/kg_fuel] Store CO emission index 
        combustor_results['SZ']['fm']['EI']['H2O'].append(EI_fm['H2O'])          # [g/kg_fuel] Store H2O emission index
        combustor_results['SZ']['fm']['EI']['soot'].append(EI_fm['soot'])        # [g/kg_fuel] Store soot emission index

    # Joint Mixing 
    mixed_gas_joint                     = ct.Solution(gas)             # [-] Joint mode gas object
    total_mass_flow                     = mdot_total_sm + mdot_total_fm # [kg/s] Total mass flow rate after slow and fast modes
    sm_qty                              = ct.Quantity(mixed_gas_sm)    # [-] Slow mode quantity
    fm_qty                              = ct.Quantity(mixed_gas_fm)    # [-] Fast mode quantity
    joint_mixture                       = sm_qty + fm_qty              # [-] Combined mixture
    mixed_gas_joint.TP                  = joint_mixture.T, joint_mixture.P # [K, Pa] Initial temperature and pressure
    mixed_gas_joint.Y                   = joint_mixture.Y              # [-] Initial composition
    mdot_total_joint                    = total_mass_flow              # [kg/s] Initial total mass flow rate
    reactor_joint                       = ct.ConstPressureReactor(mixed_gas_joint) # [-] Joint mode reactor
    sim_joint                           = ct.ReactorNet([reactor_joint]) # [-] Joint mode reactor network

    for z_joint in z_positions[int(combustor.joint_mixing_fraction * combustor.N_SZ) + 1:]:
        z_frac_joint                    = z_joint / combustor.L_SZ     # [-] Fractional position in SZ
        mdot_air_added_joint            = (beta_SA_FM * dz if z_frac_joint <= combustor.l_SA_FM else
                                           beta_DA * dz if combustor.l_DA_start <= z_frac_joint <= combustor.l_DA_end else 0.0) # [kg/s] Air mass flow rate added
        previous_mdot_total_joint       = mdot_total_joint             # [kg/s] Previous total mass flow rate
        mdot_total_joint               += mdot_air_added_joint         # [kg/s] Update total mass flow rate
        residence_time                  = dz * A_SZ * mixed_gas_joint.density / mdot_total_joint # [s] Residence time

        air_qty                         = ct.Quantity(air)             # [-] Air quantity object
        air_qty.mass                    = mdot_air_added_joint * residence_time # [kg] Mass of air added over residence time
        mix_qty                         = ct.Quantity(mixed_gas_joint) # [-] Mixture quantity object
        mix_qty.mass                    = previous_mdot_total_joint * residence_time # [kg] Mass of existing mixture over residence time
        mixture_joint                   = mix_qty + air_qty            # [-] Combined mixture

        mixed_gas_joint.TP              = mixture_joint.T, mixture_joint.P # [K, Pa] Update temperature and pressure
        mixed_gas_joint.Y               = mixture_joint.Y              # [-] Update composition

        reactor_joint                   = ct.ConstPressureReactor(mixed_gas_joint) # [-] Joint mode reactor
        sim_joint                       = ct.ReactorNet([reactor_joint]) # [-] Reactor setup
        sim_joint.advance(residence_time)                              # [-] Advance simulation

        rho_SZ_joint_i                  = mixed_gas_joint.density      # [kg/m^3] Density in joint mode
        V_SZ_joint_i                    = dz * A_SZ                    # [m^3] Volume of SZ segment
        total_moles_SZ_joint_i          = (rho_SZ_joint_i * V_SZ_joint_i) / mixed_gas_joint.mean_molecular_weight # [kmol] Total moles in segment
        PAH_conc_SZ_joint_i             = np.array([mixed_gas_joint[s].X[0] * total_moles_SZ_joint_i / V_SZ_joint_i for s in ['C12H8', 'BIPHENYL', 'FLUORENE', 'C14H10', 'C16H10']]) # [kmol/m^3] PAH concentrations
        C2H2_conc_SZ_joint_i            = mixed_gas_joint['C2H2'].X[0] * total_moles_SZ_joint_i / V_SZ_joint_i # [kmol/m^3] Acetylene concentration
        OH_SZ_joint_i                   = mixed_gas_joint['OH'].X[0] * total_moles_SZ_joint_i / V_SZ_joint_i # [kmol/m^3] OH concentration
        O2_SZ_joint_i                   = mixed_gas_joint['O2'].X[0] * total_moles_SZ_joint_i / V_SZ_joint_i # [kmol/m^3] O2 concentration
        O_SZ_joint_i                    = mixed_gas_joint['O'].X[0] * total_moles_SZ_joint_i / V_SZ_joint_i # [kmol/m^3] O concentration

        dM_dt_outflow_SZ_joint_i        = (-mdot_total_joint / (rho_SZ_joint_i * V_SZ_joint_i)) * combustor.fuel.M_mech # [kg/m^3/s] Soot mass outflow rate
        dM_dt_mech_SZ_joint_i           = soot_model.total_mech_mass(combustor.fuel.nuc_fac, combustor.fuel.sg_fac, combustor.fuel.ox_fac, combustor.fuel.L, combustor.fuel.PAH_species, combustor.fuel.radii, combustor.fuel.mu_matrix, mixed_gas_joint.T, combustor.fuel.n_C_matrix, PAH_conc_SZ_joint_i, C2H2_conc_SZ_joint_i, OH_SZ_joint_i, O2_SZ_joint_i, O_SZ_joint_i) + dM_dt_outflow_SZ_joint_i # [kg/m^3/s] Soot mass change rate
        mdot_soot_SZ_joint_i            = dM_dt_mech_SZ_joint_i * V_SZ_joint_i # [kg/s] Soot mass flow rate

        EI_joint                        = calculate_emission_indices(mixed_gas_joint, mdot_total_joint, mdot_fuel, mdot_soot_SZ_joint_i) # [-] Emission indices
        
        combustor_results['SZ']['joint']['phi'].append(mixed_gas_joint.equivalence_ratio()) # [-] Store equivalence ratio
        combustor_results['SZ']['joint']['T'].append(mixed_gas_joint.T)          # [K] Store temperature 
        combustor_results['SZ']['joint']['z'].append(z_frac_joint * 100)         # [%] Store position percentage 
        combustor_results['SZ']['joint']['EI']['CO2'].append(EI_joint['CO2'])    # [g/kg_fuel] Store CO2 emission index 
        combustor_results['SZ']['joint']['EI']['NOx'].append(EI_joint['NOx'])    # [g/kg_fuel] Store NOx emission index 
        combustor_results['SZ']['joint']['EI']['CO'].append(EI_joint['CO'])      # [g/kg_fuel] Store CO emission index 
        combustor_results['SZ']['joint']['EI']['H2O'].append(EI_joint['H2O'])    # [g/kg_fuel] Store H2O emission index
        combustor_results['SZ']['joint']['EI']['soot'].append(EI_joint['soot'])  # [g/kg_fuel] Store soot emission index

    combustor_results['SZ']['final']['EI']['soot'] = EI_joint['soot']  # [-] Store soot emission index

    return combustor_results

def calculate_emission_indices(reactor,  mdot_total, mdot_fuel, mdot_soot):
    """Calculate emission indices for combustion products"""
    gas                                 = reactor.thermo if hasattr(reactor, 'thermo') else reactor # [-] Extract gas object
    NOx_species                         = ['NO', 'NO2']              # [-] List of NOx species
    EI_NOx                              = 0.0                        # [g/kg_fuel] Initialize NOx emission index
    for species in NOx_species:
        try:
            idx                         = gas.species_index(species) # [-] Species index
            EI_NOx                     += gas.Y[idx] * 1000 * mdot_total / mdot_fuel # [g/kg_fuel] Add contribution to NOx EI
        except ValueError:
            continue
    EI                                  = {
        'CO2': gas.Y[gas.species_index('CO2')] * 1000 * mdot_total / mdot_fuel, # [g/kg_fuel] CO2 emission index
        'CO': gas.Y[gas.species_index('CO')] * 1000 * mdot_total / mdot_fuel,   # [g/kg_fuel] CO emission index
        'H2O': gas.Y[gas.species_index('H2O')] * 1000 * mdot_total / mdot_fuel, # [g/kg_fuel] H2O emission index
        'NOx': EI_NOx,                                                          # [g/kg_fuel] NOx emission index
        'soot': mdot_soot * 1000 / mdot_fuel                                   # [g/kg_fuel] Soot emission index
    }
    return EI                                                            # [-] Return emission indices dictionary