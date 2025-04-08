# RCAIDE/Library/Methods/Emissions/Chemical_Reactor_Network_Method/evaluate_cantera.py

# Created: June 2024, M. Clarke, M. Guidotti 
# Updated: Mar 2025, M. Guidotti, J. Dost, D. Mehta

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from   RCAIDE.Framework.Core import Data  
import numpy                 as np
import os
import cantera as ct

# ----------------------------------------------------------------------------------------------------------------------
#  evaluate_cantera
# ----------------------------------------------------------------------------------------------------------------------   
def evaluate_cantera(combustor,T,P,mdot_air,FAR): 

    """
    Evaluates emission indices using a Chemical Reactor Network (CRN) built in Cantera.
    
    Parameters
    ----------
    combustor : Data
        Combustor configuration data
        - diameter : float
            Combustor diameter [m]
        - length : float
            Combustor length [m]
        - number_of_combustors : int
            Number of combustors for one engine [-]
        - F_SC : float
            Fuel scale factor [-]
        - N_PZ : int
            Number of PSR in the Primary Zone [-]
        - L_PZ : float
            Primary Zone length [m]
        - S_PZ : float
            Mixing parameter in the Primary Zone [-]
        - design_equivalence_ratio_PZ : float
            Design Equivalence Ratio in Primary Zone at Maximum Throttle [-]
        - N_SZ : int
            Number of discretizations in the Secondary Zone [-]
        - f_SM : float
            Slow mode fraction [-]
        - l_SA_SM : float
            Secondary air length fraction (of L_SZ) in slow mode [-]
        - l_SA_FM : float
            Secondary air length fraction (of L_SZ) in fast mode [-]
        - l_DA_start : float
            Dilution air start length fraction (of L_SZ) [-]
        - l_DA_end : float
            Dilution air end length fraction (of L_SZ) [-]
        - joint_mixing_fraction : float
            Joint mixing fraction [-]
        - design_equivalence_ratio_SZ : float
            Design Equivalence Ratio in Secondary Zone [-]
        - air_mass_flow_rate_take_off : float
            Air mass flow rate at take-off [kg/s]
        - fuel_to_air_ratio_take_off : float
            Fuel to air ratio at take-off [-]
        - air_data : Data
            Air object containing air surrogate species
        - fuel_data : Data
            Fuel object containing fuel properties and kinetics
    
    T : float
        Stagnation Temperature entering combustors [K]
    P : float
        Stagnation Pressure entering combustors [Pa]
    mdot_air : float
        Air mass flow entering the combustor [kg/s]
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
        - EI_NOx : float
            NOx emission index [kg_NOx/kg_fuel]
        - final_phi : float
            Final equivalence ratio [-]
        - final_T : float
            Final temperature [K]
        - PZ_phi : list
            Equivalence ratio in the Primary Zone [-]
        - PZ_T : list
            Temperature in the Primary Zone [K]
        - PZ_f_psr : list
            Fraction of mass flow entering each PSR [-]
        - PZ_EI_CO2 : list
            CO2 emission index in the Primary Zone [kg_CO2/kg_fuel]
        - PZ_EI_CO : list
            CO emission index in the Primary Zone [kg_CO/kg_fuel]
        - PZ_EI_H2O : list
            H2O emission index in the Primary Zone [kg_H2O/kg_fuel]
        - PZ_EI_NOx : list
            NOx emission index in the Primary Zone [kg_NOx/kg_fuel]
        - SZ_sm_z : list
            Positions in the Secondary Zone slow mode [-]
        - SZ_sm_phi : list
            Equivalence ratio in the Secondary Zone slow mode [-]
        - SZ_sm_T : list
            Temperature in the Secondary Zone slow mode [K]
        - SZ_sm_EI_CO2 : list
            CO2 emission index in the Secondary Zone slow mode [kg_CO2/kg_fuel]
        - SZ_sm_EI_CO : list
            CO emission index in the Secondary Zone slow mode [kg_CO/kg_fuel]
        - SZ_sm_EI_H2O : list
            H2O emission index in the Secondary Zone slow mode [kg_H2O/kg_fuel]
        - SZ_sm_EI_NOx : list
            NOx emission index in the Secondary Zone slow mode [kg_NOx/kg_fuel]
        - SZ_fm_z : list
            Positions in the Secondary Zone fast mode [-]
        - SZ_fm_phi : list
            Equivalence ratio in the Secondary Zone fast mode [-]
        - SZ_fm_T : list
            Temperature in the Secondary Zone fast mode [K]
        - SZ_fm_EI_CO2 : list
            CO2 emission index in the Secondary Zone fast mode [kg_CO2/kg_fuel]
        - SZ_fm_EI_CO : list
            CO emission index in the Secondary Zone fast mode [kg_CO/kg_fuel]
        - SZ_fm_EI_H2O : list
            H2O emission index in the Secondary Zone fast mode [kg_H2O/kg_fuel]
        - SZ_fm_EI_NOx : list
            NOx emission index in the Secondary Zone fast mode [kg_NOx/kg_fuel]
        - SZ_joint_z : list
            Positions in the Secondary Zone joint mode [-]
        - SZ_joint_phi : list
            Equivalence ratio in the Secondary Zone joint mode [-]
        - SZ_joint_T : list
            Temperature in the Secondary Zone joint mode [K]
        - SZ_joint_EI_CO2 : list
            CO2 emission index in the Secondary Zone joint mode [kg_CO2/kg_fuel]
        - SZ_joint_EI_CO : list
            CO emission index in the Secondary Zone joint mode [kg_CO/kg_fuel]
        - SZ_joint_EI_H2O : list
            H2O emission index in the Secondary Zone joint mode [kg_H2O/kg_fuel]
        - SZ_joint_EI_NOx : list
            NOx emission index in the Secondary Zone joint mode [kg_NOx/kg_fuel]
    
    Notes
    -----
    This function uses Cantera to simulate the chemical kinetics and thermodynamics of the combustor. It requires the Cantera module to be installed.
    
    **Extra modules required**
    
    * Cantera
    
    **Major Assumptions**
        * The combustor operates under steady-state conditions.
        * The fuel and air are perfectly mixed before entering the combustor.
    
    **Theory**
    
    The function evaluates the emission indices by simulating the chemical reactions in the combustor using Cantera. The emissions are calculated based on the chemical kinetics and thermodynamics of the fuel and air mixture.
    
    References
    ----------
    [1] Goodwin, D. G., Speth, R. L., Moffat, H. K., & Weber, B. W. (2023). Cantera: An object-oriented software toolkit for chemical kinetics, thermodynamics, and transport processes (Version 3.0.0) [Computer software]. https://www.cantera.org
    [2] Brink, L. F. J. (2020). Modeling the impact of fuel composition on aircraft engine NOₓ, CO, and soot emissions. Master's thesis, Massachusetts Institute of Technology.
    [3] Allaire, D. L. (2006). A physics-based emissions model for aircraft gas turbine combustors. Master's thesis, Massachusetts Institute of Technology.
    [4] Lefebvre, A. H., & Ballal, D. R. (2010). Gas turbine combustion: Alternative fuels and emissions (3rd ed.). CRC Press.
    
    See Also
    --------
    RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method.evaluate_CRN_emission_indices_no_surrogate
    RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method.evaluate_CRN_emission_indices_surrogate
    """

    # ------------------------------------------------------------------------------              
    # ------------------------------ Combustor Inputs ------------------------------              
    # ------------------------------------------------------------------------------              

    rcaide_root    = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    mechanism_path = os.path.join(rcaide_root, 'Emissions', 'Chemical_Reactor_Network_Method', 'Data')
    gas            = mechanism_path + '/' + combustor.fuel_data.kinetic_mechanism

    data                  = Data()         
    data.final            = Data()
    data.final.EI         = Data()
    data.final.EI.CO2     = 0 # [kg/kg_fuel]                             
    data.final.EI.CO      = 0 # [kg/kg_fuel]                               
    data.final.EI.H2O     = 0 # [kg/kg_fuel]                               
    data.final.EI.NOx     = 0 # [kg/kg_fuel]  
    data.final.phi        = 0 # [-] 
    data.final.T          = 0 # [K] 
    data.PZ               = Data()
    data.PZ.psr           = Data()
    data.PZ.psr.phi       = [] # [-] 
    data.PZ.psr.T         = [] # [K] 
    data.PZ.psr.f_psr     = [] # [-] 
    data.PZ.psr.EI        = Data()
    data.PZ.psr.EI.CO2    = [] # [kg/kg_fuel] 
    data.PZ.psr.EI.CO     = [] # [kg/kg_fuel] 
    data.PZ.psr.EI.H2O    = [] # [kg/kg_fuel] 
    data.PZ.psr.EI.NOx    = [] # [kg/kg_fuel] 
    data.PZ.phi           = 0 # [-] 
    data.PZ.T             = 0 # [K] 
    data.PZ.f_psr         = 0 # [-] 
    data.PZ.EI            = Data()
    data.PZ.EI.CO2        = 0 # [kg/kg_fuel] 
    data.PZ.EI.CO         = 0 # [kg/kg_fuel] 
    data.PZ.EI.H2O        = 0 # [kg/kg_fuel] 
    data.PZ.EI.NOx        = 0 # [kg/kg_fuel] 
    data.SZ               = Data()
    data.SZ.sm            = Data()
    data.SZ.sm.z          = [] # [-] 
    data.SZ.sm.phi        = [] # [-] 
    data.SZ.sm.T          = [] # [K] 
    data.SZ.sm.EI         = Data()
    data.SZ.sm.EI.CO2     = [] # [kg/kg_fuel] 
    data.SZ.sm.EI.CO      = [] # [kg/kg_fuel] 
    data.SZ.sm.EI.H2O     = [] # [kg/kg_fuel] 
    data.SZ.sm.EI.NOx     = [] # [kg/kg_fuel] 
    data.SZ.fm            = Data()
    data.SZ.fm.z          = [] # [-] 
    data.SZ.fm.phi        = [] # [-] 
    data.SZ.fm.T          = [] # [K] 
    data.SZ.fm.EI         = Data()
    data.SZ.fm.EI.CO2     = [] # [kg/kg_fuel] 
    data.SZ.fm.EI.CO      = [] # [kg/kg_fuel] 
    data.SZ.fm.EI.H2O     = [] # [kg/kg_fuel] 
    data.SZ.fm.EI.NOx     = [] # [kg/kg_fuel] 
    data.SZ.joint         = Data()
    data.SZ.joint.z       = [] # [-] 
    data.SZ.joint.phi     = [] # [-] 
    data.SZ.joint.T       = [] # [K] 
    data.SZ.joint.EI      = Data()
    data.SZ.joint.EI.CO2  = [] # [kg/kg_fuel] 
    data.SZ.joint.EI.CO   = [] # [kg/kg_fuel] 
    data.SZ.joint.EI.H2O  = [] # [kg/kg_fuel] 
    data.SZ.joint.EI.NOx  = [] # [kg/kg_fuel]  

    try: 
        compute_combustor_performance(data, combustor, T, P, mdot_air, FAR, gas) # [-] Run combustor function

    except ImportError:
        print('cantera required: run pip install cantera')                                                                
     
    return data

# ----------------------------------------------------------------------
#  RQL Burner Model
# ----------------------------------------------------------------------

def compute_combustor_performance(results, combustor, Temp_air, Pres_air, mdot_air_tot, FAR, gas):

    mdot_fuel_TakeOff = combustor.fuel_to_air_ratio_take_off * combustor.air_mass_flow_rate_take_off # [kg/s] Fuel mass flow rate at Take Off
    mdot_fuel_tot     = mdot_air_tot * FAR                             # [kg/s] Fuel mass flow rate 
    mdot_air          = mdot_air_tot / combustor.number_of_combustors  # [kg/s] Air mass flow rate per combustor
    mdot_fuel         = mdot_fuel_tot / combustor.number_of_combustors # [kg/s] Fuel mass flow rate per combustor

    f_air_PZ          = mdot_fuel_TakeOff * combustor.F_SC / (combustor.design_equivalence_ratio_PZ * combustor.air_mass_flow_rate_take_off * combustor.fuel_data.stoichiometric_fuel_air_ratio) # [-] Air mass flow rate fraction in Primary Zone
    phi_sign          = (mdot_fuel * combustor.F_SC) / (mdot_air * f_air_PZ * combustor.fuel_data.stoichiometric_fuel_air_ratio) # [-] Mean equivalence ratio in the Primary Zone
    sigma_phi         = phi_sign * combustor.S_PZ                      # [-] Standard deviation of the Equivalence Ratio in the Primary Zone
    A_PZ              = np.pi * (combustor.diameter / 2) ** 2          # [m^2] Cross-sectional area of the Primary Zone
    V_PZ              = A_PZ * combustor.L_PZ                          # [m^3] Volume of the Primary Zone
    V_PZ_PSR          = V_PZ / combustor.N_PZ                          # [m^3] Volume of each PSR
    mdot_air_PZ       = f_air_PZ * mdot_air                            # [kg/s] Air mass flow rate in the Primary Zone
    phi_PSR           = np.linspace(phi_sign - 2 * sigma_phi, phi_sign + 2 * sigma_phi, combustor.N_PZ) # [-] Equivalence ratio in each PSR 
    Delta_phi         = np.abs(phi_PSR[0] - phi_PSR[1])                # [-] Equivalence ratio step in the Primary Zone

    fuel              = ct.Solution(gas)                               # [-] Fuel object
    fuel.TPX          = combustor.fuel_data.temperature, combustor.fuel_data.pressure, combustor.fuel_data.fuel_surrogate_S1                   # [K, Pa, -] Temperauture, Pressure and Mole fraction composition of fuel
    fuel_reservoir    = ct.Reservoir(fuel)                             # [-] Fuel reservoir
    air               = ct.Solution(gas)                               # [-] Air object
    air.TPX           = Temp_air, Pres_air, combustor.air_data.air_surrogate                      # [K, Pa, -] Temperauture, Pressure and Mole fraction composition of air
    air_reservoir     = ct.Reservoir(air)                              # [-] Air reservoir
    fuel_hot          = ct.Solution(gas)                               # [-] Fuel hot state
    fuel_hot.TPX      = Temp_air, Pres_air, combustor.fuel_data.fuel_surrogate_S1                     # [K, Pa, -] Temperauture, Pressure and Mole fraction composition of hot fuel
    delta_h           = np.abs(fuel.h - fuel_hot.h)                    # [J/kg] Fuel specific enthalpy difference

    PZ_Structures     = {"PSRs": {}, "MFC_AirToPSR": {}, "MFC_FuelToPSR": {}, "PSR_Networks": {}, "MFC_PSRToMixer": {}} # [-] Primary Zone structures
    mdot_PZ, f_PSR_data, mass_psr_list = [], [], [] # [-] Arrays to store results
    mixer             = ct.ConstPressureReactor(air)                   # [-] Mixer object

    phi_diff          = phi_PSR - phi_sign                             # [-] Equivalence ratio difference
    f_PSR_data        = (1 / (np.sqrt(2 * np.pi) * sigma_phi)) * np.exp(-(phi_diff ** 2) / (2 * sigma_phi ** 2)) * Delta_phi # [-] Fraction of mass flow entering each reactor
    f_PSR_data       /= np.sum(f_PSR_data)                             # [-] Normalizes mass flow rate fraction in each PSR
    
    # ------------------------------------------------------------------
    #  Primary Zone (PZ)
    # ------------------------------------------------------------------

    for i in range(combustor.N_PZ):

        f_PSR_PZ_i      = f_PSR_data[i]                                # [-] Fuel mass flow rate fraction in the PSR
        mdot_air_PZ_i   = f_PSR_PZ_i * (mdot_air_PZ + mdot_fuel) / (1 + phi_PSR[i] * combustor.fuel_data.stoichiometric_fuel_air_ratio) # [kg/s] Air mass flow rate in the PSR
        mdot_fuel_PZ_i  = mdot_air_PZ_i * phi_PSR[i] * combustor.fuel_data.stoichiometric_fuel_air_ratio # [kg/s] Fuel mass flow rate in the PSR
        mdot_total_PZ_i = mdot_air_PZ_i + mdot_fuel_PZ_i               # [kg/s] Total mass flow rate in the PSR
        mdot_PZ.append(mdot_total_PZ_i)                                # [-] Store total mass flow rate in the PSR

        h_mix_PZ_i      = (1 / (mdot_air_PZ_i + mdot_fuel_PZ_i)) * (mdot_air_PZ_i * air.h + mdot_fuel_PZ_i * fuel_hot.h - mdot_fuel_PZ_i * (combustor.fuel_data.heat_of_vaporization + delta_h)) # [J/kg] Mixture specific enthalpy
        psr_gas_PZ_i    = ct.Solution(gas)                             # [-] PSR gas object
        psr_gas_PZ_i.set_equivalence_ratio(phi_PSR[i], combustor.fuel_data.fuel_surrogate_S1, combustor.air_data.air_surrogate)  # [-] Set equivalence ratio, fuel, and air mole fractions
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
        
        EI = calculate_emission_indices(psr_PZ_i, mdot_total_PZ_i, mdot_fuel_PZ_i) # [-] Emission indices computation

        results.PZ.psr.phi.append(phi_PSR[i])                 # [-] Store Equivalence ratio
        results.PZ.psr.T.append(psr_gas_PZ_i.T)               # [K] Store Temperature
        results.PZ.psr.f_psr.append(f_PSR_PZ_i)               # [-] Store mass flow rate fraction
        results.PZ.psr.EI.NOx.append(EI['NOx'])            # [kg/kg_fuel] Store NOx emission index
        results.PZ.psr.EI.CO2.append(EI['CO2'])            # [kg/kg_fuel] Store CO2 emission index
        results.PZ.psr.EI.CO.append(EI['CO'])              # [kg/kg_fuel] Store CO emission index
        results.PZ.psr.EI.H2O.append(EI['H2O'])            # [kg/kg_fuel] Store H2O emission index\

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
  
    EI_mixer_initial = calculate_emission_indices(mixture_sum, mdot_tot_PZ, mdot_fuel) # [-] Emission indices computation

    results.PZ.phi = mixture_sum.equivalence_ratio()    # [-] Store Equivalence ratio
    results.PZ.T = mixture_sum.T                        # [K] Store Temperature
    results.PZ.EI.NOx = EI_mixer_initial['NOx']      # [kg/kg_fuel] Store NOx emission index
    results.PZ.EI.CO2 = EI_mixer_initial['CO2']      # [kg/kg_fuel] Store CO2 emission index
    results.PZ.EI.CO = EI_mixer_initial['CO']        # [kg/kg_fuel] Store CO emission index
    results.PZ.EI.H2O = EI_mixer_initial['H2O']      # [kg/kg_fuel] Store H2O emission index

    # ----------------------------------------------------------------------
    #  Secondary Zone
    # ----------------------------------------------------------------------

    combustor.L_SZ                      = combustor.length - combustor.L_PZ # [m] Secondary Zone length
    A_SZ                                = np.pi * (combustor.diameter / 2) ** 2 # [m^2] Cross-sectional area of Secondary Zone
    f_air_SA                            = mdot_fuel_TakeOff / (combustor.design_equivalence_ratio_SZ * combustor.fuel_data.stoichiometric_fuel_air_ratio * combustor.air_mass_flow_rate_take_off) # [-] Secondary air mass flow fraction
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

        EI_sm                           = calculate_emission_indices(mixed_gas_sm, mdot_total_sm, combustor.f_SM * mdot_fuel) # [-] Emission indices    

        results.SZ.sm.phi.append(mixed_gas_sm.equivalence_ratio()) # [-] Store equivalence ratio
        results.SZ.sm.T.append(mixed_gas_sm.T)                # [K] Store temperature 
        results.SZ.sm.z.append(z_frac_sm * 100)               # [%] Store position percentage 
        results.SZ.sm.EI.CO2.append(EI_sm['CO2'])          # [kg/kg_fuel] Store CO2 emission index 
        results.SZ.sm.EI.NOx.append(EI_sm['NOx'])          # [kg/kg_fuel] Store NOx emission index 
        results.SZ.sm.EI.CO.append(EI_sm['CO'])            # [kg/kg_fuel] Store CO emission index 
        results.SZ.sm.EI.H2O.append(EI_sm['H2O'])          # [kg/kg_fuel] Store H2O emission index

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

        EI_fm                           = calculate_emission_indices(mixed_gas_fm, mdot_total_fm, f_FM * mdot_fuel) # [-] Emission indices
        
        results.SZ.fm.phi.append(mixed_gas_fm.equivalence_ratio()) # [-] Store equivalence ratio
        results.SZ.fm.T.append(mixed_gas_fm.T)                # [K] Store temperature 
        results.SZ.fm.z.append(z_frac_fm * 100)               # [%] Store position percentage 
        results.SZ.fm.EI.CO2.append(EI_fm['CO2'])          # [kg/kg_fuel] Store CO2 emission index 
        results.SZ.fm.EI.NOx.append(EI_fm['NOx'])          # [kg/kg_fuel] Store NOx emission index 
        results.SZ.fm.EI.CO.append(EI_fm['CO'])            # [kg/kg_fuel] Store CO emission index 
        results.SZ.fm.EI.H2O.append(EI_fm['H2O'])          # [kg/kg_fuel] Store H2O emission index

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

        EI_joint                        = calculate_emission_indices(mixed_gas_joint, mdot_total_joint, mdot_fuel) # [-] Emission indices
        
        results.SZ.joint.phi.append(mixed_gas_joint.equivalence_ratio()) # [-] Store equivalence ratio
        results.SZ.joint.T.append(mixed_gas_joint.T)          # [K] Store temperature 
        results.SZ.joint.z.append(z_frac_joint * 100)         # [%] Store position percentage 
        results.SZ.joint.EI.CO2.append(EI_joint['CO2'])    # [kg/kg_fuel] Store CO2 emission index 
        results.SZ.joint.EI.NOx.append(EI_joint['NOx'])    # [kg/kg_fuel] Store NOx emission index 
        results.SZ.joint.EI.CO.append(EI_joint['CO'])      # [kg/kg_fuel] Store CO emission index 
        results.SZ.joint.EI.H2O.append(EI_joint['H2O'])    # [kg/kg_fuel] Store H2O emission index

    results.final.phi = mixed_gas_joint.equivalence_ratio() # [-] Store Equivalence ratio
    results.final.T = mixed_gas_joint.T                    # [K] Store Temperature
    results.final.EI.NOx = EI_joint['NOx']              # [kg/kg_fuel] Store NOx emission index
    results.final.EI.CO2 = EI_joint['CO2']              # [kg/kg_fuel] Store CO2 emission index
    results.final.EI.CO = EI_joint['CO']                # [kg/kg_fuel] Store CO emission index
    results.final.EI.H2O = EI_joint['H2O']              # [kg/kg_fuel] Store H2O emission index

    return results

def calculate_emission_indices(reactor,  mdot_total, mdot_fuel):
    """Calculate emission indices for combustion products"""
    gas                                 = reactor.thermo if hasattr(reactor, 'thermo') else reactor # [-] Extract gas object
    NOx_species                         = ['NO', 'NO2']              # [-] List of NOx species
    EI_NOx                              = 0.0                        # [kg/kg_fuel] Initialize NOx emission index
    for species in NOx_species:
        try:
            idx                         = gas.species_index(species) # [-] Species index
            EI_NOx                     += gas.Y[idx] * mdot_total / mdot_fuel # [kg/kg_fuel] Add contribution to NOx EI
        except ValueError:
            continue
    EI                                  = {
        'CO2': gas.Y[gas.species_index('CO2')] * mdot_total / mdot_fuel, # [kg/kg_fuel] CO2 emission index
        'CO': gas.Y[gas.species_index('CO')] * mdot_total / mdot_fuel,   # [kg/kg_fuel] CO emission index
        'H2O': gas.Y[gas.species_index('H2O')] * mdot_total / mdot_fuel, # [kg/kg_fuel] H2O emission index
        'NOx': EI_NOx,                                                   # [kg/kg_fuel] NOx emission index
    }
    return EI                                                            # [-] Return emission indices dictionary
