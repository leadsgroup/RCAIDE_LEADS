# RCAIDE/Library/Methods/Emissions/Chemical_Reactor_Network_Method/evaluate_cantera.py

# Created: June 2024, M. Clarke, M. Guidotti 
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from   RCAIDE.Framework.Core import Data  
import pandas                as pd
import numpy                 as np
import os

# ----------------------------------------------------------------------------------------------------------------------
#  evaluate_cantera
# ----------------------------------------------------------------------------------------------------------------------   
def evaluate_cantera(combustor,T,P,mdot,FAR): 

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
    
    high_fidelity_kin_mech  = False                                                 # [-]       True (simulation time around 300 s): Computes EI_CO2, EI_CO, EI_H2O, EI_NO2, EI_NO, EI_CSOOT; False (simulation time around 60 s): Computes EI_CO2, EI_CO, EI_H2O
    T_stag_0                = T                                                     # [K]       Stagnation Temperature entering all combustors
    P_stag_0                = P                                                     # [Pa]      Stagnation Pressure entering all combustors
    FAR                     = FAR                                                   # [-]       Fuel-to-Air ratio
    m_dot_air_tot           = mdot                                                  # [kg/s]    Air mass flow going through all combustors
    
    f_air_PZ                = combustor.f_air_PZ                                    # [-]       Fraction of total air present in the combustor that enters the Primary Zone         
    FAR_st                  = combustor.FAR_st                                      # [-]       Stoichiometric Fuel to Air ratio 
    N_comb                  = combustor.N_comb                                      # [-]       Number of can-annular combustors
    N_PZ                    = combustor.N_PZ                                        # [-]       Number of PSR (EVEN, must match the number of PSR below)
    A_PZ                    = combustor.A_PZ                                        # [m**2]    Primary Zone cross-sectional area     
    L_PZ                    = combustor.L_PZ                                        # [m]       Primary Zone length  
    N_SZ                    = combustor.N_SZ                                        # [-]       Number of dilution air inlets        
    A_SZ                    = combustor.A_SZ                                        # [m**2]    Secondary Zone cross-sectional area
    L_SZ                    = combustor.L_SZ                                        # [m]       Secondary Zone length  
    phi_SZ                  = combustor.phi_SZ                                      # [-]       Equivalence Ratio for PFR    phi_PZ_des              = 0.6                                                   # [-]       Primary Zone Design Equivalence Ratio
    S_PZ                    = combustor.S_PZ                                        # [-]       Mixing parameter, used to define the Equivalence Ratio standard deviation  
    F_SC                    = combustor.F_SC                                        # [-]       Fuel scaler
    PSR_1st_mixers          = combustor.number_of_assigned_PSR_1st_mixers           # [-]       Number of PSR assigned to the first row of mixers  
    PSR_2nd_mixers          = combustor.number_of_assigned_PSR_2nd_mixers           # [-]       Number of mixers assigned to the second row of mixers                                           
         
    m_dot_fuel_tot          = m_dot_air_tot*FAR                                     # [kg/s]    Fuel mass flow going through all combustors
    m_dot_air               = m_dot_air_tot/N_comb                                  # [kg/s]    Air mass flow inside each combustor, scaled inside each PSR to vary the Equivalence Ratio
    m_dot_fuel              = m_dot_fuel_tot/N_comb                                 # [kg/s]    Fuel mass flow inside each combustor                                                                                                                                                                    
    phi_sign                = (m_dot_fuel_tot*F_SC)/(m_dot_air_tot*f_air_PZ*FAR_st) # [-]       Primary Zone mean Equivalence Ratio
    sigma_phi               = phi_sign*S_PZ                                         # [-]       Primary Zone Equivalence Ratio standard deviation                                                                                                                                                      
    
    if high_fidelity_kin_mech:                                                               
        dict_fuel           = combustor.fuel_data.fuel_chemical_properties          # [-]       Fuel species and corresponding mole fractions for full fuel model
        kinetics_model      = combustor.fuel_data.surrogate_chemical_kinetics       # [-]       Full fuel model 
    else:                                                                                      
        dict_fuel           = combustor.fuel_data.fuel_surrogate_chemical_properties# [-]       Fuel species and corresponding mole fractions for surrogate fuel model 
        kinetics_model      = combustor.fuel_data.surrogate_chemical_kinetics       # [-]       Surrogate fuel model 
    dict_oxy                = combustor.fuel_data.air_chemical_properties           # [-]       Air species and corresponding mole fractions     
    oxidizer_model          = combustor.fuel_data.oxidizer
    
    if high_fidelity_kin_mech:                                                                           
        list_sp             = combustor.fuel_data.species_list                      # [-]       Fuel species for Emission Index analysis on the full model
    else:                                                                                       
        list_sp             = combustor.fuel_data.surrogate_species_list            # [-]       Fuel species for Emission Index analysis on the surrogate model
    
    col_names               = ['EI_' +str(sp) for sp in list_sp]                    # [-]       Define output variables 
    df                      = pd.DataFrame(columns=col_names)                       # [-]       Assign output variables space to df
    
    
    try: 
        import cantera as ct 
        gas, EI = combustor_model(ct,kinetics_model,oxidizer_model,high_fidelity_kin_mech, dict_fuel, dict_oxy, T_stag_0, P_stag_0, FAR, m_dot_fuel, m_dot_air, N_PZ, A_PZ, L_PZ, phi_sign, phi_SZ, A_SZ, L_SZ, f_air_PZ, N_SZ, sigma_phi, PSR_1st_mixers, PSR_2nd_mixers) # [-]       Run combustor function
        
        sp_idx                  = [gas.species_index(sp) for sp in list_sp]             # [-]       Retrieve the species index
        data_n                  = list(EI[sp_idx])                                      # [-]       Assign output variables  
        df.loc[0]               = data_n                                                # [-]       Assign output variables to df 
        
        results                 = Data()                                                # [-]       Create results data structure
        results.EI_CO2          = df.loc[0, 'EI_CO2']                                   # [-]       Assign CO2 Emission Index to the results data structure
        results.EI_CO           = df.loc[0, 'EI_CO']                                    # [-]       Assign CO Emission Index to the results data structure
        results.EI_H2O          = df.loc[0, 'EI_H2O']                                   # [-]       Assign H2O Emission Index to the results data structure
        results.EI_NO           = 0                                                     # [-]       Assign NO Emission Index to the results data structure
        results.EI_NO2          = 0                                                     # [-]       Assign NO2 Emission Index to the results data structure   
        
        if high_fidelity_kin_mech:   
            results.EI_NO       = df.loc[0, 'EI_NO']                                    # [-]       Assign NO Emission Index to the results data structure
            results.EI_NO2      = df.loc[0, 'EI_NO2']                                   # [-]       Assign NO2 Emission Index to the results data structure
 
    except ImportError:
        print('cantera required: run pip install cantera') 
        results                 = Data()                                               
        results.EI_CO2          = 0                               
        results.EI_CO           = 0                               
        results.EI_H2O          = 0                               
        results.EI_NO           = 0                                                    
        results.EI_NO2          = 0                                                    
        
        if high_fidelity_kin_mech:   
            results.EI_NO       = 0                                 
            results.EI_NO2      = 0                                 
     

    return results

def combustor_model(ct, kinetics_model,oxidizer_model,high_fidelity_kin_mech, dict_fuel, dict_oxy, T_stag_0, P_stag_0, FAR, m_dot_fuel, m_dot_air, N_PZ, A_PZ, L_PZ, phi_sign, phi_SZ, A_SZ, L_SZ, f_air_PZ, N_SZ, sigma_phi, PSR_1st_mixers, PSR_2nd_mixers):

    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator + "Data" + separator
    

    # ------------------------------------------------------------------------------
    # ----------------------------- Initial Parameters -----------------------------
    # ------------------------------------------------------------------------------    
      
    f_air_SZ                = 1 - f_air_PZ                                          # [-]       Fraction of total air present in the combustor that enters the Secondary Zone  
    m_dot_air_PSR           = f_air_PZ*m_dot_air                                    # [kg/s]    Air mass flow going through each PSR  
    m_dot_fuel_PSR          = m_dot_fuel                                            # [kg/s]    Fuel mass flow going through each PSR    
    m_dot_air_SZ            = (f_air_SZ*m_dot_air)/N_SZ                             # [kg/s]    Air mass flow going through each dilution air inlet (3 inlets)
    V_PZ_PSR                = (A_PZ*L_PZ)/N_PZ                                      # [m**3]    Volume of each PSR
    phi_PSR                 = np.linspace(0.001, 2*phi_sign, N_PZ)                  # [-]       Distribution of Equivalence Ratio through the PSRs
    Delta_phi               = np.abs(phi_PSR[0] - phi_PSR[1])                       # [-]       Difference between two subsequent Equivalence Ratios
   
    # ------------------------------------------------------------------------------
    # ----------------------------------- PSR #1 -----------------------------------
    # ------------------------------------------------------------------------------ 

    Fuel_list               = []                                                    # [-]       Create a list for the fuel inside each PSR
    mass_flow_rates         = []                                                    # [-]       Create a list for the mass flow rate inside each PSR
    mixer_list              = []                                                    # [-]       Create a list for the mixers inside each PSR
    
    m_i = 0                                                                         # [-]       Initialize the index of the mixer
       
    for PSR_i in range(N_PZ):     
        f_PZ_1                            = (1 / (np.sqrt(2 * np.pi) * sigma_phi)) * np.exp((-(phi_PSR[PSR_i] - phi_sign) ** 2) / (2 * sigma_phi ** 2)) * Delta_phi # [-]       Fraction of mass flow entering the PSR at the PSR equivalence ratio                                                                                       
        Fuel_list.append(ct.Solution(rel_path+kinetics_model) )                     # [-]       Import surrogate fuel kinematic mechanism
        Fuel_list[PSR_i].TP               = T_stag_0, P_stag_0                      # [-]       Set the fuel temperature and pressure
        Fuel_list[PSR_i].set_equivalence_ratio(phi_PSR[PSR_i], fuel=dict_fuel, oxidizer=dict_oxy) # [-]       Set the euivalence ratio inside the PSR
        Fuel_list[PSR_i].equilibrate('HP')                                          # [-]       Fix the specific enthalpy and pressure 
        rho                               = Fuel_list[PSR_i].density                # [kg/m**3] Fuel density
        m_dot_air_PSRl                         = m_dot_air_PSR * f_PZ_1             # [kg/s]    Air mass flow inside the PSR    
        m_dot_fuel_PSRl                        = m_dot_fuel_PSR * f_PZ_1            # [kg/s]    Fuel mass flow inside the PSR
        mass_flow_rates.append(m_dot_fuel_PSRl + m_dot_air_PSRl)                    # [kg/s]    Total mass flow inside the PSR  
        upstream                          = ct.Reservoir(Fuel_list[PSR_i])          # [-]       Create a resevoir for the Fuel upstream of the PSR
        if PSR_i % PSR_1st_mixers == 0:
            mixer_list.append(ct.IdealGasReactor(Fuel_list[PSR_i]))                 # [-]       Create the reactor for the downstream mixer
        
        PSR                               = ct.IdealGasReactor(Fuel_list[PSR_i])    # [-]       Create the reactor for the PSR 
        PSR.volume                        = V_PZ_PSR                                # [m**3]    Set the PSR volume
        inlet                             = ct.MassFlowController(upstream, PSR)    # [-]       Connect the upstream resevoir with the PSR 
        inlet.mass_flow_rate              = mass_flow_rates[PSR_i]                  # [kg/s]    Prescribe the inlet mass flow rate 
        outlet                            = ct.MassFlowController(PSR, mixer_list[m_i], mdot=mass_flow_rates[PSR_i]) # [-]       Connect the PSR with the downstream mixer with the same mass flow rate                                     
        t_res_PSR                         = (rho * V_PZ_PSR) / (mass_flow_rates[PSR_i]) # [s]       Compute the residence time inside the PSR
        sim_PSR                           = ct.ReactorNet([PSR])                    # [-]       Set the PSR simulation
        sim_PSR.advance(t_res_PSR)                                                  # [-]       Run the simulation until the residence time is reached 
           
        if PSR_i % PSR_1st_mixers != 0:                                             # [-]       If the number of PSR / number of PSRs assigned to the mixer is different from 0
            m_i += 1                                                                # [-]       Increase the index of the mixer
        
    mass_flow_rates =  np.array(mass_flow_rates)                                    # [-]       Create array of mass flow rates
    mixer_list_2    =  []                                                           # [-]       Initialize the list of mixers in the second talk
    idx   =  0                                                                      # [-]       Initialize the first index
    idx_2 =  0                                                                      # [-]       Initialize the second index
    for mixer_i in range(0, N_PZ, int(N_PZ/PSR_2nd_mixers)): 
        mixer_list_2.append(ct.IdealGasReactor(Fuel_list[mixer_i]))                 # [-]       Create the reactor for the downstream mixer        
        for n in range(int(N_PZ/PSR_1st_mixers/PSR_2nd_mixers)):
            start_index =  mixer_i +  n * 2                                         # [-]       Define the starting index
            end_index   =  mixer_i + ( n + 1 ) * 2                                  # [-]       Define the ending index
            outlet      =  ct.MassFlowController(mixer_list[idx_2], mixer_list_2[idx], mdot = np.sum(mass_flow_rates[start_index:end_index])) # [-]       Connect the mixer with the downstream mixer     
            sim_mixer   =  ct.ReactorNet([mixer_list[idx_2]])                       # [-]       Set the mixer simulation  
            sim_mixer.advance_to_steady_state()                                     # [-]       Run the simulation until it reaches steady state
            idx_2 += 1                                                              # [-]       Advance second index of one
        idx +=  1                                                                   # [-]       Advance first index of one
     
    # ------------------------------------------------------------------------------
    # ------------------------------- Mixing 1-2-3-4 -------------------------------
    # ------------------------------------------------------------------------------
    
    mixer_12345678  = ct.IdealGasReactor(Fuel_list[0])                              # [-]       Create the reactor for the downstream mixer
    for idx_3 in  range(2):
        start_index = idx_3 * 4                                                     # [-]       Define the starting index
        end_index = (idx_3 + 1) * 4                                                 # [-]       Define the ending index   
        outlet_1234             = ct.MassFlowController(mixer_list_2[idx_3], mixer_12345678, mdot = np.sum(mass_flow_rates[ start_index:end_index]) ) # [-]       Connect the mixer with the downstream mixer     
        sim_mixer_1234          = ct.ReactorNet([mixer_list_2[idx_3]])              # [-]       Set the mixer simulation  
        sim_mixer_1234.advance_to_steady_state()                                    # [-]       Run the simulation until it reaches steady state
         
    # ------------------------------------------------------------------------------
    # --------------------------- Mixing 1-2-3-4-5-6-7-8 ---------------------------
    # ------------------------------------------------------------------------------     
  
    mixer_air         = ct.IdealGasReactor(Fuel_list[0])                            # [-]       Create the reactor for the downstream mixer
    outlet_12345678   = ct.MassFlowController(mixer_12345678, mixer_air, mdot = (  np.sum(np.array(mass_flow_rates)))) # [-]       Connect the mixer with the downstream mixer     
    sim_mixer_12345678      = ct.ReactorNet([mixer_12345678])                       # [-]       Set the mixer simulation  
    sim_mixer_12345678.advance_to_steady_state()                                    # [-]       Run the simulation until it reaches steady state

    # ------------------------------------------------------------------------------
    # -------------------------------- Mixing 1-Air --------------------------------
    # ------------------------------------------------------------------------------
    
    total_mass_flow = np.sum(np.array(mass_flow_rates))                             # [kg/s]    Total mass flow entering the first mixer  

    for _ in  range(N_SZ): 
        Air                   = ct.Solution(rel_path+oxidizer_model)                # [-]       Import air kinematic mechanism           
        Air.TPX               = T_stag_0, P_stag_0, dict_oxy                        # [-]       Set the air temperature, pressure and mole fractions
        rho_air               = Air.density                                         # [kg/m**3] Fuel density
        res_air               = ct.Reservoir(Air)                                   # [-]       Create a resevoir for the air upstream of the mixer
        PFR                   = ct.IdealGasConstPressureReactor(Fuel_list[0])       # [-]       Create the reactor for the PFR
        PFR.volume            = A_SZ*(L_SZ/N_SZ)                                    # [m**3]    Set the PFR volume
        inlet_air             = ct.MassFlowController(res_air, mixer_air, mdot= m_dot_air_SZ) # [-]       Connect the upstream resevoir with the mixer
        total_mass_flow       = total_mass_flow + m_dot_air_SZ                      # [kg/s]    Total mass flow inside the mixer  
        outlet_air            = ct.MassFlowController(mixer_air, PFR, mdot=total_mass_flow ) # [-]       Connect the mixer with the downstream PFR with the same mass flow rate           
        sim_mixer_air         = ct.ReactorNet([mixer_air])                          # [-]       Set the mixer simulation  
        sim_mixer_air.advance_to_steady_state()                                     # [-]       Run the simulation until it reaches steady state
        
        # --------------------------------------------------------------------------
        # ------------------------------------ PFR ---------------------------------
        # -------------------------------------------------------------------------- 
        
        mixer_air             = ct.IdealGasReactor(Fuel_list[0])                    # [-]       Create the reactor for the downstream mixer
        outlet_PFR            = ct.MassFlowController(PFR,mixer_air , mdot=total_mass_flow ) # [-]       Connect the PFR with the downstream mixer                                 
        t_res_PFR             = (Fuel_list[0].density * PFR.volume) / (total_mass_flow) # [s]       Compute the residence time inside the PFR           
        Fuel_list[0].set_equivalence_ratio(phi_SZ, fuel=dict_fuel, oxidizer=dict_oxy) # [-]       Set the euivalence ratio inside the PFR
        sim_PFR               = ct.ReactorNet([PFR])                                # [-]       Set the PFR simulation
        sim_PFR.advance(t_res_PFR)                                                  # [-]       Run the simulation until the residence time is reached 
   
    # ------------------------------------------------------------------------------
    # ---------------------------- Additional computations -------------------------
    # ------------------------------------------------------------------------------    

    m_dot_input_combustor   = m_dot_fuel + m_dot_air                                # [kg/s]    Total mass flow rate entering a single combustor (air + fuel)
    Emission_Index          = Fuel_list[0].Y * (m_dot_input_combustor)/m_dot_fuel   # [-]       Computation of the Emission Index   

    return (Fuel_list[0], Emission_Index)  