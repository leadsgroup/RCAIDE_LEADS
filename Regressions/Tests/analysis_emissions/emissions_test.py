# Regression/scripts/Tests/emissions_test.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Core                          import Units , Data 
from RCAIDE.Library.Plots                           import *        

# python imports     
import numpy as np  
import sys
import os
import matplotlib.pyplot as plt  


sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Boeing_737    import vehicle_setup as vehicle_setup
from Boeing_737    import configs_setup as configs_setup 

# ----------------------------------------------------------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------------------------------------------------------

def main():
    emissions_methods = ['Emission_Index_Correlation_Method', 'Emission_Index_CRN_Method']
    use_surrogate     = [True, False]
    
    cantera_installation = False 
    try: 
        import cantera as ct
        cantera_installation = True 
    except:
        pass 
    
    true_EI_CO2s =  [3.16, 3.102342112842955, 3.0314334973667685]
    true_EI_H2Os =  [1.23, 1.241535923661115, 1.2007950309221447]
    i =  0
    for em in  range(2):
        for sur in  range(2):
            if em == 0 and  sur == 0:
                pass
            else:
                # vehicle data
                vehicle  = vehicle_setup()
                
                # Set up vehicle configs
                configs  = configs_setup(vehicle)
            
                # create analyses
                analyses = analyses_setup(configs,emissions_methods[em], use_surrogate[sur])
            
                # mission analyses 
                mission = mission_setup(analyses)
                
                # create mission instances (for multiple types of missions)
                missions = missions_setup(mission) 
                 
                # mission analysis 
                results = missions.base_mission.evaluate()
                
                # check results
                EI_CO2         = results.segments.cruise.conditions.emissions.index.CO2[0,0]
                EI_H2O         = results.segments.cruise.conditions.emissions.index.H2O[0,0]  
                true_EI_CO2    = true_EI_CO2s[i]
                true_EI_H2O    = true_EI_H2Os[i]   
                diff_EI_CO2    = np.abs(EI_CO2 - true_EI_CO2)
                diff_EI_H2O    = np.abs(EI_H2O - true_EI_H2O)
                
                if cantera_installation == False and  i > 0:
                    pass
                else:
                    print('EI CO2 Error: ',diff_EI_CO2)
                    assert (diff_EI_CO2/true_EI_CO2) < 1e-1
                    print('EI H2O Error: ',diff_EI_H2O)
                    assert (diff_EI_H2O/true_EI_H2O) < 1e-1
                i += 1

    plot_PZ_results(results)
    plot_SZ_results(results)
             
    return 

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs,emissions_method, use_surrogate):
    
    analyses = RCAIDE.Framework.Analyses.Analysis.Container()
    
    # build a base analysis for each config
    for tag,config in list(configs.items()):
        analysis = base_analysis(config,emissions_method, use_surrogate)
        analyses[tag] = analysis
    
    return analyses

def base_analysis(vehicle,emissions_method, use_surrogate):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    
    # ------------------------------------------------------------------
    #  Weights
    weights         = RCAIDE.Framework.Analyses.Weights.Conventional()
    weights.vehicle = vehicle
    analyses.append(weights)
    
    # ------------------------------------------------------------------
    #  Aerodynamics Analysis 
    aerodynamics                                       = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.vehicle                               = vehicle
    aerodynamics.settings.number_of_spanwise_vortices  = 5
    aerodynamics.settings.number_of_chordwise_vortices = 2       
    aerodynamics.settings.model_fuselage               = True
    aerodynamics.settings.drag_coefficient_increment   = 0.0000
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    # Emissions
    if emissions_method == "Emission_Index_Correlation_Method":
        emissions = RCAIDE.Framework.Analyses.Emissions.Emission_Index_Correlation_Method()
    elif emissions_method == "Emission_Index_CRN_Method":
        emissions = RCAIDE.Framework.Analyses.Emissions.Emission_Index_CRN_Method() 
        emissions.settings.use_surrogate     = use_surrogate 
        emissions.training.pressure          = np.linspace(2.5, 2.5, 1) *1E6
        emissions.training.temperature       = np.linspace(750, 750, 1) 
        emissions.training.air_mass_flowrate = np.linspace(40, 40, 1) 
        emissions.training.fuel_to_air_ratio = np.linspace(0.025, 0.025, 1)             
   
    emissions.vehicle = vehicle          
    analyses.append(emissions)
        
    # ------------------------------------------------------------------
    #  Energy
    energy= RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle  = vehicle 
    analyses.append(energy)
    
    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)
    
    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   
    
    # done!
    return analyses    
 
# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
    
def mission_setup(analyses):
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'cruise'
     
    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
     
    
    # ------------------------------------------------------------------    
    #   Cruise Segment: constant speed 
    # ------------------------------------------------------------------    
    segment     = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                      = 36000. * Units.ft
    segment.altitude_start                                = 36000. * Units.ft
    segment.altitude_end                                  = 36000. * Units.ft
    segment.mach_number                                   = 0.78
    segment.distance                                      = 500 * Units.km  
    segment.state.numerics.number_of_control_points       = 2   
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)    
     
    
    return mission

def missions_setup(mission):

    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)
    
    # done!
    return missions 

def plot_PZ_results(results):
    """
    Plot combustor performance results.

    Parameters
    ----------
    phi_data : list
        Equivalence ratios for each PSR
    f_PSR_data : list
        Mass flow fractions for each PSR
    T_data : list
        Temperature data for each PSR
    EI_NOx_data : list
        NOx emission index data for each PSR
    EI_CO2_data : list
        CO2 emission index data for each PSR
    EI_CO_data : list
        CO emission index data for each PSR
    EI_H2O_data : list
        H2O emission index data for each PSR
    EI_soot_data : list
        Soot emission index data for each PSR
    """
    
    fig, ax1 = plt.subplots(figsize=(10, 6))
    x_eq = [0.39775725593667544, 0.5362796833773087, 0.6715039577836412, 0.8100263852242745, 0.9485488126649076, 1.0837730870712403, 1.2222955145118735, 1.3608179419525066, 1.4960422163588394, 1.6345646437994725, 1.7730870712401061, 1.9116094986807393, 2.0468337730870716, 2.185356200527705, 2.3238786279683383, 2.4591029023746707, 2.5976253298153043, 2.7361477572559374, 2.8713720316622697, 3.0098944591029033, 3.1484168865435365]
    y_eq = [0.01113783872272403, 0.016367987211832913, 0.023003413708761064, 0.031043781175972132, 0.040233783679679544, 0.050062640833541984, 0.060019066694913636, 0.06908133197234367, 0.07622770737438132, 0.0809469069583807, 0.08260041311172316, 0.08093275138185392, 0.07619956474009593, 0.06891146505402232, 0.05996278142634287, 0.049992368507212614, 0.04014952429559157, 0.03081762898907999, 0.022891011690387694, 0.01624159813570096, 0.010997294070065292]
    color = 'black'
    ax1.set_xlabel('Equivalence ratio in primary zone [-]')
    ax1.set_ylabel('Mass flow fraction in primary zone [-]', color=color)
    line1, = ax1.plot(results.segments.cruise.conditions.emissions.index.PZ_phi, results.segments.cruise.conditions.emissions.index.PZ_f_psr, '-', color=color, label='Mass Flow Fraction RCAIDE')
    marker1, = ax1.plot(x_eq, y_eq, 'o', color=color, label='Mass Flow Fraction Literature')
    ax1.tick_params(axis='y', labelcolor=color)
    ax2 = ax1.twinx()
    x_temp = [0.39928559454902235, 0.5356771321231005, 0.6753350162208, 0.8116217841894036, 0.951137921173814, 1.0871535207517773, 1.2229534182008222, 1.3620319880094265, 1.497794907950657, 1.6368858035951999, 1.7726795381262754, 1.911807411278633, 2.0509414473489596, 2.1867783223058184, 2.3226336860165855, 2.461804699594726, 2.5976662262234624, 2.7368557285555104, 2.8727295810201845, 3.008609596402828, 3.147811424570814]
    y_temp = [1663.5687732342003, 1934.9442379182153, 2176.579925650557, 2384.758364312267, 2540.8921933085494, 2585.5018587360587, 2499.9999999999995, 2392.1933085501855, 2284.386617100371, 2184.0148698884755, 2094.795539033457, 2016.7286245353157, 1942.3791821561335, 1879.1821561338286, 1827.137546468401, 1775.0929368029738, 1726.7657992565053, 1685.873605947955, 1644.9814126394049, 1607.806691449814, 1574.349442379182]
    color = 'orange'
    ax2.set_ylabel('Temperature [K]', color=color)
    line2, = ax2.plot(results.segments.cruise.conditions.emissions.index.PZ_phi, results.segments.cruise.conditions.emissions.index.PZ_T, '-', color=color, label='Temperature RCAIDE')
    marker2, = ax2.plot(x_temp, y_temp, 'o', color=color, label='Temperature Literature')
    ax2.tick_params(axis='y', labelcolor=color)
    lines = [line1, marker1, line2, marker2]
    ax1.legend(lines, [l.get_label() for l in lines], loc='upper right')
    fig.tight_layout()
    # plt.savefig('Equivalence_ratio_temperature_primary_zone_reactor_i.png', dpi=300)

    fig, ax1 = plt.subplots(figsize=(10, 6))
    x_co = [0.3972148541114058, 0.53315649867374, 0.6724137931034483, 0.8083554376657823, 0.9476127320954907, 1.080238726790451, 1.2194960212201593, 1.3587533156498677, 1.4946949602122017, 1.6339522546419096, 1.7698938992042441, 1.909151193633952, 2.0450928381962865, 2.181034482758621, 2.320291777188329, 2.456233421750663, 2.5954907161803713, 2.7314323607427053, 2.8706896551724137, 3.006631299734748, 3.145888594164456]
    y_co = [85, 27.500000000000227, 25.000000000000227, 52.5, 160, 460, 860.0000000000001, 1150, 1322.5, 1390, 1387.5, 1342.5, 1272.5, 1202.5, 1142.5, 1090, 1040, 995, 952.5, 912.5, 875]
    color = 'tab:red'
    ax1.set_xlabel('Equivalence ratio in primary zone reactor i [-]')
    ax1.set_ylabel(r'EI CO [g/kg$_{\text{fuel}}$]', color=color)
    line1, = ax1.plot(results.segments.cruise.conditions.emissions.index.PZ_phi, results.segments.cruise.conditions.emissions.index.PZ_EI_CO, '-', color=color, label='EI CO RCAIDE')
    marker1, = ax1.plot(x_co, y_co, 'o', color=color, label='EI CO Literature')
    ax1.tick_params(axis='y', labelcolor=color)
    ax2 = ax1.twinx()
    x_nox = [0.39592426855198065, 0.5322344173409326, 0.671522258549804, 0.8063555197328927, 0.9412306360303175, 1.0827547362051522, 1.2203265177262388, 1.360164454723529, 1.4965762516472976, 1.6361929544688107, 1.772556916976195, 1.9089208794835797, 2.0485316030030445, 2.1848955655104287, 2.321259528017813, 2.460870251537278, 2.5972342140446627, 2.7368449375641277, 2.873208900071512, 3.0095728625788962, 3.1491835860983612]
    y_nox = [0.11685947922671858, 1.270051589418074, 8.22419512615133, 41.21908401876067, 73.31157664628124, 32.05197687684312, 6.004367282215881, 1.098445620639609, 0.06010394418680676, -0.07600888763457192, -0.08304054684305129, -0.09007220605154487, -0.09727128571736898, -0.10430294492584835, -0.11133460413434193, -0.11853368380016605, -0.1255653430086454, -0.13276442267448374, -0.1397960818829631, -0.1468277410914567, -0.1540268207572808]
    color = 'tab:purple'
    ax2.set_ylabel(r'EI NO$_{x}$ [g/kg$_{\text{fuel}}$]', color=color)
    line2, = ax2.plot(results.segments.cruise.conditions.emissions.index.PZ_phi, results.segments.cruise.conditions.emissions.index.PZ_EI_NOx, '-', color=color, label='EI NOx RCAIDE')
    marker2, = ax2.plot(x_nox, y_nox, 'o', color=color, label='EI NOx Literature')
    ax2.tick_params(axis='y', labelcolor=color)
    lines = [line1, marker1, line2, marker2]
    ax1.legend(lines, [l.get_label() for l in lines], loc='upper right')
    fig.tight_layout()

    # plt.savefig('Emission_Indices_CO_NOx.png', dpi=300)
    
    fig, ax1 = plt.subplots(figsize=(10, 6))
    color = 'tab:blue'
    ax1.set_xlabel('Equivalence ratio in primary zone reactor i [-]')
    ax1.set_ylabel(r'EI CO$_{2}$ [g/kg$_{\text{fuel}}$]', color=color)
    line1, = ax1.plot(results.segments.cruise.conditions.emissions.index.PZ_phi, results.segments.cruise.conditions.emissions.index.PZ_EI_CO2, '-', color=color, label='EI CO2 RCAIDE')
    ax1.tick_params(axis='y', labelcolor=color)
    ax2 = ax1.twinx()
    color = 'tab:green'
    ax2.set_ylabel(r'EI H$_{2}$O [g/kg$_{\text{fuel}}$]', color=color)
    line2, = ax2.plot(results.segments.cruise.conditions.emissions.index.PZ_phi, results.segments.cruise.conditions.emissions.index.PZ_EI_H2O, '-', color=color, label='EI H2O RCAIDE')
    ax2.tick_params(axis='y', labelcolor=color)
    lines = [line1, line2]
    ax1.legend(lines, [l.get_label() for l in lines], loc='upper right')
    fig.tight_layout()
    # plt.savefig('Emission_Indices_CO2_H2O.png', dpi=300)

    plt.figure(figsize=(10, 6))
    line1, = plt.plot(results.segments.cruise.conditions.emissions.index.PZ_phi,  results.segments.cruise.conditions.emissions.index.PZ_EI_soot, '-', color=color, label='EI soot RCAIDE')
    plt.xlabel('Equivalence ratio in primary zone reactor i [-]')
    plt.ylabel(r'EI soot at exit of primary zone reactor i [g/kg$_{\text{fuel}}$]')
    plt.grid(True, linestyle='--', alpha=0.7)
    lines = [line1]
    plt.legend(lines, [l.get_label() for l in lines], loc='upper left')
    # plt.savefig('Emission_Indices_Soot.png', dpi=300)

def plot_SZ_results(results):
    """
    Plot emission indices (EI) trends over the secondary zone (SZ) length for the slow mode.
    
    Parameters:
    - z_percentages: List of positions as percentages downstream in the SZ [%]
    - EI_CO2_list: List of EI values for CO2 [g/kg_fuel]
    - EI_NOx_list: List of EI values for NOx [g/kg_fuel]
    - EI_CO_list: List of EI values for CO [g/kg_fuel]
    - EI_H2O_list: List of EI values for H2O [g/kg_fuel]
    - EI_soot_list: List of EI values for soot [g/kg_fuel]
    """
    
    # Plot CO2 EI
    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_sm_z, results.segments.cruise.conditions.emissions.index.SZ_sm_EI_CO2, 'b-', label='Slow Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_fm_z, results.segments.cruise.conditions.emissions.index.SZ_fm_EI_CO2, 'r-', label='Fast Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_joint_z, results.segments.cruise.conditions.emissions.index.SZ_joint_EI_CO2, 'g-', label='Joint Mixing')
    plt.xlim(0, 100)
    plt.xlabel('Percentage downstream SZ [%]')
    plt.ylabel(r'EI CO$_2$ [g/kg$_{\text{fuel}}$]')
    plt.legend()
    plt.grid(True)

    # Plot NOx EI
    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_sm_z, results.segments.cruise.conditions.emissions.index.SZ_sm_EI_NOx, 'b-', label='Slow Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_fm_z, results.segments.cruise.conditions.emissions.index.SZ_fm_EI_NOx, 'r-', label='Fast Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_joint_z, results.segments.cruise.conditions.emissions.index.SZ_joint_EI_NOx, 'g-', label='Joint Mixing')
    plt.xlim(0, 100)
    plt.xlabel('Percentage downstream SZ [%]')
    plt.ylabel(r'EI NO$_x$ [g/kg$_{\text{fuel}}$]')
    plt.legend()
    plt.grid(True)

    # Plot CO EI
    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_sm_z, results.segments.cruise.conditions.emissions.index.SZ_sm_EI_CO, 'b-', label='Slow Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_fm_z, results.segments.cruise.conditions.emissions.index.SZ_fm_EI_CO, 'r-', label='Fast Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_joint_z, results.segments.cruise.conditions.emissions.index.SZ_joint_EI_CO, 'g-', label='Joint Mixing')
    plt.xlim(0, 100)
    plt.xlabel('Percentage downstream SZ [%]')
    plt.ylabel(r'EI CO [g/kg$_{\text{fuel}}$]')
    plt.legend()
    plt.grid(True)

    # Plot H2O EI
    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_sm_z, results.segments.cruise.conditions.emissions.index.SZ_sm_EI_H2O, 'b-', label='Slow Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_fm_z, results.segments.cruise.conditions.emissions.index.SZ_fm_EI_H2O, 'r-', label='Fast Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_joint_z, results.segments.cruise.conditions.emissions.index.SZ_joint_EI_H2O, 'g-', label='Joint Mixing')
    plt.xlim(0, 100)
    plt.xlabel('Percentage downstream SZ [%]')
    plt.ylabel(r'EI H$_2$O [g/kg$_{\text{fuel}}$]')
    plt.legend()
    plt.grid(True)

    # Plot Soot EI
    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_sm_z, results.segments.cruise.conditions.emissions.index.SZ_sm_EI_soot, 'b-', label='Slow Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_fm_z, results.segments.cruise.conditions.emissions.index.SZ_fm_EI_soot, 'r-', label='Fast Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_joint_z, results.segments.cruise.conditions.emissions.index.SZ_joint_EI_soot, 'g-', label='Joint Mixing')
    plt.xlim(0, 100)
    plt.xlabel('Percentage downstream SZ [%]')
    plt.ylabel(r'EI soot [g/kg$_{\text{fuel}}$]')
    plt.legend()
    plt.grid(True)

    # Plot temperature
    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_sm_z, results.segments.cruise.conditions.emissions.index.SZ_sm_T, 'b-', label='Slow Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_fm_z, results.segments.cruise.conditions.emissions.index.SZ_fm_T, 'r-', label='Fast Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_joint_z, results.segments.cruise.conditions.emissions.index.SZ_joint_T, 'g-', label='Joint Mixing')
    plt.xlim(0, 100)
    plt.xlabel('Percentage downstream SZ [%]')  
    plt.ylabel('Temperature [K]')
    plt.legend()
    plt.grid(True)

    # Plot equivalence ratio
    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_sm_z, results.segments.cruise.conditions.emissions.index.SZ_sm_phi, 'b-', label='Slow Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_fm_z, results.segments.cruise.conditions.emissions.index.SZ_fm_phi, 'r-', label='Fast Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_joint_z, results.segments.cruise.conditions.emissions.index.SZ_joint_phi, 'g-', label='Joint Mixing')
    plt.xlim(0, 100)
    plt.xlabel('Percentage downstream SZ [%]')  
    plt.ylabel('Equivalence Ratio [-]')
    plt.legend()
    plt.grid(True)
     

if __name__ == '__main__': 
    main()    
    plt.show()
        
