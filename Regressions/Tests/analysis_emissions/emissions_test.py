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
    weights.aircraft_type  =  "Transport"
    weights.vehicle = vehicle
    analyses.append(weights)
    
    # ------------------------------------------------------------------
    #  Aerodynamics Analysis 
    aerodynamics                                       = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.vehicle                               = vehicle
    aerodynamics.settings.number_of_spanwise_vortices  = 5
    aerodynamics.settings.number_of_chordwise_vortices = 2       
    aerodynamics.settings.model_fuselage               = True 
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
        
    fig, ax1 = plt.subplots(figsize=(10, 6))
    color = 'black'
    ax1.set_xlabel('Equivalence ratio in primary zone [-]')
    ax1.set_ylabel('Mass flow fraction in primary zone [-]', color=color)
    ax1.plot(results.segments.cruise.conditions.emissions.index.PZ_phi[0], results.segments.cruise.conditions.emissions.index.PZ_f_psr[0], '-', color=color, label='Mass Flow Fraction RCAIDE')
    ax1.tick_params(axis='y', labelcolor=color)
    ax2 = ax1.twinx()
    color = 'orange'
    ax2.set_ylabel('Temperature [K]', color=color)
    ax2.plot(results.segments.cruise.conditions.emissions.index.PZ_phi[0], results.segments.cruise.conditions.emissions.index.PZ_T[0], '-', color=color, label='Temperature RCAIDE')
    ax2.tick_params(axis='y', labelcolor=color)
    fig.tight_layout()

    fig, ax1 = plt.subplots(figsize=(10, 6))
    color = 'tab:red'
    ax1.set_xlabel('Equivalence ratio in primary zone reactor i [-]')
    ax1.set_ylabel(r'EI CO [kg/kg$_{\text{fuel}}$]', color=color)
    ax1.plot(results.segments.cruise.conditions.emissions.index.PZ_phi[0], results.segments.cruise.conditions.emissions.index.PZ_EI_CO[0], '-', color=color, label='EI CO RCAIDE')
    ax1.tick_params(axis='y', labelcolor=color)
    ax2 = ax1.twinx()
    color = 'tab:purple'
    ax2.set_ylabel(r'EI NO$_{x}$ [kg/kg$_{\text{fuel}}$]', color=color)
    ax2.plot(results.segments.cruise.conditions.emissions.index.PZ_phi[0], results.segments.cruise.conditions.emissions.index.PZ_EI_NOx[0], '-', color=color, label='EI NOx RCAIDE')
    ax2.tick_params(axis='y', labelcolor=color)
    fig.tight_layout()
    
    fig, ax1 = plt.subplots(figsize=(10, 6))
    color = 'tab:blue'
    ax1.set_xlabel('Equivalence ratio in primary zone reactor i [-]')
    ax1.set_ylabel(r'EI CO$_{2}$ [kg/kg$_{\text{fuel}}$]', color=color)
    ax1.plot(results.segments.cruise.conditions.emissions.index.PZ_phi[0], results.segments.cruise.conditions.emissions.index.PZ_EI_CO2[0], '-', color=color, label='EI CO2 RCAIDE')
    ax1.tick_params(axis='y', labelcolor=color)
    ax2 = ax1.twinx()
    color = 'tab:green'
    ax2.set_ylabel(r'EI H$_{2}$O [kg/kg$_{\text{fuel}}$]', color=color)
    ax2.plot(results.segments.cruise.conditions.emissions.index.PZ_phi[0], results.segments.cruise.conditions.emissions.index.PZ_EI_H2O[0], '-', color=color, label='EI H2O RCAIDE')
    ax2.tick_params(axis='y', labelcolor=color)
    fig.tight_layout()

    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.PZ_phi[0],  results.segments.cruise.conditions.emissions.index.PZ_EI_soot[0], '-', color=color, label='EI soot RCAIDE')
    plt.xlabel('Equivalence ratio in primary zone reactor i [-]')
    plt.ylabel(r'EI soot at exit of primary zone reactor i [kg/kg$_{\text{fuel}}$]')
    plt.grid(True, linestyle='--', alpha=0.7)

def plot_SZ_results(results):

    
    # Plot CO2 EI
    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_sm_z[0], results.segments.cruise.conditions.emissions.index.SZ_sm_EI_CO2[0], 'b-', label='Slow Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_fm_z[0], results.segments.cruise.conditions.emissions.index.SZ_fm_EI_CO2[0], 'r-', label='Fast Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_joint_z[0], results.segments.cruise.conditions.emissions.index.SZ_joint_EI_CO2[0], 'g-', label='Joint Mixing')
    plt.xlim(0, 100)
    plt.xlabel('Percentage downstream SZ [%]')
    plt.ylabel(r'EI CO$_2$ [kg/kg$_{\text{fuel}}$]')
    plt.legend()
    plt.grid(True)

    # Plot NOx EI
    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_sm_z[0], results.segments.cruise.conditions.emissions.index.SZ_sm_EI_NOx[0], 'b-', label='Slow Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_fm_z[0], results.segments.cruise.conditions.emissions.index.SZ_fm_EI_NOx[0], 'r-', label='Fast Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_joint_z[0], results.segments.cruise.conditions.emissions.index.SZ_joint_EI_NOx[0], 'g-', label='Joint Mixing')
    plt.xlim(0, 100)
    plt.xlabel('Percentage downstream SZ [%]')
    plt.ylabel(r'EI NO$_x$ [kg/kg$_{\text{fuel}}$]')
    plt.legend()
    plt.grid(True)

    # Plot CO EI
    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_sm_z[0], results.segments.cruise.conditions.emissions.index.SZ_sm_EI_CO[0], 'b-', label='Slow Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_fm_z[0], results.segments.cruise.conditions.emissions.index.SZ_fm_EI_CO[0], 'r-', label='Fast Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_joint_z[0], results.segments.cruise.conditions.emissions.index.SZ_joint_EI_CO[0], 'g-', label='Joint Mixing')
    plt.xlim(0, 100)
    plt.xlabel('Percentage downstream SZ [%]')
    plt.ylabel(r'EI CO [kg/kg$_{\text{fuel}}$]')
    plt.legend()
    plt.grid(True)

    # Plot H2O EI
    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_sm_z[0], results.segments.cruise.conditions.emissions.index.SZ_sm_EI_H2O[0], 'b-', label='Slow Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_fm_z[0], results.segments.cruise.conditions.emissions.index.SZ_fm_EI_H2O[0], 'r-', label='Fast Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_joint_z[0], results.segments.cruise.conditions.emissions.index.SZ_joint_EI_H2O[0], 'g-', label='Joint Mixing')
    plt.xlim(0, 100)
    plt.xlabel('Percentage downstream SZ [%]')
    plt.ylabel(r'EI H$_2$O [kg/kg$_{\text{fuel}}$]')
    plt.legend()
    plt.grid(True)

    # Plot Soot EI
    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_sm_z[0], results.segments.cruise.conditions.emissions.index.SZ_sm_EI_soot[0], 'b-', label='Slow Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_fm_z[0], results.segments.cruise.conditions.emissions.index.SZ_fm_EI_soot[0], 'r-', label='Fast Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_joint_z[0], results.segments.cruise.conditions.emissions.index.SZ_joint_EI_soot[0], 'g-', label='Joint Mixing')
    plt.xlim(0, 100)
    plt.xlabel('Percentage downstream SZ [%]')
    plt.ylabel(r'EI soot [kg/kg$_{\text{fuel}}$]')
    plt.legend()
    plt.grid(True)

    # Plot temperature
    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_sm_z[0], results.segments.cruise.conditions.emissions.index.SZ_sm_T[0], 'b-', label='Slow Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_fm_z[0], results.segments.cruise.conditions.emissions.index.SZ_fm_T[0], 'r-', label='Fast Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_joint_z[0], results.segments.cruise.conditions.emissions.index.SZ_joint_T[0], 'g-', label='Joint Mixing')
    plt.xlim(0, 100)
    plt.xlabel('Percentage downstream SZ [%]')  
    plt.ylabel('Temperature [K]')
    plt.legend()
    plt.grid(True)

    # Plot equivalence ratio
    plt.figure(figsize=(10, 6))
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_sm_z[0], results.segments.cruise.conditions.emissions.index.SZ_sm_phi[0], 'b-', label='Slow Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_fm_z[0], results.segments.cruise.conditions.emissions.index.SZ_fm_phi[0], 'r-', label='Fast Mode')
    plt.plot(results.segments.cruise.conditions.emissions.index.SZ_joint_z[0], results.segments.cruise.conditions.emissions.index.SZ_joint_phi[0], 'g-', label='Joint Mixing')
    plt.xlim(0, 100)
    plt.xlabel('Percentage downstream SZ [%]')  
    plt.ylabel('Equivalence Ratio [-]')
    plt.legend()
    plt.grid(True)
     
if __name__ == '__main__': 
    main()    
    plt.show()
        
