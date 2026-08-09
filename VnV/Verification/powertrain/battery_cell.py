# VnV/Verification/powertrain/battery_cell.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE  
from RCAIDE.Framework.Core                                            import Units, Data 
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common       import size_module_from_energy_and_power, find_mass_gain_rate, find_total_mass_gain, find_ragone_properties
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Aluminum_Air import * 
from RCAIDE.Framework.Mission.Common                                  import Conditions
from RCAIDE.Library.Plots                                             import *
from RCAIDE.Input_Output                                              import load as load_results
from RCAIDE.Input_Output                                              import save as save_results

# package imports  
import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.cm as cm

# local imports 
import sys 
import os
base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Battery_Cell   import vehicle_setup , configs_setup  
import time

# ----------------------------------------------------------------------------------------------------------------------
#  REGRESSION
# ----------------------------------------------------------------------------------------------------------------------  

def main():
    ti = time.time()
    Ereq                           = 3000*Units.Wh
    Preq                           = 2000.
    # make true only when regenerating truth values, should be left false for regression
    update_regression_values       = False

    # Lithium Air Battery Test
    lithium_air_battery_test(Ereq,Preq)

    # Aluminum Air Battery Test
    aluminum_air_battery_test(Ereq,Preq)

    # Lithium Sulfur Test
    lithium_sulphur_battery_test(Ereq,Preq)

    # Lithium-Ion Test
    lithium_ion_battery_test(update_regression_values)

    elapsed_time = time.time() - ti
    elapsed_time_min = elapsed_time / 60
    print('Elapsed time (min): ', elapsed_time_min)
    return 
    
     
def lithium_air_battery_test(Ereq,Preq): 
    battery_li_air                 = RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Lithium_Air()
    return 
   
        
def aluminum_air_battery_test(Ereq,Preq): 
    battery_al_air                 = RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Aluminum_Air()
    test_size_module_from_energy_and_power(battery_al_air, Ereq, Preq)
    test_mass_gain(battery_al_air, Preq)
    
    aluminum_mass  =  find_aluminum_mass(battery_al_air, Ereq)
    water_mass     =  find_water_mass(battery_al_air, Ereq)

    return 
   
def lithium_sulphur_battery_test(Ereq,Preq):   
    battery_li_s                   = RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Lithium_Sulfur()
    specific_energy_guess          = 400*Units.Wh/Units.kg 
    test_find_ragone_properties(specific_energy_guess,battery_li_s, Ereq,Preq) 
    plot_battery_ragone_diagram(battery_li_s,   save_filename =  'lithium_sulfur')     
    return 


def lithium_ion_battery_test(update_regression_values=False):

    # Operating conditions for battery p
    curr                  = [1.5,3]
    C_rat                 = [0.5,1]
    marker_size           = 5
    mAh                   = np.array([3800,2600])
    # PLot parameters
    marker                = ['s' ,'o' ,'P']
    linestyles            = ['-','--',':']
    linecolors            = cm.inferno(np.linspace(0.2,0.8,3))     
    plt.rcParams.update({'font.size': 12})
    fig1 = plt.figure('Cell Test') 
    fig1.set_size_inches(12,7)   
    axes1  = fig1.add_subplot(3,2,1)
    axes2  = fig1.add_subplot(3,2,2)  
    axes3  = fig1.add_subplot(3,2,3) 
    axes4  = fig1.add_subplot(3,2,4) 
    axes5  = fig1.add_subplot(3,2,5) 
    axes6  = fig1.add_subplot(3,2,6)  

    battery_chemistry     = ['lithium_ion_nmc','lithium_ion_lfp']    
    electrical_config     = ['Series','Parallel'] 
    for j in range(len(curr)):      
        for i in range(len(battery_chemistry)):
            
            vehicle  = vehicle_setup(curr[j],C_rat[j],battery_chemistry[i],electrical_config[j]) 
            
            # Set up vehicle configs
            configs  = configs_setup(vehicle)
        
            # create analyses
            analyses = analyses_setup(configs)
        
            # mission analyses
            mission  = mission_setup(analyses,vehicle,battery_chemistry[i],curr[j],mAh[i]) 
            
            # create mission instances (for multiple types of missions)
            missions = missions_setup(mission) 
             
            # mission analysis 
            results = missions.base_mission.evaluate()  
            
            # Voltage/Temperature Regression -- checked over the full time history of
            # every segment (Recharge, Discharge_1, Discharge_2), not just a single
            # sample point, so a bug affecting only part of a segment's evolution
            # (e.g. a discharge segment that starts correctly but stalls partway
            # through) can't slip past the check.
            regression_data = Data()
            for tag in ['recharge', 'discharge_1', 'discharge_2']:
                segment = results.segments[tag]
                regression_data[tag] = Data(
                    voltage_under_load = segment.conditions.energy.sources['battery_pack'].voltage_under_load[:,0],
                    cell_temperature   = segment.conditions.energy.sources['battery_pack'][battery_chemistry[i]].cell.temperature[:,0],
                )

            res_path = os.path.join(base_dir, f'battery_cell_{battery_chemistry[i]}_{electrical_config[j]}.res')
            if update_regression_values:
                save_results(regression_data, res_path)
            truth_data = load_results(res_path)

            for tag in ['recharge', 'discharge_1', 'discharge_2']:
                for field in ['voltage_under_load', 'cell_temperature']:
                    computed = regression_data[tag][field]
                    truth    = np.array(truth_data[tag][field])
                    error    = np.max(np.abs((computed - truth) / truth))
                    print(f'{tag} {field} max relative error: {error}')
                    assert error < 1e-6, f'{tag} {field} regression failed (max relative error {error})'

            # Sanity check: discharge segments must actually discharge the battery.
            # A vehicle-config plumbing bug can silently leave a discharge segment
            # evaluating with a zero (or otherwise wrong) power draw, in which case
            # it converges trivially with state_of_charge ~constant instead of
            # decreasing -- the V_ul/bat_temp regressions above are read from the
            # Recharge/Discharge_1 endpoints and are not sensitive enough to always
            # catch that on their own.
            for discharge_tag in ['discharge_1', 'discharge_2']:
                soc = results.segments[discharge_tag].conditions.energy.sources['battery_pack'][battery_chemistry[i]].cell.state_of_charge[:,0]
                soc_drop = soc[0] - soc[-1]
                assert soc_drop > 0.05, f'{discharge_tag} state_of_charge barely changed ({soc[0]} -> {soc[-1]}); battery is not actually discharging'

            for segment in results.segments.values():
                volts         = segment.conditions.energy.sources['battery_pack'].voltage_under_load[:,0]
                SOC           = segment.conditions.energy.sources['battery_pack'][battery_chemistry[i]].cell.state_of_charge[:,0]
                cell_temp     = segment.conditions.energy.sources['battery_pack'][battery_chemistry[i]].cell.temperature[:,0]
                Amp_Hrs       = segment.conditions.energy.sources['battery_pack'][battery_chemistry[i]].cell.charge_throughput[:,0]
                  
                if battery_chemistry[i] == 'lithium_ion_nmc':
                    axes1.plot(Amp_Hrs , volts , marker= marker[i], linestyle = linestyles[i],  color= linecolors[j]  , markersize=marker_size   ,label = battery_chemistry[i] + ': '+ str(C_rat[j]) + ' C') 
                    axes3.plot(Amp_Hrs , SOC   , marker= marker[i] , linestyle = linestyles[i],  color= linecolors[j], markersize=marker_size   ,label = battery_chemistry[i] + ': '+ str(C_rat[j]) + ' C') 
                    axes5.plot(Amp_Hrs , cell_temp, marker= marker[i] , linestyle = linestyles[i],  color= linecolors[j] , markersize=marker_size,label = battery_chemistry[i] + ': '+ str(C_rat[j]) + ' C')              
                else:
                    axes2.plot(Amp_Hrs , volts , marker= marker[i], linestyle = linestyles[i],  color= linecolors[j] , markersize=marker_size   ,label = battery_chemistry[i] + ': '+ str(C_rat[j]) + ' C') 
                    axes4.plot(Amp_Hrs , SOC   , marker= marker[i] , linestyle = linestyles[i],  color= linecolors[j], markersize=marker_size   ,label = battery_chemistry[i] + ': '+ str(C_rat[j]) + ' C') 
                    axes6.plot(Amp_Hrs , cell_temp, marker= marker[i] , linestyle = linestyles[i],  color= linecolors[j] , markersize=marker_size,label = battery_chemistry[i] + ': '+ str(C_rat[j]) + ' C')              
             
    legend_font_size = 6                     
    axes1.set_ylabel('Voltage $(V_{UL}$)')  
    axes1.legend(loc='upper right', ncol = 2, prop={'size': legend_font_size})  
    axes1.set_ylim([2.5,5]) 
    axes1.set_xlim([0,7])
    axes2.set_xlabel('Amp-Hours (A-hr)') 
    axes2.legend(loc='upper right', ncol = 2, prop={'size': legend_font_size})  
    axes2.set_ylim([2.5,5])   
    axes2.set_xlim([0,7])   
    axes3.set_ylabel('SOC')  
    axes3.legend(loc='upper right', ncol = 2, prop={'size': legend_font_size})  
    axes3.set_ylim([0,1]) 
    axes3.set_xlim([0,7]) 
    axes4.legend(loc='upper right', ncol = 2, prop={'size': legend_font_size})  
    axes4.set_ylim([0,1])   
    axes4.set_xlim([0,7])      
    axes5.set_xlabel('Amp-Hours (A-hr)') 
    axes5.legend(loc='upper right', ncol = 2, prop={'size': legend_font_size})
    axes5.set_ylim([273,320])
    axes5.set_xlim([0,7]) 
    axes5.set_ylabel(r'Temperature ($\degree$C)')    
    axes6.set_xlabel('Amp-Hours (A-hr)')        
    axes6.legend(loc='upper left', ncol = 2, prop={'size': legend_font_size})
    axes6.set_ylim([273,320])
    axes6.set_xlim([0,7])  
    
    return  
 
def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):    
    #   Initialize the Analyses     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle

    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    analyses.append(geometry)

    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Weights()
    weights.settings.run_weights_analysis = False
    analyses.append(weights)    
    
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)
 
    #  Planet Analysis
    planet  = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)
 
    #  Atmosphere Analysis
    atmosphere                 = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   
 
    return analyses     

def mission_setup(analyses,vehicle,battery_chemistry,current,mAh):
 
    #   Initialize the Mission 
    mission            = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag        = 'cell_cycle_test'   
    Segments           = RCAIDE.Framework.Mission.Segments 
    base_segment       = Segments.Segment()   
    time               = 0.8 * (mAh/1000)/current * Units.hrs  

    # Charge Segment 
    segment                                 = Segments.Ground.Battery_Recharge(base_segment)      
    segment.analyses.extend(analyses.charge) 
    segment.cutoff_SOC                      = 1.0  
    segment.initial_battery_conditions.state_of_charge = 0.2
    segment.tag                             = 'Recharge' 
    mission.append_segment(segment)   

         
    segment                                 = Segments.Ground.Battery_Discharge(base_segment) 
    segment.analyses.extend(analyses.discharge)  
    segment.tag                             = 'Discharge_1' 
    segment.time                            = time/2  
    segment.initial_battery_conditions.state_of_charge = 1
    mission.append_segment(segment)
    
    segment                                = Segments.Ground.Battery_Discharge(base_segment) 
    segment.tag                            = 'Discharge_2'
    segment.analyses.extend(analyses.discharge)   
    segment.time                           = time/2  
    mission.append_segment(segment)        
    
    return mission 

def missions_setup(mission): 
 
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions  

def test_mass_gain(battery,power):
    print(battery)
    mass_gain       =find_total_mass_gain(battery)
    print('mass_gain      = ', mass_gain)
    mdot            =find_mass_gain_rate(battery,power)
    print('mass_gain_rate = ', mdot)
    return

def test_size_module_from_energy_and_power(battery,energy,power):
    size_module_from_energy_and_power(battery, energy, power) 
    print(battery)
    return

def test_find_ragone_properties(specific_energy,battery,energy,power):
    find_ragone_properties( specific_energy, battery, energy,power)
    print(battery)
    print('specific_energy (Wh/kg) = ',battery.specific_energy/(Units.Wh/Units.kg))
    return

if __name__ == '__main__':
    main()
    plt.show()