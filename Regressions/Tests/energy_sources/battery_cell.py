# Regression/scripts/network_isolated_battery_cell/cell_test.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE  
from RCAIDE.Framework.Core                                            import Units, Data 
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common       import size_module_from_mass ,size_module_from_energy_and_power, find_mass_gain_rate, find_total_mass_gain, find_ragone_properties
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Aluminum_Air import * 
from RCAIDE.Framework.Mission.Common                                  import Conditions
from RCAIDE.Library.Plots                                             import * 

# package imports  
import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.cm as cm

# local imports 
import sys 
import os
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Battery_Cell   import vehicle_setup , configs_setup  

# ----------------------------------------------------------------------------------------------------------------------
#  REGRESSION
# ----------------------------------------------------------------------------------------------------------------------  

def main(): 
    Ereq                           = 3000*Units.Wh  
    Preq                           = 2000.  

    # Lithium Air Battery Test 
    lithium_air_battery_test(Ereq,Preq)
    
    # Aluminum Air Battery Test 
    aluminum_air_battery_test(Ereq,Preq)
    
    # Lithium Sulfur Test 
    lithium_sulphur_battery_test(Ereq,Preq)
        
    # Lithium-Ion Test
    lithium_ion_battery_test()
    return 
    
     
def lithium_air_battery_test(Ereq,Preq): 
    battery_li_air                 = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Air()     
    return 
   
        
def aluminum_air_battery_test(Ereq,Preq): 
    battery_al_air                 = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Aluminum_Air()    
    test_size_module_from_energy_and_power(battery_al_air, Ereq, Preq)
    test_mass_gain(battery_al_air, Preq)
    
    aluminum_mass  =  find_aluminum_mass(battery_al_air, Ereq)
    water_mass     =  find_water_mass(battery_al_air, Ereq)

    return 
   
def lithium_sulphur_battery_test(Ereq,Preq):   
    battery_li_s                   = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Sulfur()
    specific_energy_guess          = 400*Units.Wh/Units.kg 
    test_find_ragone_properties(specific_energy_guess,battery_li_s, Ereq,Preq) 
    plot_battery_ragone_diagram(battery_li_s,   save_filename =  'lithium_sulfur')     
    return 


def lithium_ion_battery_test():    
    
    # Operating conditions for battery p
    curr                  = [1.5,3]  
    C_rat                 = [0.5,1]  
    marker_size           = 5 
    mAh                   = np.array([3800,2600]) 
    V_ul_true             = np.array([[3.176391931635407,3.1422615279089],[3.176391931635407,3.1422615279089]])
    bat_temp_true         = np.array([[309.47942727882105,304.7804077267032], [309.65733640183896,304.9861235766863]])  

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
            
            # Voltage Cell Regression
            V_ul        = results.segments[0].conditions.energy.bus.battery_modules[battery_chemistry[i]].cell.voltage_under_load[2][0]   
            print('Under load voltage: ' + str(V_ul))
            V_ul_diff   = np.abs(V_ul - V_ul_true[j,i])
            print('Under load voltage difference')
            print(V_ul_diff) 
            assert np.abs((V_ul_diff)/V_ul_true[j,i]) < 1e-6  
           
            # Temperature Regression
            bat_temp        = results.segments[1].conditions.energy.bus.battery_modules[battery_chemistry[i]].cell.temperature[2][0]  
            print('Cell temperature: ' + str(bat_temp))
            bat_temp_diff   = np.abs(bat_temp  - bat_temp_true[j,i]) 
            print('cell temperature difference')
            print(bat_temp_diff)
            assert np.abs((bat_temp_diff)/bat_temp_true[j,i]) < 1e-6
       
            for segment in results.segments.values(): 
                volts         = segment.conditions.energy.bus.voltage_under_load[:,0] 
                SOC           = segment.conditions.energy.bus.battery_modules[battery_chemistry[i]].cell.state_of_charge[:,0]   
                cell_temp     = segment.conditions.energy.bus.battery_modules[battery_chemistry[i]].cell.temperature[:,0]   
                Amp_Hrs       = segment.conditions.energy.bus.battery_modules[battery_chemistry[i]].cell.charge_throughput[:,0]                   
                  
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
    
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle  = vehicle 
    analyses.append(energy)
 
    #  Planet Analysis
    planet  = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)
 
    #  Atmosphere Analysis
    atmosphere                 = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
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
    segment.initial_battery_state_of_charge = 0.2  
    segment.tag                             = 'Recharge' 
    mission.append_segment(segment)   

         
    segment                                 = Segments.Ground.Battery_Discharge(base_segment) 
    segment.analyses.extend(analyses.discharge)  
    segment.tag                             = 'Discharge_1' 
    segment.time                            = time/2  
    segment.initial_battery_state_of_charge = 1  
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