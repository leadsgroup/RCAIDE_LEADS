# Regression/scripts/network_isolated_battery_cell/cell_test.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE  
from RCAIDE.Framework.Core                                    import Units, Data  
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Aluminum_Air import * 
from RCAIDE.Framework.Mission.Common                          import Conditions
from RCAIDE.Library.Plots                                     import * 

# package imports  
import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.cm as cm

# local imports 
import sys 
import os
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Hydrogen_Fuel_Cell   import vehicle_setup , configs_setup  

# ----------------------------------------------------------------------------------------------------------------------
#  REGRESSION
# ----------------------------------------------------------------------------------------------------------------------  

def main():   
    
    # Operating conditions for battery p 
    marker_size           = 5   
    mdot_H2_true          = [3.1979336247376623e-07,6.498654180730008e-07]

    # PLot parameters 
    marker                = ['s' ,'o' ,'P']
    linestyles            = ['-','--',':']
    linecolors            = cm.inferno(np.linspace(0.2,0.8,3))     

    fuel_cell_tpye     =  ['Larminie', 'PEM']
    

    fig1 = plt.figure('Fuel Cell Test') 
    fig1.set_size_inches(6,5)   
    axes1  = fig1.add_subplot(1,1,1)
    
    for i in range(len(fuel_cell_tpye)):
        
        vehicle  = vehicle_setup(fuel_cell_tpye[i]) 
        
        # Set up vehicle configs
        configs  = configs_setup(vehicle)
    
        # create analyses
        analyses = analyses_setup(configs)
    
        # mission analyses
        mission  = mission_setup(analyses) 
        
        # create mission instances (for multiple types of missions)
        missions = missions_setup(mission) 
         
        # mission analysis 
        results = missions.base_mission.evaluate()  
        
        # Hydrogen Mass Flow Rate Regression
        fuel_cell_tag = list(results.segments[0].conditions.energy.bus.fuel_cell_stacks.keys())[0]
        mdot_H2       = results.segments[0].conditions.energy.bus.fuel_cell_stacks[fuel_cell_tag].H2_mass_flow_rate
        print('Mass Flow Rate: ' + str(mdot_H2[0,0]))
        mdot_H2_diff   = np.abs(mdot_H2[0,0] - mdot_H2_true[i]) 
        print(mdot_H2_diff) 
        assert np.abs((mdot_H2_diff)/mdot_H2_true[i]) < 1e-6  

        time     = results.segments[0].conditions.frames.inertial.time[:,0] 
        axes1.plot(time , mdot_H2 , marker= marker[i], linestyle = linestyles[i],  color= linecolors[i]  , markersize=marker_size   ,label = fuel_cell_tpye[i])             
             
    legend_font_size = 6

    plt.rcParams.update({'font.size': 22}) 
    axes1.set_ylabel('H2 Mass Flow Rate $(kg/s)$)')  
    axes1.set_xlabel('Time (s)')  
    axes1.legend(loc='upper right', ncol = 2, prop={'size': legend_font_size})  
    axes1.set_ylim([0,1E-6])  
    
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

def mission_setup(analyses):
 
    #   Initialize the Mission 
    mission            = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag        = 'cell_cycle_test'   
    Segments           = RCAIDE.Framework.Mission.Segments 
    base_segment       = Segments.Segment()    
  
    segment                                 = Segments.Ground.Battery_Discharge(base_segment) 
    segment.analyses.extend(analyses.discharge)  
    segment.tag                             = 'Discharge_1' 
    segment.time                            = 60  
    mission.append_segment(segment)
     
    return mission 

def missions_setup(mission): 
 
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions  
 
if __name__ == '__main__':
    main()
    plt.show()