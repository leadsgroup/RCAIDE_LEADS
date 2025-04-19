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
    
    true_EI_CO2s =  [3.16, 3.0996295865239563, 3.1371106320136155]
    true_EI_H2Os =  [1.34, 1.1911420639654764, 1.2053455595806213]
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
        emissions.training.pressure          = np.linspace(10,30, 1) *1E6
        emissions.training.temperature       = np.linspace(700, 900, 1) 
        emissions.training.air_mass_flowrate = np.linspace(10, 60, 1) 
        emissions.training.fuel_to_air_ratio = np.linspace(0.01, 0.05, 1)             
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
     

if __name__ == '__main__': 
    main()    
    plt.show()
        
