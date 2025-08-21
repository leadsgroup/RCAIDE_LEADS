# VnV/Verification/analysis_emissions/LTO_Emissions.py
# 
# Created:  Jun 2025, A. Molloy

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
import Boeing_787 

# ----------------------------------------------------------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------------------------------------------------------

def main():
    
    vehicle  = Boeing_787.vehicle_setup() 
    configs  = Boeing_787.configs_setup(vehicle) 
    analyses = analyses_setup(configs) 
    mission  = LTO_emisions_mission_setup(analyses) 
    missions = missions_setup(mission)

    # Step 5 execute flight profile
    results = missions.base_mission.evaluate()
    
    design_thrust = vehicle.networks.fuel.propulsors.propulsor_1.sealevel_static_thrust/1000 # kN
    num_engines   = len(vehicle.networks.fuel.propulsors)
   
    NOx_mass_g  = 0
    CO2_mass_g  = 0
    H2O_mass_g  = 0
    Soot_mass_g = 0
    for segment in results.segments: 
        NOx_mass_g    += (segment.conditions.emissions.mass.NOx[-1, 0]*1000 ) /num_engines  #   mass of pollutant (g) per engine 
        CO2_mass_g    += (segment.conditions.emissions.mass.CO2[-1, 0]*1000 ) /num_engines  #   mass of pollutant (g) per engine      
        H2O_mass_g    += (segment.conditions.emissions.mass.H2O[-1, 0]*1000 ) /num_engines #   mass of pollutant (g) per engine      
        Soot_mass_g   += (segment.conditions.emissions.mass.Soot[-1, 0]*1000 ) /num_engines  #   mass of pollutant (g) per engine
    
    LTO_NOx   = NOx_mass_g /design_thrust
    LTO_CO2   = CO2_mass_g /design_thrust
    LTO_H2O   = H2O_mass_g /design_thrust
    LTO_Soot  = Soot_mass_g /design_thrust

    # Truth values
    truth_values = {'LTO_NOx_truth': 19.640678833265977, 'LTO_CO2_truth': 4099.375502848117, 'LTO_H2O_truth': 1595.6429963617672, 'LTO_Soot_truth': 1.5567248744992848}

    diff_LTO_NOx = (LTO_NOx - truth_values['LTO_NOx_truth'])/truth_values['LTO_NOx_truth']
    diff_LTO_CO2 = (LTO_CO2 - truth_values['LTO_CO2_truth'])/truth_values['LTO_CO2_truth']
    diff_LTO_H2O = (LTO_H2O - truth_values['LTO_H2O_truth'])/truth_values['LTO_H2O_truth']
    diff_LTO_Soot = (LTO_Soot - truth_values['LTO_Soot_truth'])/truth_values['LTO_Soot_truth']

    # Print and assert errors
    print('LTO NOx Error: ',diff_LTO_NOx)
    print('LTO CO2 Error: ',diff_LTO_CO2)
    print('LTO H2O Error: ',diff_LTO_H2O)
    print('LTO Soot Error: ',diff_LTO_Soot)
    
    assert (diff_LTO_NOx/truth_values['LTO_NOx_truth']) < 1e-5
    assert (diff_LTO_CO2/truth_values['LTO_CO2_truth']) < 1e-5
    assert (diff_LTO_H2O/truth_values['LTO_H2O_truth']) < 1e-5
    assert (diff_LTO_Soot/truth_values['LTO_Soot_truth']) < 1e-5
             
    return 

def LTO_emisions_mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment() 
    base_segment.state.numerics.number_of_control_points = 4 
  
    
    segment = Segments.Ground.Test_Stand(base_segment)
    segment.tag = "LTO_Takeoff_Setting" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity                                    = 157.0 * Units['knots']  
    segment.altitude                                    = 5.0   
    segment.throttle                                    = 1
    segment.time                                        = 0.7 *  Units.minutes 
    mission.append_segment(segment)
 
    return mission   
 
def analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses


def base_analysis(vehicle):
    """This is the baseline set of analyses to be used with this vehicle. Of these, the most
    commonly changed are the weights and aerodynamics methods."""

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.vehicle = vehicle
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Conventional()
    weights.vehicle = vehicle 
    weights.settings.FLOPS.fidelity                                          = 'Complex'      
    weights.settings.weight_correction_additions.empty.structural.paint      = 450 
    weights.settings.weight_correction_additions.operational_items.ETOPS     = 7.7 * vehicle.passengers
    weights.settings.weight_correction_factors.empty.structural.landing_gear = 1.1  
    weights.settings.weight_correction_additions.empty.propulsion.battery    = 56   
    weights.settings.weight_correction_factors.empty.systems.electrical      = 2.67 
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.vehicle = vehicle    
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy = RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle = vehicle 
    analyses.append(energy)
    

    # ------------------------------------------------------------------
    # Emissions 
    emissions = RCAIDE.Framework.Analyses.Emissions.Emission_Index_Correlation_Method()            
    emissions.vehicle = vehicle          
    analyses.append(emissions)  

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    return analyses  

def missions_setup(mission):
    """This allows multiple missions to be incorporated if desired, but only one is used here."""

    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)

    return missions

def missions_setup(mission):

    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)
    
    # done!
    return missions  
     

if __name__ == '__main__': 
    main()    
    plt.show()
        
