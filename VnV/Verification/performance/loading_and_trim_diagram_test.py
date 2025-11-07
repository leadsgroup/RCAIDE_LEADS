# Regression/scripts/Tests/load_diagram_test.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Core import Units  , Container
from RCAIDE.Library.Methods.Performance.compute_load_and_trim_diagram        import compute_load_and_trim_diagram
from RCAIDE.Library.Plots.Common import set_axes, plot_style
import matplotlib.pyplot as plt
from RCAIDE.Library.Plots import  * 

# python imports      
import os
import numpy as np  
import pickle
import sys 
import numpy as np
import matplotlib.pyplot as plt

# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Embraer_190    import vehicle_setup as vehicle_setup       
from Embraer_190    import configs_setup as configs_setup 

# ----------------------------------------------------------------------------------------------------------------------
#  REGRESSION
# ----------------------------------------------------------------------------------------------------------------------  
def main(): 
    
    vehicle    = vehicle_setup()  
    for wing in vehicle.wings:
        wing.control_surfaces  = Container()   
     
    configs   = configs_setup(vehicle) 
    analyses  = analyses_setup(configs) 
    mission   = mission_setup(analyses)    
    load_data =  compute_load_and_trim_diagram(vehicle, mission)
    
    save_results(load_data,'loading_results') 
 
    LEMAC_truth = np.array([[-37.96970039,  51.73364162, 141.43698363],
                            [-37.96970039,  51.73364162, 141.43698363],
                            [-37.96970039,  51.73364162, 141.43698363]])

    plot_load_diagram(load_data) 

    LEMAC_error = np.max(abs((load_data.aerodynamic_LEMAC_location - LEMAC_truth)/LEMAC_truth))
    print(f"LEMAC error: {LEMAC_error}")
    assert LEMAC_error < 1e-4, f"LEMAC error too large: {LEMAC_error}"
        
    return

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
   
    analyses = RCAIDE.Framework.Analyses.Vehicle()
   
    # append vehicle 
    analyses.vehicle = vehicle 

    #  Geometry
    analyses.geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    
    #  Weights
    analyses.weights = RCAIDE.Framework.Analyses.Weights.Conventional() 
    analyses.weights.settings.FLOPS.fidelity                      = 'Complex'      
    analyses.weights.settings.advanced_composites                 = True 
    
    # Aerodynamics
    analyses.aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    analyses.aerodynamics.settings.number_of_spanwise_vortices   = 15
    analyses.aerodynamics.settings.number_of_chordwise_vortices  = 2 

    analyses.stability = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method()  
    analyses.stability.settings.number_of_spanwise_vortices   = 15
    analyses.stability.settings.number_of_chordwise_vortices  = 2 
 
    #  Energy
    analyses.energy = RCAIDE.Framework.Analyses.Energy.Energy() 

    #  Planet Analysis
    analyses.planet = RCAIDE.Framework.Analyses.Planets.Earth() 

    #  Atmosphere Analysis
    analyses.atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.atmosphere.features.planet = analyses.planet.features 

    return analyses    
    
def mission_setup(analyses): 

    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.solver.type = 'root_finder'
 
    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                      = 10.668 * Units.km  
    segment.air_speed                                     = 230.412 * Units['m/s']
    segment.distance                                      = 1000 * Units.nmi  
    segment.hybrid_power_split_ratio = 0.05
    segment.battery_fuel_cell_power_split_ratio = 1.0
    
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
    """This allows multiple missions to be incorporated if desired, but only one is used here."""

    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)

    return missions

def save_results(data,filename): 
    pickle_file  = filename + '.pkl'
    with open(pickle_file, 'wb') as file:
        pickle.dump(data, file) 
    return 


def load_results(filename):  
    load_file = filename + '.pkl' 
    with open(load_file, 'rb') as file:
        results = pickle.load(file) 
    return results 

 
if __name__ == '__main__': 
    main()
    plt.show()