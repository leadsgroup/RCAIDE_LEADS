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
base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Embraer_190    import vehicle_setup as E190_vehicle_setup       
from BWB            import vehicle_setup as BWB_vehicle_setup       

# ----------------------------------------------------------------------------------------------------------------------
#  REGRESSION
# ----------------------------------------------------------------------------------------------------------------------  
def main():
    # tube and wing load trim test 
    tube_and_wing_load_trim_test()
 
    # blended wing body load trim test 
    blended_wing_body_load_trim_test()
    
    return 

def tube_and_wing_load_trim_test():
    vehicle    = E190_vehicle_setup()  
    
    # take out control surfaces to make regression run faster
    for wing in vehicle.wings:
        wing.control_surfaces  = Container() 
  
    # Set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = E190_analyses_setup(configs)

    # mission analyses 
    mission = E190_mission_setup(analyses)
 
    load_data =  compute_load_and_trim_diagram( mission, cruise_segment_tag = 'cruise', discretization=  3)
    
    save_results(load_data,'taw_loading_results') 
 
    CG_Percent_of_LEMAC_truth = np.array([[-0.26135411,  0.66526503,  1.59188417],
                                        [-0.26135411,  0.66526503,  1.59188417],
                                        [-0.26135411,  0.66526503,  1.59188417]])
    plot_load_diagram(load_data,save_filename  = "TW_Aircraft_Loading_Trim_Dragram") 

    LEMAC_error = np.max(abs((load_data.trim_results.CG_percent_of_LEMAC_location - CG_Percent_of_LEMAC_truth)/CG_Percent_of_LEMAC_truth))
    print(f"LEMAC error: {LEMAC_error}")
    assert LEMAC_error < 1e-2, f"LEMAC error too large: {LEMAC_error}"
        
    return 
 

def blended_wing_body_load_trim_test():
    vehicle    = BWB_vehicle_setup()  
    
    # take out control surfaces to make regression run faster
    for wing in vehicle.wings:
        wing.control_surfaces  = Container() 
  
    # Set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = BWB_analyses_setup(configs)

    # mission analyses 
    mission = BWB_mission_setup(analyses)
 
    load_data =  compute_load_and_trim_diagram( mission, cruise_segment_tag= 'cruise', discretization=  3)
    
    save_results(load_data,'bwb_loading_results')
 
    CG_Percent_of_LEMAC_truth = np.array([[-0.00936612,  0.85303387,  1.71543386],
       [-0.00936612,  0.85303387,  1.71543386],
       [-0.00936612,  0.85303387,  1.71543386]])
    
    plot_load_diagram(load_data,save_filename  = "BWB_Aircraft_Loading_Trim_Dragram") 

    LEMAC_error = np.max(abs((load_data.trim_results.CG_percent_of_LEMAC_location - CG_Percent_of_LEMAC_truth)))
    print(f"LEMAC error: {LEMAC_error}")
    assert LEMAC_error < 5e-3, f"LEMAC error too large: {LEMAC_error}"
        
    return

def configs_setup(vehicle): 
    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'  
    configs.append(base_config) 
    return configs
  
def E190_analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = E190_base_analysis(config)
        analyses[tag] = analysis

    return analyses
 
def BWB_analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = BWB_base_analysis(config)
        analyses[tag] = analysis

    return analyses

def E190_base_analysis(vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    analyses.vehicle =  vehicle
    
    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()   
    analyses.append(geometry)

     # ------------------------------------------------------------------
    #  Weights 
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_Transport()   
    weights.settings.FLOPS.fidelity              = 'Complex' 
    weights.settings.run_center_of_gravity_analysis             = True
    weights.settings.run_moments_of_inertia_analysis            = True 
    weights.print_weight_analysis_report         = False
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis  
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis  
    stability     = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method()  
    analyses.append(stability)       

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    # done!
    return analyses

def BWB_base_analysis(vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    analyses.vehicle =  vehicle
    
    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()   
    analyses.append(geometry)

     # ------------------------------------------------------------------
    #  Weights 
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_BWB()   
    weights.settings.FLOPS.fidelity              = 'Complex' 
    weights.print_weight_analysis_report         = False
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis  
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis  
    stability     = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method()   
    analyses.append(stability)       

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    # done!
    return analyses    

def E190_mission_setup(analyses): 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.solver.type = 'root_finder'

    # ------------------------------------------------------------------    
    #   Cruise Segment 
    # ------------------------------------------------------------------    

    segment = Segments.Single_Point.Set_Speed_Set_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.base )  
    segment.altitude  =  35000 *  Units.ft
    segment.air_speed =  450 * Units['knots'] 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]   
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment) 
  
    return mission

def BWB_mission_setup(analyses): 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.solver.type = 'root_finder'

    # ------------------------------------------------------------------    
    #   Cruise Segment 
    # ------------------------------------------------------------------    

    segment = Segments.Single_Point.Set_Speed_Set_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.base )  
    segment.altitude  =  35000 *  Units.ft
    segment.air_speed =  450 * Units['knots'] 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']]   
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment) 
  
    return mission 


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