# blended_wing_body_aerodynamics_test.py

import RCAIDE
from RCAIDE.Framework.Core import Data, Units  
from RCAIDE.Library.Plots import *  
import numpy as  np 
import sys
import os

sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))

# the analysis functions
from BWB    import vehicle_setup  ,  configs_setup

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    
    vehicle  = vehicle_setup() 
    configs  = configs_setup(vehicle) 
    analyses = analyses_setup(configs)  
    mission  = mission_setup(analyses)
    missions = missions_setup(mission)  
    results  = missions.base_mission.evaluate() 

    vortex_distribution = results.segments.cruise.analyses.aerodynamics.settings.vortex_distribution
    plot_3d_vehicle_vlm_panelization(vortex_distribution=vortex_distribution,
                    save_filename               = "BWB_Top_View", 
                    show_wing_control_points    = False,  
                    show_figure                 = False)

    plot_3d_vehicle_vlm_panelization(vortex_distribution=vortex_distribution,
                    save_filename               = "BWB_Top_View",
                    show_wing_control_points    = True,  
                    show_figure                 = False)

    Cruise_CL        = results.segments.cruise.conditions.aerodynamics.coefficients.lift.total[2][0] 
    Cruise_CL_true   = 0.38410077243879337
    Cruise_CL_diff   = np.abs(Cruise_CL - Cruise_CL_true)
    print('Error: ',Cruise_CL_diff)
    assert np.abs((Cruise_CL - Cruise_CL_true)/Cruise_CL_true) < 1e-6 
    
    
    # test lopa coordianates
    LOPA_coords =  results.segments.cruise.analyses.vehicle.wings.main_wing.layout_of_passenger_accommodations.object_coordinates

    # Extract sample values from computation   
    coordinate_1_x         = LOPA_coords[22][2]
    coordinate_1_y         = LOPA_coords[34][3]  
    coordinate_2_x         = LOPA_coords[124][2]
    coordinate_2_y         = LOPA_coords[68][3] 
    coordinate_3_x         = LOPA_coords[0][2]
    coordinate_3_y         = LOPA_coords[301][3] 
    
    # thruth values 
    coordinate_1_x_thruth  = 5.6388
    coordinate_1_y_thruth  = 1.7018 
    coordinate_2_x_thruth  = 1.3716
    coordinate_2_y_thruth  = 0.6858
    coordinate_3_x_thruth  = 0.4572
    coordinate_3_y_thruth  = -3.556
    
    # Truth values  
    error = Data()  
    error.coordinate_1_x   = np.max(np.abs(coordinate_1_x - coordinate_1_x_thruth))   
    error.coordinate_1_y   = np.max(np.abs(coordinate_1_y - coordinate_1_y_thruth))    
    error.coordinate_2_x   = np.max(np.abs(coordinate_2_x - coordinate_2_x_thruth))   
    error.coordinate_2_y   = np.max(np.abs(coordinate_2_y - coordinate_2_y_thruth))    
    error.coordinate_3_x   = np.max(np.abs(coordinate_3_x - coordinate_3_x_thruth))   
    error.coordinate_3_y   = np.max(np.abs(coordinate_3_y - coordinate_3_y_thruth))    
    print('Errors:')                               
    print(error)
     
    for k,v in list(error.items()): 
        assert(np.abs(v)<1e-6)        
    
    return 

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

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
    analyses.vehicle = vehicle

    # ------------------------------------------------------------------
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.update_fuselage_properties = True 
    analyses.append(geometry)
    

    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_BWB() 
    weights.settings.FLOPS.fidelity     = 'Complex'  
    weights.settings.overwrite_center_of_gravity = True
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()   
    analyses.append(aerodynamics)
 
    # ------------------------------------------------------------------
    #  Energy
    energy = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    return analyses    
    
    

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment() 
 

    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                 = 40000 * Units['ft']  
    segment.air_speed                                                = 450 * Units['knots']
    segment.distance                                                 = 7370 * Units.km   
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment) 

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Landing Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "Landing"

    segment.analyses.extend( analyses.reverse_thrust ) 
    segment.velocity_start                                                = 160.0 * Units['knots']
    segment.velocity_end                                                  = 10 * Units.knots 
    segment.friction_coefficient                                          = 0.4
    segment.altitude                                                      = 0.0
    
    segment.assigned_control_variables.elapsed_time.active                = True  
    segment.assigned_control_variables.elapsed_time.initial_guess_values  = [[30.]]  
    segment.assigned_control_variables.elapsed_time.bounds                  = [[-10, 100000000]]
    mission.append_segment(segment)     

    return mission
 
def missions_setup(mission):
    """This allows multiple missions to be incorporated if desired, but only one is used here."""

    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)

    return missions 
if __name__ == '__main__': 
    main()     