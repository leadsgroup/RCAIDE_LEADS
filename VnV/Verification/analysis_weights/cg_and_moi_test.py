# Regression/scripts/Tests/analysis_weights/cg_and_moi_test.py
# 
# Created:  Oct 2024, A. Molloy
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# cg_and_moi_test.py

from RCAIDE.Framework.Core                                     import Units,  Data  
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia  import compute_vehicle_moment_of_inertia
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity  import compute_vehicle_center_of_gravity
from RCAIDE.Library.Methods.Geometry.Planform                  import wing_planform
from RCAIDE.Library.Mission.Common.Pre_Process                 import geometry, mass_properties
import numpy as  np
import RCAIDE
import pandas as pd
import sys   
import os

base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)

# the analysis functions
from Lockheed_C5a           import vehicle_setup as transport_setup
from Cessna_172             import vehicle_setup as general_aviation_setup
from Stopped_Rotor_EVTOL    import vehicle_setup as EVTOL_setup
from BWB                    import vehicle_setup as BWB_vehicle_setup
def main(): 
    # make true only when resizing aircraft. should be left false for regression
    update_regression_values = False  
    Transport_Aircraft_Test()
    General_Aviation_Test()
    EVTOL_Aircraft_Test(update_regression_values)
    # -------------------------------------------------------------
    # Run test only if Python version >= 3.11
    # Shapely < 2.1 (and Python < 3.11) may not include functions
    # like 'maximum_inscribed_circle' required for this test.
    # -------------------------------------------------------------
    if sys.version_info >= (3, 11):
        BWB_Test()
    else:
        print("Skipping BWB_Test():\
            Shapely lacks 'maximum_inscribed_circle' support for Python < 3.11.")
    return

def BWB_Test():

    vehicle          = BWB_vehicle_setup() 
    fuel_line        = vehicle.networks.fuel.fuel_lines.fuel_line
    fuel_line.fuel_tanks.clear()
    
    #############################################################################################################################    
     #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #------------------------------------------------------------------------------------------------------------------------- 
    # fuel tank
    fuel_tank_1                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Natural_Gas_Tank(vehicle.wings.main_wing)
    fuel_tank_1.tag                             = 'LNG_Fuel_Tank_1' 
    fuel_tank_1.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()  
    fuel_tank_1.material                        = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_1.insulation_material             = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_1.fuel.gravimetric_efficiency     = 0.5 
    fuel_tank_1.segments_bounding_tank          = ['fuselage_section_3', 'wing_section_2']        
    fuel_tank_1.segments_percent_chord_start    = [0.2,0.2]
    fuel_tank_1.segments_percent_chord_end      = [0.6,0.6]  
    fuel_tank_1.wall_thickness                  = 2*Units.inches
    fuel_line.fuel_tanks.append(fuel_tank_1)


    fuel_tank_2                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Natural_Gas_Tank(vehicle.wings.main_wing)
    fuel_tank_2.tag                           = 'LNG_Fuel_Tank_2' 
    fuel_tank_2.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()   
    fuel_tank_2.material                      = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_2.insulation_material           = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_2.fuel.gravimetric_efficiency   = 0.5
    fuel_tank_2.xz_plane_symmetric            = False
    fuel_tank_2.orientation_euler_angles      = [0,0,np.pi/2]
    fuel_tank_2.bwb_aft_tank                  = True
    fuel_tank_2.aft_tank_root_chord_bounds    = [0.65,0.9]
    fuel_tank_2.aft_tank_segment_bound        = 'fuel_wall'
    fuel_tank_2.radial_offset                 = 0.2

    fuel_line.fuel_tanks.append(fuel_tank_2)
  
    configs  = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    for analysis in analyses:
        analysis.geometry.settings.compute_fuel_volume = True
        analysis.geometry.settings.update_max_fuel = True
    mission  = mission_setup(analyses)

    geometry(mission)   
    mass_properties(mission)

    truth_moi = np.array([[ 3.58809772e+06,  2.09092723e+06, -6.63490042e+05],
                          [ 2.09092723e+06,  2.55119296e+07,  9.57721836e+03],
                          [-6.63490042e+05,  9.57721836e+03,  2.78709759e+07]])
    computed_moi = mission.segments[0].analyses.vehicle.mass_properties.moments_of_inertia.tensor
    error_matrix = abs((computed_moi - truth_moi) / truth_moi)
    assert np.all(error_matrix < 1e-2),\
        f"MOI tensor mismatch.\nExpected:\n{truth_moi}\nGot:\n{computed_moi}"

    return

def Transport_Aircraft_Test():
    vehicle = transport_setup()
    for wing in vehicle.wings: 
        wing_planform(wing) 

    # update fuel weight to 60%
    vehicle.networks.fuel.fuel_lines.fuel_line.fuel_tanks.integral_tank.fuel.mass_properties.mass = 0.6 * vehicle.networks.fuel.fuel_lines.fuel_line.fuel_tanks.integral_tank.fuel.mass_properties.mass
    vehicle.mass_properties.fuel = 0.6 * vehicle.networks.fuel.fuel_lines.fuel_line.fuel_tanks.integral_tank.fuel.mass_properties.mass
    # ------------------------------------------------------------------
    #   Weight Breakdown 
    # ------------------------------------------------------------------  
    weight_analysis                               = RCAIDE.Framework.Analyses.Weights.Conventional_Transport()
    weight_analysis.aircraft_type                 = "Transport" 
    weight_analysis.method                        = 'Raymer'
    weight_analysis.settings.use_max_fuel_weight  = False  
    weight_analysis.settings.cargo_doors_number   = 2
    weight_analysis.settings.cargo_doors_clamshell= True
    results                                       = weight_analysis.evaluate(vehicle) 

    # ------------------------------------------------------------------
    #   CG Location
    # ------------------------------------------------------------------  
    centre_of_gravity_df = pd.DataFrame(columns=[
    "Component",
    "Mass (kg)",
    "CG x (m)",
    "CG y (m)",
    "CG z (m)"
    ])
    verbose_flag = False
    CG_location,_, _, centre_of_gravity_df = compute_vehicle_center_of_gravity(vehicle,centre_of_gravity_df,
                                            overwrite_center_of_gravity =  True ,
                                            verbose=verbose_flag)     

    # ------------------------------------------------------------------
    #   Operating Aircraft MOI
    # ------------------------------------------------------------------ 
 
    moment_of_inertia_df = pd.DataFrame(columns=[
    "Component",
    "Mass (kg)",
    "Ixx (kg·m²)",
    "Iyy (kg·m²)",
    "Izz (kg·m²)",
    "Ixy (kg·m²)",
    "Ixz (kg·m²)",
    "Iyz (kg·m²)",
    ])
    overwrite_MOI = True
    verbose_flag = False
    MOI ,moment_of_inertia_df = compute_vehicle_moment_of_inertia(vehicle,moment_of_inertia_df,
                                        overwrite_moment_of_intertia = overwrite_MOI,
                                        verbose=verbose_flag)   
    
    print(vehicle.tag + ' Moment of Inertia')
    print(MOI) 
    accepted  = np.array([[ 1.61568025e+07, -2.32830644e-10, -7.93049792e+06],
       [-2.32830644e-10,  5.94821638e+07,  0.00000000e+00],
       [-7.93049792e+06,  0.00000000e+00,  5.53750895e+07]])
                          
    MOI_error     = (MOI - accepted) / accepted

    # Check the errors
    error = Data()
    error.Ixx   = MOI_error[0, 0]
    error.Iyy   = MOI_error[1, 1]
    error.Izz   = MOI_error[2, 2]
    error.Ixz   = MOI_error[2, 0]
    error.Ixy   = MOI_error[1, 0]

    print('Errors:')
    print(error)

    for k,v in list(error.items()):
        assert(np.abs(v)<1e-6) 

    return  

def General_Aviation_Test(): 
    # ------------------------------------------------------------------
    #   Weight Breakdown 
    # ------------------------------------------------------------------  
    weight_analysis               = RCAIDE.Framework.Analyses.Weights.Conventional_General_Aviation() 
    vehicle                       = general_aviation_setup() 
    for wing in vehicle.wings: 
        wing_planform(wing) 
        if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
            vehicle.reference_area = wing.areas.reference 
    results                       = weight_analysis.evaluate(vehicle) 

    # ------------------------------------------------------------------
    #   CG Location
    # ------------------------------------------------------------------    
    centre_of_gravity_df = pd.DataFrame(columns=[
    "Component",
    "Mass (kg)",
    "CG x (m)",
    "CG y (m)",
    "CG z (m)"
    ])
    verbose_flag = False
    CG_location,_, _, centre_of_gravity_df = compute_vehicle_center_of_gravity(vehicle,centre_of_gravity_df,
                                            overwrite_center_of_gravity =  True ,
                                            verbose=verbose_flag)   

    # ------------------------------------------------------------------
    #   Operating Aircraft MOI
    # ------------------------------------------------------------------    
    moment_of_inertia_df = pd.DataFrame(columns=[
    "Component",
    "Mass (kg)",
    "Ixx (kg·m²)",
    "Iyy (kg·m²)",
    "Izz (kg·m²)",
    "Ixy (kg·m²)",
    "Ixz (kg·m²)",
    "Iyz (kg·m²)",
    ])
    overwrite_MOI = True
    verbose_flag = False
    MOI ,moment_of_inertia_df = compute_vehicle_moment_of_inertia(vehicle,moment_of_inertia_df,
                                        overwrite_moment_of_intertia = overwrite_MOI,
                                        verbose=verbose_flag)   

    print(vehicle.tag + ' Moment of Inertia')
    print(MOI)

    accepted  = np.array([[2213.58651629,    0.        ,  -95.86139613],
                          [   0.        , 4733.73447153,    0.        ],
                          [ -95.86139613,    0.        , 2717.82190841]])

    MOI_error     = MOI - accepted

    # Check the errors
    error       = Data()
    error.Ixx   = MOI_error[0, 0]
    error.Iyy   = MOI_error[1, 1]
    error.Izz   = MOI_error[2, 2]
    error.Ixz   = MOI_error[2, 0]
    error.Ixy   = MOI_error[1, 0]

    print('Errors:')
    print(error)

    for k,v in list(error.items()):
        assert(np.abs(v)<1e-5)   

    return

def EVTOL_Aircraft_Test(update_regression_values):
    vehicle = EVTOL_setup(update_regression_values)
    for wing in vehicle.wings: 
        wing_planform(wing) 
        if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
            vehicle.reference_area = wing.areas.reference
    # ------------------------------------------------------------------
    #   Weight Breakdown 
    # ------------------------------------------------------------------  
    weight_analysis          = RCAIDE.Framework.Analyses.Weights.Electric_VTOL()
    weight_analysis.method    = 'Physics_Based'
    weight_analysis.aircraft_type = 'VTOL'
    weight_analysis.settings.safety_factor               = 1.5    
    weight_analysis.settings.miscelleneous_weight_factor = 1.1 
    weight_analysis.settings.disk_area_factor            = 1.15
    weight_analysis.settings.max_thrust_to_weight_ratio  = 1.1
    weight_analysis.settings.max_g_load                  = 3.8 
    results                                              = weight_analysis.evaluate(vehicle) 

    # ------------------------------------------------------------------
    #   CG Location
    # ------------------------------------------------------------------    
    centre_of_gravity_df = pd.DataFrame(columns=[
    "Component",
    "Mass (kg)",
    "CG x (m)",
    "CG y (m)",
    "CG z (m)"
    ])
    verbose_flag = False
    CG_location,_, _, centre_of_gravity_df = compute_vehicle_center_of_gravity(vehicle,centre_of_gravity_df,
                                            overwrite_center_of_gravity =  True ,
                                            verbose=verbose_flag)   

    # ------------------------------------------------------------------
    #   Operating Aircraft MOI
    # ------------------------------------------------------------------    
    moment_of_inertia_df = pd.DataFrame(columns=[
    "Component",
    "Mass (kg)",
    "Ixx (kg·m²)",
    "Iyy (kg·m²)",
    "Izz (kg·m²)",
    "Ixy (kg·m²)",
    "Ixz (kg·m²)",
    "Iyz (kg·m²)",
    ])
    overwrite_MOI = True
    verbose_flag = False
    MOI ,moment_of_inertia_df = compute_vehicle_moment_of_inertia(vehicle,moment_of_inertia_df,
                                        overwrite_moment_of_intertia = overwrite_MOI,
                                        verbose=verbose_flag)   

    print(vehicle.tag + ' Moment of Inertia')
    print(MOI) 
    accepted  = np.array([[ 9439.70980302,  -432.78287632,  -285.54130702],
                          [ -432.78287632,  9813.07098438,  -101.63196553],
                          [ -285.54130702,  -101.63196553, 17475.15176533]])
    MOI_error     = (MOI - accepted) / accepted

    # Check the errors
    error = Data()
    error.Ixx   = MOI_error[0, 0]
    error.Iyy   = MOI_error[1, 1]
    error.Izz   = MOI_error[2, 2]
    error.Ixz   = MOI_error[2, 0]
    error.Ixy   = MOI_error[1, 0]

    print('Errors:')
    print(error)

    for k,v in list(error.items()):
        assert(np.abs(v)<5e-2) # Note that EVTOL weight is an iterative process, therefore the error can be larger than expected. 

    return  

def configs_setup(vehicle):
    """This function sets up vehicle configurations for use in different parts of the mission.
    Here, this is mostly in terms of high lift settings."""

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base' 
    configs.append(base_config)
    return configs

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
    analyses.append(geometry)
    

    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_BWB() 
    weights.aircraft_type                                                    = 'BWB'
    weights.settings.FLOPS.fidelity                                          = 'Complex' 
    weights.settings.run_weights_analysis                                    = True
    weights.settings.run_center_of_gravity_analysis                          = True
    weights.settings.run_moments_of_inertia_analysis                         = True
    analyses.append(weights)

    return analyses  

def mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment() 
    base_segment.state.numerics.number_of_control_points = 16
  
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.base ) 
    segment.altitude                                                 = 35000 * Units['ft']  
    segment.air_speed                                                = 450 * Units['knots']
    segment.distance                                                 = 7370 * Units.km   
    mission.append_segment(segment)

    return mission  

if __name__ == '__main__':
    main()


