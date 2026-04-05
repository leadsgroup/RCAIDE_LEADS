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

def main(): 
    # make true only when resizing aircraft. should be left false for regression
    update_regression_values = False  
    Transport_Aircraft_Test()
    General_Aviation_Test()
    EVTOL_Aircraft_Test(update_regression_values)
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
    accepted  = np.array([[16156802.54978671,        0.        , -7824233.96896249],
                          [       0.        , 58633557.83917309,        0.        ],
                          [-7824233.96896249,        0.        , 54526483.54615998]])
                          
    MOI_error     = np.nan_to_num((MOI - accepted) / accepted)

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

    accepted  = np.array([[3092.49011892,    0.        , -278.99230136],
                          [   0.        , 5921.80746984,    0.        ],
                          [-278.99230136,    0.        , 4782.27699572]])

    MOI_error     =  np.nan_to_num((MOI - accepted) / accepted)

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
    accepted  = np.array([[ 9463.1492284 ,  -431.31503284,  -323.65112921],
       [ -431.31503284,  9992.41102398,  -101.09543924],
       [ -323.65112921,  -101.09543924, 17665.06668109]])
    MOI_error     = np.nan_to_num((MOI - accepted) / accepted)

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

if __name__ == '__main__':
    main()


