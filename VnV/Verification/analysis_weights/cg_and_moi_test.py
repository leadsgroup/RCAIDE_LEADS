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
import sys   
import os

sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))

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
    CG_location, _, _ = compute_vehicle_center_of_gravity(vehicle)  

    # ------------------------------------------------------------------
    #   Operating Aircraft MOI
    # ------------------------------------------------------------------    
    MOI  = compute_vehicle_moment_of_inertia(vehicle, CG_location) 

    print(vehicle.tag + ' Moment of Inertia')
    print(MOI) 
    accepted  = np.array([[13477607.88436136,  1060786.65213009, -2303578.72179531],
                          [ 1060786.65213009, 27157084.72163714,   144715.50530858],
                          [-2303578.72179531,   144715.50530858, 26665027.80590354]])
                          
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
    CG_location, _ , _= compute_vehicle_center_of_gravity(vehicle)  

    # ------------------------------------------------------------------
    #   Operating Aircraft MOI
    # ------------------------------------------------------------------    
    MOI  = compute_vehicle_moment_of_inertia(vehicle, CG_location) 

    print(vehicle.tag + ' Moment of Inertia')
    print(MOI)

    accepted  = np.array([[3289.69032531,  -10.87334507,  -50.30969647],
                          [ -10.87334507, 3683.8647338 ,   -9.44417805],
                          [ -50.30969647,   -9.44417805, 2795.54860064]])

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
    CG_location, _ , _=  compute_vehicle_center_of_gravity(vehicle)  

    # ------------------------------------------------------------------
    #   Operating Aircraft MOI
    # ------------------------------------------------------------------    
    MOI  = compute_vehicle_moment_of_inertia(vehicle, CG_location)

    print(vehicle.tag + ' Moment of Inertia')
    print(MOI) 
    accepted  = np.array([[ 8897.68574941,  -234.00424743,  -357.809369  ],
                          [ -234.00424743, 12111.53717803,  -175.47068943],
                          [ -357.809369  ,  -175.47068943, 19414.704072  ]])
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

if __name__ == '__main__':
    main()


