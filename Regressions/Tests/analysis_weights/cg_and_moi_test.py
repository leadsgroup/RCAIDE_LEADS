# Regression/scripts/Tests/analysis_weights/cg_and_moi_test.py
# 
# Created:  Oct 2024, A. Molloy
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# cg_and_moi_test.py

from RCAIDE.Framework.Core                                     import Units,  Data  
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia  import compute_aircraft_moment_of_inertia
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity  import compute_vehicle_center_of_gravity
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia  import compute_cuboid_moment_of_inertia

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
    
    # update fuel weight to 60%
    vehicle.networks.fuel.fuel_lines.fuel_line.fuel_tanks.wing_fuel_tank.fuel.mass_properties.mass = 0.6 * vehicle.networks.fuel.fuel_lines.fuel_line.fuel_tanks.wing_fuel_tank.fuel.mass_properties.mass

    # ------------------------------------------------------------------
    #   Weight Breakdown 
    # ------------------------------------------------------------------  
    weight_analysis                               = RCAIDE.Framework.Analyses.Weights.Conventional()
    weight_analysis.aircraft_type                 =  "Transport"
    weight_analysis.vehicle                       = vehicle
    weight_analysis.method                        = 'Raymer'
    weight_analysis.settings.use_max_fuel_weight  = False  
    weight_analysis.settings.cargo_doors_number   = 2
    weight_analysis.settings.cargo_doors_clamshell= True
    results                                       = weight_analysis.evaluate() 
    
    # ------------------------------------------------------------------
    #   CG Location
    # ------------------------------------------------------------------    
    CG_location, _ = compute_vehicle_center_of_gravity( weight_analysis.vehicle)  
    
    # ------------------------------------------------------------------
    #   Operating Aircraft MOI
    # ------------------------------------------------------------------    
    MOI, total_mass = compute_aircraft_moment_of_inertia(weight_analysis.vehicle, CG_location)

    # ------------------------------------------------------------------
    #   Payload MOI
    # ------------------------------------------------------------------    
    Cargo_MOI, mass =  compute_cuboid_moment_of_inertia(CG_location, 99790*Units.kg, 36.0, 3.66, 3, 0, 0, 0, CG_location)
    MOI             += Cargo_MOI
    total_mass      += mass
    
    print(weight_analysis.vehicle.tag + ' Moment of Intertia')
    print(MOI) 
    accepted  = np.array([[34511560.549699254, 2607978.8783662403, 3264942.719311941],
                          [2607978.8783662403, 44472082.67654222, -1.4551915228366852e-11],
                          [3264942.719311941, -1.4551915228366852e-11,62610504.49712051]]) 
    MOI_error     = MOI - accepted

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
    weight_analysis               = RCAIDE.Framework.Analyses.Weights.Conventional() 
    weight_analysis.vehicle       = general_aviation_setup() 
    weight_analysis.method        = 'FLOPS'
    weight_analysis.aircraft_type = 'General_Aviation'
    results                       = weight_analysis.evaluate() 
    
    # ------------------------------------------------------------------
    #   CG Location
    # ------------------------------------------------------------------    
    CG_location, _ = compute_vehicle_center_of_gravity(weight_analysis.vehicle)  
    
    # ------------------------------------------------------------------
    #   Operating Aircraft MOI
    # ------------------------------------------------------------------    
    MOI, total_mass = compute_aircraft_moment_of_inertia(weight_analysis.vehicle, CG_location) 

    print(weight_analysis.vehicle.tag + ' Moment of Intertia')
    print(MOI)
     
    accepted  = np.array([[2859.945595755493, 17.36583331027025, 21.28699692497282],
                          [17.36583331027025, 3742.738737133499,    0.        ],
                          [21.28699692497282,    0.        , 2713.6487787834294]])
    
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
        assert(np.abs(v)<1e-6)   
    
    return

def EVTOL_Aircraft_Test(update_regression_values):
    vehicle = EVTOL_setup(update_regression_values)
    
    # ------------------------------------------------------------------
    #   Weight Breakdown 
    # ------------------------------------------------------------------  
    weight_analysis          = RCAIDE.Framework.Analyses.Weights.Electric()
    weight_analysis.method    = 'Physics_Based'
    weight_analysis.aircraft_type = 'VTOL'
    weight_analysis.settings.safety_factor               = 1.5    
    weight_analysis.settings.miscelleneous_weight_factor = 1.1 
    weight_analysis.settings.disk_area_factor            = 1.15
    weight_analysis.settings.max_thrust_to_weight_ratio  = 1.1
    weight_analysis.settings.max_g_load                  = 3.8
    weight_analysis.vehicle                              = vehicle
    results                                              = weight_analysis.evaluate() 
    
    # ------------------------------------------------------------------
    #   CG Location
    # ------------------------------------------------------------------    
    CG_location, _ =  compute_vehicle_center_of_gravity( weight_analysis.vehicle)  
    
    # ------------------------------------------------------------------
    #   Operating Aircraft MOI
    # ------------------------------------------------------------------    
    MOI, total_mass = compute_aircraft_moment_of_inertia(weight_analysis.vehicle, CG_location)
    
    print(weight_analysis.vehicle.tag + ' Moment of Intertia')
    print(MOI) 
    accepted  = np.array([[ 6164.802836748834, -598.6564105301388, -731.9581958856566],
                          [ -598.6564105301388,  9538.034165432851,   -96.2175642864901],
                          [ -731.9581958856566, -96.2175642864901, 14316.666830474172]])
    MOI_error     = MOI - accepted

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
        assert(np.abs(v)<1e-5) 
    
    return  

if __name__ == '__main__':
    main()


