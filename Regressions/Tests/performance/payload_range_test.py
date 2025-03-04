# Regression/scripts/Tests/performance_payload_range.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Core import Units , Container
from RCAIDE.Library.Methods.Performance.compute_payload_range_diagram        import compute_payload_range_diagram

# python imports     
import numpy as np  
import sys
import matplotlib.pyplot as plt  
import os
# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Embraer_190    import vehicle_setup as E190_vehicle_setup 
from NASA_X57       import vehicle_setup as X57_vehicle_setup      

# ----------------------------------------------------------------------------------------------------------------------
#  REGRESSION
# ----------------------------------------------------------------------------------------------------------------------  
def main(): 
    fuel_payload_range_res = fuel_aircraft_payload_range()
    fuel_r                 = fuel_payload_range_res.range[-1]
    fuel_r_true            = 7043315.924243388
    print('Fuel Range: ' + str(fuel_r))
    fuel_error =  abs(fuel_r - fuel_r_true) /fuel_r_true
    assert(abs(fuel_error)<1e-6)
    
    rotor_type  = ['Blade_Element_Momentum_Theory_Helmholtz', 'Actuator_Disk']
    electric_r_truth = [37039.99999999999, 37039.99999999999]
    for i in range(len(rotor_type)):
        electric_payload_range_res = electric_aircraft_payload_range(rotor_type[i])       
        electric_r         =  electric_payload_range_res.range[-1]
        print('Electric Range: ' + str(electric_r ))
        electric_error =  abs(electric_r - electric_r_truth[i]) /electric_r_truth[i]
        assert(abs(electric_error)<1e-6)
    return 
    
    
def fuel_aircraft_payload_range():
    
    # vehicle data
    vehicle             = E190_vehicle_setup()

    # take out control surfaces to make regression run faster
    for wing in vehicle.wings:
        wing.control_surfaces  = Container()
        
    assigned_propulsors = [['starboard_propulsor','port_propulsor']]   
    altitude            = 10.668 * Units.km  
    airspeed            = 230.412 * Units['m/s']
    max_range_guess     = 1000 * Units.nmi
    

    # ------------------------------------------------------------------
    #  Weights
    weights         = RCAIDE.Framework.Analyses.Weights.Conventional()
    weights.vehicle = vehicle 

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle  = vehicle
    aerodynamics.settings.number_of_spanwise_vortices   = 5
    aerodynamics.settings.number_of_chordwise_vortices  = 2  
    
    # run payload range analysis 
    payload_range_results =  compute_payload_range_diagram(vehicle,assigned_propulsors,
                                                           weights_analysis=weights,
                                                           aerodynamics_analysis=aerodynamics,
                                                           cruise_airspeed=airspeed,
                                                           cruise_altitude=altitude, 
                                                           max_range_guess = max_range_guess)
                                   
    return payload_range_results 

def electric_aircraft_payload_range(rotor_type):
    # vehicle data
    vehicle             = X57_vehicle_setup(rotor_type)

    assigned_propulsors = [['starboard_propulsor','port_propulsor']]   
    altitude            = 15000   * Units.feet 
    airspeed            = 130 * Units.kts
    max_range_guess     = 20.   * Units.nautical_mile


    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Electric()
    weights.vehicle = vehicle

    # since OEW was not defined, we evalaute it here   
    _ = weights.evaluate()

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle  = vehicle
    aerodynamics.settings.number_of_spanwise_vortices   = 5
    aerodynamics.settings.number_of_chordwise_vortices  = 2  

    payload_range_results =  compute_payload_range_diagram(vehicle,assigned_propulsors,
                                                           weights_analysis=weights,
                                                           aerodynamics_analysis=aerodynamics,
                                                           cruise_airspeed=airspeed,
                                                           cruise_altitude=altitude, 
                                                           max_range_guess = max_range_guess)

    return payload_range_results


if __name__ == '__main__': 
    main()    
    plt.show() 