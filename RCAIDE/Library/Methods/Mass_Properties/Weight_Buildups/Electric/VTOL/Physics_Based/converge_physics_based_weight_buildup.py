# RCAIDE/Library/Methods/Weights/Buildups/eVTOL/converge_physics_based_weight_buildup.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE
import RCAIDE 

# ----------------------------------------------------------------------------------------------------------------------
# converge_physics_based_weight_buildup
# ---------------------------------------------------------------------------------------------------------------------- 
def converge_physics_based_weight_buildup(base_vehicle,
                    print_iterations              = False,
                    miscelleneous_weight_factor   = 1.1,):
    '''Converges the maximum takeoff weight of an aircraft using the eVTOL 
    weight buildup routine.  
    
    Source:
    None
    
    Assumptions:
    None
    
    Inputs:
    vehicle                     RCAIDE Config Data Stucture
    print_iterations            Boolean Flag      
    miscelleneous_weight_factor          Factor capturing uncertainty in vehicle weight [Unitless]
    speed_of_sound:             Local Speed of Sound                           [m/s]
    max_tip_mach:               Allowable Tip Mach Number                      [Unitless]
    disk_area_factor:           Inverse of Disk Area Efficiency                [Unitless]
    max_thrust_to_weight_ratio: Allowable Thrust to Weight Ratio               [Unitless]
    safety_factor               Safety Factor in vehicle design                [Unitless]
    max_g_load                  Maximum g-forces load for certification        [UNitless]
    motor_efficiency:           Motor Efficiency                               [Unitless]
    
    Outputs:
    None
    
    Properties Used:
    N/A
    '''
    

    weight_analysis          = RCAIDE.Framework.Analyses.Weights.Electric()
    weight_analysis.vehicle  = base_vehicle
    weight_analysis.settings.miscelleneous_weight_factor =  miscelleneous_weight_factor
    weight_analysis.aircraft_type = 'VTOL'
    weight_analysis.method = 'Physics_Based'
    weight_analysis.settings.safety_factor = 1.5    # CHECK THIS VALUE
    weight_analysis.settings.disk_area_factor = 1.15
    weight_analysis.settings.max_thrust_to_weight_ratio = 1.1
    weight_analysis.settings.max_g_load = 3.8
    breakdown                = weight_analysis.evaluate() 
    build_up_mass            = breakdown.total    
    diff                     = weight_analysis.vehicle.mass_properties.max_takeoff - build_up_mass
    iterations               = 0
    
    while(abs(diff)>1):
        weight_analysis.vehicle.mass_properties.max_takeoff = weight_analysis.vehicle.mass_properties.max_takeoff - diff 
        breakdown      = weight_analysis.evaluate()         
        build_up_mass  = breakdown.total    
        diff           = weight_analysis.vehicle.mass_properties.max_takeoff - build_up_mass 
        iterations     += 1
        if print_iterations:
            print(round(diff,3))
        if iterations == 100:
            print('Weight convergence failed!')
            return False 
    print('Converged MTOW = ' + str(round(weight_analysis.vehicle.mass_properties.max_takeoff)) + ' kg') 
    
    return weight_analysis.vehicle , breakdown
