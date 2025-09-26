# RCAIDE/Library/Missions/Common/Pre_Process/mass_properties.py
# 
# 
# Created: Mar 2025, M. Clarke
# Modified: Aug 2025 S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  RCAIDE
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia                             import compute_aircraft_moment_of_inertia
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity                             import compute_vehicle_center_of_gravity
from copy import deepcopy 
# ----------------------------------------------------------------------------------------------------------------------
#  mass_properties
# ----------------------------------------------------------------------------------------------------------------------  
def mass_properties(mission):
    """Calculate and update mass properties for all mission segments.
    
    Performs weight analysis, center of gravity computation, and moment of inertia 
    calculations for each mission segment. Handles multiple analysis scenarios including
    user-defined weights, MTOW-based calculations, and iterative weight convergence.
    
    Parameters
    ----------
    mission : RCAIDE.Framework.Mission
        Mission object containing segments with weight analysis requirements.
        
    Raises
    ------
    AttributeError
        If max_takeoff weight is not defined.
    AssertionError
        If payload exceeds max_payload or fuel exceeds max_fuel.
        
    Notes
    -----
    The function operates in three main modes:
    
    1. **Pre-defined weights**: Uses existing takeoff weight if provided
    2. **Simple MTOW**: Falls back to MTOW if aircraft_type undefined
    3. **Full analysis**: Performs complete weight buildup with iterations
    
    Algorithm Flow
    ~~~~~~~~~~~~~~
    
    .. code-block:: text
    
        For each segment:
        ├── Check if weights analysis exists
        ├── Validate MTOW defined (required)
        ├── If takeoff weight defined → Skip analysis
        ├── Else if aircraft_type undefined → Use MTOW
        ├── Else if no payload/fuel → Use MTOW  
        ├── Else → Perform weight analysis:
        │   ├── Check payload/fuel limits
        │   ├── If max_fuel/max_zero_fuel undefined:
        │   │   └── Iterate to convergence (max 100 iterations)
        │   │       ├── Initial guess from regression
        │   │       ├── Evaluate weights
        │   │       ├── Apply corrections
        │   │       └── Check convergence (<10 kg residual)
        │   └── Else: Single evaluation
        ├── Update CG if requested
        ├── Update MOI if requested
        └── Copy vehicle to aerodynamics
    
    Weight Equations
    ~~~~~~~~~~~~~~~~
    
    .. math::
    
        W_{takeoff} = W_{OEW} + W_{payload} + W_{fuel}
        
        W_{OEW} = W_{empty} + W_{operational}
        
        W_{max\\_zero\\_fuel} = W_{OEW} + W_{max\\_payload}
        
        W_{max\\_fuel} = W_{MTOW} - W_{OEW} - W_{min\\_payload}
    
    Initial Regression Estimates
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    When max_fuel or max_zero_fuel undefined:
    
    .. math::
    
        W_{max\\_fuel}^{(0)} = 0.477 \\cdot W_{MTOW} - 13455
        
        W_{max\\_zero\\_fuel}^{(0)} = 0.6269 \\cdot W_{MTOW} + 20505
    """
 
    for i ,  segment in enumerate(mission.segments):
        if segment.analyses.weights != None: 
            mass_properties_preprocess_routine(i, segment.analyses.weights)
             
            if segment.analyses.aerodynamics != None:   
                segment.analyses.aerodynamics.vehicle =  deepcopy(segment.analyses.weights.vehicle)
    
            if segment.analyses.stability != None:   
                segment.analyses.stability.vehicle =  deepcopy(segment.analyses.weights.vehicle)                 
        else:
            # If there is no analysis defined, it copies over the vehicle from the geometry analysis
            segment.analyses.weights = RCAIDE.Framework.Analyses.Weights.Weights()
            segment.analyses.weights.vehicle = segment.analyses.geometry.vehicle
            if segment.analyses.weights.vehicle.mass_properties.takeoff == None:
                segment.analyses.weights.vehicle.mass_properties.takeoff = segment.analyses.weights.vehicle.mass_properties.max_takeoff
    return 

def mass_properties_preprocess_routine(i, weights_analysis): 
    if weights_analysis.vehicle.mass_properties.max_takeoff == None:
        # For all weights analysis a maximum take off weight needs to be defined by the user
        raise AttributeError("Max Takeoff Weight for aircraft not defined")
    
    if weights_analysis.vehicle.mass_properties.takeoff != None:
        # If takeoff weight is defined the weight analysis is skipped it is assumed that a weight build up is not needed. 
        if weights_analysis.vehicle.mass_properties.takeoff > weights_analysis.vehicle.mass_properties.max_takeoff:
            # Lets the user know that the takeoff weight is greater than the maximum takeoff weight defined. Will still continue with simulation
            if i == 0: 
                print('\n Warning: Takeoff Weight is greater than Maximum Takeoff Weight')
    else: 

        if weights_analysis.aircraft_type == None :
            # If an aircraft type is not defined analysis cannot be performed, and the maximum take off weight will be assumed to be the takeoff weight
            if i == 0: 
                print('\n Warning: Takeoff Weight or Weight Analysis type not defined, using Maximum Takeoff Weight.')
            weights_analysis.vehicle.mass_properties.takeoff = weights_analysis.vehicle.mass_properties.max_takeoff  
        
        elif weights_analysis.vehicle.mass_properties.payload == None and weights_analysis.vehicle.mass_properties.fuel == None:
            # Without either fuel on board or payload on board, takeoff weight cannot be computed thus the takeoff weight is assumed to be max takeoff
            if i == 0: 
                print('Warning: Payload or fuel weight not defined; assuming takeoff weight is MTOW')
            weights_analysis.vehicle.mass_properties.takeoff = weights_analysis.vehicle.mass_properties.max_takeoff
        else:
            if weights_analysis.vehicle.mass_properties.payload >weights_analysis.vehicle.mass_properties.max_payload:
                        raise AssertionError('Prescribed payload is greater than maxmimum payload')

            if weights_analysis.vehicle.mass_properties.max_fuel == None or weights_analysis.vehicle.mass_properties.max_zero_fuel == None:
                # Before proceeding to the weight buildups, the buildups need either the max fuel capacity or the max zero fuel to compute OEW
                if i == 0: 
                    print('\n Warning: Max Fuel or Max Zero Fuel not defined. Iterating to find these values.')
                # Inital guess for max fuel and max zero fuel based on regressional analysis which use max takeoff weight of the aircraft
                weights_analysis.vehicle.mass_properties.max_fuel =  0.477*weights_analysis.vehicle.mass_properties.max_takeoff -13455
                weights_analysis.vehicle.mass_properties.max_zero_fuel = 0.6269*weights_analysis.vehicle.mass_properties.max_takeoff + 20505

                max_iterations = 100
                iteration = 0

                # Convergence loop
                while iteration < max_iterations:                               
                    # Run weights analysis ! 
                    _ = weights_analysis.evaluate()
                    
                    # Compute OEW 
                    weights_analysis.vehicle.mass_properties.operating_empty = weights_analysis.vehicle.mass_properties.weight_breakdown.empty.total + \
                                                                            weights_analysis.vehicle.mass_properties.weight_breakdown.operational_items.total 
                                    
                    # Apply Correction Factors if any
                    apply_correction_factors(weights_analysis)

                    weights_analysis.vehicle.mass_properties.takeoff       = weights_analysis.vehicle.mass_properties.operating_empty + weights_analysis.vehicle.mass_properties.payload + weights_analysis.vehicle.mass_properties.fuel                    
                    mew_max_zero_fuel                                      = weights_analysis.vehicle.mass_properties.operating_empty + weights_analysis.vehicle.mass_properties.max_payload
                    new_max_fuel                                           = weights_analysis.vehicle.mass_properties.max_takeoff - weights_analysis.vehicle.mass_properties.operating_empty - weights_analysis.vehicle.mass_properties.min_payload
                    residual_max_fuel                                      =  abs(new_max_fuel - weights_analysis.vehicle.mass_properties.max_fuel)
                    residual_max_zero_fuel                                 = abs(mew_max_zero_fuel - weights_analysis.vehicle.mass_properties.max_zero_fuel)
                    weights_analysis.vehicle.mass_properties.max_zero_fuel = mew_max_zero_fuel
                    weights_analysis.vehicle.mass_properties.max_fuel      = new_max_fuel
                    
                    iteration += 1
                    if residual_max_fuel < 10 and residual_max_zero_fuel <10:
                        break
                    else:
                        weights_analysis.vehicle.mass_properties.max_zero_fuel += residual_max_zero_fuel * 0.1
                        weights_analysis.vehicle.mass_properties.max_fuel      += residual_max_fuel * 0.1

            else:
                if weights_analysis.vehicle.mass_properties.fuel >weights_analysis.vehicle.mass_properties.max_fuel:
                    raise AssertionError('Prescribed fuel is greater than maxmimum fuel')   
                
                # Run weights analysis ! 
                _ = weights_analysis.evaluate()
                
                # Compute OEW 
                weights_analysis.vehicle.mass_properties.operating_empty = weights_analysis.vehicle.mass_properties.weight_breakdown.empty.total + \
                                                                           weights_analysis.vehicle.mass_properties.weight_breakdown.operational_items.total 
                                
                # Apply correction factors  if any
                apply_correction_factors(weights_analysis)

                # Compute takeoff weight and max zero fuel weight
                weights_analysis.vehicle.mass_properties.takeoff       = weights_analysis.vehicle.mass_properties.operating_empty \
                                                                       + weights_analysis.vehicle.mass_properties.payload\
                                                                       + weights_analysis.vehicle.mass_properties.fuel                    
                weights_analysis.vehicle.mass_properties.max_zero_fuel = weights_analysis.vehicle.mass_properties.operating_empty\
                                                                       + weights_analysis.vehicle.mass_properties.max_payload
                
            
            if weights_analysis.print_weight_analysis_report:
                if i == 0: 
                    print("\nPerforming Weights Analysis")
                    print("--------------------------------------------------------")
                    print("Propulsion Architecture:", weights_analysis.propulsion_architecture)
                    print("Aircraft Type          :", weights_analysis.aircraft_type)
                    print("Method                 :", weights_analysis.method)
                    def print_section(title, data):
                        print(f"{title}")
                        print(f"{'Component':<25}{'Weight (kg)':>15}")
                        print("-" * 40)
                        for item, value in data.items():
                            if item != 'total':
                                print(f"{item.replace('_', ' ').title():<25}{value:>15.2f}")
                        print("-" * 40)
                        print(f"{'Total':<25}{data.get('total', 0):>15.2f}\n")

                    print("\n=== WEIGHT BREAKDOWN REPORT ===\n")

                    # Extract data
                    structural = weights_analysis.vehicle.mass_properties.weight_breakdown.empty.get('structural', {})
                    propulsion = weights_analysis.vehicle.mass_properties.weight_breakdown.empty.get('propulsion', {})
                    systems    = weights_analysis.vehicle.mass_properties.weight_breakdown.empty.get('systems', {})
                    payload    = weights_analysis.vehicle.mass_properties.weight_breakdown.get('payload', {})
                    ops        = weights_analysis.vehicle.mass_properties.weight_breakdown.get('operational_items', {})

                    # Print sections
                    print_section("Structural Components:", structural)
                    print_section("Propulsion Components:", propulsion)
                    print_section("Systems:", systems)
                    print_section("Payload Breakdown:", payload)
                    print_section("Operational Items Breakdown:", ops)

                    # Overall Summary
                    print("Overall Summary:")
                    print(f"{'Metric':<25}{'Weight (kg)':>15}")
                    print("-" * 40)
                    print(f"{'Operating Empty Weight':<25}{weights_analysis.vehicle.mass_properties.operating_empty:>15.2f}")
                    print(f"{'Payload Weight':<25}{weights_analysis.vehicle.mass_properties.payload:>15.2f}")
                    print(f"{'Fuel Weight':<25}{weights_analysis.vehicle.mass_properties.fuel:>15.2f}")
                    print(f"{'Takeoff Weight':<25}{weights_analysis.vehicle.mass_properties.takeoff:>15.2f}")
                    print(f"{'Zero Fuel Weight':<25}{weights_analysis.vehicle.mass_properties.weight_breakdown.get('zero_fuel_weight', 0):>15.2f}")
                    print(f"{'Max Takeoff Weight':<25}{weights_analysis.vehicle.mass_properties.max_takeoff:>15.2f}")
                    print("\n===============================\n")
    
    # Compute Center of Gravity  
    CG ,_, _ = compute_vehicle_center_of_gravity(weights_analysis.vehicle, update_center_of_gravity= weights_analysis.settings.update_center_of_gravity) 
    
    # Compute Moment of Intertia
    _, _ = compute_aircraft_moment_of_inertia(weights_analysis.vehicle, CG, update_moment_of_inertia= weights_analysis.settings.update_moment_of_inertia)          

    
def apply_correction_factors(weights_analysis):
    # Apply correction factors  
    for tag, item in weights_analysis.settings.weight_correction_factors.items():
        if tag == 'empty':
            for subtag, subitem in weights_analysis.settings.weight_correction_factors[tag].items():
                for subsubtag, subsubitem in weights_analysis.settings.weight_correction_factors[tag][subtag].items():
                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag].total  -= weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag].total  -= weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    weights_analysis.vehicle.mass_properties.operating_empty -= weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag] *= subsubitem
                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag].total  += weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag].total  += weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    weights_analysis.vehicle.mass_properties.operating_empty += weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
        elif tag == 'operational_items':
            for subtag, subitem in weights_analysis.settings.weight_correction_factors[tag].items():
                weights_analysis.vehicle.mass_properties.weight_breakdown[tag].total  -= subitem
                weights_analysis.vehicle.mass_properties.operating_empty -= subitem
                weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag] *= subitem
                weights_analysis.vehicle.mass_properties.weight_breakdown[tag].total  += weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag]
                weights_analysis.vehicle.mass_properties.operating_empty += weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag]

    for tag, _ in weights_analysis.settings.weight_correction_additions.items():
        if tag == 'empty':
            for subtag, subitem in weights_analysis.settings.weight_correction_additions[tag].items():
                for subsubtag, subsubitem in weights_analysis.settings.weight_correction_additions[tag][subtag].items():
                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag] = subsubitem
                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag].total += subsubitem
                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag].total  += subsubitem
                    weights_analysis.vehicle.mass_properties.operating_empty += subsubitem
        elif tag == 'operational_items':
            for subtag, subitem in weights_analysis.settings.weight_correction_additions[tag].items():
                weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag] = subitem
                weights_analysis.vehicle.mass_properties.weight_breakdown[tag].total  += subitem
                weights_analysis.vehicle.mass_properties.operating_empty += subitem
    return
                                    