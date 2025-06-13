# RCAIDE/Library/Missions/Common/Pre_Process/mass_properties.py
# 
# 
# Created: Mar 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  RCAIDE
# ---------------------------------------------------------------------------------------------------------------------- 
import numpy as np 
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia                             import compute_aircraft_moment_of_inertia
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity                             import compute_vehicle_center_of_gravity
from copy import deepcopy
# ----------------------------------------------------------------------------------------------------------------------
#  mass_properties
# ----------------------------------------------------------------------------------------------------------------------  
def mass_properties(mission):
    """ This is  a  work in progress Documentation will be added before it is pushed to Master
    """
 
    for i ,  segment in enumerate(mission.segments):
        if segment.analyses.weights != None: 
            weights_analysis =  segment.analyses.weights      

            if weights_analysis.vehicle.mass_properties.max_takeoff == None:
                raise AttributeError("Max Takeoff Weight for aircraft not defined")
            
            if weights_analysis.vehicle.mass_properties.takeoff != None:
                if weights_analysis.vehicle.mass_properties.takeoff > weights_analysis.vehicle.mass_properties.max_takeoff:
                    if i == 0: 
                        print('\n Warning: Takeoff Weight is greater than Maximum Takeoff Weight')
            else: 

                if weights_analysis.aircraft_type == None :
                    if i == 0: 
                        print('\n Warning: Takeoff Weight or Weight Analysis type not defined, using Maximum Takeoff Weight.')
                    weights_analysis.vehicle.mass_properties.takeoff = weights_analysis.vehicle.mass_properties.max_takeoff  
                
                elif weights_analysis.vehicle.mass_properties.payload == None and weights_analysis.vehicle.mass_properties.fuel == None:
                    if i == 0: 
                        print('Warning: Payload or fuel weight not defined; assuming takeoff weight is MTOW')
                    weights_analysis.vehicle.mass_properties.takeoff = weights_analysis.vehicle.mass_properties.max_takeoff
                else:
                    if weights_analysis.vehicle.mass_properties.max_fuel == None or weights_analysis.vehicle.mass_properties.max_zero_fuel == None:
                        if i == 0: 
                            print('\n Warning: Max Fuel or Max Zero Fuel not defined. Iterating to find these values.')

                        weights_analysis.vehicle.mass_properties.max_fuel =  0.477*weights_analysis.vehicle.mass_properties.max_takeoff -13455
                        weights_analysis.vehicle.mass_properties.max_zero_fuel = 0.6269*weights_analysis.vehicle.mass_properties.max_takeoff + 20505

                        max_iterations = 100
                        iteration = 0

                        # Convergence loop
                        while iteration < max_iterations:
                            if weights_analysis.vehicle.mass_properties.payload >weights_analysis.vehicle.mass_properties.max_payload:
                                raise AssertionError('Prescribed payload is greater than maxmimum payload')
            
                            weights_analysis.vehicle.mass_properties.fuel  = 0.8* weights_analysis.vehicle.mass_properties.max_fuel

                            if weights_analysis.vehicle.mass_properties.max_fuel == None: 
                                raise AttributeError('Define Maximum Fuel Weight') 

                            if weights_analysis.vehicle.mass_properties.max_payload == None:
                                raise AttributeError('Define Payload Weight of Aircraft')        
                            
                            # Run weights analysis ! 
                            _ = weights_analysis.evaluate()
                            
                            # Compute OEW 
                            weights_analysis.vehicle.mass_properties.operating_empty = weights_analysis.vehicle.mass_properties.weight_breakdown.empty.total + \
                                                                                    weights_analysis.vehicle.mass_properties.weight_breakdown.operational_items.total 
                                            
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

                            # there is an issue with logic of using computed takeoff weight vs estimate takeoff weight 
                            weights_analysis.vehicle.mass_properties.takeoff       = weights_analysis.vehicle.mass_properties.operating_empty + weights_analysis.vehicle.mass_properties.payload + weights_analysis.vehicle.mass_properties.fuel                    
                            mew_max_zero_fuel = weights_analysis.vehicle.mass_properties.operating_empty + weights_analysis.vehicle.mass_properties.max_payload
                            new_max_fuel = weights_analysis.vehicle.mass_properties.max_takeoff - weights_analysis.vehicle.mass_properties.operating_empty - weights_analysis.vehicle.mass_properties.min_payload
                            residual_max_fuel =  abs(new_max_fuel - weights_analysis.vehicle.mass_properties.max_fuel)
                            residual_max_zero_fuel = abs(mew_max_zero_fuel - weights_analysis.vehicle.mass_properties.max_zero_fuel)
                            weights_analysis.vehicle.mass_properties.max_zero_fuel = mew_max_zero_fuel
                            weights_analysis.vehicle.mass_properties.max_fuel = new_max_fuel
                            
                            iteration += 1
                            if residual_max_fuel < 10 and residual_max_zero_fuel <10:
                                break
                            else:
                                weights_analysis.vehicle.mass_properties.max_zero_fuel += residual_max_zero_fuel * 0.1
                                weights_analysis.vehicle.mass_properties.max_fuel      += residual_max_fuel * 0.1

                    else:
                        if weights_analysis.vehicle.mass_properties.payload >weights_analysis.vehicle.mass_properties.max_payload:
                            raise AssertionError('Prescribed payload is greater than maxmimum payload')
        
                        if weights_analysis.vehicle.mass_properties.fuel >weights_analysis.vehicle.mass_properties.max_fuel:
                            raise AssertionError('Prescribed fuel is greater than maxmimum fuel')

                        if weights_analysis.vehicle.mass_properties.max_fuel == None: 
                            raise AttributeError('Define Maximum Fuel Weight') 

                        if weights_analysis.vehicle.mass_properties.max_payload == None:
                            raise AttributeError('Define Payload Weight of Aircraft')        
                        
                        # Run weights analysis ! 
                        _ = weights_analysis.evaluate()
                        
                        # Compute OEW 
                        weights_analysis.vehicle.mass_properties.operating_empty = weights_analysis.vehicle.mass_properties.weight_breakdown.empty.total + \
                                                                                weights_analysis.vehicle.mass_properties.weight_breakdown.operational_items.total 
                                        
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

                        # there is an issue with logic of using computed takeoff weight vs estimate takeoff weight 
                        weights_analysis.vehicle.mass_properties.takeoff       = weights_analysis.vehicle.mass_properties.operating_empty + weights_analysis.vehicle.mass_properties.payload + weights_analysis.vehicle.mass_properties.fuel                    
                        weights_analysis.vehicle.mass_properties.max_zero_fuel = weights_analysis.vehicle.mass_properties.operating_empty + weights_analysis.vehicle.mass_properties.max_payload
                        
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
                            print(f"{'Max Takeoff Weight':<25}{weights_analysis.vehicle.mass_properties.weight_breakdown.get('max_takeoff', 0):>15.2f}")
                            print("\n===============================\n")
                    
                
                #if weights_analysis.vehicle.mass_properties.takeoff > weights_analysis.vehicle.mass_properties.max_takeoff:
                    #print('\n Warning: Takeoff Weight is greater than Maximum Takeoff Weight')

            # Compute Center of Gravity  
            if weights_analysis.settings.update_center_of_gravity:
                CG_location, _ = compute_vehicle_center_of_gravity(weights_analysis.vehicle, update_CG= weights_analysis.settings.update_center_of_gravity)  
            else:
                CG_location = weights_analysis.vehicle.mass_properties.center_of_gravity 
            
            # Compute Moment of Intertia
            if weights_analysis.settings.update_moment_of_inertia:
                _, _ = compute_aircraft_moment_of_inertia(weights_analysis.vehicle, CG_location, update_MOI= weights_analysis.settings.update_moment_of_inertia)          
 
            if segment.analyses.aerodynamics != None:   
                segment.analyses.aerodynamics.vehicle =  deepcopy(weights_analysis.vehicle) 
    return 