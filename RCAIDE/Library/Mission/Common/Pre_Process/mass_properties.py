# RCAIDE/Library/Missions/Common/Pre_Process/mass_properties.py
# 
# 
# Created: Mar 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  RCAIDE
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia                             import compute_aircraft_moment_of_inertia
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity                             import compute_vehicle_center_of_gravity
# ----------------------------------------------------------------------------------------------------------------------
#  mass_properties
# ----------------------------------------------------------------------------------------------------------------------  
def mass_properties(mission):
    """ This is  a  work in progress Documentation will be added before it is pushed to Master
    """

    last_tag = None    
    for segment in mission.segments:
        if segment.analyses.weights != None:
            if last_tag == None:
                weights_analysis =  segment.analyses.weights
                
                if weights_analysis.vehicle.mass_properties.takeoff == 0:
                    _ = weights_analysis.evaluate() 
                
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

                    weights_analysis.vehicle.mass_properties.operating_empty =  weights_analysis.vehicle.mass_properties.weight_breakdown.empty.total \
                                                                                + weights_analysis.vehicle.mass_properties.weight_breakdown.operational_items.total 

                    if weights_analysis.vehicle.mass_properties.payload == 0 or weights_analysis.vehicle.mass_properties.fuel == 0:
                        raise AssertionError('Payload or Fuel Weight Not Defined')

                    else:
                         if weights_analysis.vehicle.mass_properties.max_payload != 0:
                              if weights_analysis.vehicle.mass_properties.payload >weights_analysis.vehicle.mass_properties.max_payload:
                                   raise AssertionError('Prescribed payload is greater than maxmimum payload')
                         if weights_analysis.vehicle.mass_properties.max_fuel != 0:
                              if weights_analysis.vehicle.mass_properties.fuel >weights_analysis.vehicle.mass_properties.max_fuel:
                                   raise AssertionError('Prescribed fuel is greater than maxmimum fuel')

                         weights_analysis.vehicle.mass_properties.takeoff = weights_analysis.vehicle.mass_properties.operating_empty \
                                                                            + weights_analysis.vehicle.mass_properties.payload \
                                                                            + weights_analysis.vehicle.mass_properties.fuel
                         weights_analysis.vehicle.mass_properties.max_zero_fuel = weights_analysis.vehicle.mass_properties.operating_empty \
                                                                            + weights_analysis.vehicle.mass_properties.max_payload 
                         if weights_analysis.print_weight_analysis_report:
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
                            systems = weights_analysis.vehicle.mass_properties.weight_breakdown.empty.get('systems', {})
                            payload = weights_analysis.vehicle.mass_properties.weight_breakdown.get('payload', {})
                            ops = weights_analysis.vehicle.mass_properties.weight_breakdown.get('operational_items', {})

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
                else:
                    print('Takeoff Weight Prescribed skipping weight analysis')        

                if weights_analysis.vehicle.mass_properties.takeoff > weights_analysis.vehicle.mass_properties.max_takeoff:
                    print('Warning: Takeoff Weight is greater than Maximum Takeoff Weight')
                    
                # CG Location  
                if weights_analysis.settings.update_center_of_gravity:
                    CG_location, _ = compute_vehicle_center_of_gravity(weights_analysis.vehicle, update_CG= weights_analysis.settings.update_center_of_gravity)  
                 
                # Operating Aircraft MOI    
                if weights_analysis.settings.update_moment_of_inertia:
                    _, _ = compute_aircraft_moment_of_inertia(weights_analysis.vehicle, CG_location, update_MOI= weights_analysis.settings.update_moment_of_inertia)          

                last_tag = segment.tag.lower()
                if segment.analyses.aerodynamics != None:   
                    segment.analyses.aerodynamics.vehicle =  weights_analysis.vehicle
            
            else:  
                segment.analyses.weights.vehicle.mass_properties      = mission.segments[last_tag].analyses.weights.vehicle.mass_properties
                if segment.analyses.aerodynamics != None:   
                    segment.analyses.aerodynamics.vehicle = mission.segments[last_tag].analyses.aerodynamics.vehicle
               
        else:
             # Creates a Weight Analysis if there doesnt exist one
            last_tag = None    
            for segment in mission.segments:
                segment.analyses.weights         = Data()
                if last_tag == None:
                    segment.analyses.weights.vehicle = segment.analyses.geometry.vehicle
                    if segment.analyses.weights.vehicle.mass_properties.takeoff == 0:
                         if segment.analyses.weights.vehicle.mass_properties.operating_empty != 0 or segment.analyses.weights.vehicle.mass_properties.payload !=0 or segment.analyses.vehicle.mass_properties.fuel != 0:
                              segment.analyses.weights.vehicle.mass_properties.takeoff = segment.analyses.weights.vehicle.mass_properties.operating_empty \
                                                                            + segment.analyses.weights.vehicle.mass_properties.payload \
                                                                            + segment.analyses.weights.vehicle.mass_properties.fuel
                              if segment.analyses.weights.vehicle.mass_properties.takeoff > segment.analyses.weights.vehicle.mass_properties.max_takeoff:
                                    print('Warning: Takeoff Weight is greater than Maximum Takeoff Weight')
                         else:
                            if segment.analyses.weights.vehicle.mass_properties.max_takeoff != 0:
                                 segment.analyses.weights.vehicle.mass_properties.takeoff = segment.analyses.weights.vehicle.mass_properties.max_takeoff
                            else: 
                                 raise AssertionError('Specify Takeoff Weight')         
                    last_tag = segment.tag.lower()
                    if segment.analyses.aerodynamics != None:
                        segment.analyses.aerodynamics.vehicle =  segment.analyses.weights.vehicle
                else:
                    segment.analyses.weights.vehicle      = mission.segments[last_tag].analyses.weights.vehicle    
                    if segment.analyses.aerodynamics != None:   
                        segment.analyses.aerodynamics = mission.segments[last_tag].analyses.aerodynamics
    return 