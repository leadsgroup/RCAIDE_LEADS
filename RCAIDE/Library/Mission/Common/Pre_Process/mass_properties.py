# RCAIDE/Library/Missions/Common/Pre_Process/mass_properties.py
# 
# 
# Created: Mar 2025, M. Clarke
# Modified: Aug 2025 S. Shekar, Jan 2026 A. Molloy

# ----------------------------------------------------------------------------------------------------------------------
#  RCAIDE
# ---------------------------------------------------------------------------------------------------------------------- 
from copy import deepcopy
import RCAIDE 
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia  import compute_vehicle_moment_of_inertia
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity  import compute_vehicle_center_of_gravity 

import numpy as np
import pandas as pd
import os
import sys

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
    3. **Full analysis**: Performs complete weight buildup with iterations, if the 
          setting update_takeoff_weight is True then it updates the aircraft takeoff weight 
          with the new one based on the buildup. otherwise the takeoff weight is not adjusted
    
    To complete a weight breakdown the methods require an aircraft to have the following
    defined: MTOW, payload and/or fuel weight, aircraft method type (included in the weights 
    analysis type most times), max fuel and max zero fuel weights or neither. If
    the user desires to have the takeoff weight calculated by the weight breakdown
    used for further analyses then specify under the weight analysis settings 
    "update_takeoff_weight == True".

    Algorithm Flow
    ~~~~~~~~~~~~~~
    
    .. code-block:: text
    
        For each segment:
        ├── Check if weights analysis exists
        ├── Validate MTOW defined (required)
        ├── If aircraft_type undefined → Use MTOW for takeoff weight
        ├── Else if no payload/fuel → Use MTOW  
        ├── Else → Perform weight analysis:
        │   ├── Check payload/fuel limits
        │   ├── If max_fuel/max_zero_fuel undefined:
        │   │   └── Iterate to convergence (max 100 iterations)
        │   │       ├── Initial guess from regression
        │   │       ├── Evaluate weights
        │   │       ├── Apply corrections
        │   │       └── Check convergence (<10 kg residual)
        │   ├── Single evaluation
        |   └── Apply correction factors if specified
        ├── Update takeoff weight if requested
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
        if segment.analyses.weights == None:
            raise AssertionError('Define weights analysis method')
        else: 
            if i ==0 or segment.analyses.geometry.settings.unique_geometry:
                mass_properties_preprocess_routine(segment, i) 
            else:
                use_previous_segment_pre_processed_data(mission,segment,i)   
    return 

def mass_properties_preprocess_routine(segment, i = 0):
    analyses         = segment.analyses
    weights_analysis = analyses.weights 

    # ---------------------------------------------------------------------------------------------------------------------------
    # STEP 1:  Pre-checks for weights analysis 
    # ---------------------------------------------------------------------------------------------------------------------------      
    if analyses.vehicle.mass_properties.max_takeoff == None:
        # For all weights analysis a maximum take off weight needs to be defined by the user
        raise AttributeError("Max Takeoff Weight for aircraft not defined")
    # orig_takeoff_weight = analyses.vehicle.mass_properties.takeoff
    
    if weights_analysis.aircraft_type == None:
        # If an aircraft type is not defined analysis cannot be performed, and the maximum take off weight will be assumed to be the takeoff weight 
        print('\n Warning: Weight Analysis type not defined')
        if analyses.vehicle.mass_properties.takeoff == None:
            print('\n Using Maximum Takeoff Weight')
            analyses.vehicle.mass_properties.takeoff = analyses.vehicle.mass_properties.max_takeoff  
    
    elif analyses.vehicle.mass_properties.takeoff == None and analyses.vehicle.mass_properties.payload == None and analyses.vehicle.mass_properties.fuel == None:
        # Without either fuel on board or payload on board, takeoff weight cannot be computed thus the takeoff weight is assumed to be max takeoff 
        print('Warning: Payload or fuel weight not defined; assuming takeoff weight is MTOW')
        analyses.vehicle.mass_properties.takeoff = analyses.vehicle.mass_properties.max_takeoff
    elif weights_analysis.settings.run_weights_analysis:
    
    # ---------------------------------------------------------------------------------------------------------------------------
    # STEP 2: Run weights analysis 
    # ---------------------------------------------------------------------------------------------------------------------------         
        if analyses.vehicle.mass_properties.payload > analyses.vehicle.mass_properties.max_payload:
            print('Warning:Prescribed payload weight is greater than maxmimum payload weight')
        if weights_analysis.settings.iterate_mtow:
            diff = 999
            max_iterations = 1000
            mtow_iterations = 0 
            while abs(diff)>0.00005 and mtow_iterations<max_iterations:
                if analyses.vehicle.mass_properties.max_zero_fuel == None:
                    max_zero_fuel_flag = True
                    # Before proceeding to the weight buildups, the buildups need either the max fuel capacity or the max zero fuel to compute OEW 
                    if i == 0 and mtow_iterations == 0:
                        print('\n Warning: Max Fuel or Max Zero Fuel not defined. Iterating to find these values.')
                    # Inital guess for max fuel and max zero fuel based on regressional analysis which use max takeoff weight of the aircraft
                    compute_max_fuel = False
                    if analyses.vehicle.mass_properties.max_fuel == None:
                        analyses.vehicle.mass_properties.max_fuel =  0.477*analyses.vehicle.mass_properties.max_takeoff -13455
                        compute_max_fuel = True
                    analyses.vehicle.mass_properties.max_zero_fuel = 0.6269*analyses.vehicle.mass_properties.max_takeoff + 20505
                    
                
                    iteration = 0

                    # Convergence loop
                    while iteration < max_iterations:                               
                        # Run weights analysis ! 
                        _ = weights_analysis.evaluate(analyses.vehicle)
                        
                        # Compute OEW
                        if weights_analysis.settings.overwrite_operating_empty_weight: 
                            analyses.vehicle.mass_properties.operating_empty = analyses.vehicle.mass_properties.weight_breakdown.empty.total +  analyses.vehicle.mass_properties.weight_breakdown.operational_items.total 
                                        
                        # Apply Correction Factors if any
                        apply_correction_factors(analyses)
                        apply_component_weights(analyses)

                        analyses.vehicle.mass_properties.takeoff         = analyses.vehicle.mass_properties.operating_empty + analyses.vehicle.mass_properties.payload + analyses.vehicle.mass_properties.fuel                    
                        mew_max_zero_fuel                                = analyses.vehicle.mass_properties.operating_empty + analyses.vehicle.mass_properties.max_payload
                        residual_max_zero_fuel                           = abs(mew_max_zero_fuel - analyses.vehicle.mass_properties.max_zero_fuel)
                        analyses.vehicle.mass_properties.max_zero_fuel   = mew_max_zero_fuel
                        
                        residual_max_fuel = 0
                        if compute_max_fuel:
                            new_max_fuel  = analyses.vehicle.mass_properties.max_takeoff - analyses.vehicle.mass_properties.operating_empty - analyses.vehicle.mass_properties.min_payload
                            residual_max_fuel =  abs(new_max_fuel - analyses.vehicle.mass_properties.max_fuel) 
                            analyses.vehicle.mass_properties.max_fuel = new_max_fuel                
                        
                        iteration += 1
                        if residual_max_fuel < 1 and residual_max_zero_fuel <1:
                            break
                        else:
                            analyses.vehicle.mass_properties.max_zero_fuel += residual_max_zero_fuel * 0.1
                            if compute_max_fuel: 
                                analyses.vehicle.mass_properties.max_fuel      += residual_max_fuel * 0.1 

                _ = weights_analysis.evaluate(analyses.vehicle) 

                if analyses.vehicle.mass_properties.payload > analyses.vehicle.mass_properties.max_payload:
                    print('Warning: Computed payload weight is greater than maxmimum payload weight')        
                
                # Compute OEW 
              
                analyses.vehicle.mass_properties.operating_empty = analyses.vehicle.mass_properties.weight_breakdown.empty.total \
                                                                            +  analyses.vehicle.mass_properties.weight_breakdown.operational_items.total 
                                
                # Apply correction factors  if any
                apply_correction_factors(analyses)
                if i == 0:
                    apply_component_weights(analyses)

                new_mtow,diff = iterate_for_mtow(analyses.vehicle.mass_properties.max_takeoff, 
                                analyses.vehicle.mass_properties.operating_empty, 
                                analyses.vehicle.mass_properties.max_payload,
                                analyses.vehicle.mass_properties.max_fuel,
                                analyses.vehicle)
                analyses.vehicle.mass_properties.max_takeoff = new_mtow

                if abs(diff)> 0.00005:
                    if max_zero_fuel_flag:
                        analyses.vehicle.mass_properties.max_zero_fuel = None
                    if compute_max_fuel:
                        analyses.vehicle.mass_properties.max_fuel = None
                mtow_iterations += 1
            
            if mtow_iterations>max_iterations:
                raise Exception('MTOW DIDNT CONVERGE')
            
        else:
            if analyses.vehicle.mass_properties.max_zero_fuel == None:
            # Before proceeding to the weight buildups, the buildups need either the max fuel capacity or the max zero fuel to compute OEW 
                if i == 0:
                    print('\n Warning: Max Fuel or Max Zero Fuel not defined. Iterating to find these values.')
                # Inital guess for max fuel and max zero fuel based on regressional analysis which use max takeoff weight of the aircraft
                compute_max_fuel = False
                if analyses.vehicle.mass_properties.max_fuel == None:
                    analyses.vehicle.mass_properties.max_fuel =  0.477*analyses.vehicle.mass_properties.max_takeoff -13455
                    compute_max_fuel = True
                analyses.vehicle.mass_properties.max_zero_fuel = 0.6269*analyses.vehicle.mass_properties.max_takeoff + 20505
                
                max_iterations = 100
                iteration = 0

                # Convergence loop
                while iteration < max_iterations:                               
                    # Run weights analysis ! 
                    _ = weights_analysis.evaluate(analyses.vehicle)
                    
                    # Compute OEW
                    if weights_analysis.settings.overwrite_operating_empty_weight: 
                        analyses.vehicle.mass_properties.operating_empty = analyses.vehicle.mass_properties.weight_breakdown.empty.total +  analyses.vehicle.mass_properties.weight_breakdown.operational_items.total 
                                    
                    # Apply Correction Factors if any
                    apply_correction_factors(analyses)
                    apply_component_weights(analyses)

                    analyses.vehicle.mass_properties.takeoff         = analyses.vehicle.mass_properties.operating_empty + analyses.vehicle.mass_properties.payload + analyses.vehicle.mass_properties.fuel                    
                    mew_max_zero_fuel                                = analyses.vehicle.mass_properties.operating_empty + analyses.vehicle.mass_properties.max_payload
                    residual_max_zero_fuel                           = abs(mew_max_zero_fuel - analyses.vehicle.mass_properties.max_zero_fuel)
                    analyses.vehicle.mass_properties.max_zero_fuel   = mew_max_zero_fuel
                    
                    residual_max_fuel = 0
                    if compute_max_fuel:
                        new_max_fuel  = analyses.vehicle.mass_properties.max_takeoff - analyses.vehicle.mass_properties.operating_empty - analyses.vehicle.mass_properties.min_payload
                        residual_max_fuel =  abs(new_max_fuel - analyses.vehicle.mass_properties.max_fuel) 
                        analyses.vehicle.mass_properties.max_fuel = new_max_fuel                
                    
                    iteration += 1
                    if residual_max_fuel < 1 and residual_max_zero_fuel <1:
                        break
                    else:
                        analyses.vehicle.mass_properties.max_zero_fuel += residual_max_zero_fuel * 0.1
                        if compute_max_fuel: 
                            analyses.vehicle.mass_properties.max_fuel      += residual_max_fuel * 0.1 

            
            
            _ = weights_analysis.evaluate(analyses.vehicle) 

            if analyses.vehicle.mass_properties.payload > analyses.vehicle.mass_properties.max_payload:
                print('Warning: Computed payload weight is greater than maxmimum payload weight')        
            
            # Compute OEW 
            if weights_analysis.settings.overwrite_operating_empty_weight: 
                analyses.vehicle.mass_properties.operating_empty = analyses.vehicle.mass_properties.weight_breakdown.empty.total \
                                                                        +  analyses.vehicle.mass_properties.weight_breakdown.operational_items.total 
                            
            # Apply correction factors  if any
            apply_correction_factors(analyses)
            if i == 0:
                apply_component_weights(analyses)
        if (analyses.vehicle.mass_properties.fuel  == 0 or analyses.vehicle.mass_properties.fuel is None) and weights_analysis.propulsion_architecture != 'Electric':
            ('Fuel Weight for the mission is not defned. Filling up the airplace till max takeoff weight')     
            analyses.vehicle.mass_properties.fuel     = analyses.vehicle.mass_properties.max_takeoff-\
                                                        (analyses.vehicle.mass_properties.operating_empty + analyses.vehicle.mass_properties.payload)


        # Compute takeoff weight and max zero fuel weight 
        if analyses.vehicle.mass_properties.takeoff == None:
            analyses.vehicle.mass_properties.takeoff = analyses.vehicle.mass_properties.operating_empty \
                                                                + analyses.vehicle.mass_properties.payload\
                                                                + analyses.vehicle.mass_properties.fuel    
        elif i == 0:
            print('\n Using user defined takeoff weight')                
        analyses.vehicle.mass_properties.max_zero_fuel = analyses.vehicle.mass_properties.operating_empty\
                                                                    + analyses.vehicle.mass_properties.max_payload 
        
        # ---------------------------------------------------------------------------------------------------------------------------
        # STEP 3: Print weight statements and apply weight factors  
        # --------------------------------------------------------------------------------------------------------------------------- 
       
        if weights_analysis.print_weight_analysis_report and type(weights_analysis) != RCAIDE.Framework.Analyses.Weights.Weights: 
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
                structural = analyses.vehicle.mass_properties.weight_breakdown.empty.get('structural', {})
                propulsion = analyses.vehicle.mass_properties.weight_breakdown.empty.get('propulsion', {})
                systems    = analyses.vehicle.mass_properties.weight_breakdown.empty.get('systems', {})
                payload    = analyses.vehicle.mass_properties.weight_breakdown.get('payload', {})
                ops        = analyses.vehicle.mass_properties.weight_breakdown.get('operational_items', {})

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
                print(f"{'Operating Empty Weight':<25}{analyses.vehicle.mass_properties.operating_empty:>15.2f}")
                print(f"{'Payload Weight':<25}{analyses.vehicle.mass_properties.payload:>15.2f}")
                print(f"{'Fuel Weight':<25}{analyses.vehicle.mass_properties.fuel:>15.2f}")
                print(f"{'Takeoff Weight':<25}{analyses.vehicle.mass_properties.takeoff:>15.2f}")
                print(f"{'Zero Fuel Weight':<25}{analyses.vehicle.mass_properties.weight_breakdown.get('zero_fuel_weight', 0):>15.2f}")
                print(f"{'Max Takeoff Weight':<25}{analyses.vehicle.mass_properties.max_takeoff:>15.2f}")
                print("\n===============================\n") 
            
        if i==0 and weights_analysis.settings.write_mass_properties:
            # -------------------------------------------------
            # File name + location 
            # -------------------------------------------------
            excel_filename = os.path.join(
                os.path.dirname(os.path.abspath(sys.argv[0])),
                os.path.splitext(os.path.basename(sys.argv[0]))[0] +
                "_weight_breakdown.xlsx"
            )

            # -------------------------------------------------
            # Collect rows
            # -------------------------------------------------
            rows = []

            # Structural
            structural = analyses.vehicle.mass_properties.weight_breakdown.empty.get("structural", {})
            for k, v in structural.items():
                if k != "total":
                    rows.append({"Section": "Structural", "Component": k.replace("_", " ").title(), "Weight (kg)": v})
            if "total" in structural:
                rows.append({"Section": "Structural", "Component": "Total", "Weight (kg)": structural["total"]})

            # Propulsion
            propulsion = analyses.vehicle.mass_properties.weight_breakdown.empty.get("propulsion", {})
            for k, v in propulsion.items():
                if k != "total":
                    rows.append({"Section": "Propulsion", "Component": k.replace("_", " ").title(), "Weight (kg)": v})
            if "total" in propulsion:
                rows.append({"Section": "Propulsion", "Component": "Total", "Weight (kg)": propulsion["total"]})

            # Systems
            systems = analyses.vehicle.mass_properties.weight_breakdown.empty.get("systems", {})
            for k, v in systems.items():
                if k != "total":
                    rows.append({"Section": "Systems", "Component": k.replace("_", " ").title(), "Weight (kg)": v})
            if "total" in systems:
                rows.append({"Section": "Systems", "Component": "Total", "Weight (kg)": systems["total"]})

            # Payload
            payload = analyses.vehicle.mass_properties.weight_breakdown.get("payload", {})
            for k, v in payload.items():
                if k != "total":
                    rows.append({"Section": "Payload", "Component": k.replace("_", " ").title(), "Weight (kg)": v})
            if "total" in payload:
                rows.append({"Section": "Payload", "Component": "Total", "Weight (kg)": payload["total"]})

            # Operational Items
            ops = analyses.vehicle.mass_properties.weight_breakdown.get("operational_items", {})
            for k, v in ops.items():
                if k != "total":
                    rows.append({"Section": "Operational Items", "Component": k.replace("_", " ").title(), "Weight (kg)": v})
            if "total" in ops:
                rows.append({"Section": "Operational Items", "Component": "Total", "Weight (kg)": ops["total"]})

            # -------------------------------------------------
            # Summary
            # -------------------------------------------------
            rows.extend([
                {"Section": "Summary", "Component": "Operating Empty Weight", "Weight (kg)": analyses.vehicle.mass_properties.operating_empty},
                {"Section": "Summary", "Component": "Payload Weight",         "Weight (kg)": analyses.vehicle.mass_properties.payload},
                {"Section": "Summary", "Component": "Fuel Weight",            "Weight (kg)": analyses.vehicle.mass_properties.fuel},
                {"Section": "Summary", "Component": "Zero Fuel Weight",       "Weight (kg)": analyses.vehicle.mass_properties.weight_breakdown.get("zero_fuel_weight", 0)},
                {"Section": "Summary", "Component": "Takeoff Weight",         "Weight (kg)": analyses.vehicle.mass_properties.takeoff},
                {"Section": "Summary", "Component": "Max Takeoff Weight",     "Weight (kg)": analyses.vehicle.mass_properties.max_takeoff},
            ])

            # -------------------------------------------------
            # Write Excel
            # -------------------------------------------------
            df = pd.DataFrame(rows)

            with pd.ExcelWriter(excel_filename, engine="openpyxl") as writer:
                df.to_excel(writer, sheet_name="Weight Breakdown", index=False)

            print(f"Weight breakdown written to Excel:\n  {excel_filename}")
    
    # ---------------------------------------------------------------------------------------------------------------------------     
    #  STEP 5: Compute Center of Gravity   
    # --------------------------------------------------------------------------------------------------------------------------- 
    if weights_analysis.settings.run_center_of_gravity_analysis:
        centre_of_gravity_df = pd.DataFrame(columns=[
        "Component",
        "Mass (kg)",
        "CG x (m)",
        "CG y (m)",
        "CG z (m)"
        ])
        if i != 0:
            verbose_flag = False
        else:
            verbose_flag = weights_analysis.print_weight_analysis_report
        _ ,_, _, centre_of_gravity_df = compute_vehicle_center_of_gravity(analyses.vehicle,centre_of_gravity_df,
                                                overwrite_center_of_gravity =  weights_analysis.settings.run_center_of_gravity_analysis ,
                                                segment=segment,
                                                verbose=verbose_flag)  

        if i==0 and weights_analysis.settings.write_mass_properties:
            # Centre of Gravity sheet
            with pd.ExcelWriter(excel_filename, engine="openpyxl",mode="a",if_sheet_exists="replace") as writer:
                centre_of_gravity_df.to_excel(writer,sheet_name="Centre of Gravity",index=False)
            print(f"CG breakdown written to Excel:\n  {excel_filename}")
        analyses.vehicle.mass_properties.center_of_gravity_breakdown = centre_of_gravity_df

    # ---------------------------------------------------------------------------------------------------------------------------         
    # STEP 6: Compute Moment of Inertia 
    # --------------------------------------------------------------------------------------------------------------------------- 
    if weights_analysis.settings.run_moments_of_inertia_analysis:
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
        overwrite_MOI = False
        tensor = analyses.vehicle.mass_properties.moments_of_inertia.tensor
        if np.all(tensor == 0):
            overwrite_MOI = True
        if i != 0:
            verbose_flag = False
        else:
            verbose_flag = weights_analysis.print_weight_analysis_report
        _ ,moment_of_inertia_df = compute_vehicle_moment_of_inertia(analyses.vehicle,moment_of_inertia_df,
                                            overwrite_moment_of_intertia = overwrite_MOI,
                                            segment=segment,
                                            verbose=verbose_flag) 
        if i==0 and weights_analysis.settings.write_mass_properties:
            # Centre of Gravity sheet
            with pd.ExcelWriter(excel_filename, engine="openpyxl",mode="a",if_sheet_exists="replace") as writer:
                moment_of_inertia_df.to_excel(writer,sheet_name="Moment of Inertia",index=False)
            print(f"MOI breakdown written to Excel:\n  {excel_filename}")
    
    
def apply_correction_factors(analyses): 
    weights_analysis = analyses.weights
    # Apply correction factors  
    for tag, item in weights_analysis.settings.weight_correction_factors.items():
        if tag == 'empty':
            for subtag, subitem in weights_analysis.settings.weight_correction_factors[tag].items():
                for subsubtag, subsubitem in weights_analysis.settings.weight_correction_factors[tag][subtag].items():
                    analyses.vehicle.mass_properties.weight_breakdown[tag].total  -= analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    analyses.vehicle.mass_properties.weight_breakdown[tag][subtag].total  -= analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    analyses.vehicle.mass_properties.operating_empty -= analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag] *= subsubitem
                    analyses.vehicle.mass_properties.weight_breakdown[tag][subtag].total  += analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    analyses.vehicle.mass_properties.weight_breakdown[tag].total  += analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                    analyses.vehicle.mass_properties.operating_empty += analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
        elif tag == 'operational_items':
            for subtag, subitem in weights_analysis.settings.weight_correction_factors[tag].items():
                analyses.vehicle.mass_properties.weight_breakdown[tag].total  -= subitem
                analyses.vehicle.mass_properties.operating_empty -= subitem
                analyses.vehicle.mass_properties.weight_breakdown[tag][subtag] *= subitem
                analyses.vehicle.mass_properties.weight_breakdown[tag].total  += analyses.vehicle.mass_properties.weight_breakdown[tag][subtag]
                analyses.vehicle.mass_properties.operating_empty += analyses.vehicle.mass_properties.weight_breakdown[tag][subtag]

    for tag, _ in weights_analysis.settings.weight_correction_additions.items():
        if tag == 'empty':
            for subtag, subitem in weights_analysis.settings.weight_correction_additions[tag].items():
                for subsubtag, subsubitem in weights_analysis.settings.weight_correction_additions[tag][subtag].items():
                    analyses.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag] = subsubitem
                    analyses.vehicle.mass_properties.weight_breakdown[tag][subtag].total += subsubitem
                    analyses.vehicle.mass_properties.weight_breakdown[tag].total  += subsubitem
                    analyses.vehicle.mass_properties.operating_empty += subsubitem
        elif tag == 'operational_items':
            for subtag, subitem in weights_analysis.settings.weight_correction_additions[tag].items():
                analyses.vehicle.mass_properties.weight_breakdown[tag][subtag] = subitem
                analyses.vehicle.mass_properties.weight_breakdown[tag].total  += subitem
                analyses.vehicle.mass_properties.operating_empty += subitem
    return

def apply_component_weights(analyses):
    weight_correction_factors = analyses.weights.settings.weight_correction_factors
    for key in analyses.vehicle.keys():
        if key =='wings':
            for wing in analyses.vehicle.wings:
                if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
                    if hasattr(weight_correction_factors.empty.structural, 'wing'):
                        wing.mass_properties.mass *= weight_correction_factors.empty.structural.wing
                if isinstance(wing, RCAIDE.Library.Components.Wings.Horizontal_Tail):
                    if hasattr(weight_correction_factors.empty.structural, 'empennage'):
                        wing.mass_properties.mass *= weight_correction_factors.empty.structural.empennage
                if isinstance(wing, RCAIDE.Library.Components.Wings.Vertical_Tail):
                    if hasattr(weight_correction_factors.empty.structural, 'empennage'):
                        wing.mass_properties.mass *= weight_correction_factors.empty.structural.empennage
        elif key == 'fuselages':
            for fuselage in analyses.vehicle.fuselages:
                if isinstance(fuselage, RCAIDE.Library.Components.Fuselages.Fuselage):
                    if hasattr(weight_correction_factors.empty.structural, 'fuselage'):
                        fuselage.mass_properties.mass *= weight_correction_factors.empty.structural.fuselage
        elif key == 'networks':
            for network in analyses.vehicle.networks:
                for propulsor in network.propulsors: 
                    if hasattr(weight_correction_factors.empty.structural, 'nacelle'):
                        propulsor.nacelle.mass_properties.mass *= weight_correction_factors.empty.structural.nacelle
                    if hasattr(weight_correction_factors.empty.propulsion, 'engines'):
                        propulsor.mass_properties.mass *= weight_correction_factors.empty.propulsion.engines
                # for fuel_line in network.fuel_lines:
                #     for converter in fuel_line.converters:
                #         if isinstance(converter,RCAIDE.Library.Components.Powertrain.Converters.Pump()):
                #             analyses.vehicle.mass_properties.weight_breakdown.empty.propulsion.converters[converter.tag] = converter.mass_properties.mass
                    # Add to this nacelles, thrust reversers, etc
        elif key == 'landing_gears':
            for landing_gear in analyses.vehicle.landing_gears:
                if hasattr(weight_correction_factors.empty.structural, 'landing_gear'):
                    landing_gear.mass_properties.mass *= weight_correction_factors.empty.structural.landing_gear
        elif key == 'booms':
            for boom in analyses.vehicle.booms:
                if hasattr(weight_correction_factors.empty.structural, 'boom'):
                    boom.mass_properties.mass *= weight_correction_factors.empty.structural.boom    
        elif key == 'systems': # If you have factor and a defined system weight, multiply those two in a calculator. 
            for system in analyses.vehicle.systems:
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Avionics:
                    if hasattr(weight_correction_factors.empty.systems, 'avionics') and system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.avionics
                    elif hasattr(weight_correction_factors.empty.systems, 'avionics') and system.mass_properties.calculated_flag == False:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.avionics = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Flight_Controls:
                    if hasattr(weight_correction_factors.empty.systems, 'control_systems') and system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.control_systems 
                    elif hasattr(weight_correction_factors.empty.systems, 'control_systems') and system.mass_properties.calculated_flag == False:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.control_systems = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Auxillary_Power_Unit: 
                    if hasattr(weight_correction_factors.empty.systems, 'apu') and system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.apu  
                    elif hasattr(weight_correction_factors.empty.systems, 'apu') and system.mass_properties.calculated_flag == False:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.apu = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Electrical: 
                    if hasattr(weight_correction_factors.empty.systems, 'electrical') and system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.electrical  
                    elif hasattr(weight_correction_factors.empty.systems, 'electrical') and system.mass_properties.calculated_flag == False:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.electrical = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Hydraulics: 
                    if hasattr(weight_correction_factors.empty.systems, 'hydraulics') and system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.hydraulics 
                    elif hasattr(weight_correction_factors.empty.systems, 'hydraulics') and system.mass_properties.calculated_flag == False:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.hydraulics = system.mass_properties.mass
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Environmental_Controls: 
                    if hasattr(weight_correction_factors.empty.systems, 'air_conditioner') and system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.air_conditioner  
                    elif hasattr(weight_correction_factors.empty.systems, 'air_conditioner') and system.mass_properties.calculated_flag == False:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.air_conditioner = system.mass_properties.mass  
                if type(system) == RCAIDE.Library.Components.Powertrain.Systems.Instruments:
                    if hasattr(weight_correction_factors.empty.systems, 'instruments') and system.mass_properties.calculated_flag:
                        system.mass_properties.mass *= weight_correction_factors.empty.systems.instruments  
                    elif hasattr(weight_correction_factors.empty.systems, 'instruments') and system.mass_properties.calculated_flag == False:
                        analyses.vehicle.mass_properties.weight_breakdown.empty.systems.instruments = system.mass_properties.mass  
                      
def iterate_for_mtow(old_mtow, oew, max_payload, max_fuel,vehicle):
    '''
    Staub factor after, Franco Staub, ex JetZero, is MTOW/(OEW + Max Fuel + Max Payload)

    '''
    overall_maximum_weight = max_payload + oew + max_fuel

    target_staub_factor = getattr(vehicle, 'staub_factor', 0)

    existing_staub_factor = old_mtow / overall_maximum_weight
    diff = existing_staub_factor - target_staub_factor

    # Proportional correction in weight-space (not absolute +0.1 kg), with damping.
    gain = 0.05
    correction_ratio = np.clip(gain * diff, -0.05, 0.05)
    new_mtow = old_mtow * (1.0 - correction_ratio)

    # Keep MTOW physically meaningful.
    return max(new_mtow, 0.0),diff
                                    

def use_previous_segment_pre_processed_data(mission,segment,i):
    '''
    Reuses previous segment pre processed data to save computational time.
    Ensures that changes in configuration are not overwritten.    
    '''
    analyses         = segment.analyses
    weights_analysis = analyses.weights 
    vehicle_1 = deepcopy(mission.segments[i-1].analyses.vehicle)
    segment.analyses.vehicle.mass_properties = vehicle_1.mass_properties
    weights_analysis.settings.iterate_mtow = False
    mass_properties_preprocess_routine(segment, i) 
    
    return
