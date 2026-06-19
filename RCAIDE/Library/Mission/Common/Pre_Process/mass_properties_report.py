# RCAIDE/Library/Methods/Mission/Common/Pre_Process/mass_properties_report.py
# 
# ----------------------------------------------------------------------------------------------------------------------
#  Imports 
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports 
import RCAIDE  

# python imports 
import os 
import sys
import pandas as pd 
# ----------------------------------------------------------------------------------------------------------------------
#  Helper Functions for Mass Properties Analysis
# ----------------------------------------------------------------------------------------------------------------------   
def print_mass_report(analyses): 
    weights_analysis = analyses.weights 
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

def write_mass_report(analyses): 
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

    return excel_filename