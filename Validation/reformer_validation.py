# reformer_test.py
# 
# Created:  Jan 2025, M. Clarke, M. Guidotti

#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
from RCAIDE.Library.Methods.Powertrain.Converters import Reformer
from RCAIDE.Library.Methods.Powertrain.Converters.Reformer.compute_reformer_performance import compute_reformer_performance

# python imports 
import pandas as pd

def main(): 

    literature_reformer = {
            "effluent_gas_flow_rate  [m**3/s]":   541, 
            "reformer_efficiency [-]":            84, 
            "hydrogen_conversion_efficiency [-]": 34, 
            "space_velocity [1/s]":               7188, 
            "liquid_space_velocity [1/s]":        1, 
            "steam_to_carbon_feed_ratio [-]":     3, 
            "oxygen_to_carbon_feed_ratio [-]":    1, 
            "fuel_to_air_ratio [-]":              4
        }

    literature_values = {
        "Test_reformer": literature_reformer,
    }

    reformer = RCAIDE.Library.Components.Powertrain.Converters.Reformer()
    reformer.tag = 'Test_reformer'

    reformer_conditions = RCAIDE.Framework.Mission.Common.Conditions()

    reformer_conditions.fuel_volume_flow_rate  = reformer.eta * 4.5e-9       # [m**3/s]        Jet-A feed rate
    reformer_conditions.steam_volume_flow_rate = reformer.eta * 1.6667e-8    # [m**3/s]        Deionized water feed rate
    reformer_conditions.air_volume_flow_rate   = reformer.eta * 1e-5         # [m**3/s]        Air feed rate

    compute_reformer_performance(reformer,reformer_conditions)

    Q_R     =  reformer_conditions.effluent_gas_flow_rate  
    eta_ref =  reformer_conditions.reformer_efficiency 
    X_H2    =  reformer_conditions.hydrogen_conversion_efficiency 
    GHSV    =  reformer_conditions.space_velocity 
    LHSV    =  reformer_conditions.liquid_space_velocity 
    S_C     =  reformer_conditions.steam_to_carbon_feed_ratio 
    O_C     =  reformer_conditions.oxygen_to_carbon_feed_ratio 
    phi     =  reformer_conditions.fuel_to_air_ratio 

    rcaide_values = {
            "effluent_gas_flow_rate  [m**3/s]":     Q_R,                     
            "reformer_efficiency [-]":            eta_ref,                 
            "hydrogen_conversion_efficiency [-]": X_H2, 
            "space_velocity [1/s]":               GHSV, 
            "liquid_space_velocity [1/s]":        LHSV, 
            "steam_to_carbon_feed_ratio [-]":     S_C, 
            "oxygen_to_carbon_feed_ratio [-]":    O_C, 
            "fuel_to_air_ratio [-]":              phi, 
        }

    def calculate_percentage_difference(simulated, reference):
        return f"{simulated} ({((simulated - reference) / reference) * 100:+.2f}%)"
    
    data = {
        "Parameter [Unit]": list(literature_values[reformer.tag].keys()),
        "Literature": list(literature_values[reformer.tag].values()),
        "RCAIDE": [calculate_percentage_difference(rcaide_values[key], literature_values[reformer.tag][key]) for key in literature_values[reformer.tag]]
    }

    df = pd.DataFrame(data)
    
    print("=" * len(df.to_markdown(index=False).split('\n')[0]))
    print(f"Reformer: {reformer.tag}")
    print("=" * len(df.to_markdown(index=False).split('\n')[0]))

    print(df.to_markdown(index=False))

if __name__ == '__main__':
    main()