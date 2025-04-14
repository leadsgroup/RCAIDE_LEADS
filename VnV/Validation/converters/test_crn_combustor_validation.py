# CRN_combustor_validation.py
#
# Created:  Apr. 2025, M. Guidotti

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
 
# RCAIDE imports
from   RCAIDE.Library.Components.Powertrain.Converters.Combustor        import Combustor
from   RCAIDE.Library.Attributes.Gases                                  import Air
from   RCAIDE.Library.Attributes.Propellants                            import Jet_A1
from   RCAIDE.Library.Methods.Emissions.Chemical_Reactor_Network_Method import evaluate_cantera

# Python imports
import pandas as pd
import numpy as np

# ----------------------------------------------------------------------
#  References
# ----------------------------------------------------------------------
# [1] Brink, L. F. J. (2020). Modeling the impact of fuel composition on 
#     aircraft engine NOₓ, CO, and soot emissions. Master's thesis, 
#     Massachusetts Institute of Technology.
# [2] Allaire, D. L. (2006). A physics-based emissions model for 
#     aircraft gas turbine combustors. Master's thesis, Massachusetts 
#     Institute of Technology.
# [3] Lefebvre, A. H., & Ballal, D. R. (2010). Gas turbine combustion: 
#     Alternative fuels and emissions (3rd ed.). CRC Press.
# [4] ICAO (2024). Aircraft Engine Emissions Databank. International Civil 
#     Aviation Organization.

# ----------------------------------------------------------------------
#  Main Function
# ----------------------------------------------------------------------

def main():
    
    ICAO_EI = {
            "EI_CO2": 3.16, 
            "EI_CO":  0.00016, 
            "EI_H2O": 1.23, 
            "EI_NOx": 0.0174, 
        }
    
    # Combustor Inputs
    combustor                                         = Combustor()    # [-] Combustor object
    combustor.tag                                     = 'CFM56-7B'     # [-] Combustor tag
    combustor.volume                                  = 0.0023         # [m**3] Combustor volume
    combustor.length                                  = 0.2            # [m] Combustor Length
    combustor.number_of_combustors                    = 1              # [-] Number of Combustors for one engine
    combustor.F_SC                                    = 1              # [-] Fuel scale factor
    combustor.N_PZ                                    = 21             # [-] Number of PSR in the Primary Zone
    combustor.L_PZ                                    = 0.05           # [m] Primary Zone length  
    combustor.S_PZ                                    = 0.39           # [-] Mixing parameter in the Primary Zone  
    combustor.design_equivalence_ratio_PZ             = 1.71           # [-] Design Equivalence Ratio in Primary Zone at Maximum Throttle  
    combustor.N_SZ                                    = 500            # [-] Number of discritizations in the Secondary Zone
    combustor.f_SM                                    = 0.6            # [-] Slow mode fraction
    combustor.l_SA_SM                                 = 0.4            # [-] Secondary air length fraction (of L_SZ) in slow mode
    combustor.l_SA_FM                                 = 0.05           # [-] Secondary air length fraction (of L_SZ) in fast mode
    combustor.l_DA_start                              = 0.95           # [-] Dilution air start length fraction (of L_SZ)
    combustor.l_DA_end                                = 1.0            # [-] Dilution air end length fraction (of L_SZ)
    combustor.joint_mixing_fraction                   = 0.6            # [-] Joint mixing fraction
    combustor.design_equivalence_ratio_SZ             = 0.61            # [-] Design Equivalence Ratio in Secondary Zone at Maximum Throttle
    combustor.air_mass_flow_rate_take_off             = 40             # [kg/s] Air mass flow rate at take-off
    combustor.fuel_to_air_ratio_take_off              = 0.025          # [-] Fuel to air ratio at take-off

    # Oxidizer Inputs
    combustor.air_data                                = Air()          # [-] Air object
    combustor.air_data.air_surrogate                  = {'O2':0.2095, 'N2':0.7809, 'AR':0.0096} # [-] Mole fractions of air surrogate species
    
    # Fuel Inputs
    combustor.fuel_data                               = Jet_A1()       # [-] Fuel object
    combustor.fuel_data.stoichiometric_fuel_air_ratio = 0.068          # [-] Stoichiometric Fuel to Air ratio
    combustor.fuel_data.heat_of_vaporization          = 360000         # [J/kg] Heat of vaporization at standard conditions
    combustor.fuel_data.temperature                   = 298.15         # [K] Temperature of fuel
    combustor.fuel_data.pressure                      = 101325         # [Pa] Pressure of fuel

    combustor.fuel_data.fuel_surrogate_S1             = {'NC12H26':0.404, 'IC8H18':0.295, 'TMBENZ' : 0.073,'NPBENZ':0.228, 'C10H8':0.02} # [-] Mole fractions of fuel surrogate species
    combustor.fuel_data.kinetic_mechanism             = 'Fuel.yaml' # [-] Kinetic mechanism for fuel surrogate species

    # RCAIDE Inputs
    Temp_air                                          = 710            # [K]    Combustor Inlet Temperature of air
    Pres_air                                          = 2600000        # [Pa]   Combustor Inlet Pressure of air
    throttle                                          = 1              # [-]    Throttle (Non linear variation with Equivalence Ratio)
    mdot_air_tot                                      = combustor.air_mass_flow_rate_take_off*throttle  # [kg/s] Air mass flow rate 
    FAR                                               = combustor.fuel_to_air_ratio_take_off *throttle # [-]    Fuel to Air ratio 
  
    # RCAIDE Simulation
    results                                           = evaluate_cantera(combustor, Temp_air, Pres_air, mdot_air_tot, FAR)

    rcaide_values = {
            "EI_CO2":            results.final.EI.CO2,                     
            "EI_CO":             results.final.EI.CO,                 
            "EI_H2O":            results.final.EI.H2O,                   
            "EI_NOx":            results.final.EI.NOx,             
        }

    def calculate_percentage_difference(simulated, reference):
        return f"{simulated:.6f} ({((simulated - reference) / reference) * 100:+.2f}%)"
    
    data = {
        "Emission Index [kg/kg_fuel]": list(rcaide_values.keys()),
        "ICAO": list(ICAO_EI.values()),
        "RCAIDE": [calculate_percentage_difference(rcaide_values[key], ICAO_EI[key]) for key in ICAO_EI]
    }

    df = pd.DataFrame(data)
    
    print("=" * len(df.to_string(index=False).split('\n')[0]))
    print(df.to_string(index=False))
    print("=" * len(df.to_string(index=False).split('\n')[0]))

    print(df.to_string(index=False))


    error = np.abs((rcaide_values["EI_CO2"] - ICAO_EI["EI_CO2"]) / ICAO_EI["EI_CO2"]) * 100
    print("\nError in CO2 [%]:", error)
    assert error < 10

    return

if __name__ == '__main__':
    main()
