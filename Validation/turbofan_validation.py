# turbofan_validation.py
#
# Created:  Mar. 2025, M. Guidotti

# References:
# [1]: "Turbofan and Turbojet Engines Database Handbook", Elodie Roux. https://ptabdata.blob.core.windows.net/files/2017/IPR2017-00999/v20_GE-1019%20Turbofan%20and%20Turbojet%20Engines%20Database%20Handbook.pdf
# [2]: "Turbofan Engine Sizing and Tradeoff Analysis via Signomial Programming", Martin A. York, Warren W. Hoburg, and Mark Drela. https://arc.aiaa.org/doi/epdf/10.2514/1.C034463
# [3]: "Airplane Design Optimization for Minimal Global Warming Impact", Pieter-Jan Proesmans and Roelof Vos. https://arc.aiaa.org/doi/full/10.2514/1.C036529

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from   RCAIDE.Framework.Core           import Units, Data   
from   RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.design_turbofan import design_turbofan
from   RCAIDE.Framework.Mission.Common import Conditions

# Python imports 
import numpy             as np                   
import pandas as pd

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():  

    altitude            = np.array([35000])*Units.feet
    mach_number         = np.array([0.8])
                        
    turbofans           = [GE90_94B()]

    gsp_values_GE90_94B = {
            "Compressor Exit Temperature [K]": 771,  # [K]
            "Compressor Exit Pressure [MPa]": 1.42,  # [MPa]
            "Turbine Inlet Temperature [K]":  1430,  # [K]
            "Turbine Inlet Pressure [MPa]":   1.35,  # [MPa]
            "Fuel Mass Flow Rate [kg/s]":     1.14,  # [kg/s]
            "TSFC [mg/(N s)]":                14.6   # [mg/(N s)]
        }

    literature_values_GE90_94B = {
            "Compressor Exit Temperature [K]":  772, # [K]
            "Compressor Exit Pressure [MPa]":  1.42, # [MPa]
            "Turbine Inlet Temperature [K]":   1430, # [K]
            "Turbine Inlet Pressure [MPa]":    1.35, # [MPa]
            "Fuel Mass Flow Rate [kg/s]":      1.16, # [kg/s]
            "TSFC [mg/(N s)]":                 14.9  # [mg/(N s)]
        }

    
    gsp_values = {
        "GE90-94B": gsp_values_GE90_94B
    }

    literature_values = {
        "GE90-94B": literature_values_GE90_94B
    }

    thrust              = np.zeros((len(altitude),len(mach_number)))
    overall_efficiency  = np.zeros((len(altitude),len(mach_number)))
    thermal_efficiency  = np.zeros((len(altitude),len(mach_number)))
    Tt_3                = np.zeros((len(altitude),len(mach_number)))
    Pt_3                = np.zeros((len(altitude),len(mach_number)))
    Tt_4                = np.zeros((len(altitude),len(mach_number)))
    Pt_4                = np.zeros((len(altitude),len(mach_number))) 
    m_dot_core          = np.zeros((len(altitude),len(mach_number)))
    fuel_flow_rate      = np.zeros((len(altitude),len(mach_number)))
    m_dot_air_tot       = np.zeros((len(altitude),len(mach_number)))
    TSFC                = np.zeros((len(altitude),len(mach_number)))
    
    for turbofan_index, turbofan in enumerate(turbofans):
        for i in range(len(altitude)): 
            for j in range(len(mach_number)):

                turbofan = turbofans[turbofan_index]
            
                planet                                            = RCAIDE.Library.Attributes.Planets.Earth()
                atmosphere                                        = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
                atmo_data                                         = atmosphere.compute_values(altitude[i])

                p                                                 = atmo_data.pressure          
                T                                                 = atmo_data.temperature       
                rho                                               = atmo_data.density          
                a                                                 = atmo_data.speed_of_sound    
                mu                                                = atmo_data.dynamic_viscosity     

                conditions                                        = RCAIDE.Framework.Mission.Common.Results() 
                conditions.freestream.altitude                    = np.atleast_1d(altitude[i])
                conditions.freestream.mach_number                 = np.atleast_1d(mach_number[j])
                conditions.freestream.pressure                    = np.atleast_1d(p)
                conditions.freestream.temperature                 = np.atleast_1d(T)
                conditions.freestream.density                     = np.atleast_1d(rho)
                conditions.freestream.dynamic_viscosity           = np.atleast_1d(mu)
                conditions.freestream.gravity                     = np.atleast_2d(planet.sea_level_gravity)
                conditions.freestream.isentropic_expansion_factor = np.atleast_1d(turbofan.working_fluid.compute_gamma(T,p))
                conditions.freestream.Cp                          = np.atleast_1d(turbofan.working_fluid.compute_cp(T,p))
                conditions.freestream.R                           = np.atleast_1d(turbofan.working_fluid.gas_specific_constant)
                conditions.freestream.speed_of_sound              = np.atleast_1d(a)
                conditions.freestream.velocity                    = np.atleast_1d(a*mach_number[j])  

                # setup conditions  
                fuel_line                                         = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
                segment                                           = RCAIDE.Framework.Mission.Segments.Segment()  
                segment.state.conditions                          = conditions     
                segment.state.conditions.energy[fuel_line.tag]    = Conditions()
                segment.state.conditions.noise[fuel_line.tag]     = Conditions()

                turbofan.append_operating_conditions(segment,segment.state.conditions.energy,segment.state.conditions.noise)

                for tag, item in turbofan.items(): 
                    if issubclass(type(item), RCAIDE.Library.Components.Component):
                        item.append_operating_conditions(segment,segment.state.conditions.energy)

                # set throttle
                segment.state.conditions.energy.propulsors[turbofan.tag].throttle[:,0] = 1  

                thrust_vector,moment,power,_,_,_                  = turbofan.compute_performance(segment.state,fuel_line)

                bypass_ratio                                      = turbofan.bypass_ratio
                high_pressure_compressor                          = turbofan.high_pressure_compressor
                combustor                                         = turbofan.combustor
                high_pressure_turbine                             = turbofan.high_pressure_turbine
                core_nozzle                                       = turbofan.core_nozzle
                fan_nozzle                                        = turbofan.fan_nozzle   

                # unpack component conditions
                turbofan_conditions                               = conditions.energy
                hpc_conditions                                    = turbofan_conditions.converters[high_pressure_compressor.tag]
                combustor_conditions                              = turbofan_conditions.converters[combustor.tag]
                hpt_conditions                                    = turbofan_conditions.converters[high_pressure_turbine.tag]
                core_nozzle_conditions                            = turbofan_conditions.converters[core_nozzle.tag]
                fan_nozzle_conditions                             = turbofan_conditions.converters[fan_nozzle.tag]

                # extract properties
                mdot_air_core                                     = turbofan_conditions.propulsors[turbofan.tag].core_mass_flow_rate
                mdot_air_fan                                      = bypass_ratio *  mdot_air_core  
                fuel_enthalpy                                     = combustor.fuel_data.specific_energy 
                mdot_fuel                                         = turbofan_conditions.propulsors[turbofan.tag].fuel_flow_rate 
                U_0                                               = a*mach_number[j] 
                h_e_f                                             = fan_nozzle_conditions.outputs.static_enthalpy
                h_e_c                                             = core_nozzle_conditions.outputs.static_enthalpy
                h_0                                               = turbofan.working_fluid.compute_cp(T,p) * T 
                h_t4                                              = combustor_conditions.outputs.stagnation_enthalpy
                h_t3                                              = hpc_conditions.outputs.stagnation_enthalpy     

                thrust[i,j]                                       = np.linalg.norm(thrust_vector)
                overall_efficiency[i,j]                           = thrust[i,j] * U_0 / (mdot_fuel * fuel_enthalpy) # Aircraft and Rocket Propulsion Eqn 2.22 
                thermal_efficiency[i,j]                           = 1 - ((mdot_air_core +  mdot_fuel)*(h_e_c -  h_0) + mdot_air_fan*(h_e_f - h_0) + mdot_fuel *h_0)/((mdot_air_core +  mdot_fuel)*h_t4 - mdot_air_core *h_t3) # Aircraft and Rocket Propulsion Eqn 5.49 
                Tt_3[i,j]                                         = hpc_conditions.outputs.stagnation_temperature 
                Pt_3[i,j]                                         = hpc_conditions.outputs.stagnation_pressure
                Tt_4[i,j]                                         = hpt_conditions.inputs.stagnation_temperature 
                Pt_4[i,j]                                         = hpt_conditions.inputs.stagnation_pressure 
                m_dot_core[i,j]                                   = turbofan_conditions.propulsors[turbofan.tag].core_mass_flow_rate   
                fuel_flow_rate[i,j]                               = turbofan_conditions.propulsors[turbofan.tag].fuel_flow_rate
                m_dot_air_tot[i,j]                                = turbofan_conditions.propulsors[turbofan.tag].core_mass_flow_rate + bypass_ratio * turbofan_conditions.propulsors[turbofan.tag].core_mass_flow_rate
                TSFC[i,j]                                         = turbofan.TSFC # [N/N-s]

        rcaide_values = {
            "Compressor Exit Temperature [K]": Tt_3[0,0],                      # [K]
            "Compressor Exit Pressure [MPa]":  Pt_3[0,0]/1e6,                  # [MPa]
            "Turbine Inlet Temperature [K]":   Tt_4[0,0],                      # [K]
            "Turbine Inlet Pressure [MPa]":    Pt_4[0,0]/1e6,                  # [MPa]
            "Fuel Mass Flow Rate [kg/s]":      fuel_flow_rate[0,0],            # [kg/s]
            "TSFC [mg/(N s)]":                 TSFC[0,0]*1e6/(Units.hour*9.81) # [mg/(N s)]
        }

        def calculate_percentage_difference(simulated, reference):
            return f"{simulated} ({((simulated - reference) / reference) * 100:+.2f}%)"
        
        data = {
            "Parameter [Unit]": list(gsp_values[turbofan.tag].keys()),
            "GSP (real data)": list(gsp_values[turbofan.tag].values()),
            "Literature": list(literature_values[turbofan.tag].values()),
            "RCAIDE": [calculate_percentage_difference(rcaide_values[key], gsp_values[turbofan.tag][key]) for key in gsp_values[turbofan.tag]]
        }

        df = pd.DataFrame(data)
        
        print("=" * len(df.to_markdown(index=False).split('\n')[0]))
        print(f"Engine: {turbofan.tag}")
        print("=" * len(df.to_markdown(index=False).split('\n')[0]))

        print(df.to_markdown(index=False))
    
    return

def GE90_94B():

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()   
    turbofan.tag                                = 'GE90-94B'
    turbofan.origin                             = [[ 0.0 , 0.0 , 0.0 ]]
    turbofan.bypass_ratio                       = 8.5                         # CHECKED Ref. [1] Page 197
    turbofan.design_altitude                    = 35000*Units.ft              # CHECKED Ref. [2] Page 9
    turbofan.design_mach_number                 = 0.8                         # CHECKED Ref. [2] Page 9
    turbofan.design_thrust                      = 72988.199552 * Units.N             # CHECKED Ref. [2] Page 9

    # working fluid                   
    turbofan.working_fluid                      = RCAIDE.Library.Attributes.Gases.Air() 
    
    # Ram inlet 
    ram                                         = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                     = 'ram' 
    turbofan.ram                                = ram 
          
    # inlet nozzle          
    inlet_nozzle                                = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                            = 'inlet nozzle'        
    inlet_nozzle.pressure_ratio                 = 0.98
    inlet_nozzle.compressibility_effects        = False
    turbofan.inlet_nozzle                       = inlet_nozzle
    
    # fan                
    fan                                         = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                     = 'fan'
    fan.polytropic_efficiency                   = 0.915                   # CHECKED Ref. [2] Page 9
    fan.pressure_ratio                          = 1.58                       # CHECKED Ref. [1] Page 197
    turbofan.fan                                = fan        

    # low pressure compressor    
    low_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                   = 'lpc'
    low_pressure_compressor.polytropic_efficiency = 0.9037                   # CHECKED Ref. [3] Page 14
    low_pressure_compressor.pressure_ratio        = 1.26                     # CHECKED Ref. [2] Page 9   
    turbofan.low_pressure_compressor              = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.9                        # CHECKED Ref. [3] Page 14
    high_pressure_compressor.pressure_ratio        = 20.003                     # CHECKED Ref. [2] Page 9
    turbofan.high_pressure_compressor              = high_pressure_compressor

    # low pressure turbine  
    low_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                       ='lpt'
    low_pressure_turbine.mechanical_efficiency     = 0.99                     # CHECKED Ref. [3] Page 14
    low_pressure_turbine.polytropic_efficiency     = 0.93                     # CHECKED Ref. [3] Page 14
    turbofan.low_pressure_turbine                  = low_pressure_turbine
   
    # high pressure turbine     
    high_pressure_turbine                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                      ='hpt'
    high_pressure_turbine.mechanical_efficiency    = 0.99                     # CHECKED Ref. [3] Page 14
    high_pressure_turbine.polytropic_efficiency    = 0.93                     # CHECKED Ref. [3] Page 14 
    turbofan.high_pressure_turbine                 = high_pressure_turbine 

    # combustor  
    combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                  = 'Comb'
    combustor.efficiency                           = 0.997                    # CHECKED Ref. [2] Page 9
    combustor.turbine_inlet_temperature            = 1430                     # CHECKED Ref. [3] Page 14
    combustor.pressure_ratio                       = 0.94                     # CHECKED Ref. [2] Page 9
    combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    turbofan.combustor                             = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.98                     # CHECKED Ref. [2] Page 9
    core_nozzle.pressure_ratio                     = 0.995 
    turbofan.core_nozzle                           = core_nozzle
             
    # fan nozzle             
    fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                 = 'fan nozzle'
    fan_nozzle.polytropic_efficiency               = 0.98                     # CHECKED Ref. [2] Page 9
    fan_nozzle.pressure_ratio                      = 0.995
    turbofan.fan_nozzle                            = fan_nozzle 
    
    # design turbofan
    design_turbofan(turbofan)  
    
    return turbofan

if __name__ == '__main__': 
    main()
