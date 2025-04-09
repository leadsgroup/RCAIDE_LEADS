# turboprop_validation.py
#
# Created:  Apr. 2025, M. Guidotti

# References:
# [1]: 

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from   RCAIDE.Framework.Core           import Units, Data   
from   RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.design_turboprop import design_turboprop
from   RCAIDE.Framework.Mission.Common import Conditions

# Python imports 
import numpy             as np                                             
import matplotlib.pyplot as plt 
import matplotlib.cm     as cm
import pandas as pd

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():  

    altitude            = np.array([35000])*Units.feet
    mach_number         = np.array([0.8])
                        
    turboprops          = [PW127XT()]

    gsp_values_PW127XT = {
            "Compressor Exit Temperature [K]":0,  # [K]
            "Compressor Exit Pressure [MPa]": 0,  # [MPa]
            "Turbine Inlet Temperature [K]":  0,  # [K]
            "Turbine Inlet Pressure [MPa]":   0,  # [MPa]
            "Fuel Mass Flow Rate [kg/s]":     0,  # [kg/s]
            "TSFC [mg/(N s)]":                0  # [mg/(N s)]
        }

    literature_PW127XT = {
            "Compressor Exit Temperature [K]":0, # [K]
            "Compressor Exit Pressure [MPa]": 0, # [MPa]
            "Turbine Inlet Temperature [K]":  0, # [K]
            "Turbine Inlet Pressure [MPa]":   0, # [MPa]
            "Fuel Mass Flow Rate [kg/s]":     0, # [kg/s]
            "TSFC [mg/(N s)]":                0  # [mg/(N s)]
        }

    gsp_values = {
        "PW127XT": gsp_values_PW127XT,
    }

    literature_values = {
        "PW127XT": literature_PW127XT,
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
    TSFC                = np.zeros((len(altitude),len(mach_number)))
    
    for turboprop_index, turboprop in enumerate(turboprops):
        for i in range(len(altitude)): 
            for j in range(len(mach_number)):

                turboprop = turboprops[turboprop_index]
            
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
                conditions.freestream.isentropic_expansion_factor = np.atleast_1d(turboprop.working_fluid.compute_gamma(T,p))
                conditions.freestream.Cp                          = np.atleast_1d(turboprop.working_fluid.compute_cp(T,p))
                conditions.freestream.R                           = np.atleast_1d(turboprop.working_fluid.gas_specific_constant)
                conditions.freestream.speed_of_sound              = np.atleast_1d(a)
                conditions.freestream.velocity                    = np.atleast_1d(a*mach_number[j])  

                # setup conditions  
                fuel_line                                         = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
                segment                                           = RCAIDE.Framework.Mission.Segments.Segment()  
                segment.state.conditions                          = conditions     
                segment.state.conditions.energy[fuel_line.tag]    = Conditions()
                segment.state.conditions.noise[fuel_line.tag]     = Conditions()

                turboprop.append_operating_conditions(segment,segment.state.conditions.energy,segment.state.conditions.noise)

                for tag, item in turboprop.items(): 
                    if issubclass(type(item), RCAIDE.Library.Components.Component):
                        item.append_operating_conditions(segment,segment.state.conditions.energy)

                # set throttle
                segment.state.conditions.energy.propulsors[turboprop.tag].throttle[:,0] = 1  

                thrust_vector,moment,power,_,_,_                  = turboprop.compute_performance(segment.state,center_of_gravity=turboprop.origin)

                lpc                                               = turboprop.compressor
                combustor                                         = turboprop.combustor
                high_pressure_turbine                             = turboprop.high_pressure_turbine
                core_nozzle                                       = turboprop.core_nozzle  

                # unpack component conditions
                turboprop_conditions                              = conditions.energy
                lpc_conditions                                    = turboprop_conditions.converters[lpc.tag]
                combustor_conditions                              = turboprop_conditions.converters[combustor.tag]
                hpt_conditions                                    = turboprop_conditions.converters[high_pressure_turbine.tag]
                core_nozzle_conditions                            = turboprop_conditions.converters[core_nozzle.tag]

                # extract properties
                mdot_air_core                                     = turboprop_conditions.propulsors[turboprop.tag].core_mass_flow_rate
                fuel_enthalpy                                     = combustor.fuel_data.specific_energy 
                mdot_fuel                                         = turboprop_conditions.propulsors[turboprop.tag].fuel_flow_rate 
                U_0                                               = a*mach_number[j] 
                h_e_c                                             = core_nozzle_conditions.outputs.static_enthalpy
                h_0                                               = turboprop.working_fluid.compute_cp(T,p) * T 
                h_t4                                              = combustor_conditions.outputs.stagnation_enthalpy   

                thrust[i,j]                                       = np.linalg.norm(thrust_vector)
                overall_efficiency[i,j]                           = turboprop_conditions.propulsors[turboprop.tag].overall_efficiency[0,0]
                thermal_efficiency[i,j]                           = turboprop_conditions.propulsors[turboprop.tag].thermal_efficiency[0,0]
                Tt_3[i,j]                                         = lpc_conditions.outputs.stagnation_temperature 
                Pt_3[i,j]                                         = lpc_conditions.outputs.stagnation_pressure
                Tt_4[i,j]                                         = hpt_conditions.inputs.stagnation_temperature 
                Pt_4[i,j]                                         = hpt_conditions.inputs.stagnation_pressure 
                m_dot_core[i,j]                                   = turboprop_conditions.propulsors[turboprop.tag].core_mass_flow_rate   
                fuel_flow_rate[i,j]                               = turboprop_conditions.propulsors[turboprop.tag].fuel_flow_rate
                TSFC[i,j]                                         = turboprop_conditions.propulsors[turboprop.tag].thrust_specific_fuel_consumption 
    
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
            "Parameter [Unit]": list(gsp_values[turboprop.tag].keys()),
            "GSP (real data)": list(gsp_values[turboprop.tag].values()),
            "Literature": list(literature_values[turboprop.tag].values()),
            "RCAIDE": [calculate_percentage_difference(rcaide_values[key], gsp_values[turboprop.tag][key]) for key in gsp_values[turboprop.tag]]
        }

        df = pd.DataFrame(data)
        
        print("=" * len(df.to_markdown(index=False).split('\n')[0]))
        print(f"Engine: {turboprop.tag}")
        print("=" * len(df.to_markdown(index=False).split('\n')[0]))

        print(df.to_markdown(index=False))
    
    return

def PW127XT():

    turboprop                                        = RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop()    
    turboprop.tag                                    = 'PW127XT'  
    turboprop.origin                                 = [[ 9.559106394 ,4.219315295, 1.616135105]]
    turboprop.design_altitude                        = 25000 * Units.ft                                 # [-]         Design Altitude 
    turboprop.design_freestream_velocity             = 141.94 * Units.meter_per_second      
    turboprop.design_thrust                          = 10000.0 * Units.N                                # [-]         Design Thrust          
    turboprop.working_fluid                          = RCAIDE.Library.Attributes.Gases.Air()            
    turboprop.design_propeller_efficiency            = 0.83                                             # [-]         Design Propeller Efficiency
    turboprop.gearbox.efficiency                     = 0.99                                             # [-]         Design Gearbox Efficiency
    
    # Ram inlet 
    ram                                              = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                          = 'ram' 
    turboprop.ram                                    = ram 
          
    # inlet nozzle          
    inlet_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                                 = 'inlet nozzle'                                       
    inlet_nozzle.pressure_ratio                      = 0.98
    inlet_nozzle.compressibility_effects             = False
    turboprop.inlet_nozzle                           = inlet_nozzle
                                                     
    # compressor                        
    compressor                                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    compressor.tag                                   = 'lpc'                   
    compressor.pressure_ratio                        = 10                   
    turboprop.compressor                             = compressor
    
    # combustor      
    combustor                                        = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                    = 'Comb'
    combustor.efficiency                             = 0.99                   
    combustor.turbine_inlet_temperature              = 1370                    
    combustor.pressure_ratio                         = 0.96                    
    combustor.fuel_data                              = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    turboprop.combustor                              = combustor
        
    # high pressure turbine         
    high_pressure_turbine                            = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                        ='hpt'
    high_pressure_turbine.mechanical_efficiency      = 0.99                       
    turboprop.high_pressure_turbine                  = high_pressure_turbine 
        
    # low pressure turbine      
    low_pressure_turbine                             = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                         ='lpt'
    low_pressure_turbine.mechanical_efficiency       = 0.99                      
    turboprop.low_pressure_turbine                   = low_pressure_turbine
    
    # core nozzle    
    core_nozzle                                      = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                  = 'core nozzle'          
    core_nozzle.pressure_ratio                       = 0.99
    turboprop.core_nozzle                            = core_nozzle
    
    # design turboprop
    design_turboprop(turboprop)
    
    return turboprop

if __name__ == '__main__': 
    main()
    plt.show()
