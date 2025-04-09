# turbojet_validation.py
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
from   RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.design_turbojet import design_turbojet
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
                        
    turbojets           = [Olympus_593()]

    gsp_values_Olympus_593 = {
            "Compressor Exit Temperature [K]":0,  # [K]
            "Compressor Exit Pressure [MPa]": 0,  # [MPa]
            "Turbine Inlet Temperature [K]":  0,  # [K]
            "Turbine Inlet Pressure [MPa]":   0,  # [MPa]
            "Fuel Mass Flow Rate [kg/s]":     0,  # [kg/s]
            "TSFC [mg/(N s)]":                0  # [mg/(N s)]
        }

    literature_values_Olympus_593 = {
            "Compressor Exit Temperature [K]":0, # [K]
            "Compressor Exit Pressure [MPa]": 0, # [MPa]
            "Turbine Inlet Temperature [K]":  0, # [K]
            "Turbine Inlet Pressure [MPa]":   0, # [MPa]
            "Fuel Mass Flow Rate [kg/s]":     0, # [kg/s]
            "TSFC [mg/(N s)]":                0  # [mg/(N s)]
        }

    gsp_values = {
        "Olympus_593": gsp_values_Olympus_593,
    }

    literature_values = {
        "Olympus_593": literature_values_Olympus_593,
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
    
    for turbojet_index, turbojet in enumerate(turbojets):
        for i in range(len(altitude)): 
            for j in range(len(mach_number)):

                turbojet = turbojets[turbojet_index]
            
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
                conditions.freestream.isentropic_expansion_factor = np.atleast_1d(turbojet.working_fluid.compute_gamma(T,p))
                conditions.freestream.Cp                          = np.atleast_1d(turbojet.working_fluid.compute_cp(T,p))
                conditions.freestream.R                           = np.atleast_1d(turbojet.working_fluid.gas_specific_constant)
                conditions.freestream.speed_of_sound              = np.atleast_1d(a)
                conditions.freestream.velocity                    = np.atleast_1d(a*mach_number[j])  

                # setup conditions  
                fuel_line                                         = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
                segment                                           = RCAIDE.Framework.Mission.Segments.Segment()  
                segment.state.conditions                          = conditions     
                segment.state.conditions.energy[fuel_line.tag]    = Conditions()
                segment.state.conditions.noise[fuel_line.tag]     = Conditions()

                turbojet.append_operating_conditions(segment,segment.state.conditions.energy,segment.state.conditions.noise)

                for tag, item in turbojet.items(): 
                    if issubclass(type(item), RCAIDE.Library.Components.Component):
                        item.append_operating_conditions(segment,segment.state.conditions.energy)

                # set throttle
                segment.state.conditions.energy.propulsors[turbojet.tag].throttle[:,0] = 1  

                thrust_vector,moment,power,_,_,_                  = turbojet.compute_performance(segment.state,center_of_gravity=turbojet.origin)

                high_pressure_compressor                          = turbojet.high_pressure_compressor
                combustor                                         = turbojet.combustor
                high_pressure_turbine                             = turbojet.high_pressure_turbine
                core_nozzle                                       = turbojet.core_nozzle  

                # unpack component conditions
                turbojet_conditions                               = conditions.energy
                hpc_conditions                                    = turbojet_conditions.converters[high_pressure_compressor.tag]
                combustor_conditions                              = turbojet_conditions.converters[combustor.tag]
                hpt_conditions                                    = turbojet_conditions.converters[high_pressure_turbine.tag]
                core_nozzle_conditions                            = turbojet_conditions.converters[core_nozzle.tag]

                # extract properties
                mdot_air_core                                     = turbojet_conditions.propulsors[turbojet.tag].core_mass_flow_rate
                fuel_enthalpy                                     = combustor.fuel_data.specific_energy 
                mdot_fuel                                         = turbojet_conditions.propulsors[turbojet.tag].fuel_flow_rate 
                U_0                                               = a*mach_number[j] 
                h_e_c                                             = core_nozzle_conditions.outputs.static_enthalpy
                h_0                                               = turbojet.working_fluid.compute_cp(T,p) * T 
                h_t4                                              = combustor_conditions.outputs.stagnation_enthalpy
                h_t3                                              = hpc_conditions.outputs.stagnation_enthalpy     

                thrust[i,j]                                       = np.linalg.norm(thrust_vector)
                overall_efficiency[i,j]                           = turbojet_conditions.propulsors[turbojet.tag].overall_efficiency[0,0]
                thermal_efficiency[i,j]                           = turbojet_conditions.propulsors[turbojet.tag].thermal_efficiency[0,0]
                Tt_3[i,j]                                         = hpc_conditions.outputs.stagnation_temperature 
                Pt_3[i,j]                                         = hpc_conditions.outputs.stagnation_pressure
                Tt_4[i,j]                                         = hpt_conditions.inputs.stagnation_temperature 
                Pt_4[i,j]                                         = hpt_conditions.inputs.stagnation_pressure 
                m_dot_core[i,j]                                   = turbojet_conditions.propulsors[turbojet.tag].core_mass_flow_rate   
                fuel_flow_rate[i,j]                               = turbojet_conditions.propulsors[turbojet.tag].fuel_flow_rate
                TSFC[i,j]                                         = turbojet_conditions.propulsors[turbojet.tag].thrust_specific_fuel_consumption 
      
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
            "Parameter [Unit]": list(gsp_values[turbojet.tag].keys()),
            "GSP (real data)": list(gsp_values[turbojet.tag].values()),
            "Literature": list(literature_values[turbojet.tag].values()),
            "RCAIDE": [calculate_percentage_difference(rcaide_values[key], gsp_values[turbojet.tag][key]) for key in gsp_values[turbojet.tag]]
        }

        df = pd.DataFrame(data)
        
        print("=" * len(df.to_markdown(index=False).split('\n')[0]))
        print(f"Engine: {turbojet.tag}")
        print("=" * len(df.to_markdown(index=False).split('\n')[0]))

        print(df.to_markdown(index=False))
    
    return

def Olympus_593():

    turbojet                              = RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet()   
    turbojet.tag                          = 'Olympus_593'
    turbojet.engine_length                = 4.039
    turbojet.nacelle_diameter             = 1.3
    turbojet.inlet_diameter               = 1.212 
    turbojet.areas.wetted                 = 30
    turbojet.design_altitude              = 60000.0*Units.ft
    turbojet.design_mach_number           = 2.02
    turbojet.design_thrust                = 10000. * Units.lbf  
    turbojet.origin                       = [[37.,5.5,-1.6]] 
    turbojet.working_fluid                = RCAIDE.Library.Attributes.Gases.Air()
   
    # Ram  
    ram                                   = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                               = 'ram' 
    turbojet.ram                          = ram 
         
    # Inlet Nozzle         
    inlet_nozzle                          = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                      = 'inlet_nozzle' 
    inlet_nozzle.polytropic_efficiency    = 1.0
    inlet_nozzle.pressure_ratio           = 1.0
    inlet_nozzle.pressure_recovery        = 0.94 
    turbojet.inlet_nozzle                 = inlet_nozzle    
          
    #  Low Pressure Compressor      
    lp_compressor                         = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    lp_compressor.tag                     = 'low_pressure_compressor' 
    lp_compressor.polytropic_efficiency   = 0.88
    lp_compressor.pressure_ratio          = 3.1     
    turbojet.low_pressure_compressor      = lp_compressor         
        
    # High Pressure Compressor        
    hp_compressor                         = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    hp_compressor.tag                     = 'high_pressure_compressor' 
    hp_compressor.polytropic_efficiency   = 0.88
    hp_compressor.pressure_ratio          = 5.0  
    turbojet.high_pressure_compressor     = hp_compressor
 
    # Low Pressure Turbine 
    lp_turbine                            = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    lp_turbine.tag                        ='low_pressure_turbine' 
    lp_turbine.mechanical_efficiency      = 0.99
    lp_turbine.polytropic_efficiency      = 0.89 
    turbojet.low_pressure_turbine         = lp_turbine      
             
    # High Pressure Turbine         
    hp_turbine                            = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    hp_turbine.tag                        ='high_pressure_turbine' 
    hp_turbine.mechanical_efficiency      = 0.99
    hp_turbine.polytropic_efficiency      = 0.87 
    turbojet.high_pressure_turbine        = hp_turbine   
          
    # Combustor   
    combustor                             = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                         = 'combustor' 
    combustor.efficiency                  = 0.94
    combustor.alphac                      = 1.0     
    combustor.turbine_inlet_temperature   = 1440.
    combustor.pressure_ratio              = 0.92
    combustor.fuel_data                   = RCAIDE.Library.Attributes.Propellants.Jet_A()     
    turbojet.combustor                    = combustor
     
    #  Afterburner  
    afterburner                           = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    afterburner.tag                       = 'afterburner' 
    afterburner.efficiency                = 0.9
    afterburner.alphac                    = 1.0     
    afterburner.turbine_inlet_temperature = 1500
    afterburner.pressure_ratio            = 1.0
    afterburner.fuel_data                 = RCAIDE.Library.Attributes.Propellants.Jet_A()     
    turbojet.afterburner                  = afterburner   
 
    # Core Nozzle 
    nozzle                                = RCAIDE.Library.Components.Powertrain.Converters.Supersonic_Nozzle()   
    nozzle.tag                            = 'core_nozzle' 
    nozzle.pressure_recovery              = 0.95
    nozzle.pressure_ratio                 = 1.    
    turbojet.core_nozzle                  = nozzle
    
    design_turbojet(turbojet) 
    
    return turbojet

if __name__ == '__main__': 
    main()
