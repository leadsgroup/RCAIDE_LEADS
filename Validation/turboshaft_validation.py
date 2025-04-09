# turboshaft_validation.py
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
from   RCAIDE.Library.Methods.Powertrain.Converters.Turboshaft import design_turboshaft
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
    
    altitude            = np.array([0])*Units.feet
    mach_number         = np.array([0.1])
                        
    turboshafts         = [Boeing_502_14()]

    literature_Boeing_502_14 = {
            "Power [W]":0, # [W]
            "Thermal Efficiency": 0, # [K]
            "PSFC [kg/(kW hr)]":  0, # [mg/(N s)]
        }

    literature_values = {
        "Boeing_502_14": literature_Boeing_502_14,
    }

    power               = np.zeros((len(altitude),len(mach_number)))
    thermal_efficiency  = np.zeros((len(altitude),len(mach_number)))
    Tt_3                = np.zeros((len(altitude),len(mach_number)))
    Pt_3                = np.zeros((len(altitude),len(mach_number)))
    Tt_4                = np.zeros((len(altitude),len(mach_number)))
    Pt_4                = np.zeros((len(altitude),len(mach_number))) 
    m_dot_core          = np.zeros((len(altitude),len(mach_number)))
    fuel_flow_rate      = np.zeros((len(altitude),len(mach_number)))
    PSFC                = np.zeros((len(altitude),len(mach_number)))
    
    for turboshaft_index, turboshaft in enumerate(turboshafts):
        for i in range(len(altitude)): 
            for j in range(len(mach_number)):

                turboshaft = turboshafts[turboshaft_index]
            
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
                conditions.freestream.isentropic_expansion_factor = np.atleast_1d(turboshaft.working_fluid.compute_gamma(T,p))
                conditions.freestream.Cp                          = np.atleast_1d(turboshaft.working_fluid.compute_cp(T,p))
                conditions.freestream.R                           = np.atleast_1d(turboshaft.working_fluid.gas_specific_constant)
                conditions.freestream.speed_of_sound              = np.atleast_1d(a)
                conditions.freestream.velocity                    = np.atleast_1d(a*mach_number[j])  

                # setup conditions  
                fuel_line                                         = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
                segment                                           = RCAIDE.Framework.Mission.Segments.Segment()  
                segment.state.conditions                          = conditions     
                segment.state.conditions.energy[fuel_line.tag]    = Conditions()
                segment.state.conditions.noise[fuel_line.tag]     = Conditions()

                turboshaft.append_operating_conditions(segment,segment.state.conditions.energy,segment.state.conditions.noise)

                for tag, item in turboshaft.items(): 
                    if issubclass(type(item), RCAIDE.Library.Components.Component):
                        item.append_operating_conditions(segment,segment.state.conditions.energy)

                # set throttle
                segment.state.conditions.energy.converters[turboshaft.tag].throttle[:,0] = 1  

                power,_,_                 = turboshaft.compute_performance(segment.state)

                lpc                                               = turboshaft.compressor
                combustor                                         = turboshaft.combustor
                high_pressure_turbine                             = turboshaft.high_pressure_turbine
                core_nozzle                                       = turboshaft.core_nozzle  

                # unpack component conditions
                turboshaft_conditions                             = conditions.energy
                lpc_conditions                                    = turboshaft_conditions.converters[lpc.tag]
                combustor_conditions                              = turboshaft_conditions.converters[combustor.tag]
                hpt_conditions                                    = turboshaft_conditions.converters[high_pressure_turbine.tag]
                core_nozzle_conditions                            = turboshaft_conditions.converters[core_nozzle.tag]

                # extract properties
                power[i,j]                                        = power[0][0]
                thermal_efficiency[i,j]                           = turboshaft_conditions.converters[turboshaft.tag].thermal_efficiency[0,0]
                Tt_3[i,j]                                         = lpc_conditions.outputs.stagnation_temperature 
                Pt_3[i,j]                                         = lpc_conditions.outputs.stagnation_pressure
                Tt_4[i,j]                                         = hpt_conditions.inputs.stagnation_temperature 
                Pt_4[i,j]                                         = hpt_conditions.inputs.stagnation_pressure 
                m_dot_core[i,j]                                   = turboshaft_conditions.converters[turboshaft.tag].fuel_flow_rate/ turboshaft_conditions.converters[turboshaft.tag].fuel_to_air_ratio
                fuel_flow_rate[i,j]                               = turboshaft_conditions.converters[turboshaft.tag].fuel_flow_rate
                PSFC[i,j]                                         = turboshaft_conditions.converters[turboshaft.tag].power_specific_fuel_consumption 
    
        rcaide_values = {
            "Power [W]": power[0,0],                       # [W]
            "Thermal Efficiency": thermal_efficiency[0,0], # [-]
            "PSFC [kg/(kW hr)]": PSFC[0,0]              # [kg/(kW hr)]
        }

        def calculate_percentage_difference(simulated, reference):
            return f"{simulated} ({((simulated - reference) / reference) * 100:+.2f}%)"
        
        data = {
            "Parameter [Unit]": list(rcaide_values.keys()),
            "Literature": list(literature_values[turboshaft.tag].values()),
            "RCAIDE": [calculate_percentage_difference(rcaide_values[key], literature_values[turboshaft.tag][key]) for key in literature_values[turboshaft.tag]]
        }

        df = pd.DataFrame(data)
        
        print("=" * len(df.to_markdown(index=False).split('\n')[0]))
        print(f"Engine: {turboshaft.tag}")
        print("=" * len(df.to_markdown(index=False).split('\n')[0]))

        print(df.to_markdown(index=False))
    
    return

def Boeing_502_14():

    turboshaft                                     = RCAIDE.Library.Components.Powertrain.Converters.Turboshaft() 
    turboshaft.tag                                 = 'Boeing_502_14'
    turboshaft.origin                              = [[13.72, 4.86,-1.1]] 
    turboshaft.engine_length                       = 0.945     
    turboshaft.bypass_ratio                        = 0    
    turboshaft.design_altitude                     = 0.01*Units.ft
    turboshaft.design_mach_number                  = 0.1   
    turboshaft.design_power                        = 148000.0*Units.W 
    turboshaft.mass_flow_rate_design               = 1.9 #[kg/s]
    
    # working fluid                                    
    turboshaft.working_fluid                       = RCAIDE.Library.Attributes.Gases.Air() 
    ram                                            = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                        = 'ram' 
    turboshaft.ram                                 = ram 
                                                   
    # inlet nozzle                                 
    inlet_nozzle                                   = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                               = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency             = 0.98
    inlet_nozzle.pressure_ratio                    = 0.98 
    turboshaft.inlet_nozzle                        = inlet_nozzle 
                                                   
    # compressor                                   
    compressor                                     = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    compressor.tag                                 = 'compressor'
    compressor.polytropic_efficiency               = 0.91
    compressor.pressure_ratio                      = 4.35  
    compressor.mass_flow_rate                      = 1.9 
    turboshaft.compressor                          = compressor

    # low pressure turbine  
    low_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                       ='lpt'
    low_pressure_turbine.mechanical_efficiency     = 0.99
    low_pressure_turbine.polytropic_efficiency     = 0.93 
    turboshaft.low_pressure_turbine                = low_pressure_turbine
   
    # high pressure turbine     
    high_pressure_turbine                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                      ='hpt'
    high_pressure_turbine.mechanical_efficiency    = 0.99
    high_pressure_turbine.polytropic_efficiency    = 0.93 
    turboshaft.high_pressure_turbine               = high_pressure_turbine 

    # combustor  
    combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                  = 'Comb'
    combustor.efficiency                           = 0.99 
    combustor.alphac                               = 1.0     
    combustor.turbine_inlet_temperature            = 889
    combustor.pressure_ratio                       = 0.95
    combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    turboshaft.combustor                           = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.95
    core_nozzle.pressure_ratio                     = 0.99  
    turboshaft.core_nozzle                         = core_nozzle

    # design turboshaft
    design_turboshaft(turboshaft)  
    
    return turboshaft

if __name__ == '__main__': 
    main()
    plt.show()
  