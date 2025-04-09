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
from   RCAIDE.Framework.Core           import Units  
from   RCAIDE.Library.Methods.Powertrain.Propulsors.Constant_Speed_Internal_Combustion_Engine import design_constant_speed_internal_combustion_engine
from   RCAIDE.Framework.Mission.Common import Conditions

# Python imports 
import numpy             as np                                         
import pandas as pd
import os

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():  

    altitude            = np.array([0.1])*Units.feet
    mach_number         = np.array([0.1])
                        
    ice_css              = [ICE_CS()]

    literature_ICE_CS = {
            "Thrust [N]":          0, 
            "Power [W]":           0, 
            "Thrust Coefficient":  0, 
            "Torque Coefficient":  0, 
            "Power Coefficient":   0, 
            "PSFC [kg/(kW hr)]":   0  
        }

    literature_values = {
        "ICE_CS": literature_ICE_CS,
    }

    thrust              = np.zeros((len(altitude),len(mach_number)))
    power               = np.zeros((len(altitude),len(mach_number)))
    thrust_coefficient  = np.zeros((len(altitude),len(mach_number)))
    torque_coefficient  = np.zeros((len(altitude),len(mach_number)))
    power_coefficient   = np.zeros((len(altitude),len(mach_number)))
    fuel_flow_rate      = np.zeros((len(altitude),len(mach_number)))
    PSFC                = np.zeros((len(altitude),len(mach_number)))

    for ice_cs_index, ice_cs in enumerate(ice_css):
        for i in range(len(altitude)): 
            for j in range(len(mach_number)):

                ice_cs = ice_css[ice_cs_index]
            
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
                conditions.freestream.isentropic_expansion_factor = np.atleast_1d(ice_cs.working_fluid.compute_gamma(T,p))
                conditions.freestream.Cp                          = np.atleast_1d(ice_cs.working_fluid.compute_cp(T,p))
                conditions.freestream.R                           = np.atleast_1d(ice_cs.working_fluid.gas_specific_constant)
                conditions.freestream.speed_of_sound              = np.atleast_1d(a)
                conditions.freestream.velocity                    = np.atleast_1d(a*mach_number[j])  

                # setup conditions  
                fuel_line                                         = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
                segment                                           = RCAIDE.Framework.Mission.Segments.Segment()  
                segment.state.conditions                          = conditions     
                segment.state.conditions.energy[fuel_line.tag]    = Conditions()
                segment.state.conditions.noise[fuel_line.tag]     = Conditions()

                ice_cs.append_operating_conditions(segment,segment.state.conditions.energy,segment.state.conditions.noise)

                for tag, item in ice_cs.items(): 
                    if issubclass(type(item), RCAIDE.Library.Components.Component):
                        item.append_operating_conditions(segment,segment.state.conditions.energy, segment.state.conditions.noise)

                # set throttle
                segment.state.conditions.energy.propulsors[ice_cs.tag].throttle[:,0] = 1  
                segment.state.conditions.frames.body.transform_to_inertial = np.array([[[1., 0., 0.],
                                                                                        [0., 1., 0.],
                                                                                        [0., 0., 1.]]])

                thrust,moment,power_mech,power_elec,_,_           = ice_cs.compute_performance(segment.state)

                # unpack component conditions
                ice_cs_conditions                                 = conditions.energy
                propeller_conditions                              = ice_cs_conditions.converters[ice_cs.propeller.tag]
                engine_conditions                                 = ice_cs_conditions.converters[ice_cs.engine.tag]

                thrust[i,j]                                       = np.linalg.norm(thrust)
                power[i,j]                                        = power_mech
                thrust_coefficient[i,j]                           = propeller_conditions.thrust_coefficient
                torque_coefficient[i,j]                           = propeller_conditions.torque_coefficient
                power_coefficient[i,j]                            = propeller_conditions.power_coefficient
                fuel_flow_rate[i,j]                               = engine_conditions.fuel_flow_rate
                PSFC[i,j]                                         = engine_conditions.power_specific_fuel_consumption 
      
        rcaide_values = {
            "Thrust [N]":           thrust[0,0],                     
            "Power [W]":            power[0,0],                 
            "Thrust Coefficient":   propeller_conditions.thrust_coefficient[0,0],                   
            "Torque Coefficient":   propeller_conditions.torque_coefficient[0,0],             
            "Power Coefficient":    propeller_conditions.power_coefficient[0,0],           
            "PSFC [kg/(kW hr)]":    PSFC[0,0] 
        }

        def calculate_percentage_difference(simulated, reference):
            return f"{simulated} ({((simulated - reference) / reference) * 100:+.2f}%)"
        
        data = {
            "Parameter [Unit]": list(rcaide_values.keys()),
            "Literature": list(literature_values[ice_cs.tag].values()),
            "RCAIDE": [calculate_percentage_difference(rcaide_values[key], literature_values[ice_cs.tag][key]) for key in literature_values[ice_cs.tag]]
        }

        df = pd.DataFrame(data)
        
        print("=" * len(df.to_markdown(index=False).split('\n')[0]))
        print(f"Engine: {ice_cs.tag}")
        print("=" * len(df.to_markdown(index=False).split('\n')[0]))

        print(df.to_markdown(index=False))
    
    return

def ICE_CS():

    current_path = os.path.dirname(os.path.abspath(__file__))
    target_path = os.path.join(current_path,'..')
    target_path = os.path.join(os.path.normpath(target_path),'Verification', 'Vehicles', 'Airfoils')

    ice_cs                                 = RCAIDE.Library.Components.Powertrain.Propulsors.Constant_Speed_Internal_Combustion_Engine()
    ice_cs.tag                             = 'ICE_CS' 
           
    engine                                 = RCAIDE.Library.Components.Powertrain.Converters.Engine()
    engine.sea_level_power                 = 180. * Units.horsepower
    engine.flat_rate_altitude              = 0.0
    engine.rated_speed                     = 2700. * Units.rpm
    engine.rated_power                     = 180.  * Units.hp   
    engine.power_specific_fuel_consumption = 0.52  
    ice_cs.engine                          = engine 
    
    prop                                   = RCAIDE.Library.Components.Powertrain.Converters.Propeller()
    prop.variable_pitch                    = True 
    prop.number_of_blades                  = 2.0
    prop.tip_radius                        = 76./2. * Units.inches
    prop.hub_radius                        = 8.     * Units.inches
    prop.cruise.design_freestream_velocity = 119.   * Units.knots
    prop.cruise.design_angular_velocity    = 2650.  * Units.rpm
    prop.cruise.design_Cl                  = 0.8
    prop.cruise.design_altitude            = 12000. * Units.feet
    prop.cruise.design_power               = .64 * 180. * Units.horsepower 
    airfoil                                = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file                = target_path + '/NACA_4412.txt'
    airfoil.polar_files                    = [target_path + '/Polars/NACA_4412_polar_Re_50000.txt' ,
                                              target_path + '/Polars/NACA_4412_polar_Re_100000.txt' ,
                                              target_path + '/Polars/NACA_4412_polar_Re_200000.txt' ,
                                              target_path + '/Polars/NACA_4412_polar_Re_500000.txt' ,
                                              target_path + '/Polars/NACA_4412_polar_Re_1000000.txt' ] 
    prop.append_airfoil(airfoil)  
    prop.airfoil_polar_stations            = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] 
    ice_cs.propeller                       = prop  

    # design propeller ICE  
    design_constant_speed_internal_combustion_engine(ice_cs)
    
    return ice_cs

if __name__ == '__main__': 
    main()
