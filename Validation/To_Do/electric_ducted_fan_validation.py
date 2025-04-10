# 
# Created:  April 2025, M. Clarke, M. Guidotti

#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
from RCAIDE.Framework.Core import Units
from   RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Ducted_Fan import design_electric_ducted_fan
from RCAIDE.Library.Methods.Powertrain                  import setup_operating_conditions  
import numpy as np

#----------------------------------------------------------------------
#   Reference Values
# ----------------------------------------------------------------------

# Ducted Fan: 

#----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():  

    altitude            = np.array([5000])*Units.feet
    mach_number         = np.array([0.8])

    bus                 = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    bus.voltage         = 110 
                        
    literature_values_Electric_Ducted_Fan = {
            "Compressor Exit Temperature [K]":0, # [K]
            "Compressor Exit Pressure [MPa]": 0, # [MPa]
            "Turbine Inlet Temperature [K]":  0, # [K]
            "Turbine Inlet Pressure [MPa]":   0, # [MPa]
            "Fuel Mass Flow Rate [kg/s]":     0, # [kg/s]
            "TSFC [mg/(N s)]":                0  # [mg/(N s)]
        }

    literature_values = {
        "Electric_Ducted_Fan": literature_values_Electric_Ducted_Fan,
    }

    thrust              = np.zeros((len(altitude),len(mach_number)))
    overall_efficiency  = np.zeros((len(altitude),len(mach_number)))
    thermal_efficiency  = np.zeros((len(altitude),len(mach_number)))
    Tt_3                = np.zeros((len(altitude),len(mach_number)))
    Pt_3                = np.zeros((len(altitude),len(mach_number)))
    Tt_4                = np.zeros((len(altitude),len(mach_number)))
    Pt_4                = np.zeros((len(altitude),len(mach_number))) 
    

    electric_ducted_fan = Electric_Ducted_Fan(bus, 'Rankine_Froude_Momentum_Theory')
    operating_state = setup_operating_conditions(electric_ducted_fan, altitude = altitude,velocity_range= np.array([20]), angle_of_attack=0)
         
    ducted_fan_conditions =  operating_state.conditions.energy.converters[electric_ducted_fan.ducted_fan.tag]
    ducted_fan_conditions.throttle = 1
    operating_state.conditions.energy.modulators[electric_ducted_fan.electronic_speed_controller.tag].outputs.voltage = 110
    
    RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Ducted_Fan.compute_electric_ducted_fan_performance(electric_ducted_fan,operating_state)
     
    results = operating_state.conditions.energy.converters[electric_ducted_fan.ducted_fan.tag] 

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
      
    print(df.to_markdown(index=False))
    
    return

def Electric_Ducted_Fan(bus, ducted_fan_type):

    center_propulsor                              = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Ducted_Fan()  
  
    # Electronic Speed Controller       
    esc                                           = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    esc.tag                                       = 'esc_1'
    esc.efficiency                                = 0.95 
    esc.bus_voltage                               = bus.voltage   
    center_propulsor.electronic_speed_controller  = esc   
        

    # Ducted_fan                            
    ducted_fan                                   = RCAIDE.Library.Components.Powertrain.Converters.Ducted_Fan()
    ducted_fan.tag                               = 'ducted_fan'
    ducted_fan.number_of_radial_stations         = 20
    ducted_fan.tip_radius                        = 0.07 * Units.meters
    ducted_fan.hub_radius                        = 0.025 * Units.meters
    # ducted_fan.exit_radius                       = 1.1 * ducted_fan.tip_radius
    # ducted_fan.blade_clearance                   = 0.001
    # ducted_fan.length                            = 10. * Units.inches 
    # ducted_fan.fan_effectiveness                 = 1.1  
    # ducted_fan.rotor_percent_x_location          = 0.4
    # ducted_fan.origin                            = [[2.,  0, 0.95]]
    # ducted_fan.stator_percent_x_location         = 0.7
    # ducted_fan.fidelity                          = ducted_fan_type  
    # ducted_fan.cruise.design_thrust              = 75
    ducted_fan.cruise.design_altitude            = 3000  * Units.m
    # ducted_fan.cruise.design_angular_velocity    = (0.8* 339.709) /  ducted_fan.tip_radius
    ducted_fan.cruise.design_freestream_velocity = 40 *  Units.mps
    # ducted_fan.cruise.design_reference_velocity  = 90 *  Units.mph 

    center_propulsor.ducted_fan                  = ducted_fan    
              
    # DC_Motor       
    motor                                         = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    motor.efficiency                              = 0.98
    motor.origin                                  = [[2.,  0, 0.95]]
    motor.nominal_voltage                         = bus.voltage 
    motor.no_load_current                         = 0.001
    center_propulsor.motor                        = motor  

    # design center propulsor 
    design_electric_ducted_fan(center_propulsor, new_regression_results = True,keep_files = True)

    return center_propulsor

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()

