# 
# Created:  April 2025, M. Clarke, M. Guidotti

#----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
from RCAIDE.Framework.Core import Units
from   RCAIDE.Library.Methods.Powertrain                  import setup_operating_conditions 
from   RCAIDE.Library.Methods.Powertrain.Converters       import Motor
from   RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan import design_ducted_fan
from   RCAIDE.Library.Methods.Powertrain.Converters.Motor import design_optimal_motor
import numpy as np

#----------------------------------------------------------------------
#   Reference Values
# ----------------------------------------------------------------------

# Ducted Fan: 

#----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():  

    tip_mach            = np.array([0.2])     
    mach_number         = np.array([0.01]) 
    altitude            = np.array([0.1]) *Units.feet

    thrust              = np.zeros((len(altitude),len(mach_number),len(tip_mach)))
    torque              = np.zeros((len(altitude),len(mach_number),len(tip_mach)))
    power               = np.zeros((len(altitude),len(mach_number),len(tip_mach)))

    bus                                              = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    bus.voltage                                      = 1000                         

    electric_ducted_fan                              = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Ducted_Fan()
    electric_ducted_fan.tag                          = 'electric_ducted_fan'

    ducted_fan                                       = RCAIDE.Library.Components.Powertrain.Converters.Ducted_Fan()
    ducted_fan.fidelity                              = 'Rankine_Froude_Momentum_Theory'
    ducted_fan.cruise.design_angular_velocity        = 0.8 * 339.709 / 0.95
    ducted_fan.cruise.design_freestream_velocity     = 90 * Units.mph
    ducted_fan.cruise.design_altitude                = 5000 * Units.ft
    DF                                               = design_ducted_fan(ducted_fan)
    electric_ducted_fan.ducted_fan                   = DF

    esc                                              = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    esc.tag                                          = 'esc_1'
    esc.efficiency                                   = 0.95 
    esc.bus_voltage                                  = bus.voltage
    electric_ducted_fan.electronic_speed_controller  = esc  

    motor                                            = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    motor.efficiency                                 = 0.95
    motor.origin                                     = [[2.,  0, 0.95]]
    motor.nominal_voltage                            = bus.voltage 
    motor.no_load_current                            = 0.001
    electric_ducted_fan.motor                        = design_optimal_motor(motor)

    bus.assigned_propulsors.append(electric_ducted_fan)    
    
    for i in range(len(altitude)): 
        for j in range(len(mach_number)):
            for k in  range(len(tip_mach)):
                
                operating_state_motor  = setup_operating_conditions(electric_ducted_fan.motor) 
        
                # Assign conditions to the motor
                motor_conditions = operating_state_motor.conditions.energy.converters[electric_ducted_fan.motor.tag]
                motor_conditions.inputs.voltage[:, 0] = bus.voltage # [V]
                motor_conditions.inputs.current[:, 0] = 100      # [A]
            
                # Run BEMT
                segment.state.conditions.expand_rows(ctrl_pts)
                ducted_fan_conditions             = segment.state.conditions.energy[bus.tag][electric_ducted_fan.tag][electric_ducted_fan.ducted_fan.tag]     
                ducted_fan_conditions.omega[:,0]  = ((tip_mach[k]*atmo_data.speed_of_sound[0,0]) /DF.tip_radius) 
                compute_ducted_fan_performance(electric_ducted_fan,segment.state,bus)
                
                thrust[i, j, k] = segment.state.conditions.energy[bus.tag][electric_ducted_fan.tag][electric_ducted_fan.ducted_fan.tag].thrust[0, 0] 
                torque[i, j, k] = segment.state.conditions.energy[bus.tag][electric_ducted_fan.tag][electric_ducted_fan.ducted_fan.tag].torque[0, 0]
                power[i, j, k]  = segment.state.conditions.energy[bus.tag][electric_ducted_fan.tag][electric_ducted_fan.ducted_fan.tag].power[0, 0]

    return

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()

