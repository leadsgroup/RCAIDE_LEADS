# RCAIDE/Library/Methods/Weights/Correlation_Buildups/General_Aviation/compute_systems_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE
import RCAIDE
from RCAIDE.Framework.Core import  Units , Data 

# ----------------------------------------------------------------------------------------------------------------------
# Systems Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_systems_weight(vehicle, V_fuel, V_int, N_tank, N_eng):
    """ output = RCAIDE.Methods.Weights.Correlations.General_Avation.systems(num_seats, ctrl_type, S_h, S_v, S_gross_w, ac_type)
        Calculate the weight of the different engine systems on the aircraft

        Source:
            Raymer, Aircraft Design: A Conceptual Approach (pg 461 in 4th edition)

        Inputs:
            V_fuel              - total fuel volume                     [meters**3]
            V_int               - internal fuel volume                  [meters**3]
            N_tank              - number of fuel tanks                  [dimensionless]
            N_eng               - number of engines                     [dimensionless]
            span                - wingspan                              [meters]
            TOW                 - gross takeoff weight of the aircraft  [kg]
            num_seats           - total number of seats on the aircraft [dimensionless]
            mach_number         - mach number                           [dimensionless]
            has_air_conditioner - integer of 1 if the vehicle has ac, 0 if not

        Outputs:
            output - a data dictionary with fields:
                W_flight_controls - weight of the flight control system [kilograms]
                W_apu - weight of the apu [kilograms]
                W_hyd_pnu - weight of the hydraulics and pneumatics [kilograms]
                W_avionics - weight of the avionics [kilograms]
                W_opitems - weight of the optional items based on the type of aircraft [kilograms]
                W_electrical - weight of the electrical items [kilograms]
                W_ac - weight of the air conditioning and anti-ice system [kilograms]
                W_furnish - weight of the furnishings in the fuselage [kilograms]
    """ 
    # unpack inputs 
    l_fuselage = 0
    for fuselage in vehicle.fuselages:
        if l_fuselage < fuselage.lengths.total:
            ref_fuselage = fuselage
            l_fuselage = ref_fuselage.lengths.total

    TOW        = vehicle.mass_properties.max_takeoff
    Nult       = vehicle.flight_envelope.ultimate_load 
    num_seats  = vehicle.passengers
    mach_number = vehicle.flight_envelope.design_mach_number
    span        = vehicle.wings.main_wing.spans.projected

    Q_tot  = V_fuel/Units.gallons
    Q_int  = V_int/Units.gallons 
    l_fus  = l_fuselage / Units.ft  # Convert meters to ft
    b_wing = span/Units.ft 
    W_0    = TOW/Units.lb

    has_air_conditioner = 0
    if 'air_conditioner' in vehicle:
        has_air_conditioner = 1
    
    # Fuel system
    W_fuel_system = 2.49*(Q_tot**.726)*((Q_tot/(Q_tot+Q_int))**.363)*(N_tank**.242)*(N_eng**.157)*Units.lb

    # Flight controls
    W_flight_controls = .053*(l_fus**1.536)*(b_wing**.371)*((Nult*W_0**(10.**(-4.)))**.8)*Units.lb
    
    # Hydraulics & Pneumatics Group Wt
    hyd_pnu_wt = (.001*W_0) * Units.lb

    # Avionics weight
    if len(vehicle.avionics) == 0:
        avionics     = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
        W_uav        = 0. 
    else:
        avionics = vehicle.avionics
        W_uav    = avionics.mass_properties.uninstalled
    
    W_avionics = 2.117*((W_uav/Units.lbs)**.933)*Units.lb 

    # Electrical Group Wt
    W_electrical = 12.57*((W_avionics/Units.lb + W_fuel_system/Units.lb)**.51)*Units.lb

    # Environmental Control 
    W_air_conditioning = has_air_conditioner*.265*(W_0**.52)*((1. * num_seats)**.68)*((W_avionics/Units.lb)**.17)*(mach_number**.08)*Units.lb

    # Furnishings Group Wt
    W_furnish = (.0582*W_0-65.)*Units.lb

    # packup outputs
    output = Data()   
    output.W_flight_control    = W_flight_controls
    output.W_hyd_pnu           = hyd_pnu_wt
    output.W_avionics          = W_avionics
    output.W_electrical        = W_electrical
    output.W_ac                = W_air_conditioning
    output.W_furnish           = W_furnish
    output.W_fuel_system       = W_fuel_system
    output.total               = output.W_flight_control + output.W_hyd_pnu \
                                  + output.W_ac + output.W_avionics + output.W_electrical \
                                  + output.W_furnish + output.W_fuel_system
    
    # Assign mass properties to components
    if has_air_conditioner:
        vehicle.air_conditioner.mass_properties.mass    = output.empty.systems.air_conditioner 
    
    avionics.mass_properties.mass           = W_avionics
    vehicle.avionics                                    = avionics

    control_systems                                  = RCAIDE.Library.Components.Component()
    control_systems.tag                              = 'control_systems'  
    electrical_systems                               = RCAIDE.Library.Components.Component()
    electrical_systems.tag                           = 'electrical_systems'
    furnishings                                      = RCAIDE.Library.Components.Component()
    furnishings.tag                                  = 'furnishings'
    air_conditioner                                  = RCAIDE.Library.Components.Component() 
    air_conditioner.tag                              = 'air_conditioner' 
    hydraulics                                       = RCAIDE.Library.Components.Component()
    hydraulics.tag                                   = 'hydraulics'  

    control_systems.mass_properties.mass    = W_flight_controls
    electrical_systems.mass_properties.mass = W_electrical
    furnishings.mass_properties.mass        = W_furnish
    air_conditioner.mass_properties.mass    = W_air_conditioning
    hydraulics.mass_properties.mass         = hyd_pnu_wt

    # assign components to vehicle
    vehicle.control_systems                             = control_systems
    vehicle.electrical_systems                          = electrical_systems
    vehicle.furnishings                                 = furnishings 
    vehicle.hydraulics                                  = hydraulics
    

    return output