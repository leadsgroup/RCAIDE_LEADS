# RCAIDE/Library/Methods/Weights/Buildups/eVTOL/compute_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE
import RCAIDE
from RCAIDE.Framework.Core                                          import Units, Data 
import RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Electric.VTOL.Physics_Based as EVTOL
import RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Electric.Common as EVTOL_Common

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
# Compute Operating Empty Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_operating_empty_weight(vehicle,settings = None): 
    """ Calculates the empty vehicle mass for an EVTOL-type aircraft including seats,
        avionics, servomotors, ballistic recovery system, rotor and hub assembly,
        transmission, and landing gear. 
        
        Sources:
        Project Vahana Conceptual Trade Study
        https://github.com/VahanaOpenSource


        Inputs: 
            vehicle:                     RCAIDE Config Data Stucture 
            max_tip_mach:               Allowable Tip Mach Number                      [Unitless]
            disk_area_factor:           Inverse of Disk Area Efficiency                [Unitless]
            max_thrust_to_weight_ratio: Allowable Thrust to Weight Ratio               [Unitless]
            safety_factor               Safety Factor in vehicle design                [Unitless]
            max_g_load                  Maximum g-forces load for certification        [UNitless]
            motor_efficiency:           Motor Efficiency                               [Unitless]

        Outputs: 
            outputs:                    Data Dictionary of Component Masses [kg]

        Output data dictionary has the following book-keeping hierarchical structure:

            Output
                Total.
                    Empty.
                        Structural.
                            Fuselage
                            Wings
                            Landing Gear
                            Rotors
                            Hubs
                        Seats
                        Battery
                        Motors
                        Servo
                    Systems.
                        Avionics
                        ECS               - Environmental Control System
                        BRS               - Ballistic Recovery System
                        Wiring            - Aircraft Electronic Wiring
                    Payload

    """
    
    if settings == None: 
        miscelleneous_weight_factor   = 1.1
        safety_factor                 = 1.5   
        disk_area_factor              = 1.15     
        max_thrust_to_weight_ratio    = 1.1
        max_g_load                    = 3.8
    else:
        miscelleneous_weight_factor   = settings.miscelleneous_weight_factor
        safety_factor                 = settings.safety_factor              
        disk_area_factor              = settings.disk_area_factor           
        max_thrust_to_weight_ratio    = settings.max_thrust_to_weight_ratio 
        max_g_load                    = settings.max_g_load  
               
    # Set up data structures for RCAIDE weight methods
    weight                                  = Data()  
    weight.battery                          = 0.0
    weight.fuel_cell                        = 0.0
    weight.payload                          = 0.0
    weight.servos                           = 0.0
    weight.hubs                             = 0.0
    weight.booms                            = 0.0
    weight.BRS                              = 0.0  
    weight.motors                           = 0.0
    weight.rotors                           = 0.0
    weight.rotors                           = 0.0
    weight.fuselage                         = 0.0
    weight.wiring                           = 0.0
    weight.wings                            = Data()
    weight.wings_total                      = 0.0
    weight.thermal_management_system       = Data()


    control_systems                                  = RCAIDE.Library.Components.Component()
    control_systems.tag                              = 'control_systems'  
    electrical_systems                               = RCAIDE.Library.Components.Component()
    electrical_systems.tag                           = 'electrical_systems'
    furnishings                                      = RCAIDE.Library.Components.Component()
    furnishings.tag                                  = 'furnishings'
    air_conditioner                                  = RCAIDE.Library.Components.Component() 
    air_conditioner.tag                              = 'air_conditioner'
    apu                                              = RCAIDE.Library.Components.Component()
    apu.tag                                          = 'apu'
    hydraulics                                       = RCAIDE.Library.Components.Component()
    hydraulics.tag                                   = 'hydraulics'
    avionics                                         = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    optionals                                        = RCAIDE.Library.Components.Component()
    optionals.tag                                    = 'optionals'
    

    vehicle.payload.passengers      = RCAIDE.Library.Components.Component()
    vehicle.payload.passengers.tag  = 'passengers'  
    vehicle.payload.baggage         = RCAIDE.Library.Components.Component()
    vehicle.payload.baggage.tag     = 'baggage'
    vehicle.payload.cargo           = RCAIDE.Library.Components.Component()   
    vehicle.payload.cargo.tag       = 'cargo'

    # assign components to vehicle
    vehicle.systems.control_systems    = control_systems
    vehicle.systems.electrical_systems = electrical_systems
    vehicle.systems.avionics           = avionics
    vehicle.systems.furnishings        = furnishings
    vehicle.systems.air_conditioner    = air_conditioner 
    vehicle.systems.apu                = apu
    vehicle.systems.hydraulics         = hydraulics
    vehicle.systems.optionals          = optionals 

    #-------------------------------------------------------------------------------
    # Default Values
    #------------------------------------------------------------------------------- 
    MTOW                          = vehicle.mass_properties.max_takeoff
    atmosphere                    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data                     = atmosphere.compute_values(0, 0) 
    rho_ref                       = atmo_data.density[0, 0] 
    maxLift                       = MTOW * max_thrust_to_weight_ratio * 9.81           
    AvgBladeCD                    = 0.012         

    #-------------------------------------------------------------------------------
    # Fixed Weights
    #-------------------------------------------------------------------------------
    weight.seats        = vehicle.passengers * 15.   * Units.kg
    weight.passengers   = vehicle.passengers * 70.   * Units.kg
    weight.avionics     = 15.                        * Units.kg
    weight.landing_gear = MTOW * 0.02                * Units.kg
    weight.ECS          = vehicle.passengers * 7.    * Units.kg

    # Determine length scale 
    length_scale = 1.  
    if len(vehicle.fuselages) == 0.:
        for wing in vehicle.wings:
            if isinstance(wing ,RCAIDE.Library.Components.Wings.Main_Wing): 
                if wing.chords.root>length_scale:
                    length_scale = wing.chords.root
                    nose_length  = 0.25*wing.chords.root
    else:
        for fuse in vehicle.fuselages:
            nose   = fuse.lengths.nose
            length = fuse.lengths.total
            if length > length_scale:
                length_scale = length
                nose_length  = nose 
                            
    #-------------------------------------------------------------------------------
    # Environmental Control System
    #-------------------------------------------------------------------------------
    vehicle.systems.air_conditioner.origin[0][0]          = 0.51 * length_scale
    vehicle.systems.air_conditioner.mass_properties.mass  = weight.ECS

    #-------------------------------------------------------------------------------
    # Network Weight
    #-------------------------------------------------------------------------------
    maxLiftPower           = 0
    total_number_of_rotors = 0
    maxVTip                = 0
    eta                    = 0
    for network in vehicle.networks: 
        for bus in network.busses:  
            #------------------------------------------------------------------------------- 
            # Payload Weight
            #-------------------------------------------------------------------------------
            if bus.payload.origin[0][0] == 0:
                bus.payload.origin[0][0]  = 0.5 * length_scale
            weight.payload  += bus.payload.mass_properties.mass * Units.kg
     
            #-------------------------------------------------------------------------------
            # Avionics Weight
            #-------------------------------------------------------------------------------
            if bus.avionics.origin[0][0] == 0:
                bus.avionics.origin[0][0]                          = 0.4 * nose_length
            bus.avionics.mass_properties.center_of_gravity[0][0]   = 0.0
            weight.avionics += bus.avionics.mass_properties.mass   
                         
            for modules in bus.battery_modules: 
                weight.battery += modules.mass_properties.mass * Units.kg 
    
            for fuel_cell in bus.fuel_cell_stacks: 
                weight.fuel_cell += fuel_cell.mass_properties.mass * Units.kg                 
        
            # Servo, Hub and BRS Weights
            lift_rotor_hub_weight   = 4.   * Units.kg
            prop_hub_weight         = 4.   * Units.kg
            lift_rotor_BRS_weight   = 16.  * Units.kg
    
            # Rotor Weight
            number_of_propellers    = 0.0  
            number_of_lift_rotors   = 0.0   
            total_number_of_rotors  = 0.0      
            lift_rotor_servo_weight = 0.0  
            
        for propulsor in network.propulsors:
            rotor = propulsor.rotor   
            if type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Propeller:
                ''' Propeller Weight '''  
                number_of_propellers       += 1   
                rTip_ref                   = rotor.tip_radius 
                bladeSol_ref               = rotor.blade_solidity 
                prop_servo_weight          = 5.2 * Units.kg  
                propeller_mass             = EVTOL_Common.compute_rotor_weight(rotor, maxLift/5.) * Units.kg
                weight.rotors              += propeller_mass 
                rotor.mass_properties.mass  =  propeller_mass + prop_hub_weight + prop_servo_weight
                maxVTip                     = rotor.cruise.design_angular_velocity * rotor.tip_radius
                weight.servos              += prop_servo_weight
                weight.hubs                += prop_hub_weight
                
            if (type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor or type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor) or type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Rotor:
                ''' Lift Rotor, Prop-Rotor or Rotor Weight '''  
                number_of_lift_rotors       += 1  
                rTip_ref                    = rotor.tip_radius
                bladeSol_ref                = rotor.blade_solidity 
                maxVTip                     = rotor.hover.design_angular_velocity * rotor.tip_radius
                lift_rotor_servo_weight     = 0.65 * Units.kg 
                if rotor.oei.design_thrust == None:
                    design_thrust = rotor.hover.design_thrust
                else:
                    design_thrust =rotor.oei.design_thrust
                lift_rotor_mass             = EVTOL_Common.compute_rotor_weight(rotor,design_thrust)
                weight.rotors               += lift_rotor_mass 
                rotor.mass_properties.mass  =  lift_rotor_mass + lift_rotor_hub_weight + lift_rotor_servo_weight
                weight.servos               += lift_rotor_servo_weight
                weight.hubs                 += lift_rotor_hub_weight 
            
            # Motor 
            eta             = propulsor.motor.efficiency  
            weight.motors  += propulsor.motor.mass_properties.mass  
                
        total_number_of_rotors  = int(number_of_lift_rotors + number_of_propellers)  
        if total_number_of_rotors > 1:
            prop_BRS_weight     = 16.   * Units.kg
        else:
            prop_BRS_weight     = 0.   * Units.kg
 
        # Add associated weights  
        weight.BRS   += (prop_BRS_weight + lift_rotor_BRS_weight)  
        maxLiftPower = 1.15*maxLift*(disk_area_factor*np.sqrt(maxLift/(2*rho_ref*np.pi*rTip_ref**2)) + bladeSol_ref*AvgBladeCD/8*maxVTip**3/(maxLift/(rho_ref*np.pi*rTip_ref**2)))
        # Tail Rotor
        if number_of_lift_rotors == 1: # this assumes that the vehicle is an electric helicopter with a tail rotor 
            maxLiftOmega   = maxVTip/rTip_ref
            maxLiftTorque  = maxLiftPower / maxLiftOmega  
            for bus in network.busses: 
                tailrotor = next(iter(bus.lift_rotors))
                weight.tail_rotor  = EVTOL_Common.compute_rotor_weight(tailrotor, 1.5*maxLiftTorque/(1.25*rTip_ref))*0.2 * Units.kg
                weight.rotors     += weight.tail_rotor 

    #-------------------------------------------------------------------------------
    # Thermal Management System Weight
    #-------------------------------------------------------------------------------
    tms_weight = 0.0
    for network in vehicle.networks:
        for coolant_line in network.coolant_lines:
            weight.thermal_management_system.battery_module = Data()  # Add container for battery module
            for i, battery_module in enumerate(coolant_line.battery_modules):
                module_key = f'module_{i+1}'  # Create unique key for each module
                weight.thermal_management_system.battery_module[module_key] = 0.0  # Initialize weight
                for HAS in battery_module:
                    weight.thermal_management_system.battery_module[module_key] = HAS.mass_properties.mass
                    tms_weight +=  HAS.mass_properties.mass

            for tag, item in coolant_line.items(): 
                if tag == 'heat_exchangers':
                    for heat_exchanger in item:
                        weight.thermal_management_system[heat_exchanger.tag] = heat_exchanger.mass_properties.mass
                        tms_weight +=  heat_exchanger.mass_properties.mass
                if tag == 'reservoirs':
                    for reservoir in item:
                        weight.thermal_management_system[reservoir.tag] = reservoir.mass_properties.mass
                        tms_weight +=  reservoir.mass_properties.mass
    weight.thermal_management_system.total = tms_weight
    #-------------------------------------------------------------------------------
    # Wing and Motor Wiring Weight
    #-------------------------------------------------------------------------------
    maxSpan =  0
    for wing in vehicle.wings: 
        maxSpan = max(wing.spans.projected, maxSpan)  
        if wing.symbolic:
            wing_weight = 0
        else:
            wing_weight               = EVTOL.compute_wing_weight(wing, vehicle, maxLift/5, safety_factor= safety_factor, max_g_load =  max_g_load )
            wing_tag                  = wing.tag
            weight.wings[wing_tag]    = wing_weight
            wing.mass_properties.mass = wing_weight 
        weight.wings_total           += wing_weight

        # compute_wiring_weight weight
        if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
            wiring_weight  = EVTOL_Common.compute_wiring_weight(wing, vehicle, maxLiftPower/(eta*total_number_of_rotors)) * Units.kg
        else:
            wiring_weight =  0
        weight.wiring  += wiring_weight 

    #-------------------------------------------------------------------------------
    # Landing Gear Weight
    #-------------------------------------------------------------------------------  
    main_landing_gear =  False
    for LG in vehicle.landing_gears:
        if isinstance(LG, RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear):
            LG.mass_properties.mass =  weight.landing_gear
            main_landing_gear = True 
    if main_landing_gear == False:
        main_gear = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()  
        main_gear.mass_properties.mass =  weight.landing_gear
        vehicle.append_component(main_gear)
        
    #-------------------------------------------------------------------------------
    # Fuselage  Weight  
    #-------------------------------------------------------------------------------
    for fuse in  vehicle.fuselages: 
        fuselage_weight = EVTOL.compute_fuselage_weight(fuse, maxSpan, MTOW )  
        fuse.mass_properties.center_of_gravity[0][0] = .45*fuse.lengths.total
        fuse.mass_properties.mass                    =  fuselage_weight + weight.passengers + weight.seats + weight.wiring + weight.BRS
        weight.fuselage += fuselage_weight  

    #-------------------------------------------------------------------------------
    # Boom Weight
    #-------------------------------------------------------------------------------
    for boom in vehicle.booms:
        boom_weight                = EVTOL.compute_boom_weight(boom) * Units.kg
        weight.booms               += boom_weight 
        boom.mass_properties.mass  =  boom_weight 
    
    #-------------------------------------------------------------------------------
    # Pack Up Outputs
    #-------------------------------------------------------------------------------
    output                                            = Data()
    output.empty                                      = Data()    
    output.empty.structural                           = Data()
    output.empty.structural.booms                     = miscelleneous_weight_factor *weight.booms 
    output.empty.structural.fusleage                  = miscelleneous_weight_factor *weight.fuselage  
    output.empty.structural.landing_gear              = miscelleneous_weight_factor *weight.landing_gear  
    output.empty.structural.wings                     = miscelleneous_weight_factor *weight.wings_total 
    output.empty.structural.total                     = weight.booms + weight.fuselage + weight.landing_gear +weight.wings_total 
             
    output.empty.propulsion                           = Data()
    output.empty.propulsion.motors                    = miscelleneous_weight_factor *weight.motors
    output.empty.propulsion.rotors                    = miscelleneous_weight_factor *weight.rotors
    output.empty.propulsion.hubs                      = miscelleneous_weight_factor *weight.hubs 
    output.empty.propulsion.servos                    = miscelleneous_weight_factor *weight.servos
    output.empty.propulsion.wiring                    = miscelleneous_weight_factor *weight.wiring 
    output.empty.propulsion.battery                   = miscelleneous_weight_factor *weight.battery
    output.empty.propulsion.fuel_cell                 = miscelleneous_weight_factor *weight.fuel_cell
    output.empty.propulsion.TMS                       = miscelleneous_weight_factor *weight.thermal_management_system.total
    output.empty.propulsion.total                     = weight.rotors + weight.hubs  +  weight.fuel_cell  +  weight.battery +  weight.motors +   weight.wiring +   weight.servos +  weight.thermal_management_system.total 

    output.empty.systems                              = Data()
    output.empty.systems.environmental_control_system = miscelleneous_weight_factor * weight.ECS
    output.empty.systems.avionics                     = miscelleneous_weight_factor * weight.avionics  
    output.empty.systems.seats                        = miscelleneous_weight_factor * weight.seats  
    output.empty.systems.balistic_recovery_system     = miscelleneous_weight_factor * weight.BRS
    output.empty.systems.total                        = weight.ECS + weight.avionics +   weight.BRS +  weight.seats  
    
    output.empty.total        = output.empty.systems.total +  output.empty.propulsion.total +  output.empty.structural.total
    output.payload            = Data()
    output.payload.total      = weight.passengers + weight.payload
    output.payload.passengers = weight.passengers  
    output.payload.payload    = weight.payload 
    output.zero_fuel_weight   = output.empty.total + output.payload.total 
    output.fuel               = 0
    output.total              = output.empty.total + output.payload.total 
    return output






