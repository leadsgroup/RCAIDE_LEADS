# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Conventional/General_Aviation/Raymer/compute_operating_empty_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE
import RCAIDE
from RCAIDE.Framework.Core import  Units , Data 
import RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.General_Aviation.Raymer as Raymer

# ----------------------------------------------------------------------------------------------------------------------
# Main Wing Weight 
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_operating_empty_weight(vehicle, settings=None):
    """ output = RCAIDE.Methods.Weights.Correlations.Tube_Wing.empty(engine,wing,aircraft,fuselage,horizontal,vertical)
        Computes the empty weight breakdown of a General Aviation type aircraft  
        
        Inputs:
            engine - a data dictionary with the fields:                    
                thrust_sls - sea level static thrust of a single engine [Newtons]

            vehicle - a data dictionary with the fields:                    
                reference_area                                                            [meters**2]
                envelope - a data dictionary with the fields:
                    ultimate_load - ultimate load of the aircraft                         [dimensionless]
                    limit_load    - limit load factor at zero fuel weight of the aircraft [dimensionless]
                
                mass_properties - a data dictionary with the fields:
                    max_takeoff   - max takeoff weight of the vehicle           [kilograms]
                    max_zero_fuel - maximum zero fuel weight of the aircraft    [kilograms]
                    cargo         - cargo weight                                [kilograms]
                
                passengers - number of passengers on the aircraft               [dimensionless]
                        
                design_dynamic_pressure - dynamic pressure at cruise conditions [Pascal]
                design_mach_number      - mach number at cruise conditions      [dimensionless]
                
                networks - a data dictionary with the fields: 
                    keys           - identifier for the type of network; different types have different fields
                        turbofan
                            thrust_sls - sealevel standard thrust                               [Newtons]             
                        internal_combustion
                            rated_power - maximum rated power of the internal combustion engine [Watts]
                        
                    number_of_engines - integer indicating the number of engines on the aircraft

                W_cargo - weight of the bulk cargo being carried on the aircraft [kilograms]
                num_seats - number of seats installed on the aircraft [dimensionless]
                ctrl - specifies if the control system is "fully powered", "partially powered", or not powered [dimensionless]
                ac - determines type of instruments, electronics, and operating items based on types: 
                    "short-range", "medium-range", "long-range", "business", "cargo", "commuter", "sst" [dimensionless]
                w2h - tail length (distance from the airplane c.g. to the horizontal tail aerodynamic center) [meters]
                
                fuel - a data dictionary with the fields: 
                    mass_properties  - a data dictionary with the fields:
                        mass -mass of fuel [kilograms]
                    density          - gravimetric density of fuel                             [kilograms/meter**3]    
                    number_of_tanks  - number of external fuel tanks                           [dimensionless]
                    internal_volume  - internal fuel volume contained in the wing              [meters**3]
                wings - a data dictionary with the fields:    
                    wing - a data dictionary with the fields:
                        span                      - span of the wing                           [meters]
                        taper                     - taper ratio of the wing                    [dimensionless]
                        thickness_to_chord        - thickness-to-chord ratio of the wing       [dimensionless]
                        chords - a data dictionary with the fields:
                            mean_aerodynamic - mean aerodynamic chord of the wing              [meters]
                            root             - root chord of the wing                          [meters]
                            
                            
                        sweeps - a data dictionary with the fields:
                            quarter_chord - quarter chord sweep angle of the wing              [radians]
                        mac                       - mean aerodynamic chord of the wing         [meters]
                        r_c                       - wing root chord                            [meters]
                        origin  - location of the leading edge of the wing relative to the front of the fuselage                                      [meters,meters,meters]
                        aerodynamic_center - location of the aerodynamic center of the horizontal_stabilizer relative to the leading edge of the wing [meters,meters,meters]
        
                    
                    
                    
                    horizontal_stabilizer - a data dictionary with the fields:
                        areas -  a data dictionary with the fields:
                            reference - reference area of the horizontal stabilizer                                    [meters**2]
                            exposed  - exposed area for the horizontal tail                                            [meters**2]
                        taper   - taper ratio of the horizontal stabilizer                                             [dimensionless]
                        span    - span of the horizontal tail                                                          [meters]
                        sweeps - a data dictionary with the fields:
                            quarter_chord - quarter chord sweep angle of the horizontal stabilizer                     [radians]
                        chords - a data dictionary with the fields:
                            mean_aerodynamic - mean aerodynamic chord of the horizontal stabilizer                     [meters]         
                            root             - root chord of the horizontal stabilizer             
                        thickness_to_chord - thickness-to-chord ratio of the horizontal tail                           [dimensionless]
                        mac     - mean aerodynamic chord of the horizontal tail                                        [meters]
                        origin  - location of the leading of the horizontal tail relative to the front of the fuselage                                                 [meters,meters,meters]
                        aerodynamic_center - location of the aerodynamic center of the horizontal_stabilizer relative to the leading edge of the horizontal stabilizer [meters,meters,meters]
        
                    vertical - a data dictionary with the fields:
                        areas -  a data dictionary with the fields:
                            reference - reference area of the vertical stabilizer         [meters**2]
                        span    - span of the vertical tail                               [meters]
                        taper   - taper ratio of the horizontal stabilizer                [dimensionless]
                        t_c     - thickness-to-chord ratio of the vertical tail           [dimensionless]
                        sweeps   - a data dictionary with the fields:
                            quarter_chord - quarter chord sweep angle of the vertical stabilizer [radians]
                        t_tail - flag to determine if aircraft has a t-tail, "yes"               [dimensionless]


                
                fuselages - a data dictionary with the fields:  
                    fuselage - a data dictionary with the fields:
                        areas             - a data dictionary with the fields:
                            wetted - wetted area of the fuselage [meters**2]
                        differential_pressure  - Maximum fuselage pressure differential   [Pascal]
                        width             - width of the fuselage                         [meters]
                        heights - a data dictionary with the fields:
                            maximum - height of the fuselage                              [meters]
                        lengths-  a data dictionary with the fields:
                            structure - structural length of the fuselage                 [meters]                     
                        mass_properties - a data dictionary with the fields:
                            volume - total volume of the fuselage                         [meters**3]
                            internal_volume - internal volume of the fuselage             [meters**3]
                        number_coach_sets - number of seats on the aircraft               [dimensionless]    
                landing_gear - a data dictionary with the fields:
                    main - a data dictionary with the fields:
                        strut_length - strut length of the main gear                      [meters]
                    nose - a data dictionary with the fields:
                        strut_length - strut length of the nose gear                      [meters]
                avionics - a data dictionary, used to determine if avionics weight is calculated, don't include if vehicle has none
                air_conditioner - a data dictionary, used to determine if air conditioner weight is calculated, don't include if vehicle has none
        
        
        Outputs:
            output - a data dictionary with fields:
                wing - wing weight                            [kilograms]
                fuselage - fuselage weight                    [kilograms]
                propulsion - propulsion                       [kilograms]
                landing_gear_main - main gear weight          [kilograms]
                landing_gear_nose - nose gear weight          [kilograms]
                horizonal_tail - horizontal stabilizer weight [kilograms]
                vertical_tail - vertical stabilizer weight    [kilograms]
                systems - total systems weight                [kilograms]
                systems - a data dictionary with fields:
                    control_systems - control systems weight  [kilograms]
                    hydraulics - hydraulics weight            [kilograms]
                    avionics - avionics weight                [kilograms]
                    electric - electrical systems weight      [kilograms]
                    air_conditioner - air conditioner weight  [kilograms]
                    furnish - furnishing weight               [kilograms]
                    fuel_system - fuel system weight          [ kilograms]
           Wing, empannage, fuselage, propulsion and individual systems masses updated with their calculated values
       Assumptions:
            calculated aircraft weight from correlations created per component of historical aircraft
        
    """     

    if settings == None: 
        use_max_fuel_weight = True 
    else:
        use_max_fuel_weight = settings.use_max_fuel_weight
        
    # Unpack inputs
    Nult        = vehicle.flight_envelope.ultimate_load 
    TOW         = vehicle.mass_properties.max_takeoff 
    num_pax     = vehicle.passengers
    W_cargo     = vehicle.mass_properties.cargo 
    mach_number = vehicle.flight_envelope.design_mach_number
 
    landing_weight              = TOW
    m_fuel                      =  0
    number_of_tanks             =  0
    V_fuel                      =  0
    V_fuel_int                  =  0
    W_energy_network_cumulative =  0
    number_of_engines           =  0
 
    for network in vehicle.networks:
        W_energy_network_total   = 0

        for fuel_line in  network.fuel_lines: 
            for fuel_tank in fuel_line.fuel_tanks: 
                m_fuel_tank     = fuel_tank.fuel.mass_properties.mass
                m_fuel          += m_fuel_tank   
                landing_weight  -= m_fuel_tank   
                number_of_tanks += 1
                V_fuel_int      += m_fuel_tank/fuel_tank.fuel.density  #assume all fuel is in integral tanks 
                V_fuel          += m_fuel_tank/fuel_tank.fuel.density #total fuel  
         
        # Electric-Powered Propulsors  
        for bus in network.busses: 
            # electrical payload 
            W_energy_network_total  += bus.payload.mass_properties.mass * Units.kg
     
            # Avionics Weight 
            W_energy_network_total  += bus.avionics.mass_properties.mass      
    
            for battery in bus.battery_modules: 
                W_energy_network_total  += battery.mass_properties.mass * Units.kg
                  
            for propulsor in bus.propulsors:
                if 'motor' in propulsor: 
                    motor_mass = propulsor.motor.mass_properties.mass       
                    W_energy_network_cumulative  += motor_mass                
        
        # Fuel network
        W_propulsion = Raymer.compute_propulsion_system_weight(network)      
                
        W_energy_network_cumulative = W_propulsion.W_prop
        number_of_engines           =  W_propulsion.number_of_engines
    
    # Main
    for wing in vehicle.wings:
        if isinstance(wing,RCAIDE.Library.Components.Wings.Main_Wing):
            W_wing    = Raymer.compute_main_wing_weight(wing, vehicle, m_fuel)
            wing.mass_properties.mass = W_wing
            
            # set main wing to be used in future horizontal tail calculations 
            main_wing  =  wing
    
    # Empennage
    l_w2h = 0
    W_tail_horizontal =  0
    W_tail_vertical   =  0
    for wing in vehicle.wings:            
        if isinstance(wing,RCAIDE.Library.Components.Wings.Horizontal_Tail):
            l_w2h              = wing.origin[0][0] + wing.aerodynamic_center[0] - main_wing.origin[0][0] - main_wing.aerodynamic_center[0] 
            W_tail_horizontal  = Raymer.compute_horizontal_tail_weight(wing, vehicle)                 
            wing.mass_properties.mass = W_tail_horizontal     
        if isinstance(wing,RCAIDE.Library.Components.Wings.Vertical_Tail):     
            W_tail_vertical   = Raymer.compute_vertical_tail_weight(wing, vehicle) 
            wing.mass_properties.mass = W_tail_vertical
    if l_w2h == 0:
        print("Warning: l_w2h is zero")

    # Fuselage
    for fuselage in  vehicle.fuselages:  
        W_fuselage  = Raymer.compute_fuselage_weight(fuselage, vehicle, l_w2h)
        fuselage.mass_properties.mass = W_fuselage
        
    # landing gear 
    strut_length_main = 0
    strut_length_nose = 0 
    nose_landing_gear = False
    main_landing_gear = False
    for LG in vehicle.landing_gears:
        if isinstance(LG, RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear):
            strut_length_main = LG.strut_length
            main_landing_gear = True
        elif isinstance(LG, RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear):
            strut_length_nose = LG.strut_length 
            nose_landing_gear = True
    W_landing_gear         = Raymer.compute_landing_gear_weight(landing_weight, Nult, strut_length_main, strut_length_nose) 
    for landing_gear in vehicle.landing_gears:
        if isinstance(landing_gear, RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear):
            landing_gear.mass_properties.mass = W_landing_gear.main
            main_landing_gear = True
        elif isinstance(landing_gear, RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear):
            landing_gear.mass_properties.mass = W_landing_gear.nose
            nose_landing_gear = True 
    if nose_landing_gear == False:
        nose_gear = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()  
        nose_gear.mass_properties.mass = W_landing_gear.nose
        vehicle.append_component(nose_gear) 
    if main_landing_gear == False:
        main_gear = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()  
        main_gear.mass_properties.mass = W_landing_gear.main
        vehicle.append_component(main_gear)

    # Calculating Empty Weight of Aircraft
    W_systems           = Raymer.compute_systems_weight(vehicle,V_fuel, V_fuel_int, number_of_tanks, number_of_engines)

    # Calculate the equipment empty weight of the aircraft

    W_empty           = (W_wing + W_fuselage + W_landing_gear.main+W_landing_gear.nose + W_energy_network_cumulative + W_systems.total + \
                          W_tail_horizontal +W_tail_vertical) 

    # packup outputs
    W_payload = Raymer.compute_payload_weight(vehicle)
    
    vehicle.payload.passengers = RCAIDE.Library.Components.Component()
    vehicle.payload.baggage    = RCAIDE.Library.Components.Component()
    vehicle.payload.cargo      = RCAIDE.Library.Components.Component()
    
    vehicle.payload.passengers.mass_properties.mass = W_payload.passengers
    vehicle.payload.baggage.mass_properties.mass    = W_payload.baggage
    vehicle.payload.cargo.mass_properties.mass      = W_payload.cargo        


    # Distribute all weight in the output fields
    output                                    = Data()
    output.empty                              = Data()
    output.empty.structural                   = Data()
    output.empty.structural.wings             = W_wing +  W_tail_horizontal + W_tail_vertical 
    output.empty.structural.fuselage          = W_fuselage
    output.empty.structural.landing_gear      = W_landing_gear.main +  W_landing_gear.nose 
    output.empty.structural.nacelle           = 0
    output.empty.structural.paint             = 0  
    output.empty.structural.total             = output.empty.structural.wings \
                                                     + output.empty.structural.fuselage  + output.empty.structural.landing_gear \
                                                     + output.empty.structural.paint + output.empty.structural.nacelle
          
    output.empty.propulsion                   = Data()
    output.empty.propulsion.total             = W_energy_network_cumulative
    output.empty.propulsion.fuel_system       = W_systems.W_fuel_system
  
    output.empty.systems                      = Data()
    output.empty.systems.control_systems      = W_systems.W_flight_control
    output.empty.systems.hydraulics           = W_systems.W_hyd_pnu
    output.empty.systems.avionics             = W_systems.W_avionics
    output.empty.systems.electrical           = W_systems.W_electrical
    output.empty.systems.air_conditioner      = W_systems.W_ac
    output.empty.systems.furnishings              = W_systems.W_furnish
    output.empty.systems.apu                  = 0
    output.empty.systems.instruments          = 0
    output.empty.systems.anti_ice             = 0
    output.empty.systems.total                = output.empty.systems.control_systems + output.empty.systems.apu \
                                                  + output.empty.systems.electrical + output.empty.systems.avionics \
                                                  + output.empty.systems.hydraulics + output.empty.systems.furnishings \
                                                  + output.empty.systems.air_conditioner + output.empty.systems.instruments \
                                                  + output.empty.systems.anti_ice
  
    output.payload                                = Data()
    output.payload                                = W_payload
    output.operational_items                      = Data() # What is the point of these items?
    output.operational_items.oper_items           = 0
    output.operational_items.flight_crew          = 0
    output.operational_items.flight_attendants    = 0
    output.operational_items.total                = 0

    output.empty.total      = output.empty.structural.total + output.empty.propulsion.total + output.empty.systems.total
    output.operating_empty  = output.empty.total + output.operational_items.total
    output.zero_fuel_weight =  output.operating_empty + output.payload.total 

    if use_max_fuel_weight:  # assume fuel is equally distributed in fuel tanks
        total_fuel_weight  = vehicle.mass_properties.max_takeoff -  output.zero_fuel_weight
        for network in vehicle.networks: 
            for fuel_line in network.fuel_lines:  
                for fuel_tank in fuel_line.fuel_tanks:
                    fuel_weight =  total_fuel_weight/number_of_tanks  
                    fuel_tank.fuel.mass_properties.mass = fuel_weight
        output.fuel = total_fuel_weight 
        output.total = output.zero_fuel_weight + output.fuel
    else:
        total_fuel_weight =  0
        for network in vehicle.networks: 
            for fuel_line in network.fuel_lines:  
                for fuel_tank in fuel_line.fuel_tanks:
                    fuel_mass =  fuel_tank.fuel.density * fuel_tank.volume
                    fuel_tank.fuel.mass_properties.mass = fuel_mass * 9.81
                    total_fuel_weight = fuel_mass * 9.81 
        output.fuel = total_fuel_weight
        output.total = output.zero_fuel_weight + output.fuel  
    
    return output