 
# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    
 
# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def setup():
    
    base_vehicle = base_setup()
    configs = configs_setup(base_vehicle)
    
    return configs

def base_setup():

    # -----------------------------------s-------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    net = Battery_Electric_Rotor()
    net.voltage = 400.

    vehicle = vsp_read('Beta_Alia_optimized_rotors_2.vsp3',units_type = 'Imperial',specified_network =net)
    # vehicle = pickle.load(open('beta.RCAIDE', 'rb'))
    # net = vehicle.networks.battery_electric_rotor
    #plot_vehicle(vehicle,plot_control_points = False)
    
    #plt.show()
    
    # Setting up other characteristics not automatically imported
    vehicle.mass_properties.takeoff           = 6999. * Units.lb
    vehicle.mass_properties.operating_empty   = 6999. * Units.lb - 875. * Units.lb    
    vehicle.mass_properties.max_takeoff       = 6999. * Units.lb         
    vehicle.mass_properties.max_payload       = 875. * Units.lb  
    
    
    # basic parameters
    vehicle.flight_envelope.ultimate_load = 5.7
    vehicle.flight_envelope.positive_limit_load    = 3.    
    
    vehicle.number_of_passengers             = 5
    
    S = 0
    for wing in vehicle.wings:
        if len(wing.Segments.keys())>0:
            wing = segment_properties(wing,update_wet_areas=False)
        
        if wing.areas.reference>S:
            S = wing.areas.reference
            
    vehicle.reference_area = S
    
    vehicle.fuselages.fuselage_1.tag = 'fuselage'
    vehicle.append_component(vehicle.fuselages.pop('fuselage_1'))
    
    fk  = list(vehicle.fuselages.keys())
    
    for fuselage in fk:
        if fuselage == 'fuselage':
            continue
        else:
            boom = Rotor_Boom(vehicle.fuselages.pop(fuselage))
            boom.diameter = boom.effective_diameter
            boom.length   = boom.lengths.total
            boom.number_of_rotors = 2
            vehicle.append_component(boom)
    

    #------------------------------------------------------------------
    # Design Battery
    #------------------------------------------------------------------
    bat                      = RCAIDE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.specific_energy      = 187.5    *Units.Wh/Units.kg    
    bat.mass_properties.mass = 3150. * Units.lb
    bat.max_voltage          = net.voltage   
    initialize_from_mass(bat)
    net.battery              = bat     
    
    
    #------------------------------------------------------------------
    # Design Payload
    #------------------------------------------------------------------
    systems                      = RCAIDE.Library.Components.Powertrain.Systems.Systems()
    systems.power_draw           = 0.
    systems.mass_properties.mass = 875. * Units.lb    
    bus.systems                  = systems

    #------------------------------------------------------------------
    # Design Avionics
    #------------------------------------------------------------------
    avionics            = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw = 400. * Units.watts
    bus.systems        = systems
    
    #------------------------------------------------------------------
    # Design Electronic Speed Controller
    #------------------------------------------------------------------
    propeller_esc            = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    propeller_esc.efficiency = 0.97
    net.propeller_esc        = propeller_esc
    rotor_esc                = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    rotor_esc.efficiency     = 0.97
    net.lift_rotor_esc       = rotor_esc
        
    
    net.identical_flag = False
    
    
    #------------------------------------------------------------------
    # Design Motors
    #------------------------------------------------------------------

    for rotor in vehicle.networks.battery_electric_rotor.lift_rotors:
        

        # run the rotor
        altitude = np.array([[1000. * Units.feet]])
        atmosphere            = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        atmosphere_conditions = atmosphere.compute_values(altitude)
        conditions            = RCAIDE.Framework.Analyses.Mission.Segments.Conditions.Aerodynamics()
        conditions.freestream.update(atmosphere_conditions)        
        
        conditions.frames.inertial.velocity_vector   = np.array([[0,0,0]])
        conditions.propulsion.throttle               = np.array([[0.5]])
        conditions.frames.body.transform_to_inertial = np.array([np.eye(3)])
        
        # Set the RPM
        rotor.inputs.omega     = np.array([[1500 * Units.rpm]])
        rotor.angular_velocity = 1500 * Units.rpm
        
        rotor.airfoil_cl_surrogates = None
        rotor.airfoil_cd_surrogates = None
        
        rotor.Wake                  = Momentum_Theory_Wake()
        
        thrust_vector, torque, power, Cp, outputs , etap = rotor.spin(conditions)
        
        rotor.design_torque = torque[0,0]
        
        rotor.design_power_coefficient = Cp
        
        # Rotor (Lift) Motor
        lift_rotor_motor                         = RCAIDE.Library.Components.Powertrain.Converters.Motor()
        lift_rotor_motor.tag                     = rotor.tag
        lift_rotor_motor.efficiency              = 0.9
        lift_rotor_motor.nominal_voltage         = bat.max_voltage*.9
        lift_rotor_motor.mass_properties.mass    = 3. * Units.kg
        lift_rotor_motor.origin                  = rotor.origin
        lift_rotor_motor.propeller_radius        = rotor.tip_radius
        lift_rotor_motor.gearbox_efficiency      = 1.0
        lift_rotor_motor.no_load_current         = 4.0
        lift_rotor_motor                         = size_optimal_motor(lift_rotor_motor,rotor)      
        net.motors.append(lift_rotor_motor)
                

    for rotor in vehicle.networks.battery_electric_rotor.rotors:
        
        #rotor.inputs.blade_pitch_command        = 0. * Units.degrees # Changing the variable pitch

        # run the rotor
        conditions = RCAIDE.Framework.Analyses.Mission.Segments.Conditions.Aerodynamics()
        atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        h          = 1000. * Units.feet
        temperature_deviation = 10.
        
        atmo_data = atmosphere.compute_values(h,temperature_deviation)
        
        conditions.freestream.density                = atmo_data.density
        conditions.freestream.dynamic_viscosity      = atmo_data.dynamic_viscosity
        conditions.freestream.speed_of_sound         = atmo_data.speed_of_sound
        conditions.freestream.temperature            = atmo_data.temperature
        conditions.frames.inertial.velocity_vector   = np.array([[100* Units.mph,0,0]])
        conditions.propulsion.throttle               = np.array([[1]])
        conditions.frames.body.transform_to_inertial = np.array([np.eye(3)])
        
        # Set the RPM
        rotor.inputs.omega     = np.array([[2000. * Units.rpm]])
        rotor.angular_velocity = 2000. * Units.rpm
        
        rotor.airfoil_cl_surrogates = None
        rotor.airfoil_cd_surrogates = None      
        rotor.design_Cl = 0.7 # This is only used for OpenVSP
        
        rotor.Wake                  = Momentum_Theory_Wake()
        
        thrust_vector, torque, power, Cp, outputs , etap = rotor.spin(conditions)
        
        rotor.design_torque = torque[0,0]
        
        rotor.design_power_coefficient = Cp
        
        # Propeller (Thrust) motor
        propeller_motor                      = RCAIDE.Library.Components.Powertrain.Converters.Motor()
        propeller_motor.efficiency           = 0.95
        propeller_motor.nominal_voltage      = bat.max_voltage*0.5
        propeller_motor.mass_properties.mass = 2.0  * Units.kg
        propeller_motor.origin               = rotor.origin
        propeller_motor.propeller_radius     = rotor.tip_radius
        propeller_motor.no_load_current      = 2.0
        propeller_motor                      = size_optimal_motor(propeller_motor,rotor)
        net.motors.append(propeller_motor)

                    
    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = RCAIDE.Library.Components.Configs.Config.Container()

    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)
    
    # ------------------------------------------------------------------
    #   Hover OEI Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'hover_OEI'

    # Remove two rotors and motors on opposite ends: propora and proporf1
    K1 = 'front_left'
    K2 = 'back_right'
    
    config.networks.battery_electric_rotor.lift_rotors.pop(K1)
    config.networks.battery_electric_rotor.lift_rotors.pop(K2)
    config.networks.battery_electric_rotor.motors.pop(K1)
    config.networks.battery_electric_rotor.motors.pop(K2)    
    
    config.networks.battery_electric_rotor.number_of_lift_rotor_engines -=2 
    
    #plot_vehicle(config,plot_control_points = False)
    #plt.show()
    
    
    configs.append(config)
    

    return configs
