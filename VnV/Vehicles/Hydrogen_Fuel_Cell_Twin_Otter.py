# VnV/Vehicles/Hydrogen_Fuel_Cell_Twin_Otter.py
# 
# 
# Created:   Jan 20245, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core                                                                import Units    
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor                          import design_electric_rotor

# python imports 
import numpy as np 
from copy import deepcopy
import os
# ----------------------------------------------------------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------------------------------------------------------
def vehicle_setup(fuel_cell_model):     

    #------------------------------------------------------------------------------------------------------------------------------------
    #   Initialize the Vehicle
    #------------------------------------------------------------------------------------------------------------------------------------

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'H2_Twin_Otter' 
 
    # ################################################# Vehicle-level Properties ########################################################  

    # mass properties
    vehicle.mass_properties.max_takeoff              = 5670  # kg 
    vehicle.mass_properties.takeoff                  = 5670  # kg 
    vehicle.mass_properties.max_zero_fuel            = 5670  # kg
    
    vehicle.flight_envelope.design_cruise_altitude   = 5000 * Units.feet
    vehicle.flight_envelope.design_dynamic_pressure  = 2130.457961
    vehicle.flight_envelope.design_mach_number       = 0.19
    vehicle.flight_envelope.ultimate_load            = 5.7
    vehicle.flight_envelope.limit_load               = 3.8       
    vehicle.flight_envelope.positive_limit_load      = 2.5  
    vehicle.flight_envelope.design_range             = 3500 * Units.nmi
    
    vehicle.reference_area                           = 39 
    vehicle.passengers                               = 19
    vehicle.systems.control                          = "fully powered"
    vehicle.systems.accessories                      = "commuter"  

         
    # ##########################################################  Wings ################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Main Wing
    #------------------------------------------------------------------------------------------------------------------------------------
    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing' 
    wing.sweeps.quarter_chord             = 0.0 * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 39 
    wing.spans.projected                  = 19.81
    wing.chords.root                      = 2.03 
    wing.chords.tip                       = 2.03 
    wing.chords.mean_aerodynamic          = 2.03 
    wing.taper                            = wing.chords.root/wing.chords.tip 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 3. * Units.degree 
    wing.twists.tip                       = 0
    wing.origin                           = [[5.38, 0, 1.35]] 
    wing.aerodynamic_center               = [[5.38 + 0.25 *wing.chords.root , 0, 1.35]]  
    wing.vertical                         = False
    wing.symmetric                        = True
    wing.high_lift                        = True 
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0  
    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    rel_path                              = os.path.dirname(ospath)   + separator
    airfoil                               = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                           = 'Clark_y' 
    airfoil.coordinate_file               = rel_path + separator + 'Airfoils' + separator + 'Clark_y.txt'   # absolute path     
    cg_x                                  = wing.origin[0][0] + 0.25*wing.chords.mean_aerodynamic
    cg_z                                  = wing.origin[0][2] - 0.2*wing.chords.mean_aerodynamic
    vehicle.mass_properties.center_of_gravity = [[cg_x,   0.  ,  cg_z ]]  # SOURCE: Design and aerodynamic analysis of a twin-engine commuter aircraft

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.0 
    segment.twist                         = 3. * Units.degree 
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 3. * Units.degree 
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    segment.has_fuel_tank                 = True
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 0
    segment.root_chord_percent            = 0.12
    segment.dihedral_outboard             = 0.
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)    
    
    # add to vehicle
    vehicle.append_component(wing)


    #------------------------------------------------------------------------------------------------------------------------------------  
    #   Horizontal Tail
    #------------------------------------------------------------------------------------------------------------------------------------    
    wing                                  = RCAIDE.Library.Components.Wings.Wing()
    wing.tag                              = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord             = 0.0 * Units.degree
    wing.thickness_to_chord               = 0.12 
    wing.areas.reference                  = 9.762 
    wing.spans.projected                  = 6.29   
    wing.chords.root                      = 1.552 
    wing.chords.tip                       = 1.552 
    wing.chords.mean_aerodynamic          = 1.552  
    wing.taper                            = 1 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degree
    wing.twists.tip                       = 0.0 * Units.degree 
    wing.origin                           = [[13.17 , 0 , 1.25]] 
    wing.aerodynamic_center               = [[13.17 , 0 , 1.25]]  
    wing.vertical                         = False
    wing.winglet_fraction                 = 0.0  
    wing.symmetric                        = True
    wing.high_lift                        = False 
    wing.dynamic_pressure_ratio           = 0.9

    # add to vehicle
    vehicle.append_component(wing)


    #------------------------------------------------------------------------------------------------------------------------------------  
    #   Vertical Stabilizer
    #------------------------------------------------------------------------------------------------------------------------------------ 
    wing                                  = RCAIDE.Library.Components.Wings.Wing()
    wing.tag                              = 'vertical_stabilizer'     
    wing.sweeps.leading_edge              = 28.6 * Units.degree 
    wing.thickness_to_chord               = 0.12 
    wing.areas.reference                  = 8.753 
    wing.spans.projected                  = 3.9 
    wing.chords.root                      = 2.975 
    wing.chords.tip                       = 1.514
    wing.chords.mean_aerodynamic          = 2.24 # incorrect 
    wing.taper                            = wing.chords.tip/wing.chords.root 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degree
    wing.twists.tip                       = 0.0 * Units.degree 
    wing.origin                           = [[ 12.222 , 0 , 0.385 ]]  
    wing.aerodynamic_center               = [[ 12.222 + 0.25 * wing.chords.root, 0 , 0.385 ]]  
    wing.vertical                         = True 
    wing.symmetric                        = False
    wing.t_tail                           = False
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0

    # add to vehicle
    vehicle.append_component(wing)

 
    # ##########################################################   Fuselage  ############################################################    
    fuselage = RCAIDE.Library.Components.Fuselages.Fuselage() 

    cabin         = RCAIDE.Library.Components.Fuselages.Cabins.Cabin() 
    economy_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 2
    economy_class.number_of_rows                      = 8
    economy_class.galley_lavatory_percent_x_locations = []  
    economy_class.emergency_exit_percent_x_locations  = []      
    economy_class.type_A_exit_percent_x_locations     = [] 
    cabin.append_cabin_class(economy_class)
    fuselage.append_cabin(cabin)
    
        
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2.
    fuselage.lengths.nose                       = 2.95  
    fuselage.lengths.tail                       = 7.57
    fuselage.lengths.cabin                      = 4.62 
    fuselage.lengths.total                      = 15.77  
    fuselage.width                              = 1.75  
    fuselage.heights.maximum                    = 1.50  
    fuselage.heights.at_quarter_length          = 1.50  
    fuselage.heights.at_three_quarters_length   = 1.50  
    fuselage.heights.at_wing_root_quarter_chord = 1.50  
    fuselage.areas.side_projected               = fuselage.lengths.total *fuselage.heights.maximum  # estimate    
    fuselage.areas.wetted                       = 2 * np.pi * fuselage.width *  fuselage.lengths.total +  2 * np.pi * fuselage.width ** 2
    fuselage.areas.front_projected              =  np.pi * fuselage.width ** 2 
    fuselage.effective_diameter                 = 1.75 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_0'
    segment.percent_x_location                  = 0
    segment.percent_z_location                  = 0
    segment.height                              = 0.01
    segment.width                               = 0.01
    fuselage.segments.append(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'
    segment.percent_x_location                  = 0.005345402
    segment.percent_z_location                  = -0.027433333/ fuselage.lengths.total	 
    segment.height                              = 0.421666667
    segment.width                               = 0.106025757
    fuselage.segments.append(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'
    segment.percent_x_location                  = 0.019706071
    segment.percent_z_location                  = 6.66667E-05/ fuselage.lengths.total	 
    segment.height                              = 0.733333333
    segment.width                               = 0.61012023
    fuselage.segments.append(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'
    segment.percent_x_location                  = 0.054892307
    segment.percent_z_location                  = 0.009233333/ fuselage.lengths.total	 
    segment.height                              = 1.008333333
    segment.width                               = 1.009178159
    fuselage.segments.append(segment)  

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'
    segment.percent_x_location                  = 0.082575704 
    segment.percent_z_location                  = 0.0459 / fuselage.lengths.total	 
    segment.height                              = 1.228333333 
    segment.width                               = 1.275456588 
    fuselage.segments.append(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'
    segment.percent_x_location                  = 0.116879689
    segment.percent_z_location                  = 0.055066667/ fuselage.lengths.total	 
    segment.height                              = 1.393333333
    segment.width                               = 1.436068974
    fuselage.segments.append(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'
    segment.percent_x_location                  = 0.151124363 
    segment.percent_z_location                  = 0.091733333 / fuselage.lengths.total	 
    segment.height                              = 1.576666667 
    segment.width                               = 1.597041074  
    fuselage.segments.append(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'
    segment.percent_x_location                  = 0.172884077 
    segment.percent_z_location                  = 0.251733333 / fuselage.lengths.total	 
    segment.height                              = 1.953333333  
    segment.width                               = 1.75  
    fuselage.segments.append(segment)
  

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'
    segment.percent_x_location                  = 0.194554824   
    segment.percent_z_location                  = 0.311733333/ fuselage.lengths.total	 	 
    segment.height                              = 2.09	 
    segment.width                               = 1.75 
    fuselage.segments.append(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'
    segment.percent_x_location                  =  0.479203019 
    segment.percent_z_location                  = 0.311733333 / fuselage.lengths.total	  
    segment.height                              = 2.09	 
    segment.width                               = 1.75 
    fuselage.segments.append(segment)
    
    

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_10'
    segment.percent_x_location                  = 0.541657475 
    segment.percent_z_location                  = 0.311733333/ fuselage.lengths.total	 	 
    segment.height                              = 2.09	 
    segment.width                               = 2 
    fuselage.segments.append(segment)
    

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'
    segment.percent_x_location                  = 0.716936232 
    segment.percent_z_location                  = 0.394233333/ fuselage.lengths.total	 	 
    segment.height                              = 1.558333333	 
    segment.width                               = 1.64
    segment.has_fuel_tank                       = True
    fuselage.segments.append(segment)
    

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_12'
    segment.percent_x_location                  = 1.0
    segment.percent_z_location                  = 0.440066667 / fuselage.lengths.total	 
    segment.height                              = 0.11	 
    segment.width                               = 0.05 
    fuselage.segments.append(segment) 
          
 

    # add to vehicle
    vehicle.append_component(fuselage)

    # ------------------------------------------------------------------
    #   Landing gear
    # ------------------------------------------------------------------  
    main_gear                                   = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.strut_length                      = 12. * Units.inches
    vehicle.append_component(main_gear) 
    nose_gear                                   = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()    
    nose_gear.strut_length                      = 6. * Units.inches 
    vehicle.append_component(nose_gear) 
 
    # ########################################################  Energy Network  #########################################################  
    net                              = RCAIDE.Framework.Networks.Fuel_Cell()   

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus and Crogenic Line 
    #------------------------------------------------------------------------------------------------------------------------------------  
    bus = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()   
      
    if fuel_cell_model == 'PEM': 
        fuel_cell_stack = RCAIDE.Library.Components.Powertrain.Converters.Proton_Exchange_Membrane_Fuel_Cell() 
        fuel_cell_stack.electrical_configuration.series             = 940
        fuel_cell_stack.electrical_configuration.parallel           = 7
        fuel_cell_stack.geometrtic_configuration.normal_count       = 940
        fuel_cell_stack.geometrtic_configuration.parallel_count     = 7
    if fuel_cell_model == 'Larminie':  
        fuel_cell_stack   = RCAIDE.Library.Components.Powertrain.Converters.Generic_Fuel_Cell_Stack() 
        fuel_cell_stack.electrical_configuration.series             = 1020
        fuel_cell_stack.electrical_configuration.parallel           = 1
        fuel_cell_stack.geometrtic_configuration.normal_count       = 1020
        fuel_cell_stack.geometrtic_configuration.parallel_count     = 1
        
    bus.fuel_cell_stacks.append(fuel_cell_stack)  
    bus.initialize_bus_properties() 
      
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Crogenic Tank
    #------------------------------------------------------------------------------------------------------------------------------------       
    cryogenic_tank_1 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(vehicle.fuselages.fuselage)    # for regression, aircraft has two tanks
    cryogenic_tank_1.fuel_selector_ratio  = 0.5 
    cryogenic_tank_1.fuel                 = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen() 
    bus.fuel_tanks.append(cryogenic_tank_1)
    
    cryogenic_tank_2 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.fuselages.fuselage)  
    cryogenic_tank_2.fuel_selector_ratio  = 0.5
    cryogenic_tank_2.fuel                 = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen() 
    bus.fuel_tanks.append(cryogenic_tank_2)    
          
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    starboard_propulsor                              = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()  
    starboard_propulsor.tag                          = 'starboard_propulsor'
    
    # Electronic Speed Controller       
    esc                                              = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    esc.tag                                          = 'esc_1'
    esc.efficiency                                   = 0.95 
    esc.origin                                       = [[3.8,2.8129,1.22 ]]  
    esc.bus_voltage                                  = bus.voltage    
    starboard_propulsor.electronic_speed_controller  = esc   
     
    # Propeller              
    propeller                                        = RCAIDE.Library.Components.Powertrain.Converters.Propeller() 
    propeller.tag                                    = 'propeller_1'  
    propeller.tip_radius                             = 2.59
    propeller.number_of_blades                       = 3
    propeller.hub_radius                             = 10.    * Units.inches

    propeller.cruise.design_freestream_velocity      = 130 * Units.kts      
    speed_of_sound                                   = 343 
    propeller.cruise.design_tip_mach                 = 0.65
    propeller.cruise.design_angular_velocity         = propeller.cruise.design_tip_mach *speed_of_sound/propeller.tip_radius
    propeller.cruise.design_Cl                       = 0.7
    propeller.cruise.design_altitude                 = 8000. * Units.feet 
    propeller.cruise.design_thrust                   = 12500  
    propeller.clockwise_rotation                     = False
    propeller.variable_pitch                         = True  
    propeller.origin                                 = [[3.5,2.8129,1.22 ]]   
    airfoil                                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                                      = 'NACA_4412' 
    airfoil.coordinate_file                          =  rel_path + 'Airfoils' + separator + 'NACA_4412.txt'   # absolute path   
    airfoil.polar_files                              = [ rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_50000.txt',
                                                        rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_100000.txt',
                                                        rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_200000.txt',
                                                        rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_500000.txt',
                                                        rel_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_1000000.txt']   
    propeller.append_airfoil(airfoil)                       
    propeller.airfoil_polar_stations                 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] 
    starboard_propulsor.rotor                        = propeller   
              
    # DC_Motor       
    motor                                            = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    motor.efficiency                                 = 0.98
    motor.origin                                     = [[4.0,2.8129,1.22 ]]   
    motor.nominal_voltage                            = bus.voltage 
    motor.no_load_current                            = 1
    starboard_propulsor.motor                        = motor 

    # design starboard propulsor 
    design_electric_rotor(starboard_propulsor)
        
    nacelle                    = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    nacelle.tag                = 'nacelle_1'
    nacelle.length             = 4
    nacelle.diameter           = 0.73480616 
    nacelle.areas.wetted       = 0.01*(2*np.pi*0.01/2)
    nacelle.origin             = [[3.5,2.8129,1]]
    nacelle.flow_through       = False  
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_1'
    nac_segment.percent_x_location = 0.0  
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)   
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_2'
    nac_segment.percent_x_location = 0.042687938 
    nac_segment.percent_z_location = 0.0284/ nacelle.length
    nac_segment.height             = 0.183333333 
    nac_segment.width              = 0.422484315 
    nacelle.append_segment(nac_segment)   
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_3'
    nac_segment.percent_x_location = 0.143080714 
    nac_segment.percent_z_location = 0.046733333/ nacelle.length
    nac_segment.height             = 0.44	 
    nac_segment.width              = 0.685705173 
    nacelle.append_segment(nac_segment)  
     
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_4'
    nac_segment.percent_x_location = 0.170379029  
    nac_segment.percent_z_location = -0.154233333/ nacelle.length
    nac_segment.height             = 0.898333333	 
    nac_segment.width              = 0.73480616 
    nacelle.append_segment(nac_segment)  
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 0.252189893  
    nac_segment.percent_z_location = -0.154233333/ nacelle.length
    nac_segment.height             = 1.008333333 
    nac_segment.width              = 0.736964445
    nacelle.append_segment(nac_segment)   
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_6'
    nac_segment.percent_x_location = 0.383860821   
    nac_segment.percent_z_location = -0.072566667/ nacelle.length
    nac_segment.height             = 0.971666667 
    nac_segment.width              = 0.736964445 
    nacelle.append_segment(nac_segment)  
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_7'
    nac_segment.percent_x_location = 0.551826736  
    nac_segment.percent_z_location = .055066667/ nacelle.length	
    nac_segment.height             = 0.77	 
    nac_segment.width              = 0.736964445  
    nacelle.append_segment(nac_segment)
    

    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_8'
    nac_segment.percent_x_location = 0.809871485   
    nac_segment.percent_z_location = 0.1284/ nacelle.length
    nac_segment.height             = 0.366666667 
    nac_segment.width              = 0.736964445 
    nacelle.append_segment(nac_segment) 
    

    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_9'
    nac_segment.percent_x_location = 1.0  
    nac_segment.percent_z_location = 0.201733333 / nacelle.length
    nac_segment.height             = 0.036666667	
    nac_segment.width              = 0.0  
    nacelle.append_segment(nac_segment) 
    
    starboard_propulsor.nacelle = nacelle      
 
    # append propulsor to distribution line 
    net.propulsors.append(starboard_propulsor) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    port_propulsor                             = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor() 
    port_propulsor.tag                         = "port_propulsor" 
            
    esc_2                                      = deepcopy(esc)
    esc_2.origin                               = [[3.8, -2.8129,1.22 ]]        
    port_propulsor.electronic_speed_controller = esc_2  

    propeller_2                                = deepcopy(propeller)
    propeller_2.tag                            = 'propeller_2' 
    propeller_2.origin                         =  [[3.5, -2.8129,1.22 ]]   
    propeller_2.clockwise_rotation             = False        
    port_propulsor.rotor                       = propeller_2  
              
    motor_2                                    = deepcopy(motor)
    motor_2.origin                             =  [[4.0, -2.8129,1.22 ]]        
    port_propulsor.motor                       = motor_2 

    nacelle_2                                  = deepcopy(nacelle)
    nacelle_2.tag                              = 'nacelle_2'
    nacelle_2.origin                           = [[ 2.81, -3.34 ,1.22]]
    port_propulsor.nacelle                     = nacelle_2         
    
    # append propulsor to distribution line 
    net.propulsors.append(port_propulsor)  
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------  
    avionics                     = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw          = 30. # Watts
    bus.avionics                 = avionics
    
    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to bus       
    bus.assigned_propulsors =  [[starboard_propulsor.tag, port_propulsor.tag]] 

    # append bus   
    net.busses.append(bus) 
    
    vehicle.append_energy_network(net)

    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
    
    return vehicle

# ---------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):

    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------  
    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'  
    configs.append(base_config)   
    
    # done!
    return configs
