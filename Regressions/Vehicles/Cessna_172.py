# Regression/scripts/Vehicles/Cessna_172.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from RCAIDE.Core import Units,Data, Container 
from RCAIDE.Methods.Geometry.Three_Dimensional                                   import compute_span_location_from_chord_length
from RCAIDE.Methods.Flight_Dynamics.Static_Stability.Approximations              import datcom
from RCAIDE.Methods.Flight_Dynamics.Static_Stability.Approximations.Supporting_Functions import trapezoid_ac_x
from RCAIDE.Methods.Propulsion import design_propeller

# python imports 
import numpy as np
def vehicle_setup(): 
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------        
    vehicle                                     = RCAIDE.Vehicle()
    vehicle.tag                                 = 'Cessna_172'
                                                
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff         = 2550. * Units.pounds
    vehicle.mass_properties.takeoff             = 2550. * Units.pounds
    vehicle.mass_properties.max_zero_fuel       = 2550. * Units.pounds
    vehicle.mass_properties.cargo               = 0. 
                                               
    # envelope properties                       
    vehicle.envelope.ultimate_load              = 5.7
    vehicle.envelope.limit_load                 = 3.8
                                                
    cruise_speed                                = 124. * Units.kts
    altitude                                    = 8500. * Units.ft
    atmo                                        = RCAIDE.Analyses.Atmospheric.US_Standard_1976()
    freestream                                  = atmo.compute_values (0.)
    freestream0                                 = atmo.compute_values (altitude)
    mach_number                                 = (cruise_speed/freestream.speed_of_sound)[0][0] 
    vehicle.design_dynamic_pressure             = ( .5 *freestream0.density*(cruise_speed*cruise_speed))[0][0]
    vehicle.design_mach_number                  =  mach_number
                                                
    # basic parameters                          
    vehicle.reference_area                      = 174. * Units.feet**2       
    vehicle.passengers                          = 4

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        

    wing                                        = RCAIDE.Components.Wings.Main_Wing()
    wing.tag                                    = 'main_wing'    
    wing.sweeps.quarter_chord                   = 0.0 * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.areas.reference                        = 174. * Units.feet**2
    wing.spans.projected                        = 36.  * Units.feet + 1. * Units.inches
    wing.chords.root                            = 66. * Units.inches
    wing.chords.tip                             = 45. * Units.inches
    wing.chords.mean_aerodynamic                = 58. * Units.inches
    wing.taper                                  = wing.chords.tip/wing.chords.root
    wing.aspect_ratio                           = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                            = 3.0 * Units.degrees
    wing.twists.tip                             = 1.5 * Units.degrees
    wing.origin                                 = [[80.* Units.inches,0,0]]
    wing.aerodynamic_center                     = [22.* Units.inches,0,0]
    wing.vertical                               = False
    wing.symmetric                              = True
    wing.high_lift                              = True 
    wing.dynamic_pressure_ratio                 = 1.0 
                                          
    # control surfaces -------------------------------------------
    flap                                        = RCAIDE.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                                    = 'flap' 
    flap.span_fraction_start                    = 0.15 
    flap.span_fraction_end                      = 0.324    
    flap.deflection                             = 1.0 * Units.deg
    flap.chord_fraction                         = 0.19    
    wing.append_control_surface(flap)           
                                                
    slat                                        = RCAIDE.Components.Wings.Control_Surfaces.Slat() 
    slat.tag                                    = 'slat' 
    slat.span_fraction_start                    = 0.324 
    slat.span_fraction_end                      = 0.963     
    slat.deflection                             = 1.0 * Units.deg
    slat.chord_fraction                         = 0.1      
    wing.append_control_surface(slat)  
    
    RCAIDE.Methods.Geometry.Two_Dimensional.Planform.wing_planform(wing) 

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        
                                                
    wing                                        = RCAIDE.Components.Wings.Wing()
    wing.tag                                    = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord                   = 0.0 * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.areas.reference                        = 5800. * Units.inches**2
    wing.spans.projected                        = 136.  * Units.inches
    wing.chords.root                            = 55. * Units.inches
    wing.chords.tip                             = 30. * Units.inches
    wing.chords.mean_aerodynamic                = 43. * Units.inches 
    wing.taper                                  = wing.chords.tip/wing.chords.root
    wing.aspect_ratio                           = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees
    wing.origin                                 = [[246.* Units.inches,0,0]]
    wing.aerodynamic_center                     = [20.* Units.inches,0,0]
    wing.vertical                               = False
    wing.symmetric                              = True
    wing.high_lift                              = False 
    wing.dynamic_pressure_ratio                 = 0.9
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing                                        = RCAIDE.Components.Wings.Wing()
    wing.tag                                    = 'vertical_stabilizer' 
    wing.sweeps.quarter_chord                   = 25. * Units.deg
    wing.thickness_to_chord                     = 0.12
    wing.areas.reference                        = 3500. * Units.inches**2
    wing.spans.projected                        = 73.   * Units.inches
    wing.chords.root                            = 66. * Units.inches
    wing.chords.tip                             = 27. * Units.inches
    wing.chords.mean_aerodynamic                = 48. * Units.inches 
    wing.taper                                  = wing.chords.tip/wing.chords.root
    wing.aspect_ratio                           = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                            = 0.0 * Units.degrees
    wing.twists.tip                             = 0.0 * Units.degrees
    wing.origin                                 = [[237.* Units.inches,0,0]]
    wing.aerodynamic_center                     = [20.* Units.inches,0,0] 
    wing.vertical                               = True 
    wing.symmetric                              = False
    wing.t_tail                                 = False 
    wing.dynamic_pressure_ratio                 = 1.0

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage                                    = RCAIDE.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage'
    fuselage.number_coach_seats                 = 4.       
    fuselage.tag                                = 'fuselage'    
    fuselage.differential_pressure              = 8*Units.psi                    # Maximum differential pressure
    fuselage.width                              = 42.         * Units.inches     # Width of the fuselage
    fuselage.heights.maximum                    = 62. * Units.inches    # Height of the fuselage
    fuselage.lengths.total                      = 326.         * Units.inches            # Length of the fuselage
    fuselage.lengths.empennage                  = 161. * Units.inches  
    fuselage.lengths.cabin                      = 105. * Units.inches
    fuselage.lengths.structure                  = fuselage.lengths.total-fuselage.lengths.empennage 
    fuselage.mass_properties.volume             = .4*fuselage.lengths.total*(np.pi/4.)*(fuselage.heights.maximum**2.) #try this as approximation
    fuselage.mass_properties.internal_volume    = .3*fuselage.lengths.total*(np.pi/4.)*(fuselage.heights.maximum**2.)
    fuselage.areas.wetted                       = 30000. * Units.inches**2.
    fuselage.seats_abreast                      = 2.
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2.
    fuselage.lengths.nose                       = 60.  * Units.inches
    fuselage.heights.at_quarter_length          = 62. * Units.inches
    fuselage.heights.at_three_quarters_length   = 62. * Units.inches
    fuselage.heights.at_wing_root_quarter_chord = 23. * Units.inches
    fuselage.areas.front_projected              = fuselage.width* fuselage.heights.maximum
    fuselage.effective_diameter                 = 50. * Units.inches

    # add to vehicle
    vehicle.append_component(fuselage)
    
    # ------------------------------------------------------------------
    #   Landing gear
    # ------------------------------------------------------------------  
    landing_gear                                = RCAIDE.Components.Landing_Gear.Landing_Gear()
    main_gear                                   = RCAIDE.Components.Landing_Gear.Main_Landing_Gear()
    nose_gear                                   = RCAIDE.Components.Landing_Gear.Nose_Landing_Gear()
    main_gear.strut_length                      = 12. * Units.inches  
    nose_gear.strut_length                      = 6. * Units.inches 
                                                
    landing_gear.main                           = main_gear
    landing_gear.nose                           = nose_gear
                                                
    #add to vehicle                             
    vehicle.landing_gear                        = landing_gear

    # ########################################################  Energy Network  #########################################################  
    net                                         = RCAIDE.Energy.Networks.Internal_Combustion_Engine()  

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus
    #------------------------------------------------------------------------------------------------------------------------------------  
    fuel_line                                   = RCAIDE.Energy.Distributors.Fuel_Line() 
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #   Fuel
    #------------------------------------------------------------------------------------------------------------------------------------  
    # fuel tank
    fuel_tank                                   = RCAIDE.Energy.Storages.Fuel_Tanks.Fuel_Tank()
    fuel_tank.origin                            = wing.origin 
    
    # fuel 
    fuel                                        = RCAIDE.Attributes.Propellants.Aviation_Gasoline() 
    fuel.mass_properties.mass                   = 319 *Units.lbs 
    fuel.mass_properties.center_of_gravity      = wing.mass_properties.center_of_gravity
    fuel.internal_volume                        = fuel.mass_properties.mass/fuel.density  
    fuel_tank.fuel                              = fuel 
    
    fuel_line.fuel_tanks.append(fuel_tank)

    #------------------------------------------------------------------------------------------------------------------------------------                                                  
    # Engine                    
    #------------------------------------------------------------------------------------------------------------------------------------  
    engine                                     = RCAIDE.Energy.Converters.Engine()
    engine.sea_level_power                     = 180. * Units.horsepower
    engine.flat_rate_altitude                  = 0.0
    engine.rated_speed                         = 2700. * Units.rpm
    engine.power_specific_fuel_consumption     = 0.52 
    fuel_line.engines.append(engine)

    #------------------------------------------------------------------------------------------------------------------------------------     
    # Prop
    #------------------------------------------------------------------------------------------------------------------------------------ 
    prop = RCAIDE.Energy.Converters.Propeller()
    prop.number_of_blades                   = 2.0
    prop.tip_radius                         = 76./2. * Units.inches
    prop.hub_radius                         = 8.     * Units.inches
    prop.cruise.design_freestream_velocity  = 119.   * Units.knots
    prop.cruise.design_angular_velocity     = 2650.  * Units.rpm
    prop.cruise.design_Cl                   = 0.8
    prop.cruise.design_altitude             = 12000. * Units.feet
    prop.cruise.design_power                = .64 * 180. * Units.horsepower
    prop.variable_pitch                     = True 
    airfoil                                 = RCAIDE.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file                 = '../../Vehicles/Airfoils/NACA_4412.txt'
    airfoil.polar_files                     = ['../../Vehicles//Airfoils/Polars/NACA_4412_polar_Re_50000.txt' ,
                                            '../../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_100000.txt' ,
                                            '../../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_200000.txt' ,
                                            '../../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_500000.txt' ,
                                            '../../Vehicles/Airfoils/Polars/NACA_4412_polar_Re_1000000.txt' ] 
    prop.append_airfoil(airfoil)            
    prop.airfoil_polar_stations             = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] 
    prop                                    = design_propeller(prop)    
    fuel_line.rotors.append(prop)  
    net.fuel_lines.append(fuel_line)

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------ 
    Wuav                                        = 2. * Units.lbs
    avionics                                    = RCAIDE.Energy.Peripherals.Avionics()
    avionics.mass_properties.uninstalled        = Wuav
    vehicle.avionics                            = avionics     

    # add the network to the vehicle
    vehicle.append_energy_network(net)
    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    #   Vehicle Definition Complete
    #------------------------------------------------------------------------------------------------------------------------------------ 

    return vehicle
  
def configs_setup(vehicle):
     # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    configs                                                    = RCAIDE.Components.Configs.Config.Container() 
    base_config                                                = RCAIDE.Components.Configs.Config(vehicle)
    base_config.networks.internal_combustion_engine.fuel_lines.fuel_line.active_propulsor_groups = ['propulsor']  # default in network 
    base_config.tag                                            = 'base'
    configs.append(base_config)
    
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------ 
    config                                                     = RCAIDE.Components.Configs.Config(base_config)
    config.tag                                                 = 'cruise' 
    configs.append(config)
    
    
    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------ 
    config                                                     = RCAIDE.Components.Configs.Config(base_config)
    config.tag                                                 = 'takeoff' 
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.V2_VS_ratio                                         = 1.21
    config.maximum_lift_coefficient                            = 2.
    
    configs.append(config)
    
    
    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config                                                     = RCAIDE.Components.Configs.Config(base_config)
    config.tag                                                 = 'landing' 
    config.wings['main_wing'].control_surfaces.flap.deflection = 20. * Units.deg
    config.Vref_VS_ratio                                       = 1.23
    config.maximum_lift_coefficient                            = 2.
                                                               
    configs.append(config)
    
    
    # done!
    return configs