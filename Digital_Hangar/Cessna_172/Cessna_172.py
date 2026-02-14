# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units   
from RCAIDE.Library.Methods.Powertrain.Propulsors.Internal_Combustion_Engine import design_internal_combustion_engine
from RCAIDE.Library.Plots                 import *    

# python imports 
import numpy as np   
from copy import deepcopy
import os
import sys

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    
    # Step 1: design a vehicle
    vehicle  = vehicle_setup()  

    try:
        import vsp as vsp
        from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle 
        export_vsp_vehicle(vehicle, 'BWB')
    except ImportError:
        pass 
    
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,export_gltf=True) 
    
    return 


def vehicle_setup():  
    local_path = sys.path[0] + os.sep   
    
    #------------------------------------------------------------------------------------------------------------------------------------
    # ################################################# Vehicle-level Properties ########################################################  
    #------------------------------------------------------------------------------------------------------------------------------------     
    vehicle                                          = RCAIDE.Vehicle()
    vehicle.tag                                      = 'Cessna_172' 
    vehicle.mass_properties.max_takeoff              = 2550. * Units.pounds
    vehicle.mass_properties.takeoff                  = 2550. * Units.pounds 
    vehicle.mass_properties.max_zero_fuel            = 1680  * Units.pounds 
    vehicle.mass_properties.max_fuel                 = 152.407
    vehicle.mass_properties.max_payload              = 910  * Units.pounds 
    vehicle.mass_properties.cargo                    = 0. 
                                               
    # envelope properties                       
    vehicle.flight_envelope.ultimate_load            = 5.7 
    vehicle.flight_envelope.positive_limit_load      = 3.8  
    vehicle.flight_envelope.design_range             = 750 * Units.nmi 
    vehicle.flight_envelope.design_dynamic_pressure  = 2755 # max speed at 10 k feet
    vehicle.flight_envelope.design_mach_number       = 0.18745866156304694
                                                
    # basic parameters                          
    vehicle.reference_area                           = 174. * Units.feet**2       
    vehicle.number_of_passengers                     = 4


    
    #------------------------------------------------------------------------------------------------------------------------------------
    # ##################################################### Landing Gear ################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    main_gear                                = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear() 
    main_gear.tire_diameter                  = 6  *  Units.inches 
    main_gear.rim_diameter                   = 3  *  Units.inches 
    main_gear.tire_width                     = 6  *  Units.inches 
    main_gear.strut_length                   = 12  * Units.ft 
    main_gear.wheels                         = 4   
    main_gear.number_of_gear_types_in_tandem = 1
    main_gear.number_of_wheels_in_gear_type  = 2
    main_gear.fairing                        = True
    main_gear.xz_plane_symmetric             = True
    main_gear.gear_extended                  = True
    vehicle.append_component(main_gear)  

    nose_gear                                = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                  =  5 *  Units.inches   
    nose_gear.rim_diameter                   =  3 *  Units.inches 
    nose_gear.tire_width                     =  5 *  Units.inches 
    nose_gear.strut_length                   =  6.* Units.ft 
    nose_gear.wheels                         = 2  
    nose_gear.fairing                        = True 
    nose_gear.gear_extended                  = True
    nose_gear.number_of_gear_types_in_tandem = 1
    nose_gear.number_of_wheels_in_gear_type  = 2    
    vehicle.append_component(nose_gear)
    

    #------------------------------------------------------------------------------------------------------------------------------------
    # ######################################################## Wings ####################################################################  
    #------------------------------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------   

    wing                                        = RCAIDE.Library.Components.Wings.Main_Wing()
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
    wing.origin                                 = [[80.* Units.inches,0,  0.820]]
    wing.aerodynamic_center                     = [22.* Units.inches,0,0]
    wing.vertical                               = False
    wing.xz_plane_symmetric                     = True
    wing.high_lift                              = True 
    wing.dynamic_pressure_ratio                 = 1.0 
                                          
    # control surfaces -------------------------------------------
    flap                                        = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap() 
    flap.tag                                    = 'flap' 
    flap.span_fraction_start                    = 0.15 
    flap.span_fraction_end                      = 0.324    
    flap.deflection                             = 1.0 * Units.deg
    flap.chord_fraction                         = 0.19    
    wing.append_control_surface(flap)           
                                                
    slat                                        = RCAIDE.Library.Components.Wings.Control_Surfaces.Slat() 
    slat.tag                                    = 'slat' 
    slat.span_fraction_start                    = 0.324 
    slat.span_fraction_end                      = 0.963     
    slat.deflection                             = 1.0 * Units.deg
    slat.chord_fraction                         = 0.1      
    wing.append_control_surface(slat)   

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        
                                                
    wing                                        = RCAIDE.Library.Components.Wings.Horizontal_Tail()
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
    wing.xz_plane_symmetric                     = True
    wing.high_lift                              = False 
    wing.dynamic_pressure_ratio                 = 0.9 
    wing_airfoil                                = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil() 
    wing.append_airfoil(wing_airfoil)    
     
    
    elevator                              = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                          = 'elevator'
    elevator.span_fraction_start          = 0.1
    elevator.span_fraction_end            = 0.9
    elevator.deflection                   = 0.0  * Units.deg
    elevator.chord_fraction               = 0.35
    wing.append_control_surface(elevator)       

    
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing                                        = RCAIDE.Library.Components.Wings.Vertical_Tail()
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
    wing.xz_plane_symmetric                     = False
    wing.t_tail                                 = False 
    wing.dynamic_pressure_ratio                 = 1.0

    rudder                                = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                            = 'rudder'
    rudder.span_fraction_start            = 0.1
    rudder.span_fraction_end              = 0.9
    rudder.deflection                     = 0.0  * Units.deg
    rudder.chord_fraction                 = 0.4
    wing.append_control_surface(rudder) 
        

    # add to vehicle
    vehicle.append_component(wing)


    #------------------------------------------------------------------------------------------------------------------------------------
    # ########################################################## Fuselage ############################################################### 
    #------------------------------------------------------------------------------------------------------------------------------------
    
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage() 
    fuselage.differential_pressure              = 8*Units.psi                    # Maximum differential pressure
    fuselage.width                              = 42.         * Units.inches     # Width of the fuselage
    fuselage.heights.maximum                    = 62. * Units.inches    # Height of the fuselage
    fuselage.lengths.total                      = 326.         * Units.inches            # Length of the fuselage
    fuselage.lengths.tail                       = 161. * Units.inches  
    fuselage.lengths.cabin                      = 105. * Units.inches 
    fuselage.volume_properties.volume           = .4*fuselage.lengths.total*(np.pi/4.)*(fuselage.heights.maximum**2.) #try this as approximation
    fuselage.volume_properties.internal         = .3*fuselage.lengths.total*(np.pi/4.)*(fuselage.heights.maximum**2.)
    fuselage.areas.wetted                       = 30000. * Units.inches**2. 
    fuselage.fineness.nose                      = 1.6
    fuselage.fineness.tail                      = 2.
    fuselage.lengths.nose                       = 60.  * Units.inches
    fuselage.heights.at_quarter_length          = 62. * Units.inches
    fuselage.heights.at_three_quarters_length   = 62. * Units.inches
    fuselage.heights.at_wing_root_quarter_chord = 23. * Units.inches
    fuselage.areas.front_projected              = fuselage.width* fuselage.heights.maximum
    fuselage.effective_diameter                 = 50. * Units.inches

    cabin              = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    economy_class      = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    cabin.origin                          = [[2, 0, 0]]
    economy_class.number_of_seats_abrest  = 2
    economy_class.number_of_rows          = 2 
    economy_class.aisle_width             = 0
    economy_class.seat_arm_rest_width     = 1 * Units.inches
    economy_class.number_of_seats         = economy_class.number_of_rows  * economy_class.number_of_seats_abrest 
    cabin.append_cabin_class(economy_class)
    
    fuselage.append_cabin(cabin)          



    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = 0.0000 
    fuselage.segments.append(segment)   
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Super_Ellipse_Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.02077 
    segment.percent_z_location                  = 0.0 
    segment.height                              = 0.31619 
    segment.width                               = 0.33071 
    fuselage.segments.append(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Rounded_Rectangle_Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.03852  
    segment.percent_z_location                  = -0.02000
    segment.height                              = 0.73441 
    segment.width                               = 0.40654 
    fuselage.segments.append(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Rounded_Rectangle_Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.16595
    segment.percent_z_location                  = -0.01226 
    segment.height                              = 1.00366 
    segment.width                               = 0.90341
    segment.radius                              = 0.29143
    fuselage.segments.append(segment)   

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Rounded_Rectangle_Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.24391 	
    segment.percent_z_location                  = 0.01016 
    segment.height                              = 1.52704 
    segment.width                               = 1.06680
    segment.radius                              = 0.43714
    fuselage.segments.append(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Rounded_Rectangle_Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  =  0.43388 
    segment.percent_z_location                  =  0.0132 
    segment.height                              = 1.50557  
    segment.width                               = 1.06680
    segment.radius                              = 0.4007
    fuselage.segments.append(segment)     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Rounded_Rectangle_Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.53122
    segment.percent_z_location                  = -0.00895
    segment.height                              = 0.97386
    segment.width                               = 0.73776
    segment.radius                              = 0.29143
    fuselage.segments.append(segment)             
     
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Ellipse_Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.74233  
    segment.percent_z_location                  = 0
    segment.height                              = 0.55067  
    segment.width                               = 0.4517  
    fuselage.segments.append(segment)

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'   
    segment.percent_x_location                  = 0.98310  
    segment.percent_z_location                  = 0  
    segment.height                              = 0.17226  
    segment.width                               = 0.18068 
    fuselage.segments.append(segment)    
        
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'     
    segment.percent_x_location                  = 1.0 
    segment.percent_z_location                  = 0.00189   
    fuselage.segments.append(segment)     

    # add to vehicle
    vehicle.append_component(fuselage)
    
    #------------------------------------------------------------------------------------------------------------------------------------
    # ########################################################## Energy Network ######################################################### 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    #initialize the fuel network
    net                                         = RCAIDE.Framework.Networks.Fuel()   

    # add the network to the vehicle
    vehicle.append_energy_network(net) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus
    #------------------------------------------------------------------------------------------------------------------------------------  
    fuel_line                                   = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()   

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Fuel Tank & Fuel
    #------------------------------------------------------------------------------------------------------------------------------------       
    fuel_tank                                   = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank() 
    fuel_tank.origin                            = vehicle.wings.main_wing.origin  
    fuel_tank.fuel                              = RCAIDE.Library.Attributes.Propellants.Aviation_Gasoline() 
    fuel_tank.fuel.mass_properties.mass         = 319 *Units.lbs 
    fuel_tank.mass_properties.center_of_gravity = wing.mass_properties.center_of_gravity 
    fuel_line.fuel_tanks.append(fuel_tank)   

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    ice_prop    = RCAIDE.Library.Components.Powertrain.Propulsors.Internal_Combustion_Engine()      
                                                     
    # Engine                     
    engine                                     = RCAIDE.Library.Components.Powertrain.Converters.Engine()
    engine.sea_level_power                     = 180. * Units.horsepower
    engine.flat_rate_altitude                  = 0.0
    engine.rated_speed                         = 2700. * Units.rpm
    engine.power_specific_fuel_consumption     = 0.52 * Units['lb/hp/hr']
    ice_prop.engine                            = engine 
     
    # Propeller 
    prop = RCAIDE.Library.Components.Powertrain.Converters.Propeller()
    prop.tag                                = 'propeller'
    prop.number_of_blades                   = 2.0
    prop.tip_radius                         = 76./2. * Units.inches
    prop.hub_radius                         = 8.     * Units.inches
    prop.cruise.design_freestream_velocity  = 119.   * Units.knots
    prop.cruise.design_angular_velocity     = 2650.  * Units.rpm
    prop.cruise.design_Cl                   = 0.8
    prop.cruise.design_altitude             = 12000. * Units.feet
    prop.cruise.design_power                = .64 * 180. * Units.horsepower
    prop.variable_pitch                     = True   
    airfoil                                 = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                             = 'NACA_4412' 
    airfoil.coordinate_file                 =  local_path + 'NACA_4412.txt'   # absolute path   
    airfoil.polar_files                     =[ local_path + 'NACA_4412_polar_Re_50000.txt',
                                               local_path + 'NACA_4412_polar_Re_100000.txt',
                                               local_path + 'NACA_4412_polar_Re_200000.txt',
                                               local_path + 'NACA_4412_polar_Re_500000.txt',
                                               local_path + 'NACA_4412_polar_Re_1000000.txt']  
    prop.append_airfoil(airfoil)      
    prop.airfoil_polar_stations             = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]   
    ice_prop.propeller                      = prop

    # design propeller ICE  
    design_internal_combustion_engine(ice_prop) 
    
    net.propulsors.append(ice_prop)

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network      
    fuel_line.assigned_propulsors =  [[ice_prop.tag]]
    
    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network      
    net.fuel_lines.append(fuel_line)            

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------ 
    Wuav                                        = 2. * Units.lbs
    avionics                                    = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.mass_properties.uninstalled        = Wuav
    vehicle.avionics                            = avionics     

    #------------------------------------------------------------------------------------------------------------------------------------ 
    #   Vehicle Definition Complete
    #------------------------------------------------------------------------------------------------------------------------------------ 

    return vehicle 
if __name__ == '__main__': 
    main()     