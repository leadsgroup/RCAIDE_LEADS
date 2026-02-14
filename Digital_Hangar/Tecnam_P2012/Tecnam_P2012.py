# Twin_Otter.py
# 
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE      
from RCAIDE.Framework.Core import Units  
from RCAIDE.Library.Methods.Powertrain.Propulsors.Internal_Combustion_Engine import design_internal_combustion_engine 
from RCAIDE.Library.Plots                                           import *      

# python imports 
import numpy as np   
from copy import deepcopy 
import sys 
import os


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    
    # Step 1: design a vehicle
    vehicle  = vehicle_setup()  

    try:
        import vsp as vsp
        from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle 
        export_vsp_vehicle(vehicle, 'Tecnam_P2012')
    except ImportError:
        pass
        
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,export_gltf=True)  
    
    return 
 
def vehicle_setup():

    local_path = sys.path[0] + os.sep  

    #------------------------------------------------------------------------------------------------------------------------------------
    #   Initialize the Vehicle
    #------------------------------------------------------------------------------------------------------------------------------------

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Tecnam_P2012'

 
    # ################################################# Vehicle-level Properties ########################################################  

    # mass properties
    vehicle.mass_properties.max_takeoff             = 3680 
    vehicle.mass_properties.max_zero_fuel           = 3680 
    vehicle.mass_properties.max_fuel                = 1190 * Units.lbs
    vehicle.mass_properties.max_payload             = 1394
    vehicle.flight_envelope.ultimate_load           = 5.7
    vehicle.flight_envelope.positive_limit_load     = 3.8 
    vehicle.reference_area                          = 25.76
    vehicle.number_of_passengers                    = 9
    vehicle.systems.control                         = "fully powered"
    vehicle.systems.accessories                     = "commuter"    
    
    cruise_speed                                    = 173.*Units['kts']    
    altitude                                        = 10000. * Units.ft
    atmo                                            = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    freestream                                      = atmo.compute_values (0.)
    freestream0                                     = atmo.compute_values (altitude)
    mach_number                                     = (cruise_speed/freestream.speed_of_sound)[0][0] 
    vehicle.flight_envelope.design_dynamic_pressure = ( .5 *freestream0.density*(cruise_speed*cruise_speed))[0][0]
    vehicle.flight_envelope.design_mach_number      = mach_number
    vehicle.flight_envelope.design_range            = 950 * Units.nmi
    vehicle.mass_properties.center_of_gravity       = [[5.1772, 0, 0.46]]

         
    # ##########################################################  Wings ################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Main Wing
    #------------------------------------------------------------------------------------------------------------------------------------
    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing' 
    wing.sweeps.quarter_chord             = 0.0 * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 25.3
    wing.spans.projected                  = 14.0 
    wing.chords.root                      = 1.91
    wing.chords.tip                       = 1.51
    wing.chords.mean_aerodynamic          = 1.81
    wing.taper                            = wing.chords.root/wing.chords.tip 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 3.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[4.447, 0., 1.216]]
    wing.aerodynamic_center               = [3., 0., 1.01] 
    wing.vertical                         = False
    wing.xz_plane_symmetric               = True
    wing.high_lift                        = True 
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0     
    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.0 
    segment.twist                         = 3. * Units.degrees   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    airfoil                               = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                           = 'NACA_63_412.txt' 
    airfoil.coordinate_file               = local_path + 'NACA_63_412.txt'      
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'outboard'
    segment.percent_span_location         = 0.5438
    segment.twist                         = 2.* Units.degrees 
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0. 
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12 
    airfoil                               = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                           = 'NACA_63_412.txt' 
    airfoil.coordinate_file               = local_path + 'NACA_63_412.txt'      
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)
    
    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'winglet'
    segment.percent_span_location         = 1.0
    segment.twist                         = 1.  * Units.degrees 
    segment.root_chord_percent            = 0.630
    segment.dihedral_outboard             = 0. * Units.degrees 
    segment.sweeps.quarter_chord          = 0. * Units.degrees 
    segment.thickness_to_chord            = 0.12 
    segment.append_airfoil(airfoil)
    wing.append_segment(segment) 

    # control surfaces -------------------------------------------
    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.68
    aileron.span_fraction_end     = 0.96
    aileron.deflection            = 0.0  * Units.deg
    aileron.chord_fraction        = 0.30
    wing.append_control_surface(aileron)   

     
    # add to vehicle
    vehicle.append_component(wing)


    #------------------------------------------------------------------------------------------------------------------------------------  
    #   Horizontal Tail
    #------------------------------------------------------------------------------------------------------------------------------------    
    wing                                  = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag                              = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord             = 0.0 * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 7.23 
    wing.spans.projected                  = 5.64  * Units.meter 
    wing.sweeps.leading_edge              = 7.5 * Units.deg 
    wing.chords.root                      = 1.35 * Units.meter 
    wing.chords.tip                       = 0.84 * Units.meter 
    wing.chords.mean_aerodynamic          = 1.10 * Units.meter  
    wing.taper                            = wing.chords.tip/wing.chords.root 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[10.366, 0., 0.414]]
    wing.aerodynamic_center               = [10.366, 0., 0.414] 
    wing.vertical                         = False
    wing.winglet_fraction                 = 0.0  
    wing.xz_plane_symmetric               = True
    wing.high_lift                        = False 
    wing.dynamic_pressure_ratio           = 0.9

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = wing.sweeps.leading_edge
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root_2'
    segment.percent_span_location         = 1.0  
    segment.root_chord_percent            = wing.taper 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0 * Units.deg
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.01
    elevator.span_fraction_end     = 1.00
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.37
    wing.append_control_surface(elevator)

    # add to vehicle
    vehicle.append_component(wing)

    #------------------------------------------------------------------------------------------------------------------------------------  
    #   Vertical Stabilizer
    #------------------------------------------------------------------------------------------------------------------------------------ 
    wing                                  = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag                              = 'vertical_stabilizer'     
    wing.sweeps.quarter_chord             = 25. * Units.deg
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 4.26 * Units['meters**2']  
    wing.spans.projected                  = 2.50 * Units.meter  
    wing.chords.root                      = 2.891 * Units.meter 
    wing.chords.tip                       = 0.8 * Units.meter 
    wing.chords.mean_aerodynamic          = 2.20   * Units.meter 
    wing.taper                            = wing.chords.tip/wing.chords.root 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degrees
    wing.twists.tip                       = 0.0 * Units.degrees 
    wing.origin                           = [[8.8 ,0, 0.623]]
    wing.aerodynamic_center               = [8.8 ,0,0]  
    wing.vertical                         = True 
    wing.xz_plane_symmetric               = False
    wing.t_tail                           = False
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0. * Units.deg
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root_2'
    segment.percent_span_location         = 0.18   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 75.667 * Units.deg
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Canted'
    segment.percent_span_location         = 0.26   
    segment.root_chord_percent            = 0.6485 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 28.0 * Units.deg
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.0   
    segment.root_chord_percent            = 0.346
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    elevator.tag                   = 'rudder'
    elevator.span_fraction_start   = 0.2
    elevator.span_fraction_end     = 1.00
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.44
    wing.append_control_surface(elevator)

    # add to vehicle
    vehicle.append_component(wing)

 
    # ##########################################################   Fuselage  ############################################################    
    fuselage = RCAIDE.Library.Components.Fuselages.Fuselage() 
    fuselage.seats_abreast                      = 2.
    fuselage.fineness.nose                      = 1.8
    fuselage.fineness.tail                      = 2.
    fuselage.lengths.nose                       = 2.8  * Units.meter
    fuselage.lengths.tail                       = 3.5  * Units.meter
    fuselage.lengths.total                      = 11.8 * Units.meter
    fuselage.lengths.cabin                      = fuselage.lengths.total - (fuselage.lengths.nose  + fuselage.lengths.tail ) 
    fuselage.lengths.fore_space                 = 0.
    fuselage.lengths.aft_space                  = 0.
    fuselage.width                              = 1.57
    fuselage.heights.maximum                    = 2
    fuselage.heights.at_quarter_length          = 2. * Units.meter
    fuselage.heights.at_three_quarters_length   = 2. * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 2. * Units.meter
    fuselage.areas.side_projected               = 16.9613 * Units.meter**2.
    fuselage.areas.wetted                       = 52.94 * Units.meter**2.
    fuselage.areas.front_projected              = 2.72 * Units.meter**2.
    fuselage.effective_diameter                 = 1.760 * Units.meter 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_0'
    segment.percent_x_location                  = 0
    segment.percent_z_location                  = -0.01025
    segment.height                              = 0.0
    segment.width                               = 0.0
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'
    segment.percent_x_location                  = 0.01941
    segment.percent_z_location                  = -0.00698
    segment.height                              = 0.50026
    segment.width                               = 0.43658
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'
    segment.percent_x_location                  = 0.06309
    segment.percent_z_location                  = -0.00197
    segment.height                              = 0.81075
    segment.width                               = 0.90653
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'
    segment.percent_x_location                  = 0.13101
    segment.percent_z_location                  = 0.00931
    segment.height                              = 1.12583
    segment.width                               = 1.37708
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'
    segment.percent_x_location                  = 0.23545
    segment.percent_z_location                  = 0.03003
    segment.height                              = 1.65695
    segment.width                               = 1.5748
    segment.curvature                           = 3
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'
    segment.percent_x_location                  = 0.296002
    segment.percent_z_location                  = 0.03026
    segment.height                              = 1.68589
    segment.width                               = 1.5748
    segment.curvature                           = 3
    fuselage.append_segment(segment)

    # Segment Left off here
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'
    segment.percent_x_location                  = 0.581
    segment.percent_z_location                  = 0.03264
    segment.height                              = 1.68589
    segment.width                               = 1.5748
    segment.curvature                           = 3
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'
    segment.percent_x_location                  = 0.70469
    segment.percent_z_location                  = 0.03426
    segment.height                              = 1.5662
    segment.width                               = 1.33392
    segment.curvature                           = 3
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'
    segment.percent_x_location                  = 0.98511 
    segment.percent_z_location                  = 0.045
    segment.height                              = 0.4667
    segment.width                               = 0.2
    fuselage.append_segment(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'
    segment.percent_x_location                  = 1.0
    segment.percent_z_location                  = 0.04133
    segment.height                              = 0.0
    segment.width                               = 0.0
    fuselage.append_segment(segment)

    # define cabin    
    cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.offset_x                                    = 2.5  
    economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 2
    economy_class.seat_pitch                          = 31 * Units.inches
    economy_class.aisle_width                         = 0.28 * Units.meters
    economy_class.seat_width                          = 16 *  Units.inches
    economy_class.number_of_rows                      = 5
    economy_class.galley_lavatory_percent_x_locations = []  
    economy_class.emergency_exit_percent_x_locations  = []      
    economy_class.type_A_exit_percent_x_locations     = [0.01,1] 
    economy_class.number_of_seats                     = economy_class.number_of_rows  * economy_class.number_of_seats_abrest 
    

    # add to vehicle
    vehicle.append_component(fuselage)
 
    # ##########################################################   Nacelles  ############################################################    
    nacelle                    = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    nacelle.tag                = 'nacelle_1'
    nacelle.length             = 3
    nacelle.diameter           = 42 * Units.inches
    nacelle.areas.wetted       = 0.01*(2*np.pi*0.01/2)
    nacelle.origin             = [[3.0,2.25,1.232]]
    nacelle.flow_through       = False  
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_0'
    nac_segment.percent_x_location = 0.0 
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)   
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_1'
    nac_segment.percent_x_location = 0.16316  
    nac_segment.height             = 0.4
    nac_segment.width              = 0.4
    nacelle.append_segment(nac_segment)   

    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_2'
    nac_segment.percent_x_location = 0.1933799  
    nac_segment.percent_z_location = -0.027046 
    nac_segment.height             = 0.587
    nac_segment.width              = 0.90
    nacelle.append_segment(nac_segment)  
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_3'
    nac_segment.percent_x_location = 0.469773  
    nac_segment.percent_z_location = 0.014215
    nac_segment.height             = 0.62
    nac_segment.width              = 0.9
    nacelle.append_segment(nac_segment)  
     
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_4'
    nac_segment.percent_x_location = 0.9
    nac_segment.percent_z_location = 0.02025
    nac_segment.height             = 0.2
    nac_segment.width              = 0.9
    nacelle.append_segment(nac_segment)  

    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 1.0
    nac_segment.percent_z_location = 0.02025
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)  
    
    vehicle.append_component(nacelle)  

    nacelle_2          = deepcopy(nacelle)
    nacelle_2.tag      = 'nacelle_2'
    nacelle_2.origin   = [[3,-2.25,1.232]]
    vehicle.append_component(nacelle_2)    
 
    # ########################################################  Energy Network  #########################################################  
    net                                         = RCAIDE.Framework.Networks.Fuel()   

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Fuel Line
    #------------------------------------------------------------------------------------------------------------------------------------  
    fuel_line                                   = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()   
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Fuel Tank & Fuel. Update Fuel tank location and size
    #------------------------------------------------------------------------------------------------------------------------------------       
    fuel_tank                                             = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank() 
    fuel_tank.origin                                      = vehicle.wings.main_wing.origin  
    fuel_tank.fuel                                        = RCAIDE.Library.Attributes.Propellants.Aviation_Gasoline() 
    fuel_tank.fuel.mass_properties.mass                   = 1190 *Units.lbs 
    fuel_tank.fuel.mass_properties.center_of_gravity      = wing.mass_properties.center_of_gravity
    fuel_tank.internal_volume                             = fuel_tank.fuel.mass_properties.mass/fuel_tank.fuel.density   
    fuel_line.fuel_tanks.append(fuel_tank)  

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Starboard Propulsor. Continental GTSIO-520-S.
    #------------------------------------------------------------------------------------------------------------------------------------   
    starboard_propulsor                        = RCAIDE.Library.Components.Powertrain.Propulsors.Internal_Combustion_Engine()      
    starboard_propulsor.origin                 = [[3.36,2.25,1.15]]
    starboard_propulsor.tag                    = 'starboard_propulsor'        

    # Engine                     
    engine                                     = RCAIDE.Library.Components.Powertrain.Converters.Engine()
    engine.sea_level_power                     = 375. * Units.horsepower 
    engine.rated_speed                         = 3350. * Units.rpm 
    engine.power_specific_fuel_consumption     = 0.40  * Units['lb/hp/hr'] # This is a very rough estimate
    engine.origin                              = [[3.75,2.25,1.15]]
    starboard_propulsor.engine                 = engine 
    # ice_prop.sealevel_static_thrust          = 2500 # N
     
    # Propeller. propeller dimensions: TCDS_EASA_A.637_TECNAM_P2012_issue_13.pdf         
    propeller                                        = RCAIDE.Library.Components.Powertrain.Converters.Propeller() 
    propeller.tag                                    = 'propeller_1'  
    propeller.tip_radius                             = 2.26/2   
    propeller.number_of_blades                       = 3
    propeller.hub_radius                             = 10.     * Units.inches 
    propeller.cruise.design_freestream_velocity      = 175.*Units['mph']   
    propeller.cruise.design_angular_velocity         = 2700. * Units.rpm 
    propeller.cruise.design_Cl                       = 0.7 
    propeller.cruise.design_altitude                 = 2500. * Units.feet 
    propeller.cruise.design_thrust                   = 5000   
    propeller.clockwise_rotation                     = False
    propeller.variable_pitch                         = True  
    propeller.origin                                 = [[3.36,2.25,1.15]]   
    airfoil                                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                                      = 'NACA_4412' 
    airfoil.coordinate_file                          = 'NACA_4412.txt'     
    airfoil.polar_files                              =[ 'NACA_4412_polar_Re_50000.txt',
                                                        'NACA_4412_polar_Re_100000.txt',
                                                        'NACA_4412_polar_Re_200000.txt',
                                                        'NACA_4412_polar_Re_500000.txt',
                                                        'NACA_4412_polar_Re_1000000.txt']   
    propeller.append_airfoil(airfoil)                       
    propeller.airfoil_polar_stations                 = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]   
    starboard_propulsor.propeller                    = propeller   
    starboard_propulsor.nacelle = nacelle  
              
    # design propeller ICE  
    design_internal_combustion_engine(starboard_propulsor)
    net.propulsors.append(starboard_propulsor) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    port_propulsor                                  = deepcopy(starboard_propulsor)
    port_propulsor.active_fuel_tanks                = ['fuel_tank'] 
    port_propulsor.tag                              = 'port_propulsor' 
    port_propulsor.origin                           = [[3.36,-2.25,1.15]]
    port_propulsor.nacelle.tag                      = 'port_propulsor_nacelle' 
    port_propulsor.nacelle.origin                   = [[3.0,-2.25,1.232]]
    port_propulsor.propeller.origin                 = [[3.36,-2.25,1.15]]
    port_propulsor.engine.origin                    = [[3.75,-2.25,1.15]]
    port_propulsor.propeller.tag                    = 'propeller_2'
    
    # append propulsor to distribution line 
    net.propulsors.append(port_propulsor) 
    fuel_line.assigned_propulsors =  [[starboard_propulsor.tag, port_propulsor.tag]]

    # append bus   
    net.fuel_lines.append(fuel_line) 
    vehicle.append_energy_network(net)

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------ 
    Wuav                                        = 2. * Units.lbs
    avionics                                    = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.mass_properties.uninstalled        = Wuav
    vehicle.avionics                            = avionics    

    return vehicle
 
if __name__ == '__main__': 
    main()    
    plt.show()


