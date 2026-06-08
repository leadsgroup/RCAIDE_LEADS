# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units    
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor   import design_electric_rotor  
from RCAIDE.Library.Plots                 import *       
from RCAIDE import  load 
from RCAIDE import  save  


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
    redesign_rotors = False  
    vehicle  = vehicle_setup(redesign_rotors)

    try:
        import vsp as vsp
        from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle 
        export_vsp_vehicle(vehicle, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Tilt_Stopped_Rotor_V_Tail'))
    except ImportError:
        pass
        
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,save_filename=os.path.join(os.path.dirname(os.path.abspath(__file__)),'Tilt_Stopped_Rotor_V_Tail'),export_gltf=True,show_figure=True)  
    
    return 
 
def vehicle_setup(redesign_rotors = False): 
    local_path        = sys.path[0] + os.sep
    airfoil_file_path =  os.path.join(os.path.split(sys.path[0])[0], 'Airfoils_and_Polars') + os.sep 
    polar_file_path   =  os.path.join(os.path.join(os.path.split(sys.path[0])[0], 'Airfoils_and_Polars') , 'Polars') + os.sep  
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------             
    vehicle               = RCAIDE.Vehicle()
    vehicle.tag           = 'Tilt_Stopped_Rotor_V_Tail'
    vehicle.configuration = 'eVTOL'
     
    #------------------------------------------------------------------------------------------------------------------------------------
    # ################################################# Vehicle-level Properties #####################################################  
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # mass properties 
    vehicle.mass_properties.max_takeoff                = 2404 
    vehicle.mass_properties.takeoff                    = vehicle.mass_properties.max_takeoff
    vehicle.mass_properties.operating_empty            = vehicle.mass_properties.max_takeoff
    vehicle.mass_properties.center_of_gravity          = [[ 2.1345, 0 , 0 ]] 
    vehicle.mass_properties.moments_of_inertia.tensor  = np.array([[164627.7,0.0,0.0],[0.0,471262.4,0.0],[0.0,0.0,554518.7]])
    vehicle.flight_envelope.ultimate_load              = 5.7   
    vehicle.flight_envelope.positive_limit_load        = 3.  
    vehicle.number_of_passengers                       = 5 
        
    #------------------------------------------------------------------------------------------------------------------------------------
    # ######################################################## Wings ####################################################################  
    #------------------------------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
    wing                          = RCAIDE.Library.Components.Wings.Main_Wing() 
    wing.aspect_ratio             = 8.95198  # will  be overwritten
    wing.sweeps.quarter_chord     = 0.0  
    wing.thickness_to_chord       = 0.14 
    wing.taper                    = 0.292
    wing.spans.projected          = 11.82855
    wing.total_length             = 1.75
    wing.chords.root              = 1.75
    wing.chords.tip               = 1.0
    wing.chords.mean_aerodynamic  = 1.5
    wing.dihedral                 = 0.0  
    wing.areas.reference          = 15.629
    wing.twists.root              = 4. * Units.degrees
    wing.twists.tip               = 0. 
    wing.origin                   = [[1.5, 0., 0.991]]
    wing.aerodynamic_center       = [ 1.567, 0., 0.991]    
    wing.winglet_fraction         = 0.0  
    wing.xz_plane_symmetric       = True
    wing.vertical                 = False
    airfoil                       = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file       = 'NACA_63_412.txt'
    
    # Segment                                  
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'Section_1'   
    segment.percent_span_location = 0.0
    segment.twist                 = 4. * Units.degrees 
    segment.root_chord_percent    = 1. 
    segment.dihedral_outboard     = 8 * Units.degrees
    segment.sweeps.quarter_chord  = 0.9  * Units.degrees 
    segment.thickness_to_chord    = 0.16  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)               
    
    # Segment                                   
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'Section_2'    
    segment.percent_span_location = 3.5/wing.spans.projected
    segment.twist                 = 3. * Units.degrees 
    segment.root_chord_percent    = 1.4000/1.7500
    segment.dihedral_outboard     = 0.0 * Units.degrees
    segment.sweeps.quarter_chord  = 1.27273 * Units.degrees 
    segment.thickness_to_chord    = 0.16  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)               
     
    # Segment                                  
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'Section_3'   
    segment.percent_span_location = 11.3/wing.spans.projected 
    segment.twist                 = 2.0 * Units.degrees 
    segment.root_chord_percent    = 1.000/1.7500
    segment.dihedral_outboard     = -25.000* Units.degrees 
    segment.sweeps.quarter_chord  = 20.000* Units.degrees 
    segment.thickness_to_chord    = 0.16  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)     
    
    # Segment                                  
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'Section_4'   
    segment.percent_span_location = 11.6/wing.spans.projected 
    segment.twist                 = 0.0 * Units.degrees 
    segment.root_chord_percent    = 0.9/1.7500
    segment.dihedral_outboard     = -40. * Units.degrees 
    segment.sweeps.quarter_chord  = 45.0 * Units.degrees 
    segment.thickness_to_chord    = 0.16  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)  
    
    # Segment                                  
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'Section_5'   
    segment.percent_span_location = 1.0
    segment.twist                 = 0.0 * Units.degrees 
    segment.root_chord_percent    = 0.35/1.7500
    segment.dihedral_outboard     = 0  * Units.degrees 
    segment.sweeps.quarter_chord  = 0  * Units.degrees 
    segment.thickness_to_chord    = 0.16  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)                  
                                          
    # control surfaces ------------------------------------------- 
    flap                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                      = 'flap'
    flap.span_fraction_start      = 0.2
    flap.span_fraction_end        = 0.5
    flap.deflection               = 0.0 * Units.degrees 
    flap.chord_fraction           = 0.20
    wing.append_control_surface(flap)  
    

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.7
    aileron.span_fraction_end     = 0.9 
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.2
    wing.append_control_surface(aileron)      
 
    vehicle.append_component(wing)  
    
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #   Horizontal Tail
    #------------------------------------------------------------------------------------------------------------------------------------
    wing                          = RCAIDE.Library.Components.Wings.Horizontal_Tail() 
    wing.aspect_ratio             = 5
    wing.sweeps.quarter_chord     = 25. * Units.degrees
    wing.thickness_to_chord       = 0.12 
    wing.spans.projected          = 2.71805
    wing.taper                    = 0.35
    wing.chords.root              = 1.0
    wing.total_length             = 1.0
    wing.chords.tip               = wing.chords.root * wing.taper 
    wing.dihedral                 = 45 *Units.degrees
    wing.areas.reference          = 2.14279 
    wing.twists.root              = 0.0
    wing.twists.tip               = 0.0
    wing.origin                   = [[  5.374 ,0.0 ,  0.49]]
    wing.aerodynamic_center       = [   5.374, 0.0,   0.49]  
    wing.xz_plane_symmetric       = True      

    # Segment                                  
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'root'   
    segment.percent_span_location = 0
    segment.twist                 = 0.0 * Units.degrees 
    segment.root_chord_percent    = 1
    segment.dihedral_outboard     = 45. * Units.degrees 
    segment.sweeps.quarter_chord  = 30.0 * Units.degrees  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)  
    
    # Segment                                  
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'tip'   
    segment.percent_span_location = 1.0
    segment.twist                 = 0.0 * Units.degrees 
    segment.root_chord_percent    = 0.35
    segment.dihedral_outboard     = 0  * Units.degrees 
    segment.sweeps.quarter_chord  = 0  * Units.degrees  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)
    
    elevator                              = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                          = 'elevator'
    elevator.span_fraction_start          = 0.6
    elevator.span_fraction_end            = 0.9
    elevator.deflection                   = 0.0  * Units.deg
    elevator.chord_fraction               = 0.4
    wing.append_control_surface(elevator)       
    

    rudder                                = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                            = 'rudder'
    rudder.span_fraction_start            = 0.1
    rudder.span_fraction_end              = 0.5
    rudder.deflection                     = 0.0  * Units.deg
    rudder.chord_fraction                 = 0.4
    wing.append_control_surface(rudder)  
        
    # add to vehicle  
    vehicle.append_component(wing)     
      
    #------------------------------------------------------------------------------------------------------------------------------------
    # ##########################################################   Fuselage  ############################################################   
    #------------------------------------------------------------------------------------------------------------------------------------ 
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 2.  
    fuselage.seat_pitch                         = 3.  
    fuselage.fineness.nose                      = 0.88   
    fuselage.fineness.tail                      = 1.13   
    fuselage.lengths.nose                       = 0.5  
    fuselage.lengths.tail                       = 1.5
    fuselage.lengths.cabin                      = 4.46 
    fuselage.lengths.total                      = 6.46
    fuselage.width                              = 1.75
    fuselage.heights.maximum                    = 4.65 * Units.feet      # change 
    fuselage.heights.at_quarter_length          = 3.75 * Units.feet      # change 
    fuselage.heights.at_wing_root_quarter_chord = 4.65 * Units.feet      # change 
    fuselage.heights.at_three_quarters_length   = 4.26 * Units.feet      # change 
    fuselage.areas.wetted                       = 236. * Units.feet**2   # change 
    fuselage.areas.front_projected              = 0.14 * Units.feet**2   # change 
    fuselage.effective_diameter                 = 1.276     # change 
    fuselage.differential_pressure              = 0. 

    # define cabin    
    cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.origin                                      = [[1, 0, 0]]
    cabin.offset_x = 1.0
    economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 2
    economy_class.number_of_rows                      = 3 
    economy_class.seat_arm_rest_width                 = 2 *  Units.inches 
    economy_class.seat_width                          = 15 *  Units.inches
    economy_class.aisle_width                         = 0  *  Units.inches  
    economy_class.number_of_seats                     = economy_class.number_of_rows  * economy_class.number_of_seats_abrest 
    cabin.append_cabin_class(economy_class)
    fuselage.append_cabin(cabin)
        
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0 
    segment.percent_z_location                  = 0.   
    segment.height                              = 0. 
    segment.width                               = 0.
    segment.curvature                           = 3
    fuselage.append_segment(segment)                     
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 0.10912/fuselage.lengths.total 
    segment.percent_z_location                  = 0.00849
    segment.height                              = 0.481 
    segment.width                               = 0.553 
    segment.curvature                           = 3
    fuselage.append_segment(segment)           
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.47804/fuselage.lengths.total
    segment.percent_z_location                  = 0.02874
    segment.height                              = 1.00
    segment.width                               = 0.912 
    segment.curvature                           = 3
    fuselage.append_segment(segment)                     
                                                
    # Segment                                            
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.161  
    segment.percent_z_location                  = 0.04348  
    segment.height                              = 1.41
    segment.width                               = 1.174  
    fuselage.append_segment(segment)                     
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.284 
    segment.percent_z_location                  = 0.05435 
    segment.height                              = 1.62
    segment.width                               = 1.276  
    segment.curvature                           = 3
    fuselage.append_segment(segment)              
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 3.43026/fuselage.lengths.total
    segment.percent_z_location                  = 0.31483/fuselage.lengths.total 
    segment.height                              = 1.409
    segment.width                               = 1.121 
    segment.curvature                           = 3
    fuselage.append_segment(segment)                     
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 4.20546/fuselage.lengths.total
    segment.percent_z_location                  = 0.32216/fuselage.lengths.total
    segment.height                              = 1.11
    segment.width                               = 0.833
    segment.curvature                           = 3
    fuselage.append_segment(segment)                  
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 4.99358/fuselage.lengths.total
    segment.percent_z_location                  = 0.37815/fuselage.lengths.total
    segment.height                              = 0.78
    segment.width                               = 0.512 
    segment.curvature                           = 3
    fuselage.append_segment(segment)                  
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 1.
    segment.percent_z_location                  = 0.55/fuselage.lengths.total
    segment.height                              = 0.195  
    segment.width                               = 0.130
    segment.curvature                           = 3
    fuselage.append_segment(segment)                   
                                                
    vehicle.append_component(fuselage) 
    
    #------------------------------------------------------------------------------------------------------------------------------------
    # ##########################################################  Booms  ################################################################  
    #------------------------------------------------------------------------------------------------------------------------------------          
    boom                                    = RCAIDE.Library.Components.Booms.Boom()
    boom.tag                                = 'boom_1r'
    boom.configuration                      = 'boom'  
    boom.origin                             = [[ 0.5, 1.75,  1]]  
    boom.seats_abreast                      = 0.  
    boom.seat_pitch                         = 0.0 
    boom.fineness.nose                      = 0.950   
    boom.fineness.tail                      = 1.029   
    boom.lengths.nose                       = 0.2 
    boom.lengths.tail                       = 0.2
    boom.lengths.cabin                      = 4.15
    boom.lengths.total                      = 4.2
    boom.width                              = 0.15 
    boom.heights.maximum                    = 0.15  
    boom.heights.at_quarter_length          = 0.15  
    boom.heights.at_three_quarters_length   = 0.15 
    boom.heights.at_wing_root_quarter_chord = 0.15 
    boom.areas.wetted                       = 0.018
    boom.areas.front_projected              = 0.018 
    boom.effective_diameter                 = 0.15  
    boom.differential_pressure              = 0.  
    boom.symmetric                          = True 
    boom.index                              = 1
    
    # Segment  
    segment                           = RCAIDE.Library.Components.Booms.Segments.Segment() 
    segment.tag                       = 'segment_1'   
    segment.percent_x_location        = 0.
    segment.percent_z_location        = 0.0 
    segment.height                    = 0.05  
    segment.width                     = 0.05   
    boom.append_segment(segment)           
    
    # Segment                                   
    segment                           = RCAIDE.Library.Components.Booms.Segments.Segment()
    segment.tag                       = 'segment_2'   
    segment.percent_x_location        = 0.03
    segment.percent_z_location        = 0. 
    segment.height                    = 0.15 
    segment.width                     = 0.15 
    boom.append_segment(segment) 
    
    # Segment                                   
    segment                           = RCAIDE.Library.Components.Booms.Segments.Segment()
    segment.tag                       = 'segment_3'    
    segment.percent_x_location        = 0.97
    segment.percent_z_location        = 0. 
    segment.height                    = 0.15
    segment.width                     = 0.15
    boom.append_segment(segment)           
    
    # Segment                                  
    segment                           = RCAIDE.Library.Components.Booms.Segments.Segment()
    segment.tag                       = 'segment_4'   
    segment.percent_x_location        = 1.   
    segment.percent_z_location        = 0.   
    segment.height                    = 0.05   
    segment.width                     = 0.05   
    boom.append_segment(segment)           
    
    # add to vehicle
    vehicle.append_component(boom)   
    
    # add left long boom 
    boom              = deepcopy(vehicle.booms.boom_1r)
    boom.origin[0][1] = -boom.origin[0][1]
    boom.tag          = 'boom_1l' 
    vehicle.append_component(boom)         
     
    # add left long boom 
    boom               = deepcopy(vehicle.booms.boom_1r)
    boom.origin        = [[ 0.5,   3.25,   1.050]] 
    boom.tag           = 'boom_2r' 
    boom.lengths.total = 4.16
    vehicle.append_component(boom)  
     
    # add inner left boom 
    boom               = deepcopy(vehicle.booms.boom_1r)
    boom.origin        = [[0.5, - 3.25 ,    1.050 ]]   
    boom.lengths.total = 4.16
    boom.tag           = 'boom_2l' 
    vehicle.append_component(boom)
    

    # add left long boom 
    boom               = deepcopy(vehicle.booms.boom_1r)
    boom.origin        = [[ 0.5, 4.75,   1.050]] 
    boom.tag           = 'boom_3r' 
    boom.lengths.total = 4.16
    vehicle.append_component(boom)  
     
    # add inner left boom 
    boom               = deepcopy(vehicle.booms.boom_1r)
    boom.origin        = [[ 0.5, -4.75,    1.050 ]]   
    boom.lengths.total = 4.16
    boom.tag           = 'boom_3l' 
    vehicle.append_component(boom)    
    
    
    
    #------------------------------------------------------------------------------------------------------------------------------------
    # ########################################################  Energy Network  ######################################################### 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    network                                                = RCAIDE.Framework.Networks.Electric()   
    #==================================================================================================================================== 
    # Tilt Rotor Bus 
    #====================================================================================================================================          
    prop_rotor_bus                           = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    prop_rotor_bus.tag                       = 'prop_rotor_bus'
    prop_rotor_bus.origin                    = [[2.43775609, 0 , 1.2]]
    prop_rotor_bus.number_of_battery_modules = 2    

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus Battery
    #------------------------------------------------------------------------------------------------------------------------------------ 
    battery_module                                                    = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC() 
    battery_module.tag                                                = 'bus_battery'
    battery_module.electrical_configuration.series                    = 140  
    battery_module.electrical_configuration.parallel                  = 30  
    battery_module.geometrtic_configuration.normal_count              = 168
    battery_module.geometrtic_configuration.parallel_count            = 25
     
    modules_origins = [[0.25 , 0.0, 0.0],[1.5 , 0.0, 0.0]]  # large prop-rotor modules are beneath floor
    for m_i in range(prop_rotor_bus.number_of_battery_modules):
        module =  deepcopy(battery_module)
        module.tag = 'nmc_module_' + str(m_i+1) 
        module.origin = [modules_origins[m_i]]
        prop_rotor_bus.battery_modules.append(module) 
    prop_rotor_bus.initialize_bus_properties()

    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Front Propulsors 
    #------------------------------------------------------------------------------------------------------------------------------------    
     
    # Define Lift Propulsor Container 
    prop_rotor_propulsor                                = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
    prop_rotor_propulsor.tag                            = 'prop_rotor_propulsor'       
              
    # Electronic Speed Controller           
    prop_rotor_esc                                     = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    prop_rotor_esc.efficiency                          = 0.95    
    prop_rotor_esc.tag                                 = 'prop_rotor_esc_1'
    prop_rotor_esc.bus_voltage                         =  prop_rotor_bus.voltage
    prop_rotor_propulsor.electronic_speed_controller   = prop_rotor_esc  
    
    # Lift Rotor Design
    g                                             = 9.81                                    # gravitational acceleration   
    Hover_Load                                    = vehicle.mass_properties.takeoff*g *1.1  # hover load   

    prop_rotor                                    = RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor()   
    prop_rotor.tag                                = 'prop_rotor'   
    prop_rotor.tip_radius                         = 1.51 / 2
    prop_rotor.hub_radius                         = 0.15 * prop_rotor.tip_radius
    prop_rotor.number_of_blades                   = 4

    prop_rotor.hover.design_altitude              = 40 * Units.feet  
    prop_rotor.hover.design_thrust                = Hover_Load/12
    prop_rotor.hover.design_freestream_velocity   = np.sqrt(prop_rotor.hover.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))
    
    prop_rotor.oei.design_altitude                = 40 * Units.feet  
    prop_rotor.oei.design_thrust                  = Hover_Load/11 
    prop_rotor.oei.design_freestream_velocity     = np.sqrt(prop_rotor.oei.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))
    
    prop_rotor.cruise.design_altitude             = 1500 * Units.feet
    prop_rotor.cruise.design_thrust               = 3150 / 6
    prop_rotor.cruise.design_freestream_velocity  = 130.* Units['mph']      
    
    airfoil                                       = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file                       =   airfoil_file_path+ 'NACA_4412.txt'
    airfoil.polar_files                           = [polar_file_path + 'NACA_4412_polar_Re_50000.txt' ,
                                                     polar_file_path + 'NACA_4412_polar_Re_100000.txt' ,
                                                     polar_file_path + 'NACA_4412_polar_Re_200000.txt' ,
                                                     polar_file_path + 'NACA_4412_polar_Re_500000.txt' ,
                                                     polar_file_path + 'NACA_4412_polar_Re_1000000.txt',
                                                     polar_file_path + 'NACA_4412_polar_Re_3500000.txt',
                                                     polar_file_path + 'NACA_4412_polar_Re_5000000.txt',
                                                     polar_file_path + 'NACA_4412_polar_Re_7500000.txt' ]
    prop_rotor.append_airfoil(airfoil)                
    prop_rotor.airfoil_polar_stations = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]     
    prop_rotor_propulsor.rotor =  prop_rotor   
    
    
    #------------------------------------------------------------------------------------------------------------------------------------               
    # Front Rotor Motor  
    #------------------------------------------------------------------------------------------------------------------------------------    
    prop_rotor_motor                         = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    prop_rotor_motor.efficiency              = 0.95
    prop_rotor_motor.nominal_voltage         = prop_rotor_bus.voltage * 0.75 
    prop_rotor_motor.no_load_current         = 0.01     
    prop_rotor_propulsor.motor               = prop_rotor_motor
     
 
    # ##########################################################   Nacelles  ############################################################    
    nacelle                           = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    nacelle.tag                       = 'nacelle_1'
    nacelle.length                    = 0.4 
    nacelle.diameter                  = 0.2 
    nacelle.orientation_euler_angles  = [0, -90*Units.degrees,0.]
    nacelle.flow_through              = False  
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_1'
    nac_segment.percent_x_location = 0.0  
    nac_segment.height             = 0.0
    nac_segment.width              = 0.0
    nacelle.append_segment(nac_segment)

    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_2'
    nac_segment.percent_x_location = 0.05  
    nac_segment.height             = 0.1 
    nac_segment.width              = 0.1
    nac_segment.curvature          = 3
    nacelle.append_segment(nac_segment)
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_3'
    nac_segment.percent_x_location = 0.2  
    nac_segment.height             = 0.15
    nac_segment.width              = 0.15
    nac_segment.curvature          = 3
    nacelle.append_segment(nac_segment)   
    
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_4'
    nac_segment.percent_x_location = 0.5  
    nac_segment.height             = 0.3
    nac_segment.width              = 0.2
    nac_segment.curvature          = 3
    nacelle.append_segment(nac_segment)  
     
    nac_segment                    = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nac_segment.tag                = 'segment_5'
    nac_segment.percent_x_location = 1.0  
    nac_segment.height             = 0.3
    nac_segment.width              = 0.15
    nac_segment.curvature          = 3
    nacelle.append_segment(nac_segment)    
    
    prop_rotor_propulsor.nacelle = nacelle
    

    if redesign_rotors:
        design_electric_rotor(prop_rotor_propulsor)
        save_propulsor(prop_rotor_propulsor, os.path.join(local_path, 'tilt_rotor_propulsor.res'))
    else:
        regression_prop_rotor_propulsor = deepcopy(prop_rotor_propulsor)        
        design_electric_rotor(regression_prop_rotor_propulsor, iterations=2)
        loaded_propulsor = load_propulsor(os.path.join(local_path, 'tilt_rotor_propulsor.res'))

        for key,item in prop_rotor_propulsor.rotor.items():
            prop_rotor_propulsor.rotor[key] = loaded_propulsor.rotor[key] 
        for key,item in prop_rotor_propulsor.motor.items():
            prop_rotor_propulsor.motor[key] = loaded_propulsor.motor[key]        
        
    # Front Rotors Locations 
    origins       =  [[0.5, -1.75 ,1.4],  [0.5, 1.75 ,1.4] , [ 0.5 , -3.25  , 1.4] ,[ 0.5 ,  3.25, 1.4], [ 0.5 , -4.75  , 1.4] ,[ 0.5,4.75, 1.4]]
    rotor_origins =  [[0.5, -1.75 ,1.3],  [0.5, 1.75 ,1.3] , [ 0.5 , -3.25  , 1.3] ,[ 0.5 ,  3.25, 1.3], [ 0.5 , -4.75  , 1.3] ,[ 0.5,4.75, 1.3]]
   
    assigned_propulsor_list = []         
    for i in range(len(origins)): 
        propulsor_i                                       = deepcopy(prop_rotor_propulsor)
        propulsor_i.tag                                   = 'prop_rotor_propulsor_' + str(i + 1)
        propulsor_i.rotor.tag                             = 'prop_rotor_' + str(i + 1) 
        propulsor_i.rotor.origin                          = [rotor_origins[i]] 
        propulsor_i.rotor.orientation_euler_angles        = [0, 90*Units.degrees,0.] 
        propulsor_i.motor.tag                             = 'prop_rotor_motor_' + str(i + 1)   
        propulsor_i.motor.origin                          = [origins[i]]   
        propulsor_i.electronic_speed_controller.tag       = 'prop_rotor_esc_' + str(i + 1)  
        propulsor_i.electronic_speed_controller.origin    = [origins[i]]  
        propulsor_i.nacelle.tag                           = 'prop_rotor_nacelle_' + str(i + 1)  
        propulsor_i.nacelle.origin                        = [origins[i]]   
        network.propulsors.append(propulsor_i) 
        assigned_propulsor_list.append(propulsor_i.tag)
        
    prop_rotor_bus.assigned_propulsors = [assigned_propulsor_list] 
    
    # Avionics   
    avionics                       = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw            = 10. # Watts  
    avionics.mass_properties.mass  = 1.0 * Units.kg
    prop_rotor_bus.avionics        = avionics    

    # append forward bus
    network.busses.append(prop_rotor_bus)    
    
        
    #==================================================================================================================================== 
    # Rear Bus 
    #====================================================================================================================================          
    lift_rotor_bus                           = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    lift_rotor_bus.tag                       = 'lift_rotor_bus'  
    lift_rotor_bus.number_of_battery_modules =  2
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus Battery
    #------------------------------------------------------------------------------------------------------------------------------------ 
    battery_module                                                    = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC() 
    battery_module.tag                                                = 'lift_bus_battery'
    battery_module.electrical_configuration.series                    = 140   
    battery_module.electrical_configuration.parallel                  = 10  
    battery_module.geometrtic_configuration.normal_count              = 56
    battery_module.geometrtic_configuration.parallel_count            = 25

    modules_origins = [[4, 0.0, 0.0],[4, 0.0, 0.2 ]]  # rear modules are stacked inside cabin
    for m_i in range(lift_rotor_bus.number_of_battery_modules):
        module =  deepcopy(battery_module)
        module.tag = 'nmc_module_' + str(m_i+1) 
        module.origin = [modules_origins[m_i]]
        lift_rotor_bus.battery_modules.append(module) 
    lift_rotor_bus.initialize_bus_properties()
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Lift Propulsors 
    #------------------------------------------------------------------------------------------------------------------------------------    
     
    # Define Lift Propulsor Container 
    lift_propulsor                                         = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()  
              
    # Electronic Speed Controller           
    lift_rotor_esc                                         = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller() 
    lift_rotor_esc.efficiency                              = 0.95
    lift_rotor_esc.bus_voltage                             = lift_rotor_bus.voltage
    lift_propulsor.electronic_speed_controller             = lift_rotor_esc 
           
    # Lift Rotor Design              
    lift_rotor                                             = RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor()   
    lift_rotor.tip_radius                                  = 1.51 / 2
    lift_rotor.hub_radius                                  = 0.15* lift_rotor.tip_radius 
    lift_rotor.number_of_blades                            = 4

    lift_rotor.hover.design_altitude                       = 40 * Units.feet  
    lift_rotor.hover.design_thrust                         = Hover_Load/12
    lift_rotor.hover.design_freestream_velocity            = np.sqrt(lift_rotor.hover.design_thrust/(2*1.2*np.pi*(lift_rotor.tip_radius**2)))
               
    lift_rotor.oei.design_altitude                         = 40 * Units.feet  
    lift_rotor.oei.design_thrust                           = Hover_Load/11
    lift_rotor.oei.design_freestream_velocity              = np.sqrt(lift_rotor.oei.design_thrust/(2*1.2*np.pi*(lift_rotor.tip_radius**2)))
     
    airfoil                                                = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file                                =  airfoil_file_path+'NACA_4412.txt'
    airfoil.polar_files                                    = [polar_file_path + 'NACA_4412_polar_Re_50000.txt' ,
                                                              polar_file_path + 'NACA_4412_polar_Re_100000.txt' ,
                                                              polar_file_path + 'NACA_4412_polar_Re_200000.txt' ,
                                                              polar_file_path + 'NACA_4412_polar_Re_500000.txt' ,
                                                              polar_file_path + 'NACA_4412_polar_Re_1000000.txt',
                                                              polar_file_path + 'NACA_4412_polar_Re_3500000.txt',
                                                              polar_file_path + 'NACA_4412_polar_Re_5000000.txt',
                                                              polar_file_path + 'NACA_4412_polar_Re_7500000.txt' ]
    lift_rotor.append_airfoil(airfoil)                         
    lift_rotor.airfoil_polar_stations                      = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] 
    lift_propulsor.rotor =  lift_rotor          
    
    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Motor  
    #------------------------------------------------------------------------------------------------------------------------------------    
    lift_rotor_motor                   = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    lift_rotor_motor.efficiency        = 0.95
    lift_rotor_motor.nominal_voltage   = lift_rotor_bus.voltage*0.75  
    lift_rotor_motor.no_load_current   = 0.01 
    lift_propulsor.motor               = lift_rotor_motor 

    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Nacelle
    #------------------------------------------------------------------------------------------------------------------------------------
    lift_propulsor_nacelle                           =  deepcopy(nacelle)
    lift_propulsor_nacelle.orientation_euler_angles  = [0, -90*Units.degrees,0*Units.degrees]       
    lift_propulsor.nacelle                           = lift_propulsor_nacelle  
 
    if redesign_rotors:
        design_electric_rotor(lift_propulsor)
        save_propulsor(lift_propulsor, os.path.join(local_path, 'lift_rotor_propulsor.res'))
    else:
        regression_prop_rotor_propulsor = deepcopy(lift_propulsor)        
        design_electric_rotor(regression_prop_rotor_propulsor, iterations=2)
        loaded_propulsor = load_propulsor(os.path.join(local_path, 'lift_rotor_propulsor.res'))

        for key,item in lift_propulsor.rotor.items():
            lift_propulsor.rotor[key] = loaded_propulsor.rotor[key] 
        for key,item in lift_propulsor.motor.items():
            lift_propulsor.motor[key] = loaded_propulsor.motor[key]  
 
    # Front Rotors Locations
    origins       =  [[ 4.7, -1.75 ,1.4],  [ 4.7, 1.75 ,1.4] , [ 4.7, -3.25  , 1.4] ,[ 4.7,  3.25, 1.4], [ 4.7, -4.75  , 1.4] ,[  4.7 , 4.75, 1.4]]   
    rotor_origins =  [[ 4.7, -1.75 ,1.3],  [ 4.7, 1.75 ,1.3] , [ 4.7, -3.25  , 1.3] ,[ 4.7,  3.25, 1.3], [ 4.7, -4.75  , 1.3] ,[  4.7 , 4.75, 1.3]]   
    
    assigned_propulsor_list = []          
    for i in range(len(origins)): 
        propulsor_i                                       = deepcopy(lift_propulsor)
        propulsor_i.tag                                   = 'lift_rotor_propulsor_' + str(i + 1)
        propulsor_i.rotor.tag                             = 'lift_rotor_' + str(i + 1) 
        propulsor_i.rotor.origin                          = [rotor_origins[i]]  
        propulsor_i.motor.tag                             = 'lift_rotor_motor_' + str(i + 1)   
        propulsor_i.motor.origin                          = [origins[i]]  
        propulsor_i.electronic_speed_controller.tag       = 'lift_rotor_esc_' + str(i + 1)  
        propulsor_i.electronic_speed_controller.origin    = [origins[i]]  
        propulsor_i.nacelle.tag                           = 'lift_rotor_nacelle_' + str(i + 1)  
        propulsor_i.nacelle.origin                        = [origins[i]]    
        network.propulsors.append(propulsor_i) 
        assigned_propulsor_list.append(propulsor_i.tag) 
    lift_rotor_bus.assigned_propulsors = [assigned_propulsor_list] 

 
    # Avionics                            
    avionics                                               = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw                                    = 10. # Watts  
    avionics.mass_properties.mass                          = 1.0 * Units.kg
    lift_rotor_bus.avionics                                = avionics    

   
    network.busses.append(lift_rotor_bus)       
        
    # append energy network 
    vehicle.append_energy_network(network)   

    return vehicle

def load_propulsor(filename):
    propulsor =  load(filename)
    return propulsor

def save_propulsor(propulsor, filename):
    save(propulsor, filename)
    return
 
if __name__ == '__main__': 
    main()     