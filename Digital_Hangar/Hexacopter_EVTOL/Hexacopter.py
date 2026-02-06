
#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units 
from RCAIDE.Library.Methods.Geometry.Planform                     import segment_properties,wing_planform  
from RCAIDE.Library.Methods.Geometry.Airfoil.compute_airfoil_properties import compute_airfoil_properties 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor  import design_electric_rotor
from RCAIDE.Library.Plots                                         import * 
from RCAIDE import  load 
from RCAIDE import  save  

# python imports 
import os
import numpy as np 
from copy import deepcopy
import pickle
import  pandas as pd
import matplotlib.pyplot as plt  
  
# ----------------------------------------------------------------------------------------------------------------------
#  Main 
# ----------------------------------------------------------------------------------------------------------------------  
def main():           
         
    
    # Step 1: design a vehicle
    vehicle  = vehicle_setup()  

    try:
        import vsp as vsp
        from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle 
        export_vsp_vehicle(vehicle, 'Hexacopter')
    except ImportError:
        pass 
    
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,export_gltf=True)  
         
  
    return 
# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup(): 
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    vehicle                                     = RCAIDE.Vehicle()
    vehicle.tag                                 = 'Hexacopter'
    vehicle.configuration                       = 'eVTOL'
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties 
    vehicle.mass_properties.max_takeoff         = 3300 
    vehicle.mass_properties.takeoff             = vehicle.mass_properties.max_takeoff
    vehicle.mass_properties.operating_empty     = vehicle.mass_properties.max_takeoff 
    vehicle.mass_properties.center_of_gravity   = [[2.6, 0., 0. ] ] 
                                                
    # This needs updating                       
    vehicle.number_of_passengers                = 6
    vehicle.reference_area                      = 73  * Units.feet**2 
    vehicle.flight_envelope.ultimate_load       = 5.7   
    vehicle.flight_envelope.positive_limit_load = 3.  
                                                
    wing                                        = RCAIDE.Library.Components.Wings.Main_Wing()   
    wing.tag                                    = 'main_wing'   
    wing.aspect_ratio                           = 0.5 
    wing.sweeps.quarter_chord                   = 0.  
    wing.thickness_to_chord                     = 0.01   
    wing.spans.projected                        = 0.01  
    wing.chords.root                            = 0.01
    wing.total_length                           = 0.01
    wing.chords.tip                             = 0.01
    wing.chords.mean_aerodynamic                = 0.01
    wing.dihedral                               = 0.0  
    wing.areas.reference                        = 0.0001 
    wing.areas.wetted                           = 0.01
    wing.areas.exposed                          = 0.01  
    wing.symbolic                               = True 
    wing.xz_plane_symmetric                     = True 
    
    vehicle.append_component(wing)
    
    # ------------------------------------------------------    
    # FUSELAGE    
    # ------------------------------------------------------    
    
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 2.  
    fuselage.seat_pitch                         = 3.  
    fuselage.fineness.nose                      = 0.88   
    fuselage.fineness.tail                      = 1.13   
    fuselage.lengths.nose                       = 0.5 
    fuselage.lengths.tail                       = 0.5
    fuselage.lengths.cabin                      = 4.
    fuselage.lengths.total                      = 5.
    fuselage.width                              = 1.8
    fuselage.heights.maximum                    = 1.8
    fuselage.heights.at_quarter_length          = 1.8
    fuselage.heights.at_wing_root_quarter_chord = 1.8
    fuselage.heights.at_three_quarters_length   = 1.8
    fuselage.areas.wetted                       = 19.829265
    fuselage.areas.front_projected              = 1.4294246 
    fuselage.effective_diameter                 = 1.4
    fuselage.differential_pressure              = 1. 
    
    # Segment  
    segment                          = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                      = 'segment_0'   
    segment.percent_x_location       = 0.  
    segment.percent_z_location       = 0.0 
    segment.height                   = 0.1   
    segment.width                    = 0.1   
    fuselage.append_segment(segment)            
                                                
    # Segment                                   
    segment                         = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                     = 'segment_1'   
    segment.percent_x_location      = 0.200/4.
    segment.percent_z_location      = 0.1713/4.
    segment.height                  = 0.737
    segment.width                   = 1.2   
    fuselage.append_segment(segment)            
                                                
    # Segment                                   
    segment                         = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                     = 'segment_2'   
    segment.percent_x_location      = 0.8251/4.
    segment.percent_z_location      = 0.2840/4.
    segment.height                  = 1.40 
    segment.width                   = 1.8 
    fuselage.append_segment(segment)            
                                                
    # Segment                                  
    segment                         = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                     = 'segment_3'   
    segment.percent_x_location      = 3.342/4.
    segment.percent_z_location      = 0.356/4.
    segment.height                  = 1.40
    segment.width                   = 1.8 
    fuselage.append_segment(segment)  
                                                
    # Segment                                   
    segment                         = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                     = 'segment_4'   
    segment.percent_x_location      = 3.70004/4.
    segment.percent_z_location      = 0.4636/4.
    segment.height                  = 0.9444
    segment.width                   = 1.2 
    fuselage.append_segment(segment)             
    
    # Segment                                   
    segment                         = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                     = 'segment_5'   
    segment.percent_x_location      = 1.
    segment.percent_z_location      = 0.6320/4.
    segment.height                  = 0.1    
    segment.width                   = 0.1    
    fuselage.append_segment(segment)             
    
                                                 
    # add to vehicle
    vehicle.append_component(fuselage)   
    

    #------------------------------------------------------------------------------------------------------------------------------------
    # ########################################################  Energy Network  ######################################################### 
    #------------------------------------------------------------------------------------------------------------------------------------
    # define network
    network                           = RCAIDE.Framework.Networks.Electric() 
    network.charging_power            = 1000
    
    #==================================================================================================================================== 
    # Lift Bus 
    #====================================================================================================================================          
    bus                               = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    bus.tag                           = 'bus'
    bus.number_of_battery_modules     = 1
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus Battery
    #------------------------------------------------------------------------------------------------------------------------------------ 
    battery_module                                                    = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC() 
    battery_module.electrical_configuration.series                    = 150   
    battery_module.electrical_configuration.parallel                  = 270  
    battery_module.geometrtic_configuration.normal_count              = 900
    battery_module.geometrtic_configuration.parallel_count            = 45
    battery_module.geometrtic_configuration.stacking_rows             = 9
    battery_module.origin                                             = [[0.5, 0, 0]]
    bus.battery_modules.append(battery_module)    
    bus.initialize_bus_properties()

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Lift Propulsors 
    #------------------------------------------------------------------------------------------------------------------------------------    
     
    # Define Lift Propulsor Container 
    propulsor                                              = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
    propulsor.tag                                          = 'propulsor'
    propulsor.wing_mounted                                 = True         
              
    # Electronic Speed Controller           
    lift_rotor_esc                                         = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller() 
    lift_rotor_esc.efficiency                              = 0.95     
    lift_rotor_esc.bus_voltage                             = bus.voltage
    lift_rotor_esc.origin                                  = [[-0.073 ,  1.950 , 1.2]] 
    propulsor.electronic_speed_controller                  = lift_rotor_esc 
           
    # Lift Rotor Design              
    g                                                      = 9.81                                     
    Hover_Load                                             = vehicle.mass_properties.takeoff * g * 1.1 
    
    #lift_rotor                                             = RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor()    
    #lift_rotor.active                                      = True           
    #lift_rotor.tip_radius                                  = 2.5
    #lift_rotor.hub_radius                                  = 0.15 * lift_rotor.tip_radius 
    #lift_rotor.number_of_blades                            = 3
    
    #lift_rotor.hover.design_altitude                       = 40 * Units.feet  
    #lift_rotor.hover.design_thrust                         = Hover_Load/6
    #lift_rotor.hover.design_freestream_velocity            = np.sqrt(lift_rotor.hover.design_thrust/(2*1.2*np.pi*(lift_rotor.tip_radius**2)))
    
    #lift_rotor.oei.design_altitude                         = 40 * Units.feet  
    #lift_rotor.oei.design_thrust                           = Hover_Load/5  
    #lift_rotor.oei.design_freestream_velocity              = np.sqrt(lift_rotor.oei.design_thrust/(2*1.2*np.pi*(lift_rotor.tip_radius**2)))
    
    #airfoil                                                = RCAIDE.Library.Components.Airfoils.Airfoil()   
    #airfoil.coordinate_file                                = 'NACA_4412.txt'
    #airfoil.polar_files                                    = ['NACA_4412_polar_Re_50000.txt' ,
                                                             #'NACA_4412_polar_Re_100000.txt' ,
                                                             #'NACA_4412_polar_Re_200000.txt' ,
                                                              #'NACA_4412_polar_Re_500000.txt' ,
                                                              #'NACA_4412_polar_Re_1000000.txt',
                                                              #'NACA_4412_polar_Re_3500000.txt',
                                                              #'NACA_4412_polar_Re_5000000.txt',
                                                              #'NACA_4412_polar_Re_7500000.txt' ]
    #lift_rotor.append_airfoil(airfoil)                         
    #lift_rotor.airfoil_polar_stations                      = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0] 
    

    lift_rotor                               = RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor()
    lift_rotor.inputs                        = Data() 
    lift_rotor.inputs.blade_pitch_command    = 0 
    lift_rotor.inputs.y_axis_rotation        = 0.
    lift_rotor.tag                           = 'BO_105_40_percent_scale'
    lift_rotor.hub_radius                    = lift_rotor.tip_radius*0.1 
    lift_rotor.tip_radius                    = 2.5
    lift_rotor.hub_radius                    = 0.1 
    lift_rotor.number_of_blades              = 3     
    lift_rotor.thrust_angle                  = 0.
    lift_rotor.airfoil_flag                  = True 
    lift_rotor.hover.design_lift_coefficient = 0.7 
    num_sec                                  = 20
    non_dim_r                                = np.linspace(lift_rotor.hub_radius,0.99,num_sec)
    lift_rotor.radius_distribution           = non_dim_r*lift_rotor.tip_radius
    lift_rotor.thickness_to_chord            = np.ones(num_sec)*0.12
    lift_rotor.chord_distribution            = np.ones(num_sec)*0.225
    lift_rotor.max_thickness_distribution    = lift_rotor.thickness_to_chord* lift_rotor.chord_distribution
    lift_rotor.twist_distribution            = np.flip(np.linspace(90,75,num_sec))*Units.degrees 
    lift_rotor.airfoil_polar_stations        = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
    airfoil                                = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file                =  'NACA_23012.txt'
    airfoil.polar_files                    = ['NACA_23012_polar_Re_50000.txt',
                                                'NACA_23012_polar_Re_100000.txt',
                                                'NACA_23012_polar_Re_200000.txt',
                                                'NACA_23012_polar_Re_500000.txt',
                                                'NACA_23012_polar_Re_1000000.txt']
    airfoil.geometry                       = import_airfoil_geometry(airfoil.coordinate_file,airfoil.number_of_points)
    airfoil.polars                         = compute_airfoil_properties(airfoil.geometry,airfoil.polar_files)
    lift_rotor.append_airfoil(airfoil)       
    propulsor.rotor                                        = lift_rotor

    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Motor  
    #------------------------------------------------------------------------------------------------------------------------------------    
    lift_rotor_motor                                       = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    lift_rotor_motor.efficiency                            = 0.9
    lift_rotor_motor.nominal_voltage                       = bus.voltage * 3/4    
    lift_rotor_motor.no_load_current                       = 2      
    propulsor.motor                                        = lift_rotor_motor
     
    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Nacelle
    #------------------------------------------------------------------------------------------------------------------------------------     
    nacelle                                                = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.tag                                            = 'rotor_nacelle'
    nacelle.length                                         = 0.4
    nacelle.diameter                                       = 2.6 * 2
    nacelle.inlet_diameter                                 = 2.55 * 2     
    nacelle.orientation_euler_angles                       = [0,-90 * Units.degrees,0.]    
    nacelle.flow_through                                   = True  
    nacelle.areas.wetted                                   = np.pi*nacelle.diameter*nacelle.length
    nacelle_airfoil                                        = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code                     = '4305'
    nacelle.append_airfoil(nacelle_airfoil) 
    propulsor.nacelle                                      = nacelle
     
    #design_electric_rotor(propulsor)  
              
    origins = [[ -1.5,2.6,1.8],[ -1.5,-2.6,1.8], [2.5,6.0,1.8] ,[2.5,-6.,1.8], [6.5,2.6,1.8] ,[6.5,-2.6,1.8]]  
    
    for i in range(len(origins)): 
        propulsor_i                                       = deepcopy(propulsor)
        propulsor_i.tag                                   = 'rotor_propulsor_' + str(i + 1)
        propulsor_i.rotor.tag                             = 'rotor_' + str(i + 1) 
        propulsor_i.rotor.origin                          = [origins[i]]  
        propulsor_i.motor.tag                             = 'rotor_motor_' + str(i + 1)   
        propulsor_i.motor.origin                          = [origins[i]]  
        propulsor_i.electronic_speed_controller.tag       = 'rotor_esc_' + str(i + 1)  
        propulsor_i.electronic_speed_controller.origin    = [origins[i]]  
        propulsor_i.nacelle.tag                           = 'rotor_nacelle_' + str(i + 1)  
        propulsor_i.nacelle.origin                        = [origins[i]]   
        network.propulsors.append(propulsor_i)    
 
                             
    # Avionics                            
    avionics                                              = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw                                   = 10.   
    avionics.mass_properties.mass                         = 1.0 * Units.kg
    bus.avionics                                          = avionics    

   
    network.busses.append(bus)       
        
     
    vehicle.append_energy_network(network) 
 
    return vehicle


if __name__ == '__main__': 
    main()    
    plt.show()
