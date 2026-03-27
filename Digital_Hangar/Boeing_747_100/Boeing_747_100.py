# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units   
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan    import design_turbofan 
from RCAIDE.Library.Plots                 import *       

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
        export_vsp_vehicle(vehicle, 'Boeing_747_100')
    except ImportError:
        pass 
    
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,export_gltf=True)  
    return  

def vehicle_setup(): 

    airfoil_file_path =  os.path.join(os.path.split(sys.path[0])[0], 'Airfoils_and_Polars') + os.sep 
    polar_file_path   =  os.path.join(os.path.join(os.path.split(sys.path[0])[0], 'Airfoils_and_Polars') , 'Polars') + os.sep  
    
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    
    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Boeing_747_100'

    # ################################################# Vehicle-level Properties #################################################   
    vehicle.mass_properties.max_takeoff               = 323000. * Units.kilogram 
    vehicle.mass_properties.takeoff                   = 300000 * Units.kilogram 
    vehicle.mass_properties.operating_empty           = 162500 * Units.kilogram   
    vehicle.mass_properties.max_zero_fuel             = 238780.0 * Units.kilogram 
    vehicle.mass_properties.max_fuel                  = 143475.0 * Units.kilogram  
    vehicle.mass_properties.moments_of_inertia.tensor = np.array([[2.4661e7,0.0,-5.48775e7],[0.0,4.48505e7,0.0],[-5.48775e7,0.0,6.73435e7]]) 
    vehicle.mass_properties.max_payload               = 76280.  * Units.kilogram  
    vehicle.mass_properties.center_of_gravity         = [[32.00, 0, 0]]
    vehicle.flight_envelope.ultimate_load             = 3.75  
    vehicle.flight_envelope.positive_limit_load       = 2.5   
    vehicle.flight_envelope.design_mach_number        = 0.85   
    vehicle.flight_envelope.design_cruise_altitude    = 35000.0*Units.feet 
    vehicle.flight_envelope.design_range              = 7305.0 * Units.nmi 
    vehicle.reference_area                            = 572.68 * Units['meters**2']   
    vehicle.number_of_passengers                      = 366  
    vehicle.systems.control                           = "fully powered"
    vehicle.systems.accessories                       = "long range" 
    
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing'
    wing.aspect_ratio                     = 6.5
    wing.sweeps.quarter_chord             = 37 * Units.deg
    wing.thickness_to_chord               = 0.1
    wing.spans.projected                  = 61 
    wing.chords.root                      = 16.9 * Units.meter
    wing.chords.tip                       = 4 * Units.meter
    wing.taper                            = wing.chords.tip / wing.chords.root
    wing.chords.mean_aerodynamic          = 9.722 * Units.meter 
    wing.areas.reference                  = 572.68
    wing.areas.wetted                     = 1203.0 
    wing.twists.root                      = 0.0 * Units.degrees 
    wing.twists.tip                       = -3.0 * Units.degrees 
    wing.origin                           = [[17.864,0,-1.557]]
    wing.aerodynamic_center               = [0,0,0] 
    wing.vertical                         = False
    wing.dihedral                         = 5.5 * Units.degrees 
    wing.xz_plane_symmetric               = True  
    wing.dynamic_pressure_ratio           = 1.0
        
    # Wing Segments 
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 5.5 * Units.degrees
    segment.sweeps.quarter_chord          = 36.4 * Units.degrees
    segment.thickness_to_chord            = .1
    root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    root_airfoil.coordinate_file          = airfoil_file_path + 'transonic_wing_root_section_airfoil.txt'
    segment.append_airfoil(root_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Yehudi'
    segment.percent_span_location         = 0.44
    segment.twist                         = 0.0 * Units.deg
    segment.root_chord_percent            = 0.53
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 5.5 * Units.degrees
    segment.sweeps.quarter_chord          = 38. * Units.degrees
    segment.thickness_to_chord            = 0.1
    yehudi_airfoil                        = RCAIDE.Library.Components.Airfoils.Airfoil()
    yehudi_airfoil.coordinate_file        = airfoil_file_path +'transonic_wing_inboard_section_airfoil.txt'
    segment.append_airfoil(yehudi_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'Tip'
    segment.percent_span_location         = 1.
    segment.twist                         = -3. * Units.degrees
    segment.root_chord_percent            = 0.235
    segment.thickness_to_chord            = 0.1
    segment.dihedral_outboard             = 0.
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = .1
    tip_airfoil                           = RCAIDE.Library.Components.Airfoils.Airfoil()
    tip_airfoil.coordinate_file           = airfoil_file_path +'transonic_wing_tip_section_airfoil.txt'
    segment.append_airfoil(tip_airfoil)
    wing.append_segment(segment)
    
 
    
    # control surfaces -------------------------------------------
    slat                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Slat()
    slat.tag                      = 'slat'
    slat.span_fraction_start      = 0.2
    slat.span_fraction_end        = 0.963
    slat.deflection               = 0.0 * Units.degrees
    slat.chord_fraction           = 0.075
    wing.append_control_surface(slat)

    flap                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                      = 'flap'
    flap.span_fraction_start      = 0.1
    flap.span_fraction_end        = 0.67
    flap.deflection               = 0.0 * Units.degrees
    flap.configuration_type       = 'Double_slotted'
    flap.chord_fraction           = 0.20
    wing.append_control_surface(flap)

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.68
    aileron.span_fraction_end     = 0.963
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.20
    wing.append_control_surface(aileron)
    

    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing     = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 4.0
    wing.sweeps.quarter_chord    = 37.50 * Units.deg  
    wing.thickness_to_chord      = 0.1
    wing.taper                   = 0.213  
    wing.spans.projected         = 22.96 
    wing.chords.root             = 9.4 
    wing.chords.tip              = 2.0
    wing.chords.mean_aerodynamic = 5.7
    wing.areas.reference         = 65.44
    wing.areas.exposed           = 61.0    # Exposed area of the horizontal tail
    wing.areas.wetted            = 137.424     # Wetted area of the horizontal tail
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees 
    wing.origin                  = [[57.843, 0, 3.197]]
    wing.aerodynamic_center      = [0,0,0] 
    wing.vertical                = False
    wing.xz_plane_symmetric      = True 
    wing.dynamic_pressure_ratio  = 0.9


    # Wing Segments
    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'root_segment'
    segment.percent_span_location  = 0.0
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 1.0
    segment.dihedral_outboard      = 7.5 * Units.degrees
    segment.sweeps.quarter_chord   = 37.3570  * Units.degrees 
    segment.thickness_to_chord     = .1
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.212               
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = .1
    wing.append_segment(segment)
    
        

    # control surfaces -------------------------------------------
    elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                   = 'elevator'
    elevator.span_fraction_start   = 0.09
    elevator.span_fraction_end     = 0.9
    elevator.deflection            = 0.0  * Units.deg
    elevator.chord_fraction        = 0.3
    wing.append_control_surface(elevator)

    # add to vehicle
    vehicle.append_component(wing)
    

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'

    wing.aspect_ratio            = 1.323
    wing.sweeps.quarter_chord    = 42.3  * Units.deg   
    wing.thickness_to_chord      = 0.1
    wing.taper                   = 0.287

    wing.spans.projected         = 10.59
    wing.total_length            = wing.spans.projected 
    
    wing.chords.root             = 13.567 
    wing.chords.tip              = 3.9 
    wing.chords.mean_aerodynamic = 9.86

    wing.areas.reference         = 84.833
    wing.areas.wetted            = 178.15 
    
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [[53.50, 0, 4.754]]
    wing.aerodynamic_center      = [0,0,0]

    wing.vertical                = True
    wing.xz_plane_symmetric      = False
    wing.t_tail                  = False

    wing.dynamic_pressure_ratio  = 1.0

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 0 * Units.degrees
    segment.sweeps.quarter_chord          = 62.6 * Units.degrees  
    segment.thickness_to_chord            = .1
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_1'
    segment.percent_span_location         = 0.115
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.811
    segment.dihedral_outboard             = 0. * Units.degrees
    segment.sweeps.quarter_chord          = 42.857 * Units.degrees   
    segment.thickness_to_chord            = .1
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'segment_2'
    segment.percent_span_location         = 1.0
    segment.twist                         = 0. * Units.deg
    segment.root_chord_percent            = 0.287
    segment.dihedral_outboard             = 0.0 * Units.degrees
    segment.sweeps.quarter_chord          = 0.0    
    segment.thickness_to_chord            = .1  
    wing.append_segment(segment)
    
 
    # control surfaces -------------------------------------------
    rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                   = 'rudder'
    rudder.span_fraction_start   = 0.1 
    rudder.span_fraction_end     = 0.95 
    rudder.deflection            = 0 
    rudder.chord_fraction        = 0.33  
    wing.append_control_surface(rudder)    
    
    
        

    # add to vehicle
    vehicle.append_component(wing)
    

    # ################################################# Fuselage ################################################################ 
        
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage() 
    fuselage.number_coach_seats                 = vehicle.number_of_passengers 
    fuselage.seats_abreast                      = 10
    fuselage.seat_pitch                         = 0.9     * Units.meter 
    fuselage.fineness.nose                      = 1.4
    fuselage.fineness.tail                      = 3. 
    fuselage.lengths.nose                       = 8.5   * Units.meter
    fuselage.lengths.tail                       = 21.0   * Units.meter
    fuselage.lengths.total                      = 68.6 * Units.meter  
    fuselage.lengths.fore_space                 = 2.    * Units.meter
    fuselage.lengths.aft_space                  = 15.    * Units.meter
    fuselage.width                              = 6.88  * Units.meter
    fuselage.heights.maximum                    = 8.278  * Units.meter
    fuselage.effective_diameter                 = 6.38     * Units.meter
    fuselage.areas.side_projected               = fuselage.heights.maximum * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.wetted                       = np.pi * fuselage.width/2 * fuselage.lengths.total * Units['meters**2'] 
    fuselage.areas.front_projected              = np.pi * fuselage.width/2      * Units['meters**2']  
    fuselage.differential_pressure              = 5.0e4 * Units.pascal
    fuselage.heights.at_quarter_length          = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_three_quarters_length   = fuselage.heights.maximum * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = fuselage.heights.maximum* Units.meter
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  = 0.00 
    segment.height                              = 0.000 
    segment.width                               = 0.000  
    fuselage.append_segment(segment)   
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_1'    
    segment.percent_x_location                  = 0.00221 
    segment.percent_z_location                  = 0.00102 
    segment.height                              = 1.08323
    segment.width                               = 1.1818
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.00461 
    segment.percent_z_location                  = 0.00102 
    segment.height                              = 1.53878 
    segment.width                               = 1.5816
    fuselage.append_segment(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.016 
    segment.percent_z_location                  = 0.00205 
    segment.height                              = 2.54694 
    segment.width                               = 2.65306 
    fuselage.append_segment(segment)   

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.03394 	
    segment.percent_z_location                  = 0.00359 
    segment.height                              = 3.92653 
    segment.width                               = 3.67347 
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.0626 
    segment.percent_z_location                  = 0.00768 
    segment.height                              = 5.73061 
    segment.width                               = 4.7449 
    fuselage.append_segment(segment)     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.07797 
    segment.percent_z_location                  = 0.01178 
    segment.height                              = 6.79184 
    segment.width                               = 5.15306 
    fuselage.append_segment(segment)             
     
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.09075 
    segment.percent_z_location                  = 0.01358 
    segment.height                              = 7.32245 
    segment.width                               = 5.47147 
    fuselage.append_segment(segment)    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.10966 
    segment.percent_z_location                  = 0.01511 
    segment.height                              = 7.90612 
    segment.width                               = 5.86535 
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'     
    segment.percent_x_location                  = 0.12489 
    segment.percent_z_location                  = 0.01486
    segment.height                              = 8.0631
    segment.width                               = 6.08122
    fuselage.append_segment(segment)     
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_10'     
    segment.percent_x_location                  = 0.17716 
    segment.percent_z_location                  = 0.01332
    segment.height                              = 8.27755
    segment.width                               = 6.73469
    fuselage.append_segment(segment)   
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'     
    segment.percent_x_location                  = 0.2396 
    segment.percent_z_location                  = 0.01178
    segment.height                              = 8.22449
    segment.width                               = 6.93878
    fuselage.append_segment(segment)    
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_12'     
    segment.percent_x_location                  = 0.34226
    segment.percent_z_location                  = 0.00589
    segment.height                              = 7.53469
    segment.width                               = 6.88776
    fuselage.append_segment(segment)             
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_13'     
    segment.percent_x_location                  = 0.68625 
    segment.percent_z_location                  = 0.01332
    segment.height                              = 7.26939
    segment.width                               = 6.88776
    fuselage.append_segment(segment)               
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_14'     
    segment.percent_x_location                  = 0.7728 
    segment.percent_z_location                  = 0.02408
    segment.height                              = 6.04898
    segment.width                               = 6.37755
    fuselage.append_segment(segment)
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_15'     
    segment.percent_x_location                  = 0.88616 
    segment.percent_z_location                  = 0.03868
    segment.height                              = 4.35102
    segment.width                               = 3.67347
    fuselage.append_segment(segment)
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_16'     
    segment.percent_x_location                  = 0.93925 
    segment.percent_z_location                  = 0.04611
    segment.height                              = 3.50204
    segment.width                               = 2.19388
    fuselage.append_segment(segment)
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_17'     
    segment.percent_x_location                  = 0.99233 
    segment.percent_z_location                  = 0.05149
    segment.height                              = 2.54694
    segment.width                               = 0.96939
    fuselage.append_segment(segment)
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_18'     
    segment.percent_x_location                  = 1.00 
    segment.percent_z_location                  = 0.05686
    segment.height                              = 0.0
    segment.width                               = 0.0
    fuselage.append_segment(segment)        


    # add to vehicle
    vehicle.append_component(fuselage) 

    # ################################################# Landing Gear #############################################################   
    # ------------------------------------------------------------------        
    # Landing Gear
    # Source: https://www.boeing.com/content/dam/boeing/boeingdotcom/commercial/airports/acaps/747_123sp.pdf 
    # ------------------------------------------------------------------  
    main_gear               = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.tire_diameter = 50.0 * Units.inches
    main_gear.strut_length  = 10.0 * Units.ft 
    main_gear.units         = 4    # Number of main landing gear
    main_gear.wheels        = 4    # Number of wheels on the main landing gear
    vehicle.append_component(main_gear)  

    nose_gear               = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()       
    nose_gear.tire_diameter = 40. * Units.inches
    nose_gear.units         = 1    # Number of nose landing gear
    nose_gear.wheels        = 2    # Number of wheels on the nose landing gear
    nose_gear.strut_length  = 8.0 * Units.ft 
    vehicle.append_component(nose_gear)
     
 
    # ################################################# Energy Network #######################################################          
    #------------------------------------------------------------------------------------------------------------------------- 
    #  Turbofan Network
    #-------------------------------------------------------------------------------------------------------------------------   
    net                                         = RCAIDE.Framework.Networks.Fuel() 
    
    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                   = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  
           
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan_1                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() 
    turbofan_1.tag                                = 'propulsor_1'
    turbofan_1.active_fuel_tanks                  = ['fuel_tank']   
    turbofan_1.origin                             = [[13.72, 4.86,-1.1]] 
    turbofan_1.engine_length                      = 3.934     
    turbofan_1.bypass_ratio                       = 5.0   
    turbofan_1.design_altitude                    = 0.0*Units.ft
    turbofan_1.design_mach_number                 = 0.01   
    turbofan_1.design_thrust                      = 193000.0* Units.N 

    # fan                
    fan                                         = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                     = 'fan'
    fan.polytropic_efficiency                   = 0.93
    fan.pressure_ratio                          = 1.67   
    turbofan_1.fan                              = fan        

    # working fluid                   
    turbofan_1.working_fluid                    = RCAIDE.Library.Attributes.Gases.Air() 

    
    # Ram inlet 
    ram                                         = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                     = 'ram' 
    turbofan_1.ram                              = ram 
          
    # inlet nozzle          
    inlet_nozzle                                = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                            = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency          = 0.98
    inlet_nozzle.pressure_ratio                 = 0.98 
    turbofan_1.inlet_nozzle                     = inlet_nozzle 

    # low pressure compressor    
    low_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                   = 'lpc'
    low_pressure_compressor.polytropic_efficiency = 0.91
    low_pressure_compressor.pressure_ratio        = 1.9   
    turbofan_1.low_pressure_compressor            = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.91
    high_pressure_compressor.pressure_ratio        = 8.4    
    turbofan_1.high_pressure_compressor            = high_pressure_compressor

    # low pressure turbine  
    low_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                       ='lpt'
    low_pressure_turbine.mechanical_efficiency     = 0.99
    low_pressure_turbine.polytropic_efficiency     = 0.93 
    turbofan_1.low_pressure_turbine                = low_pressure_turbine
   
    # high pressure turbine     
    high_pressure_turbine                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                      ='hpt'
    high_pressure_turbine.mechanical_efficiency    = 0.99
    high_pressure_turbine.polytropic_efficiency    = 0.93 
    turbofan_1.high_pressure_turbine               = high_pressure_turbine 

    # combustor  
    combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                  = 'Comb'
    combustor.efficiency                           = 0.99 
    combustor.alphac                               = 1.0     
    combustor.turbine_inlet_temperature            = 1500
    combustor.pressure_ratio                       = 0.95
    combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    turbofan_1.combustor                           = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.95
    core_nozzle.pressure_ratio                     = 0.99  
    turbofan_1.core_nozzle                         = core_nozzle
             
    # fan nozzle             
    fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                 = 'fan nozzle'
    fan_nozzle.polytropic_efficiency               = 0.95
    fan_nozzle.pressure_ratio                      = 0.99 
    turbofan_1.fan_nozzle                          = fan_nozzle 
    
    # design turbofan
    design_turbofan(turbofan_1)  
    # append propulsor to distribution line
    

    # Nacelle 
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                            = 2.30
    nacelle.length                              = 2.70
    nacelle.tag                                 = 'nacelle_1'
    nacelle.inlet_diameter                      = 2.2
    nacelle.origin                              = [[32.483, -21.000,-1.95]] 
    nacelle.areas.wetted                        = 1.1*np.pi*nacelle.diameter*nacelle.length
    nacelle_airfoil                             = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code          = '2410'
    nacelle.append_airfoil(nacelle_airfoil) 
    turbofan_1.nacelle                          = nacelle
    
    net.propulsors.append(turbofan_1)
    

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Propulsor 2 (Inner Port Side)
    #------------------------------------------------------------------------------------------------------------------------------------      
    # copy turbofan
    turbofan_2                                  = deepcopy(turbofan_1)
    turbofan_2.active_fuel_tanks                = ['fuel_tank'] 
    turbofan_2.tag                              = 'propulsor_2' 
    turbofan_2.origin                           = [[24.72,-11.685,-2.6]]
    turbofan_2.nacelle.origin                   = [[24.72,-11.685,-2.6]]
         
    # append propulsor to distribution line 
    net.propulsors.append(turbofan_2)

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Propulsor 3 (Inner Starboard Side)
    #------------------------------------------------------------------------------------------------------------------------------------      
    # copy turbofan
    turbofan_3                                  = deepcopy(turbofan_1)
    turbofan_3.active_fuel_tanks                = ['fuel_tank'] 
    turbofan_3.tag                              = 'propulsor_3' 
    turbofan_3.origin                           = [[24.72, 11.685,-2.6]]
    turbofan_3.nacelle.origin                   = [[24.72, 11.685,-2.6]]
         
    # append propulsor to distribution line 
    net.propulsors.append(turbofan_3)
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Propulsor 4 (Outer Starboard Side)
    #------------------------------------------------------------------------------------------------------------------------------------      
    # copy turbofan
    turbofan_4                                  = deepcopy(turbofan_1)
    turbofan_4.active_fuel_tanks                = ['fuel_tank'] 
    turbofan_4.tag                              = 'propulsor_4' 
    turbofan_4.origin                           = [[32.483, 21.000,-1.95]]
    turbofan_4.nacelle.origin                   = [[32.483, 21.000,-1.95]]
         
    # append propulsor to distribution line 
    net.propulsors.append(turbofan_4)    
        
    #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #------------------------------------------------------------------------------------------------------------------------- 
    # fuel tank
    fuel_tank                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank() 
    fuel_tank.fuel                                   = RCAIDE.Library.Attributes.Propellants.Jet_A1()      
    fuel_line.fuel_tanks.append(fuel_tank)
    
    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network      
    fuel_line.assigned_propulsors =  [[turbofan_1.tag, turbofan_2.tag, turbofan_3.tag, turbofan_4.tag]]

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network      
    net.fuel_lines.append(fuel_line)        
    
    # Append energy network to aircraft 
    vehicle.append_energy_network(net)       

    return vehicle
  

if __name__ == '__main__': 
    main()     