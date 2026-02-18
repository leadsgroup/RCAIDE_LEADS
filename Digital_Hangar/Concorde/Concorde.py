# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units   
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet             import design_turbojet
from RCAIDE.Library.Plots                 import *      
from RCAIDE.Library.Methods.Performance   import *  

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
        export_vsp_vehicle(vehicle, 'Concorde')
    except ImportError:
        pass 
    
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,export_gltf=True)  
    return  

def vehicle_setup(): 

    airfoil_file_path =  os.path.join(os.path.split(sys.path[0])[0], '_Airfoils_and_Polars') + os.sep 
    polar_file_path   =  os.path.join(os.path.join(os.path.split(sys.path[0])[0], '_Airfoils_and_Polars') , 'Polars') + os.sep
    
    #------------------------------------------------------------------------------------------------------------------------------------
    # ################################################# Vehicle-level Properties ########################################################  
    #------------------------------------------------------------------------------------------------------------------------------------
    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Concorde'
    
    # mass properties
    vehicle.mass_properties.max_takeoff     = 185000.   # kg
    vehicle.mass_properties.operating_empty = 78700.   # kg
    vehicle.mass_properties.takeoff         = 185000   # kg, adjusted due to significant fuel burn on runway
    vehicle.mass_properties.cargo           = 1000.  * Units.kilogram   
    vehicle.mass_properties.max_zero_fuel   = 92000.
    vehicle.mass_properties.max_fuel        = 95680
        
    # envelope properties 

    vehicle.flight_envelope.design_cruise_altitude   = 40000 * Units.feet 
    vehicle.flight_envelope.design_dynamic_pressure  = 55169 
    vehicle.flight_envelope.design_mach_number       = 2.05 
    vehicle.flight_envelope.ultimate_load            = 5.7
    vehicle.flight_envelope.limit_load               = 3.8       
    vehicle.flight_envelope.positive_limit_load      = 2.5  
    vehicle.flight_envelope.design_range             = 4488  * Units.nmi    
  
    # basic parameters  
    vehicle.reference_area                 = 358.25      
    vehicle.number_of_passengers           = 100
    vehicle.systems.control                = "fully powered" 
    vehicle.systems.accessories            = "sst"
    vehicle.maximum_cross_sectional_area   = 13.9
    vehicle.total_length                   = 61.66
    vehicle.design_mach_number             = 2.02
    vehicle.design_range                   = 4505 * Units.miles
    vehicle.design_cruise_alt              = 60000.0 * Units.ft
    

    
    #------------------------------------------------------------------------------------------------------------------------------------
    # ##################################################### Landing Gear ################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    main_gear                                = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
    main_gear.origin                         = [[17, 0, -11]]
    main_gear.tire_diameter                  =  47    *  Units.inches  
    main_gear.rim_diameter                   =  22.1  *  Units.inches 
    main_gear.tire_width                     =  15.75 *  Units.inches 
    main_gear.strut_length                   =  12    * Units.ft 
    main_gear.wheels                         = 8   
    main_gear.number_of_gear_types_in_tandem = 2
    main_gear.number_of_wheels_in_gear_type  = 2  
    main_gear.xz_plane_symmetric             = True
    vehicle.append_component(main_gear)  

    nose_gear                                = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()
    nose_gear.origin                         = [[35, 0, -11]]
    nose_gear.tire_diameter                  = 31 *  Units.inches    
    nose_gear.rim_diameter                   = 14  *  Units.inches 
    nose_gear.tire_width                     = 10.75  *  Units.inches 
    nose_gear.strut_length                   = 12 * Units.ft 
    nose_gear.wheels                         = 2   
    nose_gear.number_of_gear_types_in_tandem = 1
    nose_gear.number_of_wheels_in_gear_type  = 2    
    vehicle.append_component(nose_gear)
    
        
    
    #------------------------------------------------------------------------------------------------------------------------------------
    # ######################################################## Wings ####################################################################  
    #------------------------------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------     
    
    wing = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'
    
    wing.aspect_ratio            = 1.83
    wing.sweeps.quarter_chord    = 59.5 * Units.deg
    wing.sweeps.leading_edge     = 66.5 * Units.deg
    wing.thickness_to_chord      = 0.03
    wing.taper                   = 0.
    
    wing.spans.projected           = 25.6    
    
    wing.chords.root               = 33.8
    wing.total_length              = 33.8
    wing.chords.tip                = 1.1
    wing.chords.mean_aerodynamic   = 18.4
    
    wing.areas.reference           = 358.25 
    wing.areas.wetted              = 601.
    wing.areas.exposed             = 326.5
    wing.areas.affected            = .6*wing.areas.reference
    
    wing.twists.root               = 0.0 * Units.degrees
    wing.twists.tip                = 0.0 * Units.degrees
    
    wing.origin                    = [[14,0,-.8]]
    wing.aerodynamic_center        = [35,0,0] 
    
    wing.vertical                  = False
    wing.xz_plane_symmetric        = True
    wing.high_lift                 = True
    wing.vortex_lift               = True
    wing.high_mach                 = True 
    wing.dynamic_pressure_ratio    = 1.0
     
    
    # set root sweep with inner section
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'section_1'
    segment.percent_span_location = 0.
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 1
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 67. * Units.deg
    segment.thickness_to_chord    = 0.03
    wing_airfoil                  = RCAIDE.Library.Components.Airfoils.Airfoil()   
    wing_airfoil.coordinate_file  = airfoil_file_path +  'NACA65_203.txt'  
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)
    
    # set section 2 start point
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'section_2'
    segment.percent_span_location = (6.15 * 2) /wing.spans.projected
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 13.8/wing.chords.root
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 48. * Units.deg
    segment.thickness_to_chord    = 0.03
    wing_airfoil                  = RCAIDE.Library.Components.Airfoils.Airfoil()   
    wing_airfoil.coordinate_file  = airfoil_file_path +  'NACA65_203.txt'
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)
    
    
    # set section 3 start point
    segment = RCAIDE.Library.Components.Wings.Segments.Segment() 
    segment.tag                   = 'section_3'
    segment.percent_span_location = (12.1 *2) /wing.spans.projected
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 4.4/wing.chords.root
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 71. * Units.deg 
    segment.thickness_to_chord    = 0.03
    wing_airfoil                  = RCAIDE.Library.Components.Airfoils.Airfoil()   
    wing_airfoil.coordinate_file  = airfoil_file_path +  'NACA65_203.txt'
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)  
    
    # set tip
    segment = RCAIDE.Library.Components.Wings.Segments.Segment() 
    segment.tag                   = 'tip'
    segment.percent_span_location = 1.
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 1.1/wing.chords.root
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 0.
    segment.thickness_to_chord    = 0.03
    wing_airfoil                  = RCAIDE.Library.Components.Airfoils.Airfoil()   
    wing_airfoil.coordinate_file  = airfoil_file_path +  'NACA65_203.txt'
    segment.append_airfoil(wing_airfoil)
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
    flap.span_fraction_start      = 0.42
    flap.span_fraction_end        = 0.62  
    flap.deflection               = 0.0 * Units.degrees
    flap.configuration_type       = 'single_slotted'
    flap.chord_fraction           = 0.14
    wing.append_control_surface(flap)

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.62
    aileron.span_fraction_end     = 0.83 
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.22 
    wing.append_control_surface(aileron)
        
    spoiler                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    spoiler.tag                   = 'elevator'
    spoiler.span_fraction_start   = 0.08
    spoiler.span_fraction_end     = 0.24
    spoiler.deflection            = 0.0 * Units.degrees
    spoiler.chord_fraction        = 0.1
    wing.append_control_surface(spoiler)       
    
    
    # add to vehicle
    vehicle.append_component(wing)
    
    
    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------
    
    wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'    
    
    wing.aspect_ratio            = 0.74     
    wing.sweeps.quarter_chord    = 60 * Units.deg
    wing.thickness_to_chord      = 0.04
    wing.taper                   = 0.14 
    wing.spans.projected         = 6.0     
    wing.chords.root             = 14.5
    wing.total_length            = 14.5
    wing.chords.tip              = 2.7
    wing.chords.mean_aerodynamic = 8.66 
    wing.areas.reference         = 33.91     
    wing.areas.wetted            = 76. 
    wing.areas.exposed           = 38.
    wing.areas.affected          = 33.91 
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees   
    wing.origin                  = [[42.,0,1.]]
    wing.aerodynamic_center      = [50,0,0]     
    wing.vertical                = True 
    wing.xz_plane_symmetric      = False
    wing.t_tail                  = False
    wing.high_mach               = True    
    wing.dynamic_pressure_ratio  = 1.0  

    # set root sweep with inner section
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'section_1'
    segment.percent_span_location = 0.0
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 14.5/14.5
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 63. * Units.deg
    segment.thickness_to_chord    = 0.04
    tail_airfoil                  = RCAIDE.Library.Components.Airfoils.Airfoil() 
    tail_airfoil.coordinate_file  =  airfoil_file_path +  'supersonic_tail.txt'  
    segment.append_airfoil(tail_airfoil)
    wing.append_segment(segment)
    
    # set mid section start point
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'section_2'
    segment.percent_span_location = 2.4/(6.0) + wing.segments['section_1'].percent_span_location
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 7.5/14.5
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 40. * Units.deg
    segment.thickness_to_chord    = 0.04
    tail_airfoil                  = RCAIDE.Library.Components.Airfoils.Airfoil() 
    tail_airfoil.coordinate_file  =  airfoil_file_path +  'supersonic_tail.txt'  
    segment.append_airfoil(tail_airfoil)
    wing.append_segment(segment)
    
    # set tip
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'tip'
    segment.percent_span_location = 1.
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 2.7/14.5
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 0.
    segment.thickness_to_chord    = 0.04
    tail_airfoil                  = RCAIDE.Library.Components.Airfoils.Airfoil() 
    tail_airfoil.coordinate_file  =  airfoil_file_path +  'supersonic_tail.txt'  
    segment.append_airfoil(tail_airfoil)
    wing.append_segment(segment)    

    # add to vehicle
    vehicle.append_component(wing)    


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------
    
    fuselage                                        = RCAIDE.Library.Components.Fuselages.Fuselage() 
    fuselage.seats_abreast                          = 4
    fuselage.seat_pitch                             = 38. * Units.inches 
    fuselage.fineness.nose                          = 4.3
    fuselage.fineness.tail                          = 6.4 
    fuselage.lengths.total                          = 61.66   
    fuselage.width                                  = 2.88 
    fuselage.heights.maximum                        = 3.32    
    fuselage.heights.maximum                        = 3.32   
    fuselage.supersonic                             = True 
    fuselage.heights.at_quarter_length              = 3.32    
    fuselage.heights.at_wing_root_quarter_chord     = 3.32    
    fuselage.heights.at_three_quarters_length       = 3.32    
    fuselage.areas.wetted                           = 442.
    fuselage.areas.front_projected                  = 11.9 
    fuselage.effective_diameter                     = 3.1 
    fuselage.differential_pressure                  = 7.4e4 * Units.pascal    # Maximum differential pressure


    cabin                                               = RCAIDE.Library.Components.Fuselages.Cabins.Cabin()
    cabin.origin                                        =  [[8.5, 0, -0.5]]  
    business_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business() 
    business_class.number_of_seats_abrest               = 4
    business_class.number_of_rows                       = 32  
    business_class.seat_arm_rest_width                  = 3 *  Units.inches 
    business_class.seat_width                           = 17 *  Units.inches
    business_class.aisle_width                          = 13  *  Units.inches  
    business_class.galley_lavatory_percent_x_locations  = [0, 0.5, 0.51, 1]       
    business_class.type_A_exit_percent_x_locations      = [0.02, 0.4, 1]
    cabin.append_cabin_class(business_class)  
    fuselage.append_cabin(cabin)          
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0000
    segment.percent_z_location                  =  -0.61 /fuselage.lengths.total  
    fuselage.append_segment(segment)   
     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 3.02870/fuselage.lengths.total   
    segment.percent_z_location                  = -0.3583/fuselage.lengths.total     
    segment.height                              = 1.4502  
    segment.width                               = 1.567  
    fuselage.append_segment(segment)
    

    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  =   5.7742/fuselage.lengths.total   
    segment.percent_z_location                  =  -0.1500/fuselage.lengths.total    
    segment.height                              = 2.356  
    segment.width                               = 2.3429  
    fuselage.append_segment(segment)
    

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  =  9.0791/fuselage.lengths.total    
    segment.percent_z_location                  = 0  
    segment.height                              = 3.0581  
    segment.width                               = 2.741  
    fuselage.append_segment(segment) 
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_4'    
    segment.percent_x_location                  = 12.384/fuselage.lengths.total  
    segment.percent_z_location                  = 0  
    segment.height                              = 3.3200 
    segment.width                               = 2.880  
    fuselage.append_segment(segment)
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 43.228 /fuselage.lengths.total    
    segment.percent_z_location                  = 0 
    segment.height                              = 3.3200   
    segment.width                               = 2.8800  
    fuselage.append_segment(segment)

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  =  47.5354/fuselage.lengths.total     
    segment.percent_z_location                  =  0.100/fuselage.lengths.total     
    segment.height                              = 2.952  
    segment.width                               = 2.8800  
    fuselage.append_segment(segment)   

                 
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 1  
    segment.percent_z_location                  = 1.2332/fuselage.lengths.total   
    segment.height                              = 0.00100  
    segment.width                               = 0.00100  
    fuselage.append_segment(segment)
    
    
    vehicle.append_component(fuselage)

    #------------------------------------------------------------------------------------------------------------------------------------
    # ########################################################## Energy Network ######################################################### 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    #initialize the fuel network
    net                                            = RCAIDE.Framework.Networks.Fuel() 
    net.identical_propulsors                       = True 
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------------------  
    fuel_line                                     = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line() 
    

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Inner Right Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    outer_right_turbojet                          = RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet()  
    outer_right_turbojet.tag                      = 'outer_right_turbojet'     
    outer_right_turbojet.length                   = 4.039
    outer_right_turbojet.nacelle_diameter         = 1.3
    outer_right_turbojet.inlet_diameter           = 1.212 
    outer_right_turbojet.areas.wetted             = 30
    outer_right_turbojet.design_altitude          = 60000.0*Units.ft
    outer_right_turbojet.design_mach_number       = 2.02
    outer_right_turbojet.design_thrust            = 10000. * Units.lbf  
    outer_right_turbojet.origin                   = [[37.,5.5,-1.6]] 
    outer_right_turbojet.working_fluid            = RCAIDE.Library.Attributes.Gases.Air()
    
    # Ram  
    ram                                           = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                       = 'ram' 
    outer_right_turbojet.ram                      = ram 
         
    # Inlet Nozzle         
    inlet_nozzle                                  = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                              = 'inlet_nozzle' 
    inlet_nozzle.polytropic_efficiency            = 1.0
    inlet_nozzle.pressure_ratio                   = 1.0
    inlet_nozzle.pressure_recovery                = 0.94 
    outer_right_turbojet.inlet_nozzle             = inlet_nozzle    
          
    #  Low Pressure Compressor      
    lp_compressor                                 = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    lp_compressor.tag                             = 'low_pressure_compressor' 
    lp_compressor.polytropic_efficiency           = 0.88
    lp_compressor.pressure_ratio                  = 3.1     
    outer_right_turbojet.low_pressure_compressor  = lp_compressor         
        
    # High Pressure Compressor        
    hp_compressor                                 = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    hp_compressor.tag                             = 'high_pressure_compressor' 
    hp_compressor.polytropic_efficiency           = 0.88
    hp_compressor.pressure_ratio                  = 5.0  
    outer_right_turbojet.high_pressure_compressor = hp_compressor
 
    # Low Pressure Turbine 
    lp_turbine                                    = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    lp_turbine.tag                                ='low_pressure_turbine' 
    lp_turbine.mechanical_efficiency              = 0.99
    lp_turbine.polytropic_efficiency              = 0.89 
    outer_right_turbojet.low_pressure_turbine     = lp_turbine      
             
    # High Pressure Turbine         
    hp_turbine                                    = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    hp_turbine.tag                                ='high_pressure_turbine' 
    hp_turbine.mechanical_efficiency              = 0.99
    hp_turbine.polytropic_efficiency              = 0.87 
    outer_right_turbojet.high_pressure_turbine    = hp_turbine   
          
    # Combustor   
    combustor                                     = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                 = 'combustor' 
    combustor.efficiency                          = 0.96
    combustor.alphac                              = 1.0     
    combustor.turbine_inlet_temperature           = 1440.
    combustor.pressure_ratio                      = 0.92
    combustor.fuel_data                           = RCAIDE.Library.Attributes.Propellants.Jet_A()     
    outer_right_turbojet.combustor                = combustor
     
    #  Afterburner  
    afterburner                                   = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    afterburner.tag                               = 'afterburner' 
    afterburner.efficiency                        = 0.95
    afterburner.alphac                            = 1.0     
    afterburner.turbine_inlet_temperature         = 1500
    afterburner.pressure_ratio                    = 1.0
    afterburner.fuel_data                         = RCAIDE.Library.Attributes.Propellants.Jet_A()     
    outer_right_turbojet.afterburner              = afterburner   
 
    # Core Nozzle 
    nozzle                                        = RCAIDE.Library.Components.Powertrain.Converters.Supersonic_Nozzle()   
    nozzle.tag                                    = 'core_nozzle' 
    nozzle.pressure_recovery                      = 0.95
    nozzle.pressure_ratio                         = 1.    
    outer_right_turbojet.core_nozzle              = nozzle
    
    # design turbojet 
    design_turbojet(outer_right_turbojet) 

    nacelle                                     = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    nacelle.diameter                            = 1.3
    nacelle.tag                                 = 'nacelle_1'
    nacelle.origin                              = [[37.,5.5,-1.6]] 
    nacelle.length                              = 10
    nacelle.inlet_diameter                      = 1.1 
    nacelle.areas.wetted                        = 30.
    
    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Rounded_Rectangle_Segment()
    nac_segment.tag                             = 'segment_1' 
    nac_segment.orientation_euler_angles        = [0., -45*Units.degrees,0.]     
    nac_segment.percent_x_location              = 0.0  
    nac_segment.height                          = 2.12
    nac_segment.width                           = 1.5
    nac_segment.curvature                       = 10
    nacelle.append_segment(nac_segment)         

    nac_segment                                 = RCAIDE.Library.Components.Nacelles.Segments.Rounded_Rectangle_Segment()
    nac_segment.tag                             = 'segment_2'
    nac_segment.percent_x_location              = 1.0
    nac_segment.height                          = 1.5
    nac_segment.width                           = 1.5
    nac_segment.curvature                       = 10
    nacelle.append_segment(nac_segment)      
    outer_right_turbojet.nacelle = nacelle  
    net.propulsors.append(outer_right_turbojet) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Inner Right Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    inner_right_turbojet                     = deepcopy(outer_right_turbojet) 
    inner_right_turbojet.tag                 = 'inner_right_turbojet'      
    inner_right_turbojet.origin              = [[37.,4,-1.6]]     
    nacelle_2                                = deepcopy(nacelle)
    nacelle_2.tag                            = 'nacelle_2'
    nacelle_2.origin                         = [[37.,4,-1.6]]
    inner_right_turbojet.nacelle = nacelle_2 
    net.propulsors.append(inner_right_turbojet) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Inner Right Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------    
    inner_left_turbojet                     = deepcopy(outer_right_turbojet)    
    inner_left_turbojet.tag                 = 'inner_left_turbojet'  
    inner_left_turbojet.origin              = [[37.,-4,-1.6]]   
    nacelle_3                               = deepcopy(nacelle)
    nacelle_3.tag                           = 'nacelle_3'
    nacelle_3.origin                        = [[37.,-4,-1.6]]
    inner_left_turbojet.nacelle = nacelle_3 
    net.propulsors.append(inner_left_turbojet) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Inner Left Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------    
    outer_left_turbojet                     = deepcopy(outer_right_turbojet)
    outer_left_turbojet.tag                 = 'outer_left_turbojet'      
    outer_left_turbojet.origin              = [[37.,-5.5,-1.6]]   
    nacelle_4                               = deepcopy(nacelle)
    nacelle_4.tag                           = 'nacelle_4'
    nacelle_4.origin                        = [[37.,-5.5,-1.6]]
    outer_left_turbojet.nacelle = nacelle_4
    net.propulsors.append(outer_left_turbojet) 
 
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Fuel Tank & Fuel
    #------------------------------------------------------------------------------------------------------------------------------------   
    fuel_tank                                      = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank()
    fuel_tank.tag                                  = 'tank_9'
    fuel_tank.mass_properties.center_of_gravity    = np.array([[26.5,0,0]])
    fuel_tank.mass_properties.mass                 = 11096
    fuel_tank.fuel_selector_ratio                  = 1/8
    fuel_tank.fuel                                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(fuel_tank) 
    
    fuel_tank                                      = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank()
    fuel_tank.tag                                  = 'tank_10'
    fuel_tank.mass_properties.center_of_gravity    = np.array([[28.7,0,0]])
    fuel_tank.mass_properties.mass                 = 11943
    fuel_tank.fuel_selector_ratio                  = 1/8
    fuel_tank.fuel                                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(fuel_tank) 
    
    fuel_tank                                      = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank()
    fuel_tank.tag                                  = 'tank_1_and_4'
    fuel_tank.mass_properties.center_of_gravity    = np.array([[31.0,0,0]])
    fuel_tank.mass_properties.mass                 = 4198+4198
    fuel_tank.fuel_selector_ratio                  = 1/8
    fuel_tank.fuel                                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(fuel_tank) 
    
    fuel_tank                                      = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank()
    fuel_tank.tag                                  = 'tank_5_and_8'
    fuel_tank.mass_properties.center_of_gravity    = np.array([[32.9,0,0]])
    fuel_tank.mass_properties.mass                 = 7200+12838
    fuel_tank.fuel_selector_ratio                  = 1/8
    fuel_tank.fuel                                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(fuel_tank) 
    
    fuel_tank                                      = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank()
    fuel_tank.tag                                  = 'tank_6_and_7'
    fuel_tank.mass_properties.center_of_gravity    = np.array([[37.4,0,0]])
    fuel_tank.mass_properties.mass                 = 11587+7405
    fuel_tank.fuel_selector_ratio                  = 1/8
    fuel_tank.fuel                                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(fuel_tank) 
    
    fuel_tank                                      = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank()
    fuel_tank.tag                                  = 'tank_5A_and_7A'
    fuel_tank.mass_properties.center_of_gravity    = np.array([[40.2,0,0]])
    fuel_tank.mass_properties.mass                 = 2225+2225
    fuel_tank.fuel_selector_ratio                  = 1/8
    fuel_tank.fuel                                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(fuel_tank) 
    
    fuel_tank                                      = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank()
    fuel_tank.tag                                  = 'tank_2_and_3'
    fuel_tank.mass_properties.center_of_gravity    = np.array([[40.2,0,0]])
    fuel_tank.mass_properties.mass                 = 4570+4570
    fuel_tank.fuel_selector_ratio                  = 1/8
    fuel_tank.fuel                                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(fuel_tank)  
 
    fuel_tank = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank()
    fuel_tank.tag                                  = 'tank_11'
    fuel_tank.mass_properties.center_of_gravity    = np.array([[49.8,0,0]])
    fuel_tank.mass_properties.mass                 = 10415
    fuel_tank.fuel_selector_ratio                  = 1/8
    fuel_tank.fuel                                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(fuel_tank)
     
    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network      
    fuel_line.assigned_propulsors =  [[outer_left_turbojet.tag,inner_left_turbojet.tag, outer_right_turbojet.tag, inner_right_turbojet.tag]]    
    
     # Append fuel line to network      
    net.fuel_lines.append(fuel_line)    
  
    #------------------------------------------------------------------------------------------------------------------------------------          
    # Append energy network to aircraft 
    vehicle.append_energy_network(net)     

    return vehicle
 
if __name__ == '__main__': 
    main()    
