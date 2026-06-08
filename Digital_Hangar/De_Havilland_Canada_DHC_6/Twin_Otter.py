# Twin_Otter.py
# 
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE      
from RCAIDE.Framework.Core import Units  
from   RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop        import design_turboprop   
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
        export_vsp_vehicle(vehicle, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'Twin_Otter'))
    except ImportError:
        pass
        
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle,save_filename=os.path.join(os.path.dirname(os.path.abspath(__file__)),'Twin_Otter'),export_gltf=True,show_figure=True)  
    
    return 
 
def vehicle_setup():

    airfoil_file_path =  os.path.join(os.path.split(sys.path[0])[0], 'Airfoils_and_Polars') + os.sep 
    polar_file_path   =  os.path.join(os.path.join(os.path.split(sys.path[0])[0], 'Airfoils_and_Polars') , 'Polars') + os.sep
    #------------------------------------------------------------------------------------------------------------------------------------
    #   Initialize the Vehicle
    #------------------------------------------------------------------------------------------------------------------------------------ 

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Twin_Otter'

 
    # ################################################# Vehicle-level Properties ########################################################  

    # mass properties
    vehicle.mass_properties.max_takeoff   = 5670. # kg 
    vehicle.mass_properties.takeoff       = 5670. # kg 
    vehicle.mass_properties.max_zero_fuel = 5670. # kg 
    vehicle.mass_properties.max_payload   = 1414. # kg
    vehicle.mass_properties.max_fuel      = 1138. # kg
    vehicle.reference_area                = 39 
    vehicle.number_of_passengers                    = 18
    vehicle.systems.control               = "fully powered"
    vehicle.systems.accessories           = "commuter"    
     
    vehicle.flight_envelope.design_cruise_altitude   = 5000 * Units.feet
    vehicle.flight_envelope.design_dynamic_pressure  = 2130.457961
    vehicle.flight_envelope.design_mach_number       = 0.19
    vehicle.flight_envelope.ultimate_load            = 5.7
    vehicle.flight_envelope.limit_load               = 3.8       
    vehicle.flight_envelope.positive_limit_load      = 2.5  
    vehicle.flight_envelope.design_range             = 3500 * Units.nmi

    
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
    main_gear.symmetric                      = True
    main_gear.origin = [[8.0, 0, 0]]
    vehicle.append_component(main_gear)  

    nose_gear                                = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()   
    nose_gear.tire_diameter                  =  5 *  Units.inches   
    nose_gear.rim_diameter                   =  3 *  Units.inches 
    nose_gear.tire_width                     =  5 *  Units.inches 
    nose_gear.strut_length                   =  6.* Units.ft 
    nose_gear.origin = [[2.0, 0, 0]]
    nose_gear.wheels                         = 2   
    nose_gear.number_of_gear_types_in_tandem = 1
    nose_gear.number_of_wheels_in_gear_type  = 2    
    vehicle.append_component(nose_gear)
            

         
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
    wing.chords.root                      = 2.04 
    wing.chords.tip                       = 2.02 
    wing.chords.mean_aerodynamic          = 2.03 
    wing.taper                            = wing.chords.root/wing.chords.tip 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 3. * Units.degree 
    wing.twists.tip                       = 0
    wing.origin                           = [[5.38, 0, 1.35]] 
    wing.aerodynamic_center               = [[5.38 + 0.25 *wing.chords.root , 0, 1.35]]  
    wing.vertical                         = False
    wing.xz_plane_symmetric               = True 
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0        
    vehicle.mass_properties.center_of_gravity = [[6.3133,   0.  ,  0.38 ]]  # SOURCE: Design and aerodynamic analysis of a twin-engine commuter aircraft

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'inboard'
    segment.percent_span_location         = 0.0 
    segment.twist                         = 3. * Units.degree 
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0. * Units.degree 
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    airfoil                               = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                           = 'Clark_y' 
    airfoil.coordinate_file               = airfoil_file_path + 'Clark_y.txt'   
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.
    segment.twist                         = 0
    segment.root_chord_percent            = 0.999
    segment.dihedral_outboard             = 0.
    segment.sweeps.quarter_chord          = 0.
    segment.thickness_to_chord            = 0.12
    airfoil                               = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.tag                           = 'Clark_y' 
    airfoil.coordinate_file               = airfoil_file_path + 'Clark_y.txt'   
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)     
    
    # control surfaces -------------------------------------------
    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.55
    aileron.span_fraction_end     = 0.98
    aileron.deflection            = 0.0  * Units.deg
    aileron.chord_fraction        = 0.25 
    wing.append_control_surface(aileron)  

    # control surfaces -------------------------------------------
    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    aileron.tag                   = 'flap'
    aileron.span_fraction_start   = 0.15
    aileron.span_fraction_end     = 0.55
    aileron.deflection            = 0.0  * Units.deg
    aileron.chord_fraction        = 0.25 
    wing.append_control_surface(aileron)      
    
    # add to vehicle
    vehicle.append_component(wing)


    #------------------------------------------------------------------------------------------------------------------------------------  
    #   Horizontal Tail
    #------------------------------------------------------------------------------------------------------------------------------------    
    wing                                  = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag                              = 'horizontal_stabilizer' 
    wing.sweeps.quarter_chord            = 0.01 * Units.degree
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
    wing.origin                           = [[12.96 , 0 , 1.25]] 
    wing.aerodynamic_center               = [12.96 + wing.chords.root /4 , 0 , 1.25]
    wing.vertical                         = False
    wing.winglet_fraction                 = 0.0  
    wing.xz_plane_symmetric               = True 
    wing.dynamic_pressure_ratio           = 0.9

     # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'root'
    segment.percent_span_location         = 0.0   
    segment.root_chord_percent            = 1. 
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'yip'
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
    elevator.chord_fraction        = 0.45
    wing.append_control_surface(elevator)

    # add to vehicle
    vehicle.append_component(wing)


    #------------------------------------------------------------------------------------------------------------------------------------  
    #   Vertical Stabilizer
    #------------------------------------------------------------------------------------------------------------------------------------ 
    wing                                  = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag                              = 'vertical_stabilizer'     
    wing.sweeps.quarter_chord             = 23.73 * Units.degree 
    wing.thickness_to_chord               = 0.12 
    wing.areas.reference                  = 8.2 
    wing.spans.projected                  = 3.5
    wing.chords.root                      = 3.0 
    wing.chords.tip                       = 1.68
    wing.chords.mean_aerodynamic          = 2.34 
    wing.taper                            = wing.chords.tip/wing.chords.root 
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference 
    wing.twists.root                      = 0.0 * Units.degree
    wing.twists.tip                       = 0.0 * Units.degree 
    wing.origin                           = [[ 12.222 , 0 , 0.75 ]] 
    wing.aerodynamic_center               = [ 12.222 + 0.25 * wing.chords.root, 0 , 0.385 ]
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
    segment.sweeps.quarter_chord          = 23.73 * Units.deg
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # Wing Segments
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = 'tip'
    segment.percent_span_location         = 1.0   
    segment.root_chord_percent            = wing.taper
    segment.dihedral_outboard             = 0.  
    segment.sweeps.quarter_chord          = 0 * Units.deg
    segment.thickness_to_chord            = 0.1
    wing.append_segment(segment)

    # control surfaces -------------------------------------------
    rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                   = 'rudder'
    rudder.span_fraction_start   = 0.01
    rudder.span_fraction_end     = 1.0
    rudder.deflection            = 0.0  * Units.deg
    rudder.chord_fraction        = 0.44
    wing.append_control_surface(rudder)

    # add to vehicle
    vehicle.append_component(wing)

 
 
    # ##########################################################   Fuselage  ############################################################    
    fuselage = RCAIDE.Library.Components.Fuselages.Fuselage() 

    # define cabin
    cabin                                             = RCAIDE.Library.Components.Fuselages.Cabins.Cabin() 
    cabin.origin                                      = [[3.5,0, 0]]
    economy_class                                     = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
    economy_class.number_of_seats_abrest              = 3
    economy_class.seat_arm_rest_width                 = 0
    economy_class.number_of_rows                      = 6
    economy_class.aisle_width                         = 8  *  Units.inches   
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
    segment.height                              = 0.0
    segment.width                               = 0.0
    fuselage.segments.append(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1a'
    segment.percent_x_location                  = 0.00985
    segment.percent_z_location                  = 0 
    segment.height                              = 0.629
    segment.width                               = 0.56185
    fuselage.segments.append(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'
    segment.percent_x_location                  = 0.019706071
    segment.percent_z_location                  = 0.0	 
    segment.height                              = 0.8130
    segment.width                               = 0.7152
    fuselage.segments.append(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'
    segment.percent_x_location                  = 0.054892307
    segment.percent_z_location                  = 0.00152
    segment.height                              = 1.10
    segment.width                               = 1.11
    fuselage.segments.append(segment)  

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'
    segment.percent_x_location                  = 0.11688
    segment.percent_z_location                  = 0.0049
    segment.height                              = 1.47905
    segment.width                               = 1.5
    segment.curvature                           = 2.5
    fuselage.segments.append(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'
    segment.percent_x_location                  = 0.14226 
    segment.percent_z_location                  = 0.00582 
    segment.height                              = 1.6
    segment.width                               = 1.6
    segment.curvature                           = 3
    fuselage.segments.append(segment) 

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'
    segment.percent_x_location                  = 0.17164 
    segment.percent_z_location                  = 0.01737	 
    segment.height                              = 2.07562 
    segment.width                               = 1.72  
    segment.curvature                           = 3.5
    fuselage.segments.append(segment)
  

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'
    segment.percent_x_location                  = 0.19455   
    segment.percent_z_location                  = 0.01836 	 
    segment.height                              = 2.17	 
    segment.width                               = 1.75 
    segment.curvature                           = 4
    fuselage.segments.append(segment)

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'
    segment.percent_x_location                  =  0.54 
    segment.percent_z_location                  = 0.01977  
    segment.height                              = 2.09	 
    segment.width                               = 1.75 
    segment.curvature                           = 4
    fuselage.segments.append(segment)
    
    

    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'
    segment.percent_x_location                  = 0.98
    segment.percent_z_location                  = 0.03867	 	 
    segment.height                              = 0.36	 
    segment.width                               = 0.05 
    fuselage.segments.append(segment)
    
    # Segment
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'
    segment.percent_x_location                  = 1.0
    segment.percent_z_location                  = 0.03586	 
    segment.height                              = 0.0	 
    segment.width                               = 0.0 
    fuselage.segments.append(segment) 
    
    # add to vehicle
    vehicle.append_component(fuselage)
    

    # ########################################################  Energy Network  #########################################################  
    net                                         = RCAIDE.Framework.Networks.Fuel()    

    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                       = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  
 
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------    
    starboard_propulsor                              = RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop()    
    starboard_propulsor.tag                          = 'starboard_propulsor' 
    starboard_propulsor.origin                       = [[3.5,2.8129,1.22 ]]
    starboard_propulsor.design_altitude              = 10000*Units.ft                                   # [-]         Design Altitude
    starboard_propulsor.design_mach_number           = 0.27                                              # [-]         Design Mach number
    starboard_propulsor.design_thrust                = 3500.0 * Units.N                                  # [-]         Design Thrust 
    starboard_propulsor.working_fluid                = RCAIDE.Library.Attributes.Gases.Air()          
    starboard_propulsor.gearbox.efficiency           = 0.99   
    starboard_propulsor.design_power                 = 462334                                          # [-]         Design Gearbox Efficiency
    starboard_propulsor.specific_fuel_consumption_reduction_factor = -3.5

    #Propeller Design              
    propeller                                        = RCAIDE.Library.Components.Powertrain.Converters.Propeller()   
    propeller.tag                                    = 'starboard_propulsor_propeller' 
    propeller.origin                                 = [[3.75, 2.8129,1.22 ]]
    propeller.active                                 = True          
    propeller.tip_radius                             = 2.59/2
    propeller.hub_radius                             = 0.1 
    propeller.number_of_blades                       = 3   
    propeller.design_efficiency                      = 0.83      
    propeller.design_angular_velocity                = 2200.0 * Units.rpm        # https://paracleteaviation.com/wp-content/uploads/2017/10/Twin-Otter-Type-Data-Sheet.pdf
    propeller.design_thrust                          = starboard_propulsor.design_thrust              
    propeller.design_altitude                        = starboard_propulsor.design_altitude                                        
    propeller.design_freestream_velocity             = starboard_propulsor.design_freestream_velocity                                                               
    starboard_propulsor.propeller                    = propeller     
    

    # Ram inlet 
    ram                                              = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                          = 'ram' 
    starboard_propulsor.ram                          = ram 
          
    # inlet nozzle          
    inlet_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                                 = 'inlet nozzle'                                       
    inlet_nozzle.pressure_ratio                      = 0.98
    inlet_nozzle.compressibility_effects             = False
    starboard_propulsor.inlet_nozzle                 = inlet_nozzle
                                                     
    # compressor                        
    compressor                                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    compressor.tag                                   = 'lpc'                   
    compressor.pressure_ratio                        = 10                   
    starboard_propulsor.compressor                   = compressor
    
    # combustor      
    combustor                                        = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                    = 'Comb'
    combustor.efficiency                             = 0.99                   
    combustor.turbine_inlet_temperature              = 1370                    
    combustor.pressure_ratio                         = 0.96 
    combustor.air_mass_flow_rate_take_off            = 40             # [kg/s] Air mass flow rate at take-off
    combustor.fuel_to_air_ratio_take_off             = 0.025          # [-] Fuel to air ratio at take-off
    combustor.air_data                               = RCAIDE.Library.Attributes.Gases.Air()          # [-] Air object
    combustor.fuel_data                              = RCAIDE.Library.Attributes.Propellants.Jet_A1()       # [-] Fuel object  
    starboard_propulsor.combustor                    = combustor
        
    # high pressure turbine         
    high_pressure_turbine                            = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                        ='hpt'
    high_pressure_turbine.mechanical_efficiency      = 0.99                       
    starboard_propulsor.high_pressure_turbine        = high_pressure_turbine 
        
    # low pressure turbine      
    low_pressure_turbine                             = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                         ='lpt'
    low_pressure_turbine.mechanical_efficiency       = 0.99                      
    starboard_propulsor.low_pressure_turbine         = low_pressure_turbine
    
    # core nozzle    
    core_nozzle                                      = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                  = 'core nozzle'          
    core_nozzle.pressure_ratio                       = 0.99
    starboard_propulsor.core_nozzle                  = core_nozzle
    
    # design starboard_propulsor
    design_turboprop(starboard_propulsor)
    
    #########################################################   Nacelles  ############################################################    
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
 
    net.propulsors.append(starboard_propulsor) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Port Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------      
    # copy turboprop
    port_propulsor                                  = deepcopy(starboard_propulsor) 
    port_propulsor.tag                              = 'port_propulsor' 
    port_propulsor.origin                           = [[3.5, -2.8129,1.22 ]]  # change origin 
    port_propulsor.nacelle.tag                      = 'port_propulsor_nacelle' 
    port_propulsor.nacelle.origin                   = [[3.5, -2.8129,1.22 ]]
    port_propulsor.propeller.origin                 = [[3.75, -2.8129,1.22 ]]
         
    # append propulsor to distribution line 
    net.propulsors.append(port_propulsor) 

    # ------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    # ------------------------------------------------------------------------------------------------------------------------- 
    # fuel tank
    fuel_tank                                             = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank() 
    fuel_tank.origin                                      = vehicle.wings.main_wing.origin   
    fuel_tank.fuel                                        = RCAIDE.Library.Attributes.Propellants.Jet_A1()    
    fuel_tank.fuel.mass_properties.mass                   = 0  
    fuel_tank.fuel.mass_properties.center_of_gravity      = wing.mass_properties.center_of_gravity
    fuel_tank.internal_volume                             = fuel_tank.fuel.mass_properties.mass/fuel_tank.fuel.density   
    fuel_line.fuel_tanks.append(fuel_tank) 

    fuel_line.assigned_propulsors =  [[starboard_propulsor.tag, port_propulsor.tag]]

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to network      
    net.fuel_lines.append(fuel_line)        
    
    # Append energy network to aircraft 
    vehicle.append_energy_network(net)    
 
    
    return vehicle  
if __name__ == '__main__':
    main() 
