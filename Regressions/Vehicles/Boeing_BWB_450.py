# Boeing_BWB_450.py 
""" setup file for the BWB vehicle
"""
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units, Data       
from RCAIDE.Library.Methods.Geometry.Planform               import segment_properties    
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan_Propulsor   import design_turbofan   
from RCAIDE.Library.Plots                                   import *     
 
# python imports 
import numpy as np  
from copy import deepcopy 
import os

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------


def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    vehicle     = RCAIDE.Vehicle()
    vehicle.tag = 'Boeing_BWB_450'    

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.max_takeoff               = 823000. * Units.lb
    vehicle.mass_properties.takeoff                   = 823000. * Units.lb
    vehicle.mass_properties.max_zero_fuel             = 0.9 * vehicle.mass_properties.max_takeoff
    vehicle.mass_properties.cargo                     = 00.  * Units.kilogram   

    # envelope properties
    vehicle.flight_envelope.ultimate_load = 2.5
    vehicle.flight_envelope.limit_load    = 1.5
    vehicle.flight_envelope.design_mach_number = 0.8
    vehicle.flight_envelope.design_range = 8000 * Units.nmi

    # basic parameters
    vehicle.reference_area         = 7840. * 2 * Units.feet**2       
    vehicle.passengers             = 450.
    vehicle.systems.control        = "fully powered" 
    vehicle.systems.accessories    = "medium range"


    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        
    wing = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.aspect_ratio            = 289.**2 / (7840. * 2)
    wing.thickness_to_chord      = 0.15
    wing.taper                   = 0.0138
    wing.spans.projected         = 289.0 * Units.feet  
    wing.chords.root             = 145.0 * Units.feet
    wing.chords.tip              = 3.5  * Units.feet
    wing.chords.mean_aerodynamic = 80. * Units.feet
    wing.areas.reference         = 7840. * 2 * Units.feet**2
    wing.sweeps.quarter_chord    = 33. * Units.degrees
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees
    wing.dihedral                = 2.5 * Units.degrees
    wing.origin                  = [[0.,0.,0]]
    wing.aerodynamic_center      = [0,0,0] 
    wing.vertical                = False
    wing.symmetric               = True
    wing.high_lift               = True
    wing.dynamic_pressure_ratio  = 1.0

    segment = RCAIDE.Library.Components.Wings.Segments.Segment()

    segment.tag                   = 'section_1'
    segment.percent_span_location = 0.0
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 1.
    segment.dihedral_outboard     = 0. * Units.degrees
    segment.sweeps.quarter_chord  = 40.0 * Units.degrees
    segment.thickness_to_chord    = 0.165
    segment.vsp_mesh              = Data()
    segment.vsp_mesh.inner_radius    = 4.
    segment.vsp_mesh.outer_radius    = 4.
    segment.vsp_mesh.inner_length    = .14
    segment.vsp_mesh.outer_length    = .14   
    wing.append_segment(segment)
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                      = 'section_2'
    segment.percent_span_location    = 0.052
    segment.twist                    = 0. * Units.deg
    segment.root_chord_percent       = 0.921
    segment.dihedral_outboard        = 0.   * Units.degrees
    segment.sweeps.quarter_chord     = 52.5 * Units.degrees
    segment.thickness_to_chord       = 0.167
    segment.vsp_mesh                 = Data()
    segment.vsp_mesh.inner_radius    = 4.
    segment.vsp_mesh.outer_radius    = 4.
    segment.vsp_mesh.inner_length    = .14
    segment.vsp_mesh.outer_length    = .14
    wing.append_segment(segment)

    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                      = 'section_3'
    segment.percent_span_location    = 0.138
    segment.twist                    = 0. * Units.deg
    segment.root_chord_percent       = 0.76
    segment.dihedral_outboard        = 1.85 * Units.degrees
    segment.sweeps.quarter_chord     = 36.9 * Units.degrees  
    segment.thickness_to_chord       = 0.171
    segment.vsp_mesh                 = Data()
    segment.vsp_mesh.inner_radius    = 4.
    segment.vsp_mesh.outer_radius    = 4.
    segment.vsp_mesh.inner_length    = .14
    segment.vsp_mesh.outer_length    = .14   
    wing.append_segment(segment)
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                      = 'section_4'
    segment.percent_span_location    = 0.221
    segment.twist                    = 0. * Units.deg
    segment.root_chord_percent       = 0.624
    segment.dihedral_outboard        = 1.85 * Units.degrees
    segment.sweeps.quarter_chord     = 30.4 * Units.degrees    
    segment.thickness_to_chord       = 0.175
    segment.vsp_mesh                 = Data()
    segment.vsp_mesh.inner_radius    = 4.
    segment.vsp_mesh.outer_radius    = 2.8
    segment.vsp_mesh.inner_length    = .14
    segment.vsp_mesh.outer_length    = .14 
    wing.append_segment(segment)
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'section_5'
    segment.percent_span_location = 0.457
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 0.313
    segment.dihedral_outboard     = 1.85  * Units.degrees
    segment.sweeps.quarter_chord  = 30.85 * Units.degrees
    segment.thickness_to_chord    = 0.118
    wing.append_segment(segment)
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'section_6'
    segment.percent_span_location = 0.568
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 0.197
    segment.dihedral_outboard     = 1.85 * Units.degrees
    segment.sweeps.quarter_chord  = 34.3 * Units.degrees
    segment.thickness_to_chord    = 0.10
    wing.append_segment(segment)
    
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'section_7'
    segment.percent_span_location = 0.97
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 0.086
    segment.dihedral_outboard     = 73. * Units.degrees
    segment.sweeps.quarter_chord  = 55. * Units.degrees
    segment.thickness_to_chord    = 0.10
    wing.append_segment(segment)

    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'tip'
    segment.percent_span_location = 1
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 0.0241
    segment.dihedral_outboard     = 0. * Units.degrees
    segment.sweeps.quarter_chord  = 0. * Units.degrees
    segment.thickness_to_chord    = 0.10
    wing.append_segment(segment)
    
    # Fill out more segment properties automatically
    wing = segment_properties(wing)        


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
    flap.span_fraction_start      = 0.2
    flap.span_fraction_end        = 0.7
    flap.deflection               = 0.0 * Units.degrees
    flap.configuration_type       = 'double_slotted'
    flap.chord_fraction           = 0.30
    wing.append_control_surface(flap)

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.7
    aileron.span_fraction_end     = 0.963
    aileron.deflection            = 30.0 * Units.degrees
    aileron.chord_fraction        = 0.16
    wing.append_control_surface(aileron)


    # add to vehicle
    vehicle.append_component(wing)

    #------------------------------------------------------------------------------------------------------------------------- 
    #  Turbofan Network
    #-------------------------------------------------------------------------------------------------------------------------     
    fuselage = RCAIDE.Library.Components.Fuselages.Blended_Wing_Body_Fuselage() 
    fuselage.aft_centerbody_area   = 1350
    fuselage.aft_centerbody_taper  = 0.5
    fuselage.cabin_area            = 15*450 
    fuselage.number_coach_seats    = vehicle.passengers 
    fuselage.seats_abreast         = 15
    fuselage.seat_pitch            = 1 * Units.meter
    fuselage.lengths.total         = 100
    fuselage.width                 = 15
    
    # add to vehicle
    vehicle.append_component(fuselage)    
    
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
    turbofan                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() 
    turbofan.tag                                = 'center_propulsor' 
    turbofan.origin                             = [[120.0 *Units.feet, 0.0*Units.feet, 6.5*Units.feet]] 
    turbofan.engine_length                      = 2.71     
    turbofan.bypass_ratio                       = 8.4
    turbofan.design_altitude                    = 0. * Units.km
    turbofan.design_mach_number                 = 0.01
    turbofan.design_thrust                      = 2.0*512000/3 * Units.N
             
    # fan                
    fan                                         = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                     = 'fan'
    fan.polytropic_efficiency                   = 0.93
    fan.pressure_ratio                          = 1.58
    turbofan.fan                                = fan        
                   
    # working fluid                   
    turbofan.working_fluid                      = RCAIDE.Library.Attributes.Gases.Air() 
    ram                                         = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                     = 'ram' 
    turbofan.ram                                = ram 
          
    # inlet nozzle          
    inlet_nozzle                                = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                            = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency          = 1.0
    inlet_nozzle.pressure_ratio                 = 1.0
    turbofan.inlet_nozzle                       = inlet_nozzle 

    # low pressure compressor    
    low_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                   = 'lpc'
    low_pressure_compressor.polytropic_efficiency = 0.91
    low_pressure_compressor.pressure_ratio        = 1.1
    turbofan.low_pressure_compressor              = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.91
    high_pressure_compressor.pressure_ratio        = 23.0
    turbofan.high_pressure_compressor              = high_pressure_compressor

    # low pressure turbine  
    low_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                       ='lpt'
    low_pressure_turbine.mechanical_efficiency     = 0.99
    low_pressure_turbine.polytropic_efficiency     = 0.93 
    turbofan.low_pressure_turbine                  = low_pressure_turbine
   
    # high pressure turbine     
    high_pressure_turbine                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                      ='hpt'
    high_pressure_turbine.mechanical_efficiency    = 0.99
    high_pressure_turbine.polytropic_efficiency    = 0.93 
    turbofan.high_pressure_turbine                 = high_pressure_turbine 

    # combustor  
    combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                  = 'combustor'
    combustor.efficiency                           = 1.0
    combustor.alphac                               = 1.0     
    combustor.turbine_inlet_temperature            = 1592. * Units.kelvin
    combustor.pressure_ratio                       = 0.95
    combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A1()  
    turbofan.combustor                             = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.95
    core_nozzle.pressure_ratio                     = 0.99  
    turbofan.core_nozzle                           = core_nozzle
             
    # fan nozzle             
    fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                 = 'fan nozzle'
    fan_nozzle.polytropic_efficiency               = 0.95
    fan_nozzle.pressure_ratio                      = 0.99 
    turbofan.fan_nozzle                            = fan_nozzle 
    
    # design turbofan
    design_turbofan(turbofan)  

    # Nacelle 
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                            = 3.96 * Units.meters 
    nacelle.length                              = 289. * Units.inches
    nacelle.tag                                 = 'nacelle_1'
    nacelle.inlet_diameter                      = 2.0
    nacelle.origin                              = [[120.0 *Units.feet, 0.0*Units.feet, 6.5*Units.feet]] 
    nacelle.areas.wetted                        = 1.1*np.pi*nacelle.diameter*nacelle.length 
    nacelle_airfoil                             = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code          = '2410'
    nacelle.append_airfoil(nacelle_airfoil)  
    turbofan.nacelle                            = nacelle 
    net.propulsors.append(turbofan) 

    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Inner Right Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    right_turbofan                     = deepcopy(turbofan) 
    right_turbofan.tag                 = 'right_propulsor' 
    right_turbofan.origin              = [[120.0 *Units.feet, 25.0*Units.feet, 6.5*Units.feet]] 
    right_nacelle                      = deepcopy(nacelle)
    right_nacelle.tag                  = 'right_nacelle'
    right_nacelle.origin               = [[120.0 *Units.feet, 25.0*Units.feet, 6.5*Units.feet]]
    right_turbofan.nacelle = right_nacelle
    net.propulsors.append(right_turbofan) 
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Inner Right Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------    
    left_turbofan                    = deepcopy(turbofan) 
    left_turbofan.tag                 = 'left_propulsor' 
    left_turbofan.origin              = [[120.0 *Units.feet, -25.0*Units.feet, 6.5*Units.feet]]  
    left_nacelle                      = deepcopy(nacelle)
    left_nacelle.tag                  = 'left_nacelle'
    left_nacelle.origin               = [[120.0 *Units.feet, -25.0*Units.feet, 6.5*Units.feet]]
    left_turbofan.nacelle = left_nacelle
    net.propulsors.append(left_turbofan)
       
    #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #------------------------------------------------------------------------------------------------------------------------- 
    # fuel tank
    fuel_tank                                   = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank()  
    fuel_tank.fuel                              = RCAIDE.Library.Attributes.Propellants.Jet_A1()   
    fuel_tank.fuel.mass_properties.mass         = vehicle.mass_properties.max_takeoff-vehicle.mass_properties.max_fuel
    fuel_tank.origin                            = vehicle.wings.main_wing.mass_properties.center_of_gravity      
    fuel_tank.mass_properties.center_of_gravity = vehicle.wings.main_wing.aerodynamic_center
    fuel_tank.volume                            = fuel_tank.fuel.mass_properties.mass/fuel_tank.fuel.density   
    
    # apend fuel tank to dataclass of fuel tanks on fuel line 
    fuel_line.fuel_tanks.append(fuel_tank)

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to bus       
    fuel_line.assigned_propulsors =  [[turbofan.tag, right_turbofan.tag, left_turbofan.tag]]
    

    # Append fuel line to Network      
    net.fuel_lines.append(fuel_line)   

    # Append energy network to aircraft 
    vehicle.append_energy_network(net)   

    return vehicle


# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'  
    configs.append(base_config)
 
    return configs
