# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE

from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Visualization.Performance.Aerodynamics.Vehicle                 import *  
from RCAIDE.Visualization.Performance.Mission                              import *  
from RCAIDE.Visualization.Performance.Aerodynamics.Rotor import * 
from RCAIDE.Visualization.Performance.Energy.Battery                       import *   
from RCAIDE.Visualization.Geometry                                         import *

# python imports 
import numpy as np
import pylab as plt
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
        export_vsp_vehicle(vehicle, 'AS2.vsp')
    except ImportError:
        pass
       
    
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle)  
    
    return


def vehicle_setup():

    ospath      = os.path.abspath(__file__)
    separator   = os.path.sep
    airfoil_path    = os.path.dirname(ospath) + separator  + '..' + separator  
    local_path  = os.path.dirname(ospath) + separator       
        

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Aerion_AS2'    

    # ################################################# Vehicle-level Properties #################################################   
    vehicle.mass_properties.max_takeoff               = 121000 * Units.lb
    vehicle.mass_properties.operating_empty           = 57801  * Units.lb
    vehicle.mass_properties.takeoff                   = 1.0 * vehicle.mass_properties.max_takeoff
    vehicle.mass_properties.max_zero_fuel             = 0.7 * vehicle.mass_properties.max_takeoff
    vehicle.mass_properties.cargo                     = 0.  * Units.kilogram   
    vehicle.mass_properties.center_of_gravity         = [60 * Units.feet, 0, 0]
    vehicle.mass_properties.moments_of_inertia.tensor = [[10 ** 5, 0, 0],[0, 10 ** 6, 0,],[0,0, 10 ** 7]]
    vehicle.flight_envelope.ultimate_load             = 2.5
    vehicle.flight_envelope.positive_limit_load       = 1.5
    vehicle.reference_area                            = 125
    vehicle.number_of_passengers                      = 8
    vehicle.systems.control                           = "fully powered" 
    vehicle.systems.accessories                       = "long range"


    # ################################################# Wings #############################################################
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
    
    wing                          = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                      = 'main_wing'
    wing.aspect_ratio             = 20.4*20.4/125.
    wing.sweeps.quarter_chord     = 0 * Units.deg
    wing.thickness_to_chord       = 0.03
    wing.taper                    = 0.8
    wing.span_efficiency          = 0.9
    wing.spans.projected          = 20.4
    wing.chords.root              = 13.4  
    wing.chords.tip               = 3.6  
    wing.chords.mean_aerodynamic  = 7.5
    wing.total_length             = 13.4
    wing.areas.reference          = 125
    wing.twists.root              = 0.0 * Units.degrees
    wing.twists.tip               = 0.0 * Units.degrees
    wing.origin                   = [21.2,0,0]
    wing.aerodynamic_center       = [27.0,0,0] 
    wing.vertical                 = False
    wing.xz_plane_symmetric       = True
    wing.high_lift                = True
    wing.high_mach                = True
    wing.transition_x_upper       = 0.9
    wing.transition_x_lower       = 0.9
    wing.dynamic_pressure_ratio   = 1.0
    wing_airfoil                  = RCAIDE.Library.Components.Airfoils.Airfoil()   
    wing_airfoil.coordinate_file  =  airfoil_path + 'Airfoils' + separator + 'NACA65_203.txt'
    wing_airfoil.polar_files      = [airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA65_203_polar_Re_50000.txt' ,
                                                     airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA65_203_polar_Re_100000.txt' ,
                                                     airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA65_203_polar_Re_200000.txt' ,
                                                     airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA65_203_polar_Re_500000.txt' ,
                                                     airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA65_203_polar_Re_1000000.txt',
                                                     airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA65_203_polar_Re_3500000.txt',
                                                     airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA65_203_polar_Re_5000000.txt',
                                                     airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA65_203_polar_Re_7500000.txt' ]    

    # set section 1 start point
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'section_1'
    segment.percent_span_location = 0.
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 13.4/13.4
    segment.thickness_to_chord    = 0.03
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 54. * Units.deg
    wing.append_segment(segment)
    
    # set section 2 start point
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'section_2'
    segment.percent_span_location = 2.6/10.2 + wing.Segments['section_1'].percent_span_location
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 7.8/13.4
    segment.thickness_to_chord    = 0.03
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 11. * Units.deg
    wing.append_segment(segment)
    
    # set section 3 start point
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'section_3'
    segment.percent_span_location = 1 
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 4.1/13.2
    segment.thickness_to_chord    = 0.03
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 0. * Units.deg  
    wing.append_segment(segment)
    
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------
    
    wing = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 10.4*10.4/40.
    wing.sweeps.quarter_chord    = -10. * Units.deg
    wing.thickness_to_chord      = 0.03
    wing.taper                   = 0.0
    wing.span_efficiency         = 0.9
    wing.spans.projected         = 10.4
    wing.chords.root             = 6.2
    wing.chords.tip              = 0.0    
    wing.chords.mean_aerodynamic = 4.0
    wing.total_length            = 6.2
    wing.areas.reference         = 40.
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees  
    wing.origin                  = [45.3,0,3.1]
    wing.aerodynamic_center      = [48.5,0,3.1]
    wing.vertical                = False 
    wing.xz_plane_symmetric      = True
    wing.high_mach               = True
    wing.transition_x_upper      = 0.9
    wing.transition_x_lower      = 0.9
    wing.dynamic_pressure_ratio  = 0.9  
    
    wing_airfoil = RCAIDE.Library.Components.Airfoils.Airfoil()
    wing_airfoil.coordinate_file = 'supertail_refined.dat' 
        
    # set section 1
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'section_1'
    segment.percent_span_location = 0.
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 6.2/6.2
    segment.thickness_to_chord    = 0.03
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 2.5 * Units.deg
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)    
    
    # set section 2
    segment = RCAIDE.Library.Components.Wings.Segment()
    segment.tag                   = 'section_2'
    segment.percent_span_location = 4.6/5.2
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 2.3/6.2
    segment.thickness_to_chord    = 0.03
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 65. * Units.deg
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)
    
    # set tip 
    segment = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'tip'
    segment.percent_span_location = 1
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 0.01
    segment.thickness_to_chord    = 0.03
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 0. * Units.deg
    segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)
    
    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'    

    wing.aspect_ratio            = 2.4*2.4/14.5
    wing.sweeps.quarter_chord    = 60. * Units.deg
    wing.thickness_to_chord      = 0.04
    wing.taper                   = 1. 
    wing.span_efficiency         = 0.9
    wing.spans.projected         = 2.4
    wing.chords.root             = 7.5
    wing.chords.tip              = 5.4
    wing.chords.mean_aerodynamic = 6.7
    wing.total_length            = 9.9
    wing.areas.reference         = 14.5
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees  
    wing.origin                  = [41.,0.,0.7]
    wing.aerodynamic_center      = [46.,0,1.5]    
    wing.vertical                = True 
    wing.xz_plane_symmetric      = False
    wing.t_tail                  = True
    wing.high_mach               = True
    wing.transition_x_upper      = 0.8
    wing.transition_x_lower      = 0.8
    wing.dynamic_pressure_ratio  = 1.0
    
    tail_airfoil = RCAIDE.Library.Components.Airfoils.Airfoil()
    tail_airfoil.coordinate_file = 'supertail_refined.dat' 
        
    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage = RCAIDE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage'

    fuselage.seats_abreast                      = 2
    fuselage.seat_pitch                         = 1
    fuselage.fineness.nose                      = 6.7 
    fuselage.fineness.tail                      = 6.3
    fuselage.lengths.total                      = 51.8
    fuselage.width                              = 2.6
    fuselage.heights.maximum                    = 2.5
    fuselage.heights.at_quarter_length          = 2.5
    fuselage.heights.at_three_quarters_length   = 1.9
    fuselage.heights.at_wing_root_quarter_chord = 1.9
    fuselage.areas.side_projected               = 100.0
    fuselage.areas.wetted                       = 300.0
    fuselage.areas.front_projected              = 2.5*2.5*np.pi/4.
    fuselage.effective_diameter                 = 3.74
    fuselage.differential_pressure              = 5.0e4 * Units.pascal 
    
    fuselage.OpenVSP_values                     = Data()

    fuselage.OpenVSP_values.nose                = Data()
    fuselage.OpenVSP_values.nose.top            = Data()
    fuselage.OpenVSP_values.nose.side           = Data()
    fuselage.OpenVSP_values.nose.top.angle      = 20.0
    fuselage.OpenVSP_values.nose.top.strength   = 0.75
    fuselage.OpenVSP_values.nose.side.angle     = 20.0
    fuselage.OpenVSP_values.nose.side.strength  = 0.75  
    fuselage.OpenVSP_values.nose.TB_Sym         = True
    fuselage.OpenVSP_values.nose.z_pos          = -.01
    
    fuselage.OpenVSP_values.tail                = Data()
    fuselage.OpenVSP_values.tail.top            = Data()
    fuselage.OpenVSP_values.tail.side           = Data()    
    fuselage.OpenVSP_values.tail.bottom         = Data()
    fuselage.OpenVSP_values.tail.top.angle      = 0.0
    fuselage.OpenVSP_values.tail.top.strength   = 0.0    

    # add to vehicle
    vehicle.append_component(fuselage)


    # ------------------------------------------------------------------
    #   Turbojet Network
    # ------------------------------------------------------------------    

    turbofan = RCAIDE.Components.Energy.Networks.Turbofan()
    turbofan.tag = 'turbofan'

    turbofan.number_of_engines = 2.0
    turbofan.bypass_ratio      = 4.0
    turbofan.engine_length     = 9.0
    turbofan.nacelle_diameter  = 1.4
    turbofan.inlet_diameter    = 1.3
    turbofan.areas             = Data()
    turbofan.areas.wetted      = 1.4*np.pi*7.0
    turbofan.origin            = [[35.,-2.3,1.3],[35.,2.3,1.3]]

    turbofan.working_fluid = RCAIDE.Attributes.Gases.Air()


    # ------------------------------------------------------------------
    #   Component 1 - Ram

    ram     = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag = 'ram'

    turbofan.append(ram)


    # ------------------------------------------------------------------
    #  Component 2 - Inlet Nozzle

    inlet_nozzle                       = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                   = 'inlet_nozzle'
    inlet_nozzle.polytropic_efficiency = 0.98
    inlet_nozzle.pressure_ratio        = 1.0
    turbofan.append(inlet_nozzle)


    # ------------------------------------------------------------------
    #  Component 3 - Low Pressure Compressor

    compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    compressor.tag                   = 'low_pressure_compressor'
    compressor.polytropic_efficiency = 0.91
    compressor.pressure_ratio        = 3.1  
    turbofan.append(compressor)


    # ------------------------------------------------------------------
    #  Component 4 - High Pressure Compressor
    
    compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    compressor.tag                   = 'high_pressure_compressor'
    compressor.polytropic_efficiency = 0.91
    compressor.pressure_ratio        = 5.0   
    turbofan.append(compressor)


    # ------------------------------------------------------------------
    #  Component 5 - Low Pressure Turbine

    turbine                       = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    turbine.tag                   = 'low_pressure_turbine'
    turbine.mechanical_efficiency = 0.99
    turbine.polytropic_efficiency = 0.93     
    turbofan.append(turbine)


    # ------------------------------------------------------------------
    #  Component 6 - High Pressure Turbine

    turbine                       = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    turbine.tag                   = 'high_pressure_turbine'
    turbine.mechanical_efficiency = 0.99
    turbine.polytropic_efficiency = 0.93     
    turbofan.append(turbine)


    # ------------------------------------------------------------------
    #  Component 7 - Combustor

    combustor                           = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                       = 'combustor'
    combustor.efficiency                = 0.99 
    combustor.alphac                    = 1.0     
    combustor.turbine_inlet_temperature = 1450.
    combustor.pressure_ratio            = 1.0
    combustor.fuel_data                 = RCAIDE.Attributes.Propellants.Jet_A()    
    turbofan.append(combustor)


    # ------------------------------------------------------------------
    #  Component 8 - Core Nozzle

    nozzle                       = RCAIDE.Library.Components.Powertrain.Converters.Supersonic_Nozzle()   
    nozzle.tag                   = 'core_nozzle'
    nozzle.polytropic_efficiency = 0.95
    nozzle.pressure_ratio        = 0.99    
    turbofan.append(nozzle)


    # ------------------------------------------------------------------
    #  Component 9 - Fan Nozzle

    nozzle                       = RCAIDE.Library.Components.Powertrain.Converters.Supersonic_Nozzle()   
    nozzle.tag                   = 'fan_nozzle'
    nozzle.polytropic_efficiency = 0.95
    nozzle.pressure_ratio        = 0.99    
    turbofan.append(nozzle)


    # ------------------------------------------------------------------
    #  Component 10 - Fan

    fan                       = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                   = 'fan'
    fan.polytropic_efficiency = 0.93
    fan.pressure_ratio        = 1.7    
    turbofan.append(fan)


    # ------------------------------------------------------------------
    # Component 10 - Thrust Computation
    thrust                          = RCAIDE.Components.Energy.Processes.Thrust()       
    thrust.tag                      = 'compute_thrust'

    thrust.total_design             = 40000 * Units.lbf #Newtons
    altitude                        = 0.0*Units.ft
    mach_number                     = 0.01
    isa_deviation                   = 0.
    turbofan.thrust                 = thrust

    turbofan_sizing(turbofan,mach_number,altitude)   
    vehicle.append_component(turbofan)      
    
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------
    return vehicle
   # ----------------------------------------------------------------------
   #   Define the Configurations
   # ---------------------------------------------------------------------

def configs_setup(vehicle):
      
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs         = RCAIDE.Library.Components.Configs.Config.Container()

    base_config     = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config                          = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                      = 'cruise'

    configs.append(config)
    
    config.maximum_lift_coefficient = 1.2
    
    # ------------------------------------------------------------------
    #   Initial Configuration
    # ------------------------------------------------------------------    
    
    config     = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'initial'

    configs.append(config)

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config                                = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                            = 'takeoff'

    config.wings['main_wing'].flaps.angle = 20. * Units.deg
    config.wings['main_wing'].slats.angle = 25. * Units.deg

    config.V2_VS_ratio                    = 1.21
    config.maximum_lift_coefficient       = 2.

    configs.append(config)


    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    
    config                                = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                            = 'landing'

    config.wings['main_wing'].flaps_angle = 30. * Units.deg
    config.wings['main_wing'].slats_angle = 25. * Units.deg

    config.Vref_VS_ratio                  = 1.23
    config.maximum_lift_coefficient       = 2.

    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config                                = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                            = 'short_field_takeoff'
    
    config.wings['main_wing'].flaps.angle = 20. * Units.deg
    config.wings['main_wing'].slats.angle = 25. * Units.deg

    config.V2_VS_ratio                    = 1.21
    config.maximum_lift_coefficient       = 2. 
    

    
    configs.append(config)
    
    return configs


def simple_sizing(configs):

    base                               = configs.base
    base.pull_base()

    
    base.mass_properties.max_zero_fuel = 0.9 * base.mass_properties.max_takeoff 

    fuselage_width                     = base.fuselages['fuselage'].width
    fuselage_width                     = base.fuselages['fuselage'].width

    # wing areas
    for wing in base.wings:
        reference_root_chord           = wing.chords.root
        reference_tip_chord            = wing.chords.tip
        span                           = wing.spans.projected
        wing_root_chord                = (reference_tip_chord - reference_root_chord)/span * fuselage_width
        wing.areas.exposed             = 2*(wing.areas.reference - (reference_root_chord + wing_root_chord)*fuselage_width)
        
        if wing.thickness_to_chord < 0.05:
            wing.areas.wetted          = 2.003* wing.areas.exposed
        else:
            wing.areas.wetted          = (1.977 + 0.54*wing.thickness_to_chord )* wing.areas.exposed
            
        wing.areas.affected            = 0.6 * wing.areas.wetted
        

    base.fuselages['fuselage'].number_coach_seats = base.passengers


    base.store_diff()


    # done!
    return


if __name__ == '__main__': 
    
    main()