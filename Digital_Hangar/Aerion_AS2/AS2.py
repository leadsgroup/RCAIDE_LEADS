# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units   
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan             import design_turbofan
from RCAIDE.Library.Plots                 import *      
from RCAIDE.Library.Methods.Performance   import *  

# python imports 
import numpy as np   
from copy import deepcopy 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    
    # Step 1: design a vehicle
    vehicle  = vehicle_setup()  

    try:
        import vsp as vsp
        from RCAIDE.Framework.External_Interfaces.OpenVSP import export_vsp_vehicle 
        export_vsp_vehicle(vehicle, 'AS2')
    except ImportError:
        pass
       
    
    # Step 2: plot vehicle 
    plot_3d_vehicle(vehicle)  
    
    return  


def vehicle_setup():

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    

    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Aerion_AS2'    


    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    

    # mass properties
    vehicle.mass_properties.max_takeoff               = 121000 * Units.lb # 121000 for full mission, 110000 otherwise
    vehicle.mass_properties.operating_empty           = 57801  * Units.lb
    vehicle.mass_properties.takeoff                   = 1.0 * vehicle.mass_properties.max_takeoff
    vehicle.mass_properties.max_zero_fuel             = 0.7 * vehicle.mass_properties.max_takeoff # unknown
    vehicle.mass_properties.cargo                     = 0.  * Units.kilogram   

    vehicle.mass_properties.center_of_gravity         = [60 * Units.feet, 0, 0] # not accurate
    vehicle.mass_properties.moments_of_inertia.tensor = [[10 ** 5, 0, 0],[0, 10 ** 6, 0,],[0,0, 10 ** 7]] # not accurate

    # envelope properties
    vehicle.flight_envelope.ultimate_load = 2.5
    vehicle.flight_envelope.positive_limit_load    = 1.5

    # basic parameters
    vehicle.reference_area         = 125.
    vehicle.number_of_passengers             = 8
    vehicle.systems.control        = "fully powered" 
    vehicle.systems.accessories    = "long range"


    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------        

    wing = RCAIDE.Components.Wings.Main_Wing()
    wing.tag = 'main_wing'

    wing.aspect_ratio            = 20.4*20.4/125.
    wing.sweeps.quarter_chord    = 0 * Units.deg
    wing.thickness_to_chord      = 0.03
    wing.taper                   = 0.8
    wing.span_efficiency         = 0.9

    wing.spans.projected         = 20.4

    wing.chords.root             = 13.4 # 8.2 or 13.4
    wing.chords.tip              = 3.6 # 0 or 3.6
    wing.chords.mean_aerodynamic = 7.5
    
    wing.total_length            = 13.4

    wing.areas.reference         = 125. 

    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees

    wing.origin                  = [21.2,0,0] # 26.4 or 21.2
    wing.aerodynamic_center      = [27.0,0,0] 

    wing.vertical                = False
    wing.xz_plane_symmetric      = True
    wing.high_lift               = True
    wing.high_mach               = True
    wing.transition_x_upper      = 0.9
    wing.transition_x_lower      = 0.9

    wing.dynamic_pressure_ratio  = 1.0
    
    wing_airfoil = RCAIDE.Library.Components.Airfoils.Airfoil()
    wing_airfoil.coordinate_file = 'NACA65_203.dat' 
        
    #wing.append_airfoil(wing_airfoil)     
    
    segment = RCAIDE.Components.Wings.Segment()
    segment.tag                   = 'section_1'
    segment.percent_span_location = 0.
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 13.4/13.4
    segment.thickness_to_chord    = 0.03
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 54. * Units.deg
    #segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)    
    
    segment = RCAIDE.Components.Wings.Segment()
    segment.tag                   = 'section_2'
    segment.percent_span_location = 2.6/10.2 + wing.Segments['section_1'].percent_span_location
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 7.8/13.4
    segment.thickness_to_chord    = 0.03
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 11. * Units.deg
    #segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)   
    
    segment = RCAIDE.Components.Wings.Segment()
    segment.tag                   = 'section_3'
    segment.percent_span_location = 1 #6.8/10.2 + wing.Segments['section_2'].percent_span_location
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 4.1/13.2
    segment.thickness_to_chord    = 0.03
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 0. * Units.deg  # 73. * Units.deg
    #segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)       

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------        

    wing = RCAIDE.Components.Wings.Wing()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 10.4*10.4/40.
    wing.sweeps.quarter_chord    = -10. * Units.deg
    wing.thickness_to_chord      = 0.03
    wing.taper                   = 0.0 # probably wrong
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
        
    #wing.append_airfoil(wing_airfoil)     
    
    segment = RCAIDE.Components.Wings.Segment()
    segment.tag                   = 'section_1'
    segment.percent_span_location = 0.
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 6.2/6.2
    segment.thickness_to_chord    = 0.03
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 2.5 * Units.deg
    #segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)    
    
    segment = RCAIDE.Components.Wings.Segment()
    segment.tag                   = 'section_2'
    segment.percent_span_location = 4.6/5.2
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 2.3/6.2
    segment.thickness_to_chord    = 0.03
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 65. * Units.deg
    #segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment)       

    segment = RCAIDE.Components.Wings.Segment()
    segment.tag                   = 'tip'
    segment.percent_span_location = 1
    segment.twist                 = 0. * Units.deg
    segment.root_chord_percent    = 0.01
    segment.thickness_to_chord    = 0.03
    segment.dihedral_outboard     = 0.
    segment.sweeps.quarter_chord  = 0. * Units.deg
    #segment.append_airfoil(wing_airfoil)
    wing.append_segment(segment) 
    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = RCAIDE.Components.Wings.Wing()
    wing.tag = 'vertical_stabilizer'    

    wing.aspect_ratio            = 2.4*2.4/14.5
    wing.sweeps.quarter_chord    = 60. * Units.deg
    wing.thickness_to_chord      = 0.04
    wing.taper                   = 1. # probably wrong
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
        
    #wing.append_airfoil(tail_airfoil)     

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage = RCAIDE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage'

    fuselage.seats_abreast         = 2
    fuselage.seat_pitch            = 1

    fuselage.fineness.nose         = 6.7 # want .3348 or 17.34, base is 11.44
    fuselage.fineness.tail         = 6.3

    #fuselage.lengths.nose          = 6.4
    #fuselage.lengths.tail          = 8.0
    #fuselage.lengths.cabin         = 28.85
    fuselage.lengths.total         = 51.8
    #fuselage.lengths.fore_space    = 6.
    #fuselage.lengths.aft_space     = 5.

    fuselage.width                 = 2.6

    fuselage.heights.maximum       = 2.5
    fuselage.heights.at_quarter_length          = 2.5
    fuselage.heights.at_three_quarters_length   = 1.9
    fuselage.heights.at_wing_root_quarter_chord = 1.9

    fuselage.areas.side_projected  = 100.0
    fuselage.areas.wetted          = 300.0
    fuselage.areas.front_projected = 2.5*2.5*np.pi/4.

    fuselage.effective_diameter    = 3.74 #4.0

    fuselage.differential_pressure = 5.0e4 * Units.pascal # Maximum differential pressure
    
    fuselage.OpenVSP_values = Data() # VSP uses degrees directly
    
    fuselage.OpenVSP_values.nose = Data()
    fuselage.OpenVSP_values.nose.top = Data()
    fuselage.OpenVSP_values.nose.side = Data()
    fuselage.OpenVSP_values.nose.top.angle = 20.0
    fuselage.OpenVSP_values.nose.top.strength = 0.75
    fuselage.OpenVSP_values.nose.side.angle = 20.0
    fuselage.OpenVSP_values.nose.side.strength = 0.75  
    fuselage.OpenVSP_values.nose.TB_Sym = True
    fuselage.OpenVSP_values.nose.z_pos = -.01
    
    fuselage.OpenVSP_values.tail = Data()
    fuselage.OpenVSP_values.tail.top = Data()
    fuselage.OpenVSP_values.tail.side = Data()    
    fuselage.OpenVSP_values.tail.bottom = Data()
    fuselage.OpenVSP_values.tail.top.angle = 0.0
    fuselage.OpenVSP_values.tail.top.strength = 0.0    

    # add to vehicle
    vehicle.append_component(fuselage)


    # ------------------------------------------------------------------
    #   Turbojet Network
    # ------------------------------------------------------------------    

    #instantiate the gas turbine network
    turbofan = RCAIDE.Components.Energy.Networks.Turbofan()
    turbofan.tag = 'turbofan'

    # setup
    turbofan.number_of_engines = 2.0
    turbofan.bypass_ratio      = 4.0
    turbofan.engine_length     = 9.0
    turbofan.nacelle_diameter  = 1.4
    turbofan.inlet_diameter    = 1.3
    turbofan.areas             = Data()
    turbofan.areas.wetted      = 1.4*np.pi*7.0
    turbofan.origin            = [[35.,-2.3,1.3],[35.,2.3,1.3]]

    # working fluid
    turbofan.working_fluid = RCAIDE.Attributes.Gases.Air()


    # ------------------------------------------------------------------
    #   Component 1 - Ram

    # to convert freestream static to stagnation quantities

    # instantiate
    ram = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag = 'ram'

    # add to the network
    turbofan.append(ram)


    # ------------------------------------------------------------------
    #  Component 2 - Inlet Nozzle

    # instantiate
    inlet_nozzle = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag = 'inlet_nozzle'

    # setup
    inlet_nozzle.polytropic_efficiency = 0.98
    inlet_nozzle.pressure_ratio        = 1.0

    # add to network
    turbofan.append(inlet_nozzle)


    # ------------------------------------------------------------------
    #  Component 3 - Low Pressure Compressor

    # instantiate 
    compressor = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    compressor.tag = 'low_pressure_compressor'

    # setup
    compressor.polytropic_efficiency = 0.91
    compressor.pressure_ratio        = 3.1  

    # add to network
    turbofan.append(compressor)


    # ------------------------------------------------------------------
    #  Component 4 - High Pressure Compressor

    # instantiate
    compressor = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    compressor.tag = 'high_pressure_compressor'

    # setup
    compressor.polytropic_efficiency = 0.91
    compressor.pressure_ratio        = 5.0   

    # add to network
    turbofan.append(compressor)


    # ------------------------------------------------------------------
    #  Component 5 - Low Pressure Turbine

    # instantiate
    turbine = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    turbine.tag='low_pressure_turbine'

    # setup
    turbine.mechanical_efficiency = 0.99
    turbine.polytropic_efficiency = 0.93     

    # add to network
    turbofan.append(turbine)


    # ------------------------------------------------------------------
    #  Component 6 - High Pressure Turbine

    # instantiate
    turbine = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    turbine.tag='high_pressure_turbine'

    # setup
    turbine.mechanical_efficiency = 0.99
    turbine.polytropic_efficiency = 0.93     

    # add to network
    turbofan.append(turbine)


    # ------------------------------------------------------------------
    #  Component 7 - Combustor

    # instantiate    
    combustor = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag = 'combustor'

    # setup
    combustor.efficiency                = 0.99 
    combustor.alphac                    = 1.0     
    combustor.turbine_inlet_temperature = 1450.
    combustor.pressure_ratio            = 1.0
    combustor.fuel_data                 = RCAIDE.Attributes.Propellants.Jet_A()    

    # add to network
    turbofan.append(combustor)


    # ------------------------------------------------------------------
    #  Component 8 - Core Nozzle

    # instantiate
    nozzle = RCAIDE.Library.Components.Powertrain.Converters.Supersonic_Nozzle()   
    nozzle.tag = 'core_nozzle'

    # setup
    nozzle.polytropic_efficiency = 0.95
    nozzle.pressure_ratio        = 0.99    

    # add to network
    turbofan.append(nozzle)


    # ------------------------------------------------------------------
    #  Component 9 - Fan Nozzle

    # instantiate
    nozzle = RCAIDE.Library.Components.Powertrain.Converters.Supersonic_Nozzle()   
    nozzle.tag = 'fan_nozzle'

    # setup
    nozzle.polytropic_efficiency = 0.95
    nozzle.pressure_ratio        = 0.99    

    # add to network
    turbofan.append(nozzle)


    # ------------------------------------------------------------------
    #  Component 10 - Fan

    # instantiate
    fan = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag = 'fan'

    # setup
    fan.polytropic_efficiency = 0.93
    fan.pressure_ratio        = 1.7    

    # add to network
    turbofan.append(fan)


    # ------------------------------------------------------------------
    #Component 10 : thrust (to compute the thrust)
    thrust = RCAIDE.Components.Energy.Processes.Thrust()       
    thrust.tag ='compute_thrust'

    #total design thrust (includes all the engines)
    thrust.total_design             = 40000 * Units.lbf #Newtons

    # Note: Sizing builds the propulsor. It does not actually set the size of the turbojet
    #design sizing conditions
    altitude      = 0.0*Units.ft
    mach_number   = 0.01
    isa_deviation = 0.

    # add to network
    turbofan.thrust = thrust

    #size the turbojet
    turbofan_sizing(turbofan,mach_number,altitude)   

    # add  gas turbine network gt_engine to the vehicle
    vehicle.append_component(turbofan)      
    
    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------  return vehicle
    return vehicle
 

if __name__ == '__main__': 
    main()