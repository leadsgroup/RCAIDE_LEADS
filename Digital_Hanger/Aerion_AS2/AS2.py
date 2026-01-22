# AS2.py
# 
# Created: Dec 2017, M. Clarke

""" setup file for a mission with a concorde
"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
##import vsp 
#from RCAIDE.Input_Output.OpenVSP.vsp_write import write
#from RCAIDE.Input_Output.OpenVSP.get_vsp_areas import get_vsp_areas 

from RCAIDE.Core import Units, Data 
from RCAIDE.Methods.Propulsion.turbofan_sizing import turbofan_sizing 
from RCAIDE.Visualization.Performance.Aerodynamics.Vehicle                 import *  
from RCAIDE.Visualization.Performance.Mission                              import *  
from RCAIDE.Visualization.Performance.Aerodynamics.Rotor import *  
from RCAIDE.Visualization.Performance.Energy.Battery                       import *   
from RCAIDE.Visualization.Performance.Noise                                import *  
from RCAIDE.Visualization.Geometry.Three_Dimensional.plot_3d_vehicle       import plot_3d_vehicle 
from RCAIDE.Visualization.Geometry                                         import *

import numpy as np
import pylab as plt

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():

    configs, analyses = full_setup()

    simple_sizing(configs)

    configs.finalize()
    analyses.finalize()

    # weight analysis
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate()      

    # mission analysis
    mission = analyses.missions.base
    results = mission.evaluate()
    
    #save_results(results)
    
    plot_mission(results)
    
    plt.show()
    
    return

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup():

    # vehicle data
    vehicle  = vehicle_setup()
    #write(vehicle, "AS2")
    configs  = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(configs_analyses)
    missions_analyses = missions_setup(mission)

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()
    analyses.configs  = configs_analyses
    analyses.missions = missions_analyses
    
    return configs, analyses

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = RCAIDE.Framework.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Weights_Transport()
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Supersonic_Zero()    
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    
    # Not yet available for this configuration

    # ------------------------------------------------------------------
    #  Energy
    energy= RCAIDE.Framework.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors #what is called throughout the mission (at every time step))
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    # done!
    return analyses    

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
# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
      
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs = RCAIDE.Library.Components.Configs.Config.Container()

    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cruise'

    configs.append(config)
    
    config.maximum_lift_coefficient = 1.2
    
    # ------------------------------------------------------------------
    #   Initial Configuration
    # ------------------------------------------------------------------    

    # This configuration is not updated as the optimization progresses
    # It is used to determine new wing dimensions
    
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'initial'

    configs.append(config)

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'takeoff'

    config.wings['main_wing'].flaps.angle = 20. * Units.deg
    config.wings['main_wing'].slats.angle = 25. * Units.deg

    config.V2_VS_ratio = 1.21
    config.maximum_lift_coefficient = 2.

    configs.append(config)


    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'

    config.wings['main_wing'].flaps_angle = 30. * Units.deg
    config.wings['main_wing'].slats_angle = 25. * Units.deg

    config.Vref_VS_ratio = 1.23
    config.maximum_lift_coefficient = 2.

    configs.append(config)
    
    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'
    
    config.wings['main_wing'].flaps.angle = 20. * Units.deg
    config.wings['main_wing'].slats.angle = 25. * Units.deg

    config.V2_VS_ratio = 1.21
    config.maximum_lift_coefficient = 2. 
    
    # payload?
    
    configs.append(config)
    
    return configs

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results,line_style='bo-'):

    axis_font = {'fontname':'Arial', 'size':'14'}    

    # ------------------------------------------------------------------
    #   Propulsion
    # ------------------------------------------------------------------


    fig = plt.figure("Propulsion",figsize=(8,6))
    for segment in results.segments.values():

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        Thrust = segment.conditions.frames.body.thrust_force_vector[:,0] /Units.lbf
        eta  = segment.conditions.propulsion.throttle[:,0]

        axes = fig.add_subplot(2,1,1)
        axes.plot( time , Thrust , line_style )
        axes.set_ylabel('Thrust (lbf)',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(2,1,2)
        axes.plot( time , eta , line_style )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('eta (lb/lbf-hr)',axis_font)
        axes.grid(True)	

    # ------------------------------------------------------------------
    #   Aerodynamics
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Coefficients",figsize=(8,10))
    for segment in results.segments.values():

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        CLift  = segment.conditions.aerodynamics.lift_coefficient[:,0]
        CDrag  = segment.conditions.aerodynamics.drag_coefficient[:,0]
        aoa = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        l_d = CLift/CDrag

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , CLift , line_style )
        axes.set_ylabel('Lift Coefficient',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , l_d , line_style )
        axes.set_ylabel('L/D',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,3)
        axes.plot( time , aoa , 'ro-' )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('AOA (deg)',axis_font)
        axes.grid(True)

    # ------------------------------------------------------------------
    #   Drag
    # ------------------------------------------------------------------
    fig = plt.figure("Drag Components",figsize=(8,10))
    axes = plt.gca()
    for i, segment in enumerate(results.segments.values()):

        time   = segment.conditions.frames.inertial.time[:,0] / Units.min
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
        cdp = drag_breakdown.parasite.total[:,0]
        cdi = drag_breakdown.induced.total[:,0]
        cdc = drag_breakdown.compressible.total[:,0]
        cdm = drag_breakdown.miscellaneous.total[:,0]
        cd  = drag_breakdown.total[:,0]

        if line_style == 'bo-':
            axes.plot( time , cdp , 'ko-', label='CD parasite' )
            axes.plot( time , cdi , 'bo-', label='CD induced' )
            axes.plot( time , cdc , 'go-', label='CD compressibility' )
            axes.plot( time , cdm , 'yo-', label='CD miscellaneous' )
            axes.plot( time , cd  , 'ro-', label='CD total'   )
            if i == 0:
                axes.legend(loc='upper center')            
        else:
            axes.plot( time , cdp , line_style )
            axes.plot( time , cdi , line_style )
            axes.plot( time , cdc , line_style )
            axes.plot( time , cdm , line_style )
            axes.plot( time , cd  , line_style )            

    axes.set_xlabel('Time (min)')
    axes.set_ylabel('CD')
    axes.grid(True)

    # ------------------------------------------------------------------
    #   Altitude, Vehicle Weight, Mach Number
    # ------------------------------------------------------------------

    fig = plt.figure("Altitude_sfc_weight",figsize=(8,10))
    for segment in results.segments.values():

        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        mass     = segment.conditions.weights.total_mass[:,0] / Units.lb
        altitude = segment.conditions.freestream.altitude[:,0] / Units.feet
        mach     = segment.conditions.freestream.mach_number[:,0]

        axes = fig.add_subplot(3,1,1)
        axes.plot( time , altitude , line_style )
        axes.set_ylabel('Altitude (ft)',axis_font)
        axes.grid(True)

        axes = fig.add_subplot(3,1,2)
        axes.plot( time , mass , 'ro-' )
        axes.set_ylabel('Weight (lb)',axis_font)
        axes.grid(True)
        
        axes = fig.add_subplot(3,1,3)
        axes.plot( time , mach , line_style )
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Mach Number',axis_font)
        axes.grid(True)        

    return

def simple_sizing(configs):

    base = configs.base
    base.pull_base()

    # zero fuel weight
    base.mass_properties.max_zero_fuel = 0.9 * base.mass_properties.max_takeoff 

    fuselage_width = base.fuselages['fuselage'].width

    # wing areas
    #for wing in base.wings:
        #wing.areas.wetted   = 2.0 * wing.areas.reference
        #wing.areas.exposed  = 0.8 * wing.areas.wetted
        #wing.areas.affected = 0.6 * wing.areas.wetted
        
    fuselage_width = base.fuselages['fuselage'].width

    # wing areas
    for wing in base.wings:
        reference_root_chord = wing.chords.root
        reference_tip_chord = wing.chords.tip
        span = wing.spans.projected
        wing_root_chord = (reference_tip_chord - reference_root_chord)/span * fuselage_width
        wing.areas.exposed = 2*(wing.areas.reference - (reference_root_chord + wing_root_chord)*fuselage_width)
        
        if wing.thickness_to_chord < 0.05:
            wing.areas.wetted  = 2.003* wing.areas.exposed
        else:
            wing.areas.wetted  = (1.977 + 0.54*wing.thickness_to_chord )* wing.areas.exposed
            
        wing.areas.affected = 0.6 * wing.areas.wetted
        
    # fuselage seats
    base.fuselages['fuselage'].number_coach_seats = base.passengers

    # diff the new data
    base.store_diff()


    # done!
    return

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses):
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Analyses.Mission.Sequential_Segments()
    mission.tag = 'baseline_mission'

    #airport
    airport = RCAIDE.Attributes.Airports.Airport()
    airport.altitude   =  0.0  * Units.ft
    airport.delta_isa  =  0.0
    airport.atmosphere = RCAIDE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport    

    # unpack Segments module
    Segments = RCAIDE.Framework.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()

    #--------------------------------------------------------------------
    # S-512 4000 nm Mission Profile
    #--------------------------------------------------------------------

    # ------------------------------------------------------------------
    #   First Climb Segment
    # ------------------------------------------------------------------

    segment = Segments.Climb.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "climb_1"

    segment.analyses.extend( analyses.base )

    ones_row = segment.state.ones_row
    segment.state.unknowns.body_angle = ones_row(1) * 5.56 * Units.deg   

    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 10.668   * Units.km
    segment.mach_start     = 0.227
    segment.mach_end       = 0.8
    segment.climb_rate     = 15.25 * Units['m/s'] 

    # add to misison
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Climb Segment
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "climb_2"

    segment.analyses.extend( analyses.base )
    
    segment.altitude_start = 10.668 * Units.km
    segment.altitude_end   = 15.24   * Units.km
    segment.mach_start     = 0.8
    segment.mach_end       = 1.4
    segment.climb_rate     = 10.16  * Units['m/s']   

    # add to mission
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------    
    #   Cruise Segment
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.analyses.extend( analyses.base )
    segment.mach       = 1.4
    segment.distance   = 6850.22 * Units.km   

    mission.append_segment(segment)

    # ------------------------------------------------------------------    
    #   First Descent Segment
    # ------------------------------------------------------------------    

    segment = Segments.Descent.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "descent_1"

    segment.analyses.extend( analyses.base )
    segment.altitude_start = 15.24   * Units.km
    segment.altitude_end = 1.524   * Units.km
    segment.mach_start   = 1.4
    segment.mach_end     = 0.385 # purposely changed it from 0.308 so I can easily uncomment section
    segment.descent_rate = 15.044   * Units['m/s']   

    # add to mission
    mission.append_segment(segment)
  

    ## ------------------------------------------------------------------    
    ##   Loiter Segment
    ## ------------------------------------------------------------------    

    #segment = Segments.Cruise.Constant_Mach_Constant_Altitude_Loiter(base_segment)
    #segment.tag = "loiter"

    #segment.analyses.extend(analyses.base)
    #segment.mach       = 0.308 # purposely changed it from 0.308 so I can easily uncomment section
    #segment.time       = 1.0* Units.hour  

    ## add to mission
    #mission.append_segment(segment)    

   
    ## ------------------------------------------------------------------    
    ##   Alternate Airport Segment
    ## ------------------------------------------------------------------    

    #segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    #segment.tag = "alt_airport"

    #segment.analyses.extend( analyses.base )
    #segment.mach       = 0.385
    #segment.distance   = 187.5 * Units.km  

    ## add to mission
    #mission.append_segment(segment)  
    
    
    # ------------------------------------------------------------------    
    #   Second Descent Segment
    # ------------------------------------------------------------------    

    segment = Segments.Descent.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "descent_2"

    segment.analyses.extend( analyses.base )
    segment.altitude_start = 1.524   * Units.km
    segment.altitude_end = 0.0   * Units.km
    segment.mach_start   = 0.385 
    segment.mach_end     = 0.0
    segment.descent_rate = 3.365   * Units['m/s']   

    # append to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------    
    #   Mission definition complete    
    # ------------------------------------------------------------------

    return mission

def missions_setup(base_mission):

    # the mission container
    missions = RCAIDE.Framework.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission

    # done!
    return missions  

def save_results(results):

    seg_count  = len(results.segments)
    result_mat = np.zeros((9*16,10))
    
    j = 0
    for segment in results.segments.values():    
        for i in xrange(16):
            time   = segment.conditions.frames.inertial.time[i,0] / Units.min
            CLift  = segment.conditions.aerodynamics.lift_coefficient[i,0]
            CDrag  = segment.conditions.aerodynamics.drag_coefficient[i,0]
            AoA    = segment.conditions.aerodynamics.angle_of_attack[i,0] / Units.deg
            l_d = CLift/CDrag
        
            drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
            cdp = drag_breakdown.parasite.total[i,0]
            cdi = drag_breakdown.induced.total[i,0]
            cdc = drag_breakdown.compressible.total[i,0]   
            cd  = drag_breakdown.total[i,0]
        
            mdot   = segment.conditions.weights.vehicle_mass_rate[i,0]
            thrust =  segment.conditions.frames.body.thrust_force_vector[i,0]
            sfc    = 3600. * mdot / 0.1019715 / thrust
        
            result_mat[j] = np.array([time,CLift,CDrag,l_d,cdp,cdi,cdc,cd,sfc,AoA])
            j += 1
        
    np.save('Concorde_new.npy',result_mat)
        
    return

if __name__ == '__main__': 
    
    main()