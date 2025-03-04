# Solar_UAV.py 

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE 
from RCAIDE.Framework.Core import Units   
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor     import design_propeller
from RCAIDE.Library.Methods.Energy.Sources.Batteries.Common import size_module_from_mass

# python imports 
import numpy as np   
# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup(): 
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------    
    
    vehicle = RCAIDE.Vehicle()
    vehicle.tag = 'Solar'
    
    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------    
    # mass properties
    vehicle.mass_properties.takeoff                  = 250. * Units.kg
    vehicle.mass_properties.operating_empty          = 250. * Units.kg
    vehicle.mass_properties.max_takeoff              = 250. * Units.kg 
    
    # basic parameters
    vehicle.reference_area                           = 80.       
    vehicle.flight_envelope.ultimate_load            = 2.0
    vehicle.flight_envelope.positive_limit_load      = 1.5
    vehicle.flight_envelope.maximum_dynamic_pressure = 0.5*1.225*(40.**2.) #Max q

    # ------------------------------------------------------------------        
    #   Main Wing
    # ------------------------------------------------------------------   

    wing                         = RCAIDE.Library.Components.Wings.Wing()
    wing.tag                     = 'main_wing' 
    wing.areas.reference         = vehicle.reference_area
    wing.spans.projected         = 40.0 * Units.meter
    wing.aspect_ratio            = (wing.spans.projected**2)/wing.areas.reference 
    wing.sweeps.quarter_chord    = 0.0 * Units.deg
    wing.symmetric               = True
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 1.0
    wing.vertical                = False
    wing.high_lift               = True 
    wing.dynamic_pressure_ratio  = 1.0
    wing.chords.mean_aerodynamic = wing.areas.reference/wing.spans.projected
    wing.chords.root             = wing.areas.reference/wing.spans.projected
    wing.chords.tip              = wing.areas.reference/wing.spans.projected
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees
    wing.highlift                = False  
    wing.vertical                = False 
    wing.number_ribs             = 26.
    wing.number_end_ribs         = 2.
    wing.transition_x_upper      = 0.6
    wing.transition_x_lower      = 1.0
    wing.origin                  = [[3.0,0.0,0.0]] # meters
    wing.aerodynamic_center      = [3.0,0.0,0.0] # meters
    
    # add to vehicle
    vehicle.append_component(wing)
    
    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------   
    wing                         = RCAIDE.Library.Components.Wings.Wing()
    wing.tag                     = 'horizontal_stabilizer' 
    wing.aspect_ratio            = 20. 
    wing.sweeps.quarter_chord    = 0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 1.0
    wing.areas.reference         = vehicle.reference_area * .15
    wing.areas.wetted            = 2.0 * wing.areas.reference
    wing.areas.exposed           = 0.8 * wing.areas.wetted
    wing.areas.affected          = 0.6 * wing.areas.wetted       
    wing.spans.projected         = np.sqrt(wing.aspect_ratio*wing.areas.reference)
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees 
    wing.vertical                = False 
    wing.symmetric               = True
    wing.dynamic_pressure_ratio  = 0.9      
    wing.number_ribs             = 5.0
    wing.chords.root             = wing.areas.reference/wing.spans.projected
    wing.chords.tip              = wing.areas.reference/wing.spans.projected
    wing.chords.mean_aerodynamic = wing.areas.reference/wing.spans.projected  
    wing.origin                  = [[10.,0.0,0.0]] # meters
    wing.aerodynamic_center      = [0.5,0.0,0.0] # meters
  
    # add to vehicle
    vehicle.append_component(wing)    
    
    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------
    
    wing                         = RCAIDE.Library.Components.Wings.Wing()
    wing.tag                     = 'vertical_stabilizer' 
    wing.aspect_ratio            = 20.       
    wing.sweeps.quarter_chord    = 0 * Units.deg
    wing.thickness_to_chord      = 0.12
    wing.taper                   = 1.0
    wing.areas.reference         = vehicle.reference_area * 0.1
    wing.spans.projected         = np.sqrt(wing.aspect_ratio*wing.areas.reference) 
    wing.chords.root             = wing.areas.reference/wing.spans.projected
    wing.chords.tip              = wing.areas.reference/wing.spans.projected
    wing.chords.mean_aerodynamic = wing.areas.reference/wing.spans.projected 
    wing.areas.wetted            = 2.0 * wing.areas.reference
    wing.areas.exposed           = 0.8 * wing.areas.wetted
    wing.areas.affected          = 0.6 * wing.areas.wetted    
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees  
    wing.origin                  = [[10.,0.0,0.0]] # meters
    wing.aerodynamic_center      = [0.5,0.0,0.0] # meters
    wing.symmetric               = True  
    wing.vertical                = True 
    wing.t_tail                  = False
    wing.dynamic_pressure_ratio  = 1.0
    wing.number_ribs             = 5.
  
    # add to vehicle
    vehicle.append_component(wing)
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Electric Network
    #------------------------------------------------------------------------------------------------------------------------------------  
    #initialize the electric network
    net            = RCAIDE.Framework.Networks.Solar()  
 
    logic                 = RCAIDE.Library.Components.Powertrain.Modulators.Solar_Logic()
    logic.system_voltage  = 40.0
    logic.MPPT_efficiency = 0.95
    net.solar_logic       = logic 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus
    #------------------------------------------------------------------------------------------------------------------------------------  
    bus                              = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus() 

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Solar Panel
    #------------------------------------------------------------------------------------------------------------------------------------  
    panel                      = RCAIDE.Library.Components.Powertrain.Sources.Solar_Panels.Solar_Panel()
    panel.area                 = vehicle.reference_area * 0.9
    panel.efficiency           = 0.25
    panel.mass_properties.mass = panel.area*(0.60 * Units.kg)
    bus.solar_panel            = panel     

    #------------------------------------------------------------------------------------------------------------------------------------           
    # Battery
    #------------------------------------------------------------------------------------------------------------------------------------  
    bat                      = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC() 
    bat.tag                  = 'li_ion_battery' 
    bat.mass_properties.mass = 90.0 * Units.kg
    bat.cell.specific_energy = 600. * Units.Wh/Units.kg
    bat.cell.resistance      = 0.05
    bat.maximum_voltage      = 45.0
    size_module_from_mass(bat)  
    bat.voltage              = bat.maximum_voltage   
    bus.initialize_bus_properties()
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------   
    propulsor                                         = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()  
    propulsor.tag                                     = 'propulsor' 
             
    # Electronic Speed Controller                  
    esc                                               = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    esc.tag                                           = 'esc_1'
    esc.efficiency                                    = 0.95 
    propulsor.electronic_speed_controller             = esc   
     
    # Propeller              
    propeller                                        = RCAIDE.Library.Components.Powertrain.Converters.Propeller() 
    propeller.tag                                    = 'propeller_1' 
    propeller.tip_radius                             =  4.25 * Units.meters 
    propeller.number_of_blades                       = 2
    propeller.hub_radius                             = 0.05 * Units.meters
    propeller.cruise.design_freestream_velocity      = 40.0 * Units['m/s']
    propeller.cruise.design_angular_velocity         = 150. * Units['rpm']
    propeller.cruise.design_Cl                       = 0.7 
    propeller.cruise.design_altitude                 =  14.0 * Units.km
    propeller.cruise.design_thrust                   =  110.   
    design_propeller(propeller)    
    propulsor.rotor                                  = propeller   
              
    # DC_Motor       
    motor                                            = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor() 
    motor.resistance                                 = 0.008
    motor.no_load_current                            = 4.5  * Units.ampere
    motor.speed_constant                             = 120. * Units['rpm'] # RPM/volt converted to (rad/s)/volt    
    motor.rotor_radius                               = propeller.tip_radius
    motor.rotor_Cp                                   = propeller.cruise.design_power_coefficient
    motor.gear_ratio                                 = 12. # Gear ratio
    motor.gearbox_efficiency                         = .98 # Gear box efficiency
    motor.expected_current                           = 160. # Expected current
    motor.mass_properties.mass                       = 2.0  * Units.kg  
    propulsor.motor                                  = motor  

 
    # ##########################################################   Nacelles  ############################################################    
    nacelle              = RCAIDE.Library.Components.Nacelles.Nacelle()
    nacelle.tag          = 'nacelle_1'
    nacelle.diameter     = 0.2 * Units.meters
    nacelle.length       = 0.01 * Units.meters 
    nacelle.areas.wetted =  nacelle.length *(2*np.pi*nacelle.diameter/2.)
    
    propulsor.nacelle = nacelle
    
    # append propulsor to distribution line 
    net.propulsors.append(propulsor)
    
    #------------------------------------------------------------------------------------------------------------------------------------           
    # Payload 
    #------------------------------------------------------------------------------------------------------------------------------------  
    payload                      = RCAIDE.Library.Components.Payloads.Payload()
    payload.power_draw           = 50. # Watts
    payload.mass_properties.mass = 5.0 * Units.kg
    bus.payload                  = payload

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Avionics
    #------------------------------------------------------------------------------------------------------------------------------------  
    avionics                     = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw          = 50. # Watts
    bus.avionics                 = avionics   
 
    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to bus       
    bus.assigned_propulsors =  [[propulsor.tag]]
 
    # append bus   
    net.busses.append(bus)
    
    vehicle.append_energy_network(net) 
    
    return vehicle

# ----------------------------------------------------------------------
#   Define the Configurations
# --------------------------------------------------------------------- 

def configs_setup(vehicle):

    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------  
    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'  
    configs.append(base_config)   

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------ 
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cruise' 
    configs.append(config)  
    
    return configs
 