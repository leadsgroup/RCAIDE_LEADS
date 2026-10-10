# AVL_test.py
# 
# Created:  Dec 2023, M. Clarke 

""" setup file for segment test regression with a Boeing 737"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units ,  Data
from RCAIDE.Library.Plots             import *       

# python imports 
import numpy as np
import pylab as plt 
import sys
import os
import time
 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    ti = time.time()
    
 
    # materials 
    material            = RCAIDE.Library.Attributes.Materials.Acrylic()    
    material            = RCAIDE.Library.Attributes.Materials.Magnesium()  
    material            = RCAIDE.Library.Attributes.Materials.Titanium()   
    material            = RCAIDE.Library.Attributes.Materials.Aluminum()    
    material            = RCAIDE.Library.Attributes.Materials.Aluminum_Alloy()  
    material            = RCAIDE.Library.Attributes.Materials.Perfluoroalkoxy()  
    material            = RCAIDE.Library.Attributes.Materials.Polyetherimide()  
    material            = RCAIDE.Library.Attributes.Materials.Polyimide()  
    material            = RCAIDE.Library.Attributes.Materials.Polytetrafluoroethylene()
    material            = RCAIDE.Library.Attributes.Materials.CrossLinked_Polyethylene()
    material            = RCAIDE.Library.Attributes.Materials.Stainless_Steel_304()
    material            = RCAIDE.Library.Attributes.Materials.Aerogel()
    material            = RCAIDE.Library.Attributes.Materials.Polyurethane_Foam()
    material            = RCAIDE.Library.Attributes.Materials.Vacuum_Jacketed_Multilayer_Insulation()
    material            = RCAIDE.Library.Attributes.Materials.Cycom_5320()
    

      
    # gases 
    working_fluid                       = RCAIDE.Library.Attributes.Gases.CO2()        
    working_fluid                       = RCAIDE.Library.Attributes.Gases.Steam()      
   
    # cryogens
    cryogens =  RCAIDE.Library.Attributes.Cryogens.Cryogen()  
    cryogens =  RCAIDE.Library.Attributes.Cryogens.Liquid_Hydrogen()  
    
    # propellants 
    propellant  = RCAIDE.Library.Attributes.Propellants.Aviation_Gasoline() 
    propellant  = RCAIDE.Library.Attributes.Propellants.Ethane() 
    propellant  = RCAIDE.Library.Attributes.Propellants.Ethanol() 
    propellant  = RCAIDE.Library.Attributes.Propellants.Propanol() 
    propellant  = RCAIDE.Library.Attributes.Propellants.Methane() 
    propellant  = RCAIDE.Library.Attributes.Propellants.Propane()
    propellant  = RCAIDE.Library.Attributes.Propellants.Gaseous_Hydrogen()
    propellant  = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    propellant  = RCAIDE.Library.Attributes.Propellants.Alcohol_Mixture()
    propellant  = RCAIDE.Library.Attributes.Propellants.Alkane_Mixture()
    propellant  = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()
    propellant  = RCAIDE.Library.Attributes.Propellants.Butanol()
    propellant  = RCAIDE.Library.Attributes.Propellants.Liquid_Petroleum_Gas()
    propellant  = RCAIDE.Library.Attributes.Propellants.Jet_A1()    
    propellant  = RCAIDE.Library.Attributes.Propellants.JP7()

    # networks
    network =  RCAIDE.Framework.Networks.Hydrogen()

    # powertrain base classes
    distributor        = RCAIDE.Library.Components.Powertrain.Distributors.Distributor()
    modulator           = RCAIDE.Library.Components.Powertrain.Modulators.Modulator()
    source              = RCAIDE.Library.Components.Powertrain.Sources.Source()

    # booms
    boom      = RCAIDE.Library.Components.Booms.Boom()
    segment_1 = RCAIDE.Library.Components.Booms.Segments.Circle_Segment()
    boom.append_segment(segment_1)
    segment_2 = RCAIDE.Library.Components.Booms.Segments.Ellipse_Segment()
    boom.append_segment(segment_2)
    segment_3 = RCAIDE.Library.Components.Booms.Segments.Rounded_Rectangle_Segment()
    boom.append_segment(segment_3)
    segment_4 = RCAIDE.Library.Components.Booms.Segments.Super_Ellipse_Segment()
    boom.append_segment(segment_4)
    segment_5 = RCAIDE.Library.Components.Booms.Segments.Segment()
    boom.append_segment(segment_5) 

    # nacelles
    nacelle      = RCAIDE.Library.Components.Nacelles.Stack_Nacelle()
    segment_1 = RCAIDE.Library.Components.Nacelles.Segments.Circle_Segment()
    nacelle.append_segment(segment_1)
    segment_2 = RCAIDE.Library.Components.Nacelles.Segments.Ellipse_Segment()
    nacelle.append_segment(segment_2)
    segment_3 = RCAIDE.Library.Components.Nacelles.Segments.Rounded_Rectangle_Segment()
    nacelle.append_segment(segment_3)
    segment_4 = RCAIDE.Library.Components.Nacelles.Segments.Super_Ellipse_Segment()
    nacelle.append_segment(segment_4)
    segment_5 = RCAIDE.Library.Components.Nacelles.Segments.Segment()
    nacelle.append_segment(segment_5)
    

    # aerodynamics-propulsion coupling (joint rotor solve not yet implemented)
    aero_propulsion_coupling_test()

    elapsed_time = time.time() - ti
    elapsed_time_min = elapsed_time / 60
    print('Elapsed time (min): ', elapsed_time_min)
    return

def aero_propulsion_coupling_test():
    """Runs one solver iteration of a cruise segment with aerodynamics-propulsion coupling enabled."""
    vehicles_path = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Vehicles"))
    if vehicles_path not in sys.path:
        sys.path.insert(0, vehicles_path)
    from Electric_Twin_Otter import vehicle_setup

    vehicle = vehicle_setup('lithium_ion_nmc', None)
    for network in vehicle.networks:
        network.aero_propulsion_coupling = True

    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle
    weights = RCAIDE.Framework.Analyses.Weights.Electric_General_Aviation()
    analyses.append(weights)
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.overwrite_reference = False
    analyses.append(geometry)
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.number_of_spanwise_vortices  = 5
    aerodynamics.settings.number_of_chordwise_vortices = 2
    analyses.append(aerodynamics)
    analyses.append(RCAIDE.Framework.Analyses.Energy.Energy())
    analyses.append(RCAIDE.Framework.Analyses.Planets.Earth())
    analyses.append(RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976())

    mission  = RCAIDE.Framework.Mission.Sequential_Segments()
    segment  = RCAIDE.Framework.Mission.Segments.Cruise.Constant_Speed_Constant_Altitude()
    segment.tag = 'cruise'
    segment.analyses.extend(analyses)
    segment.altitude                                                 = 5000 * Units.feet
    segment.air_speed                                                = 130 * Units.kts
    segment.distance                                                 = 10 * Units.nmi
    segment.initial_battery_conditions.state_of_charge               = 1.0
    segment.state.numerics.number_of_control_points                  = 2
    segment.state.numerics.mission_solver.max_evaluations            = 1
    segment.state.numerics.mission_solver.print_output               = False
    segment.flight_dynamics.force_x                                  = True
    segment.flight_dynamics.force_z                                  = True
    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.pitch_angle.active            = True
    mission.append_segment(segment)
    mission.evaluate()
    return
    
    
if __name__ == '__main__': 
    main()    
