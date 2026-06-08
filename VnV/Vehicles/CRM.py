# VnV/Vehicles/CRM.py
# 
# 
# Created:  April 2026, A. Molloy 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units, Data           
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan    import design_turbofan      
from RCAIDE.Library.Plots                                                 import *     
import RCAIDE.Framework.External_Interfaces.OpenVSP as openvsp

# python imports 
import numpy as np  
from copy import deepcopy
import matplotlib.pyplot as plt  
import os
import sys

def vehicle_setup() :
                
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------      
    vehicle = RCAIDE.Vehicle()
    
    # ################################################# Vehicle-level Properties #################################################   
    vehicle.tag = 'CRM' 
    # vehicle.mass_properties.max_takeoff               = 227930
    # vehicle.mass_properties.takeoff                   = 227930 
    # vehicle.mass_properties.max_zero_fuel             = 161025.0 * Units.kilogram   
    # vehicle.mass_properties.max_fuel                  = 101323 * Units.kilogram    
    # vehicle.mass_properties.max_payload               = 44000
    # vehicle.mass_properties.center_of_gravity         = [[27.0, 0, 0]]
    # vehicle.flight_envelope.ultimate_load             = 3.5
    # vehicle.flight_envelope.positive_limit_load       = 2.5  
    # vehicle.flight_envelope.negative_limit_load       = 1
    # vehicle.flight_envelope.design_mach_number        = 0.85  
    # vehicle.flight_envelope.design_cruise_altitude    = 35000.0*Units.feet 
    # vehicle.flight_envelope.design_range              = 7305.0 * Units.nmi
    # vehicle.reference_area                            = 395.0 * Units['meters**2']    
    # vehicle.number_of_passengers                      = number_of_passengers 
    # vehicle.systems.control                           = "fully powered" 
    # vehicle.systems.accessories                       = "long range"

    vehicle.reference_area = 4500.0 * Units['ft**2'] 
    print(f"reference Area{vehicle.reference_area/Units['ft**2'] }")
    vehicle.mass_properties.center_of_gravity[0][0] = 1325.90 * Units.inches
    vehicle.mass_properties.center_of_gravity[0][2] = 0


    # ------------------------------------------------------------------
    #   Main Wing 
    # ------------------------------------------------------------------

    wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                              = 'main_wing'
    wing.aspect_ratio                     = 8.30
    wing.sweeps.quarter_chord             = 35 * Units.deg
    wing.thickness_to_chord               = 0.115
    wing.spans.projected                  = 2304 * Units.inches
    wing.chords.root                      = 536.18 * Units.inches
    wing.chords.tip                       = 107.4 * Units.inches
    wing.taper                            = wing.chords.tip / wing.chords.root
    wing.chords.mean_aerodynamic          = 265.44 * Units.inches 
    wing.areas.reference                  = 4500.0 * Units['ft**2']  #639362 * Units['inches']**2
    wing.areas.wetted                     = 2*wing.areas.reference
    wing.origin                           = [[904.294* Units.inches,0,172.126* Units.inches]] 
    wing.aerodynamic_center               = [0,0,0] 
    wing.vertical                         = False
    wing.dihedral                         = 7.0 * Units.degrees 
    wing.xz_plane_symmetric               = True 
    wing.dynamic_pressure_ratio           = 1.0

    wing.chords.mean_aerodynamic = 275.80 * Units.inches 
    wing.transition_x_upper    = 0.01
    wing.transition_x_lower    = 0.01
        
    # Wing Segments
    ospath                                = os.path.abspath(__file__)
    separator                             = os.path.sep
    rel_path                              = os.path.dirname(ospath) + separator
    
    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '0'
    segment.percent_span_location         = 0.0 
    segment.root_chord_percent            = 1.
    segment.dihedral_outboard             = 0.8569 * Units.degrees
    segment.sweeps.quarter_chord          = 27.57 * Units.degrees
    segment.twist                         = 6.72 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+"CRM_Airfoils"+ separator + 'main_wing_airfoil_XSec_0.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '1'
    segment.percent_span_location         = 0.1 
    segment.root_chord_percent            = 0.881
    segment.dihedral_outboard             = 1.127 * Units.degrees
    segment.sweeps.quarter_chord          = 27.57 * Units.degrees
    segment.twist                         = 2.3907593412849977 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator + "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_1.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)


    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '2'
    segment.percent_span_location         = 0.15 
    segment.root_chord_percent            = 0.8126
    segment.dihedral_outboard             = 0.5057 * Units.degrees
    segment.sweeps.quarter_chord          = 27.57 * Units.degrees
    segment.twist                         = 3.158605874891446 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator + "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_2.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '3'
    segment.percent_span_location         = 0.20 
    segment.root_chord_percent            = 0.747
    segment.dihedral_outboard             = 0.5433 * Units.degrees
    segment.sweeps.quarter_chord          = 27.57 * Units.degrees
    segment.twist                         = 3.07 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_3.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '4'
    segment.percent_span_location         = 0.25 
    segment.root_chord_percent            = 0.684
    segment.dihedral_outboard             = 0.9652 * Units.degrees
    segment.sweeps.quarter_chord          = 27.57 * Units.degrees
    segment.twist                         = 2.23 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_4.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '5'
    segment.percent_span_location         = 0.30 
    segment.root_chord_percent            = 0.621
    segment.dihedral_outboard             = 1.461 * Units.degrees
    segment.sweeps.quarter_chord          = 27.57 * Units.degrees
    segment.twist                         = 1.5 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_5.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '6'
    segment.percent_span_location         = 0.35 
    segment.root_chord_percent            = 0.558
    segment.dihedral_outboard             = 1.7585 * Units.degrees
    segment.sweeps.leading_edge           = 37.25 * Units.degrees
    segment.twist                         = 0.898 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_6.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '7'
    segment.percent_span_location         = 0.37 
    segment.root_chord_percent            = 0.533
    segment.dihedral_outboard             = 2.0 * Units.degrees
    segment.sweeps.leading_edge           = 37.25 * Units.degrees
    segment.twist                         = 0.7182181454843295 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_7.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '8'
    segment.percent_span_location         = 0.40 
    segment.root_chord_percent            = 0.517
    segment.dihedral_outboard             = 2.574 * Units.degrees
    segment.sweeps.leading_edge           = 37.27 * Units.degrees
    segment.twist                         = 0.3522 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_8.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '9'
    segment.percent_span_location         = 0.45 
    segment.root_chord_percent            = 0.491
    segment.dihedral_outboard             = 3.418 * Units.degrees
    segment.sweeps.leading_edge           = 37.27 * Units.degrees
    segment.twist                         = -0.36 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_9.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '10'
    segment.percent_span_location         = 0.50 
    segment.root_chord_percent            = 0.464
    segment.dihedral_outboard             = 4.86 * Units.degrees
    segment.sweeps.leading_edge           = 37.27 * Units.degrees
    segment.twist                         = -0.82 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_10.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '11'
    segment.percent_span_location         = 0.55 
    segment.root_chord_percent            = 0.438
    segment.dihedral_outboard             = 5.49 * Units.degrees
    segment.sweeps.leading_edge          = 37.27 * Units.degrees
    segment.twist                         = -1.02 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_11.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '12'
    segment.percent_span_location         = 0.60 
    segment.root_chord_percent            = 0.412
    segment.dihedral_outboard             = 5.49 * Units.degrees
    segment.sweeps.leading_edge          = 37.27 * Units.degrees
    segment.twist                         = -1.276 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_12.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '13'
    segment.percent_span_location         = 0.65 
    segment.root_chord_percent            = 0.385
    segment.dihedral_outboard             = 6.36 * Units.degrees
    segment.sweeps.leading_edge           = 37.27 * Units.degrees
    segment.twist                         = -1.54 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_13.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '14'
    segment.percent_span_location         = 0.70 
    segment.root_chord_percent            = 0.359
    segment.dihedral_outboard             = 7.129 * Units.degrees
    segment.sweeps.leading_edge           = 37.27 * Units.degrees
    segment.twist                         = -1.74 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_14.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '15'
    segment.percent_span_location         = 0.75 
    segment.root_chord_percent            = 0.332
    segment.dihedral_outboard             = 7.92 * Units.degrees
    segment.sweeps.leading_edge           = 37.27 * Units.degrees
    segment.twist                         = -1.88 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_15.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '16'
    segment.percent_span_location         = 0.80 
    segment.root_chord_percent            = 0.306
    segment.dihedral_outboard             = 8.68 * Units.degrees
    segment.sweeps.leading_edge           = 37.27 * Units.degrees
    segment.twist                         = -2.10 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_16.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '17'
    segment.percent_span_location         = 0.85 
    segment.root_chord_percent            = 0.2795
    segment.dihedral_outboard             = 9.30 * Units.degrees
    segment.sweeps.leading_edge           = 37.27 * Units.degrees
    segment.twist                         = -2.35 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_17.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '18'
    segment.percent_span_location         = 0.90 
    segment.root_chord_percent            = 0.253
    segment.dihedral_outboard             = 9.76 * Units.degrees
    segment.sweeps.leading_edge           = 37.27 * Units.degrees
    segment.twist                         = -2.64 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_18.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '19'
    segment.percent_span_location         = 0.95 
    segment.root_chord_percent            = 0.227
    segment.dihedral_outboard             = 9.94 * Units.degrees
    segment.sweeps.leading_edge           = 37.27 * Units.degrees
    segment.twist                         = -3.16 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_19.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                           = '20'
    segment.percent_span_location         = 1.0 
    segment.root_chord_percent            = 0.200
    segment.dihedral_outboard             = 0
    segment.sweeps.leading_edge           = 0
    segment.twist                         = -3.75 * Units.degrees
    section_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
    section_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator+ "CRM_Airfoils" + separator + 'main_wing_airfoil_XSec_20.dat'
    segment.append_airfoil(section_airfoil)
    wing.append_segment(segment)

    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing     = RCAIDE.Library.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'

    wing.aspect_ratio            = 4.8335
    wing.sweeps.leading_edge     = 40.15 * Units.deg  
    wing.thickness_to_chord      = 0.09
    wing.taper                   = 0.351
    wing.spans.projected         = 847 * Units.inches
    wing.chords.root             = 255 * Units.inches
    wing.chords.tip              = 89.5 * Units.inches
    wing.chords.mean_aerodynamic = 172.25 * Units.inches
    wing.areas.reference         = 145895.75 * Units['inches']**2
    wing.areas.exposed           = 0.95*wing.areas.reference       # Exposed area of the horizontal tail
    wing.areas.wetted            = 2.1*wing.areas.reference      # Wetted area of the horizontal tail
    wing.twists.root             = 0.0 * Units.degrees
    wing.twists.tip              = 0.0 * Units.degrees 
    wing.origin                  = [[2179.5 * Units.inches, 0, 248.146 * Units.inches]]
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
    segment.sweeps.leading_edge    = 40.15  * Units.degrees 
    segment.thickness_to_chord     = 0.1
    wing.append_segment(segment)

    segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                    = 'tip_segment'
    segment.percent_span_location  = 1.
    segment.twist                  = 0. * Units.deg
    segment.root_chord_percent     = 0.351               
    segment.dihedral_outboard      = 0 * Units.degrees
    segment.sweeps.quarter_chord   = 0 * Units.degrees  
    segment.thickness_to_chord     = 0.08
    wing.append_segment(segment) 

    vehicle.append_component(wing)
    
    # ################################################# Fuselage ################################################################ 
        
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage()  
    fuselage.seats_abreast                      = 9
    fuselage.seat_pitch                         = 0.9     * Units.meter 
    fuselage.fineness.nose                      = 1.4
    fuselage.fineness.tail                      = 3. 
    fuselage.lengths.nose                       = 339   * Units.inches
    fuselage.lengths.tail                       = 719   * Units.inches
    fuselage.lengths.total                      = 2470 * Units.inches  
    fuselage.lengths.fore_space                 = 1.    * Units.meter
    fuselage.lengths.aft_space                  = 10.    * Units.meter
    fuselage.width                              = 245.88  * Units.inches
    fuselage.heights.maximum                    = 245.97  * Units.inches
    fuselage.effective_diameter                 = 245.92  * Units.inches
    fuselage.areas.side_projected               = fuselage.heights.maximum * fuselage.lengths.total
    fuselage.areas.wetted                       = np.pi*fuselage.width/2*(fuselage.width/2+ np.sqrt( fuselage.lengths.nose **2 +(fuselage.width/2)**2)) + \
                                                 np.pi*fuselage.width/2*(fuselage.width/2+ np.sqrt( fuselage.lengths.tail**2 +(fuselage.width/2)**2))+ \
                                                    np.pi * fuselage.width * ( fuselage.lengths.total - (fuselage.lengths.tail+ fuselage.lengths.nose))
    fuselage.areas.front_projected              = np.pi * (fuselage.width/2) **2   
    fuselage.differential_pressure              = 5.0e4 * Units.pascal
    fuselage.heights.at_quarter_length          = fuselage.heights.maximum 
    fuselage.heights.at_three_quarters_length   = fuselage.heights.maximum
    fuselage.heights.at_wing_root_quarter_chord = fuselage.heights.maximum   
    fuselage.origin = [[91.25* Units.inches, 0, 202.986* Units.inches]] 

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
    segment.percent_x_location                  = 0.01 
    segment.percent_z_location                  = -0.0015 
    segment.height                              = 60.98 * Units.inches
    segment.width                               = 55.88 * Units.inches
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.01868 
    segment.percent_z_location                  = -0.0011 
    segment.height                              = 84.79 * Units.inches
    segment.width                               = 81.63 * Units.inches
    fuselage.append_segment(segment)      
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.0298 
    segment.percent_z_location                  = 0.0017 
    segment.height                              = 122.47 * Units.inches 
    segment.width                               = 108.92 * Units.inches
    fuselage.append_segment(segment)   

    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.0599 	
    segment.percent_z_location                  = 0.004 
    segment.height                              = 178.65 * Units.inches
    segment.width                               = 159.95 * Units.inches
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 0.1005
    segment.percent_z_location                  = 0.0052 
    segment.height                              = 222.23 * Units.inches 
    segment.width                               = 213.27 * Units.inches
    fuselage.append_segment(segment)     
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 0.1375 
    segment.percent_z_location                  = 0.0067 
    segment.height                              = 241.72 * Units.inches
    segment.width                               = 240.42 * Units.inches
    fuselage.append_segment(segment)             
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 0.1633 
    segment.percent_z_location                  = 0.0073 
    segment.height                              = 245.97 * Units.inches
    segment.width                               = 245.88 * Units.inches
    fuselage.append_segment(segment)    
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 0.31656
    segment.percent_z_location                  = 0.0073 
    segment.height                              = 245.97 * Units.inches
    segment.width                               = 245.88 * Units.inches
    fuselage.append_segment(segment)   
    
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_9'     
    segment.percent_x_location                  = 0.51255 
    segment.percent_z_location                  = 0.0075
    segment.height                              = 245.97 * Units.inches
    segment.width                               = 245.88 * Units.inches
    fuselage.append_segment(segment)     
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_10'     
    segment.percent_x_location                  = 0.70925 
    segment.percent_z_location                  = 0.0075
    segment.height                              = 245.97 * Units.inches
    segment.width                               = 245.8846 * Units.inches
    fuselage.append_segment(segment)   
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_11'     
    segment.percent_x_location                  = 0.79979 
    segment.percent_z_location                  = 0.0135
    segment.height                              = 209.279 * Units.inches
    segment.width                               = 212.55 * Units.inches
    fuselage.append_segment(segment)    
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_12'     
    segment.percent_x_location                  = 0.8325
    segment.percent_z_location                  = 0.0156
    segment.height                              = 188.64 * Units.inches
    segment.width                               = 190.94 * Units.inches
    fuselage.append_segment(segment)             
        
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_13'     
    segment.percent_x_location                  = 0.91627
    segment.percent_z_location                  = 0.021
    segment.height                              = 121.3 * Units.inches
    segment.width                               = 102.46 * Units.inches
    fuselage.append_segment(segment)               
       
    # Segment                                   
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_14'     
    segment.percent_x_location                  = 1.0 
    segment.percent_z_location                  = 0.04098
    segment.height                              = 0.0
    segment.width                               = 0.0
    fuselage.append_segment(segment)

    # add to vehicle
    vehicle.append_component(fuselage)

    # ################################################# Energy Network #######################################################          
    #------------------------------------------------------------------------------------------------------------------------- 
    #  Turbofan Network: Based on 787 network
    #-------------------------------------------------------------------------------------------------------------------------   
    net                                         = RCAIDE.Framework.Networks.Fuel() 

    #------------------------------------------------------------------------------------------------------------------------- 
    # Fuel Distrubition Line 
    #------------------------------------------------------------------------------------------------------------------------- 
    fuel_line                                   = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()  

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Starboard Propulsor
    # Sources: https://www.researchgate.net/publication/320798360_Performance_Analysis_of_Cold_Sections_of_High_BYPASS_Ratio_Turbofan_Aeroengine/figures?lo=1

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Propulsor: Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------         
    turbofan1                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()   
    turbofan1.origin                             = [[969.298*Units.inches, 384.169*Units.inches,75.863*Units.inches]]
    turbofan1.tag                                = 'propulsor_1'    
    turbofan1.length                             = 4.928                      
    turbofan1.diameter                           = 2.822                  
    turbofan1.bypass_ratio                       = 9.1                        
    turbofan1.design_altitude                    = 36000*Units.ft             
    turbofan1.design_mach_number                 = 0.85                     
    turbofan1.design_thrust                      = 80000* Units.N
    
    # working fluid                   
    turbofan1.working_fluid                      = RCAIDE.Library.Attributes.Gases.Air() 
    
    # Ram inlet 
    ram                                          = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                      = 'ram' 
    turbofan1.ram                                = ram 
            
    # inlet nozzle          
    inlet_nozzle                                = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
    inlet_nozzle.tag                            = 'inlet nozzle'
    inlet_nozzle.polytropic_efficiency          = 0.98                                      
    inlet_nozzle.pressure_ratio                 = 1
    inlet_nozzle.compressibility_effects        = False
    turbofan1.inlet_nozzle                       = inlet_nozzle
    
    # fan                
    fan                                         = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
    fan.tag                                     = 'fan'
    fan.polytropic_efficiency                   = 0.98                 
    fan.pressure_ratio                          = 1.4                    
    turbofan1.fan                                = fan        

    # low pressure compressor    
    low_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    low_pressure_compressor.tag                   = 'lpc'
    low_pressure_compressor.polytropic_efficiency = 0.98                  
    low_pressure_compressor.pressure_ratio        = 1.3                     
    turbofan1.low_pressure_compressor              = low_pressure_compressor

    # high pressure compressor  
    high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
    high_pressure_compressor.tag                   = 'hpc'
    high_pressure_compressor.polytropic_efficiency = 0.98                        
    high_pressure_compressor.pressure_ratio        = 23.9                    
    turbofan1.high_pressure_compressor              = high_pressure_compressor

    # low pressure turbine  
    low_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    low_pressure_turbine.tag                       ='lpt'
    low_pressure_turbine.mechanical_efficiency     = 0.99                     
    low_pressure_turbine.polytropic_efficiency     = 0.98                    
    turbofan1.low_pressure_turbine                  = low_pressure_turbine
    
    # high pressure turbine     
    high_pressure_turbine                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
    high_pressure_turbine.tag                      ='hpt'
    high_pressure_turbine.mechanical_efficiency    = 0.99                     
    high_pressure_turbine.polytropic_efficiency    = 0.98                     
    turbofan1.high_pressure_turbine                 = high_pressure_turbine 

    # combustor  
    combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
    combustor.tag                                  = 'Comb'
    combustor.efficiency                           = 0.997                    
    combustor.turbine_inlet_temperature            = 1440                  
    combustor.pressure_ratio                       = 0.94                     
    combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    turbofan1.combustor                             = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.98                     
    core_nozzle.pressure_ratio                     = 0.995
    core_nozzle.diameter                           = 1.5  # may be incorrect 
    turbofan1.core_nozzle                          = core_nozzle
        
    # fan nozzle             
    fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                 = 'fan nozzle'
    fan_nozzle.polytropic_efficiency               = 0.98                     # CHECKED Ref. [2] Page 9
    fan_nozzle.pressure_ratio                      = 0.995 
    fan_nozzle.diameter                            = 2.822 # may be incorrect         
    turbofan1.fan_nozzle                           = fan_nozzle
    
    # design turbofan
    design_turbofan(turbofan1)

    # Nacelle 
    nacelle                                     = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.diameter                            = 3.556
    nacelle.length                              = 4.9
    nacelle.tag                                 = 'nacelle_1'
    nacelle.inlet_diameter                      = 2.5
    nacelle.origin                              = [[969.298*Units.inches, 384.169*Units.inches,75.863*Units.inches]]
    nacelle.areas.wetted                        = np.pi*nacelle.diameter*nacelle.length
    nacelle_airfoil                             = RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil()
    nacelle_airfoil.NACA_4_Series_code          = '0010'
    nacelle.append_airfoil(nacelle_airfoil) 
    turbofan1.nacelle                            = nacelle
    
    net.propulsors.append(turbofan1)

        #------------------------------------------------------------------------------------------------------------------------------------  
    # Propulsor: Propulsor 2 (Inner Port Side)
    #------------------------------------------------------------------------------------------------------------------------------------      
    # copy turbofan
    turbofan2                                  = deepcopy(turbofan1)
    turbofan2.active_fuel_tanks                = ['fuel_tank'] 
    turbofan2.tag                              = 'propulsor_2' 
    turbofan2.origin                           = [[969.298*Units.inches, -384.169*Units.inches,75.863*Units.inches]]
    turbofan2.nacelle.origin                   = [[969.298*Units.inches, -384.169*Units.inches,75.863*Units.inches]]
        
    # append propulsor to distribution line 
    net.propulsors.append(turbofan2)

    #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #------------------------------------------------------------------------------------------------------------------------- 
    # fuel tank
    fuel_tank_1                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing) 
    fuel_tank_1.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A1()    
    fuel_tank_1.segments_bounding_tank       = ['0', '1']  
    fuel_tank_1.segments_percent_chord_start = [0.1, 0.1]
    fuel_tank_1.segments_percent_chord_end   = [0.7, 0.7] 
    fuel_line.fuel_tanks.append(fuel_tank_1)
    
    fuel_tank_2                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing) 
    fuel_tank_2.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A1()    
    fuel_tank_2.segments_bounding_tank       = ['0','1']  
    fuel_tank_2.segments_percent_chord_start = [0.1, 0.1]
    fuel_tank_2.segments_percent_chord_end   = [0.7, 0.7] 
    fuel_line.fuel_tanks.append(fuel_tank_2)    
 

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Assign propulsors to fuel line to network      
    fuel_line.assigned_propulsors     =  [['propulsor_1', 'propulsor_2']]

    #------------------------------------------------------------------------------------------------------------------------------------   
    # Append fuel line to fuel line to network      
    net.fuel_lines.append(fuel_line)        
 
    # Append energy network to aircraft 
    vehicle.append_energy_network(net)  
    
    return vehicle 

# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    """This function sets up vehicle configurations for use in different parts of the mission.
    Here, this is mostly in terms of high lift settings."""

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
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