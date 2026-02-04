''' 
# Tilt_Stopped_Rotor.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

'''
#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units    
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor  import design_electric_rotor
from RCAIDE.Library.Plots                                         import * 
from RCAIDE import  load 
from RCAIDE import  save  

import os
import numpy as np 
from copy import deepcopy
import matplotlib.pyplot as plt 
import  pickle
# ----------------------------------------------------------------------------------------------------------------------
#  REGRESSION
# ----------------------------------------------------------------------------------------------------------------------  
def main():           
         
    # vehicle data
    new_geometry    = True
    redesign_rotors = False  
    if new_geometry :
        vehicle  = vehicle_setup(redesign_rotors)
        save_aircraft_geometry(vehicle , 'Tilt_Stopped_Rotor_Conv_Tail')
    else: 
        vehicle = load_aircraft_geometry('Tilt_Stopped_Rotor_Conv_Tail')

    plot_3d_vehicle(vehicle) 
        
    # Set up configs
    configs  = configs_setup(vehicle)

    # vehicle analyses
    analyses = analyses_setup(configs)

    # mission analyses
    #mission  = mission_setup(analyses)
    mission = complete_mission_setup(analyses)
    missions = missions_setup(mission) 
     
    results = missions.base_mission.evaluate() 
    display_stability_derivatives(results.segments[0])
    #print(f"force_x: {results.segments[0].state.residuals.force_x[0]}")
    #print(f"force_y: {results.segments[0].state.residuals.force_y[0]}")
    #print(f"force_z: {results.segments[0].state.residuals.force_z[0]}")
    # print(f"moment_x: {results.segments[0].state.residuals.moment_x[0]}")
    # print(f"moment_y: {results.segments[0].state.residuals.moment_y[0]}")
    # print(f"moment_z: {results.segments[0].state.residuals.moment_z[0]}")
     
    # plot the results 
    plot_results(results)    
    plt.show()
           
          
    return
 
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
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.vehicle                             = vehicle
    geometry.settings.overwrite_reference        = True
    geometry.settings.update_wing_properties     = True
    geometry.settings.update_center_of_gravity  = True
    analyses.append(geometry)


    # ------------------------------------------------------------------
    #  Weights
    weights         = RCAIDE.Framework.Analyses.Weights.Electric()
    weights.aircraft_type = "VTOL"
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics         = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    aerodynamics.stability_derivatives.CX_alpha     = None
    aerodynamics.stability_derivatives.CX_u         = None
    aerodynamics.stability_derivatives.CY_beta      = -0.195398
    aerodynamics.stability_derivatives.CY_r         = None
    aerodynamics.stability_derivatives.CZ_alpha     = 4.837822
    aerodynamics.stability_derivatives.CZ_u         = None
    aerodynamics.stability_derivatives.CZ_q         = None
    aerodynamics.stability_derivatives.CL_beta      = -0.12228
    aerodynamics.stability_derivatives.CL_p         = None
    aerodynamics.stability_derivatives.CL_r         = None
    aerodynamics.stability_derivatives.CM_alpha     = -1.1509 
    aerodynamics.stability_derivatives.CM_u         = None
    aerodynamics.stability_derivatives.CM_q         = None
    aerodynamics.stability_derivatives.CN_beta      = 0.074425
    aerodynamics.stability_derivatives.CN_p         = None
    aerodynamics.stability_derivatives.CN_r         = None
    aerodynamics.stability_derivatives.CY_delta_a   = 0.000623 * 180/np.pi
    aerodynamics.stability_derivatives.CL_delta_a   = 0.001974 * 180/np.pi
    aerodynamics.stability_derivatives.CLift_delta_e= 0.009275 * 180/np.pi
    aerodynamics.stability_derivatives.CN_delta_a   = -0.000021 * 180/np.pi
    aerodynamics.stability_derivatives.CM_delta_e   = -0.024291 * 180/np.pi
    aerodynamics.stability_derivatives.CY_delta_r   = -0.001918 * 180/np.pi
    aerodynamics.stability_derivatives.CL_delta_r   = -0.000195 * 180/np.pi
    aerodynamics.stability_derivatives.CN_delta_r   = 0.001046 * 180/np.pi
    aerodynamics.stability_derivatives.CM_delta_f   = None   
    analyses.append(aerodynamics)


    # ------------------------------------------------------------------
    #  Stability Analysis
    stability         = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method() 
    analyses.append(stability)    

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy() 
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




# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def vehicle_setup(redesign_rotors=True) : 

    ospath      = os.path.abspath(__file__)
    separator   = os.path.sep
    airfoil_path    = os.path.dirname(ospath) + separator  + '..' + separator  
    local_path  = os.path.dirname(ospath) + separator          
 

    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------             
    vehicle               = RCAIDE.Vehicle()
    vehicle.tag           = 'Tilt_Stopped_Rotor_Conv_Tail'
    vehicle.configuration = 'eVTOL'
     
    #------------------------------------------------------------------------------------------------------------------------------------
    # ################################################# Vehicle-level Properties #####################################################  
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # mass properties 
    vehicle.mass_properties.max_takeoff       = 2700 
    vehicle.mass_properties.takeoff           = vehicle.mass_properties.max_takeoff
    vehicle.mass_properties.operating_empty   = vehicle.mass_properties.max_takeoff
    vehicle.mass_properties.center_of_gravity = [[ 2.1345, 0 , 0 ]] 
    vehicle.mass_properties.moments_of_inertia.tensor = np.array([[164627.7,0.0,0.0],[0.0,471262.4,0.0],[0.0,0.0,554518.7]])
    vehicle.flight_envelope.ultimate_load            = 5.7   
    vehicle.flight_envelope.positive_limit_load               = 3.
    vehicle.flight_envelope.mach_number              = 0.15
    vehicle.number_of_passengers                               = 5
    vehicle.design_dynamic_pressure                  = 1929 # THis value comes form the Navion
    vehicle.reference_area                           = 15.629
        
    #------------------------------------------------------------------------------------------------------------------------------------
    # ######################################################## Wings ####################################################################  
    #------------------------------------------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------
    wing                          = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                      = 'main_wing'  
    wing.aspect_ratio             = 8.95198  # will  be overwritten
    wing.sweeps.quarter_chord     = 0.0  
    wing.thickness_to_chord       = 0.14 
    wing.taper                    = 0.292
    wing.spans.projected          = 11.82855
    wing.total_length             = 1.75
    wing.chords.root              = 1.75
    wing.chords.tip               = 1.0
    wing.chords.mean_aerodynamic  = 1.5
    wing.dihedral                 = 0.0  
    wing.areas.reference          = 15.629
    wing.twists.root              = 4. * Units.degrees
    wing.twists.tip               = 0. 
    wing.origin                   = [[1.5, 0., 0.991]]
    wing.aerodynamic_center       = [ 1.567, 0., 0.991]    
    wing.winglet_fraction         = 0.0  
    wing.xz_plane_symmetric       = True
    wing.vertical                 = False
    airfoil                       = RCAIDE.Library.Components.Airfoils.Airfoil()
    airfoil.coordinate_file       = '/Users/siripunn/Desktop/LEADS_WORK/LEADS_Research/RCAIDE_LEADS/Digital_Hangar/Tilt_Stopped_Rotor_EVTOL/NACA_63_412.txt'
    
    # Segment                                  
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'Section_1'   
    segment.percent_span_location = 0.0
    segment.twist                 = 4. * Units.degrees 
    segment.root_chord_percent    = 1. 
    segment.dihedral_outboard     = 8 * Units.degrees
    segment.sweeps.quarter_chord  = 0.9  * Units.degrees 
    segment.thickness_to_chord    = 0.16  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)               
    
    # Segment                                   
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'Section_2'    
    segment.percent_span_location = 3.5/wing.spans.projected
    segment.twist                 = 3. * Units.degrees 
    segment.root_chord_percent    = 1.4000/1.7500
    segment.dihedral_outboard     = 0.0 * Units.degrees
    segment.sweeps.quarter_chord  = 1.27273 * Units.degrees 
    segment.thickness_to_chord    = 0.16  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)               
     
    # Segment                                  
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'Section_3'   
    segment.percent_span_location = 11.3/wing.spans.projected 
    segment.twist                 = 2.0 * Units.degrees 
    segment.root_chord_percent    = 1.000/1.7500
    segment.dihedral_outboard     = 35.000* Units.degrees 
    segment.sweeps.quarter_chord  = 45.000* Units.degrees 
    segment.thickness_to_chord    = 0.16  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)     
    
    # Segment                                  
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'Section_4'   
    segment.percent_span_location = 11.6/wing.spans.projected 
    segment.twist                 = 0.0 * Units.degrees 
    segment.root_chord_percent    = 0.9/1.7500
    segment.dihedral_outboard     = 60. * Units.degrees 
    segment.sweeps.quarter_chord  = 70.0 * Units.degrees 
    segment.thickness_to_chord    = 0.16  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)  
    
    # Segment                                  
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'Section_5'   
    segment.percent_span_location = 1.0
    segment.twist                 = 0.0 * Units.degrees 
    segment.root_chord_percent    = 0.35/1.7500
    segment.dihedral_outboard     = 0  * Units.degrees 
    segment.sweeps.quarter_chord  = 0  * Units.degrees 
    segment.thickness_to_chord    = 0.16  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)                 
     
                                          
    # control surfaces ------------------------------------------- 
    flap                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                      = 'flap'
    flap.span_fraction_start      = 0.2
    flap.span_fraction_end        = 0.5
    flap.deflection               = 0.0 * Units.degrees 
    flap.chord_fraction           = 0.20
    wing.append_control_surface(flap)  
    

    aileron                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron()
    aileron.tag                   = 'aileron'
    aileron.span_fraction_start   = 0.7
    aileron.span_fraction_end     = 0.9 
    aileron.deflection            = 0.0 * Units.degrees
    aileron.chord_fraction        = 0.2
    wing.append_control_surface(aileron)      

        
    # add to vehicle 
    vehicle.append_component(wing)  
    
     
    # ------------------------------------------------------------------        
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------       
    wing                                  = RCAIDE.Library.Components.Wings.Horizontal_Tail() 
    wing.sweeps.leading_edge              = 6 * Units.degrees 
    wing.thickness_to_chord               = 0.12
    wing.areas.reference                  = 4   
    wing.spans.projected                  = 4 
    wing.chords.root                      = 0.75
    wing.chords.mean_aerodynamic          = 0.75
    wing.chords.tip                       = 0.75
    wing.taper                            = wing.chords.tip/wing.chords.root
    wing.aspect_ratio                     = wing.spans.projected**2. / wing.areas.reference
    wing.twists.root                      = 0 * Units.degrees  
    wing.twists.tip                       = 0 * Units.degrees   
    wing.origin                           = [[ 6.54518625 , 0., 0.203859697]]
    wing.aerodynamic_center               = [[ 6.545186254 + 0.25*wing.spans.projected, 0., 0.5]] 
    wing.vertical                         = False 
    wing.xz_plane_symmetric               = True
    wing.high_lift                        = False 
    wing.dynamic_pressure_ratio           = 0.9 
    
    # Segment                                  
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'Root'   
    segment.percent_span_location = 0.0
    segment.root_chord_percent    = 1. 
    segment.sweeps.leading_edge   = 6 * Units.degrees 
    segment.thickness_to_chord    = 0.12  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)  
    
    # Segment                                  
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'Tip'   
    segment.percent_span_location = 1.0
    segment.root_chord_percent    = wing.taper
    segment.thickness_to_chord    = 0.12  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)
    
        
    elevator                              = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
    elevator.tag                          = 'elevator'
    elevator.span_fraction_start          = 0.1
    elevator.span_fraction_end            = 0.9
    elevator.deflection                   = 0.0  * Units.deg
    elevator.chord_fraction               = 0.35
    wing.append_control_surface(elevator)         
 
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------ 
    wing                                  = RCAIDE.Library.Components.Wings.Vertical_Tail() 
    wing.sweeps.leading_edge              = 20 * Units.degrees 
    wing.thickness_to_chord               = 0.125
    wing.areas.reference                  = 1.163 
    wing.spans.projected                  = 1.4816
    wing.chords.root                      = 1.2176
    wing.chords.tip                       = 1.2176
    wing.chords.tip                       = 0.5870 
    wing.aspect_ratio                     = 1.8874 
    wing.taper                            = 0.4820 
    wing.chords.mean_aerodynamic          = 0.9390 
    wing.twists.root                      = 0 * Units.degrees  
    wing.twists.tip                       = 0 * Units.degrees   
    wing.origin                           = [[ 7.127369987, 0., 0.5]]
    wing.aerodynamic_center               = [ 7.49778005775, 0., 0.5] 
    wing.vertical                         = True 
    wing.xz_plane_symmetric               = False
    wing.t_tail                           = False
    wing.winglet_fraction                 = 0.0  
    wing.dynamic_pressure_ratio           = 1.0
    
    # Segment                                  
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'Root'   
    segment.percent_span_location = 0.0
    segment.root_chord_percent    = 1. 
    segment.sweeps.leading_edge   = 20 * Units.degrees 
    segment.thickness_to_chord    = 0.125  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)  
    
    # Segment                                  
    segment                       = RCAIDE.Library.Components.Wings.Segments.Segment()
    segment.tag                   = 'Tip'   
    segment.percent_span_location = 1.0
    segment.root_chord_percent    = wing.taper
    segment.thickness_to_chord    = 0.125  
    segment.append_airfoil(airfoil)
    wing.append_segment(segment)      
    
    rudder                                = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    rudder.tag                            = 'rudder'
    rudder.span_fraction_start            = 0.1
    rudder.span_fraction_end              = 0.9
    rudder.deflection                     = 0.0  * Units.deg
    rudder.chord_fraction                 = 0.4
    wing.append_control_surface(rudder) 
     
    vehicle.append_component(wing)
      
    #------------------------------------------------------------------------------------------------------------------------------------
    # ##########################################################   Fuselage  ############################################################   
    #------------------------------------------------------------------------------------------------------------------------------------ 
    fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage()
    fuselage.tag                                = 'fuselage' 
    fuselage.seats_abreast                      = 2.  
    fuselage.seat_pitch                         = 3.  
    fuselage.fineness.nose                      = 0.88   
    fuselage.fineness.tail                      = 1.13   
    fuselage.lengths.nose                       = 0.5  
    fuselage.lengths.tail                       = 1.5
    fuselage.lengths.cabin                      = 4.46 
    fuselage.lengths.total                      = 6.46
    fuselage.width                              = 1.75
    fuselage.heights.maximum                    = 4.65 * Units.feet      # change 
    fuselage.heights.at_quarter_length          = 3.75 * Units.feet      # change 
    fuselage.heights.at_wing_root_quarter_chord = 4.65 * Units.feet      # change 
    fuselage.heights.at_three_quarters_length   = 4.26 * Units.feet      # change 
    fuselage.areas.wetted                       = 236. * Units.feet**2   # change 
    fuselage.areas.front_projected              = 0.14 * Units.feet**2   # change 
    fuselage.effective_diameter                 = 1.276     # change 
    fuselage.differential_pressure              = 0. 
    
    # Segment  
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment() 
    segment.tag                                 = 'segment_0'    
    segment.percent_x_location                  = 0.0 
    segment.percent_z_location                  = 0.     # change  
    segment.height                              = 0.049 
    segment.width                               = 0.032 
    fuselage.append_segment(segment)                     
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_1'   
    segment.percent_x_location                  = 0.10912/fuselage.lengths.total 
    segment.percent_z_location                  = 0.00849
    segment.height                              = 0.481 
    segment.width                               = 0.553 
    fuselage.append_segment(segment)           
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_2'   
    segment.percent_x_location                  = 0.47804/fuselage.lengths.total
    segment.percent_z_location                  = 0.02874
    segment.height                              = 1.00
    segment.width                               = 0.912 
    fuselage.append_segment(segment)                     
                                                
    # Segment                                            
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_3'   
    segment.percent_x_location                  = 0.161  
    segment.percent_z_location                  = 0.04348  
    segment.height                              = 1.41
    segment.width                               = 1.174  
    fuselage.append_segment(segment)                     
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_4'   
    segment.percent_x_location                  = 0.284 
    segment.percent_z_location                  = 0.05435 
    segment.height                              = 1.62
    segment.width                               = 1.276  
    fuselage.append_segment(segment)              
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_5'   
    segment.percent_x_location                  = 3.43026/fuselage.lengths.total
    segment.percent_z_location                  = 0.31483/fuselage.lengths.total 
    segment.height                              = 1.409
    segment.width                               = 1.121 
    fuselage.append_segment(segment)                     
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_6'   
    segment.percent_x_location                  = 4.20546/fuselage.lengths.total
    segment.percent_z_location                  = 0.32216/fuselage.lengths.total
    segment.height                              = 1.11
    segment.width                               = 0.833
    fuselage.append_segment(segment)                  
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_7'   
    segment.percent_x_location                  = 4.99358/fuselage.lengths.total
    segment.percent_z_location                  = 0.37815/fuselage.lengths.total
    segment.height                              = 0.78
    segment.width                               = 0.512 
    fuselage.append_segment(segment)                  
                                                
    # Segment                                             
    segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
    segment.tag                                 = 'segment_8'   
    segment.percent_x_location                  = 1.
    segment.percent_z_location                  = 0.55/fuselage.lengths.total
    segment.height                              = 0.195  
    segment.width                               = 0.130 
    fuselage.append_segment(segment)                   
                                                
    vehicle.append_component(fuselage) 
    
    #------------------------------------------------------------------------------------------------------------------------------------
    # ##########################################################  Booms  ################################################################  
    #------------------------------------------------------------------------------------------------------------------------------------          
    boom                                    = RCAIDE.Library.Components.Booms.Boom()
    boom.tag                                = 'boom_1r'
    boom.configuration                      = 'boom'  
    boom.origin                             = [[   0.036, 2.25,  1]]  
    boom.seats_abreast                      = 0.  
    boom.seat_pitch                         = 0.0 
    boom.fineness.nose                      = 0.950   
    boom.fineness.tail                      = 1.029   
    boom.lengths.nose                       = 0.2 
    boom.lengths.tail                       = 0.2
    boom.lengths.cabin                      = 4.15
    boom.lengths.total                      = 4.2
    boom.width                              = 0.15 
    boom.heights.maximum                    = 0.15  
    boom.heights.at_quarter_length          = 0.15  
    boom.heights.at_three_quarters_length   = 0.15 
    boom.heights.at_wing_root_quarter_chord = 0.15 
    boom.areas.wetted                       = 0.018
    boom.areas.front_projected              = 0.018 
    boom.effective_diameter                 = 0.15  
    boom.differential_pressure              = 0.  
    boom.symmetric                          = True 
    boom.index                              = 1
    
    # Segment  
    segment                           = RCAIDE.Library.Components.Booms.Segments.Segment()
    segment.tag                       = 'segment_1'   
    segment.percent_x_location        = 0.
    segment.percent_z_location        = 0.0 
    segment.height                    = 0.05  
    segment.width                     = 0.05   
    boom.append_segment(segment)           
    
    # Segment                                   
    segment                           = RCAIDE.Library.Components.Booms.Segments.Segment()
    segment.tag                       = 'segment_2'   
    segment.percent_x_location        = 0.03
    segment.percent_z_location        = 0. 
    segment.height                    = 0.15 
    segment.width                     = 0.15 
    boom.append_segment(segment) 
    
    # Segment                                   
    segment                           = RCAIDE.Library.Components.Booms.Segments.Segment()
    segment.tag                       = 'segment_3'    
    segment.percent_x_location        = 0.97
    segment.percent_z_location        = 0. 
    segment.height                    = 0.15
    segment.width                     = 0.15
    boom.append_segment(segment)           
    
    # Segment                                  
    segment                           = RCAIDE.Library.Components.Booms.Segments.Segment()
    segment.tag                       = 'segment_4'   
    segment.percent_x_location        = 1.   
    segment.percent_z_location        = 0.   
    segment.height                    = 0.05   
    segment.width                     = 0.05   
    boom.append_segment(segment)           
    
    # add to vehicle
    vehicle.append_component(boom)   
    
    # add left long boom 
    boom              = deepcopy(vehicle.booms.boom_1r)
    boom.origin[0][1] = -boom.origin[0][1]
    boom.tag          = 'boom_1l' 
    vehicle.append_component(boom)         
     
    # add left long boom 
    boom              = deepcopy(vehicle.booms.boom_1r)
    boom.origin       = [[     0.110,    2.25,   1.050]] 
    boom.tag          = 'boom_2r' 
    boom.lengths.total                      = 4.16
    vehicle.append_component(boom)  
     
    # add inner left boom 
    boom              = deepcopy(vehicle.booms.boom_1r)
    boom.origin       = [[     0.110, - 5.7 ,    1.050 ]]   
    boom.lengths.total                      = 4.16
    boom.tag          = 'boom_2l' 
    vehicle.append_component(boom)
    

    # add left long boom 
    boom              = deepcopy(vehicle.booms.boom_1r)
    boom.origin       = [[     0.110, 9.2,   1.050]] 
    boom.tag          = 'boom_3r' 
    boom.lengths.total                      = 4.16
    vehicle.append_component(boom)  
     
    # add inner left boom 
    boom              = deepcopy(vehicle.booms.boom_1r)
    boom.origin       = [[     0.110, -  9.2,    1.050 ]]   
    boom.lengths.total                      = 4.16
    boom.tag          = 'boom_3l' 
    vehicle.append_component(boom)    
    
    
    
    #------------------------------------------------------------------------------------------------------------------------------------
    # ########################################################  Energy Network  ######################################################### 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    network                                                = RCAIDE.Framework.Networks.Electric()   
    #==================================================================================================================================== 
    # Tilt Rotor Bus 
    #====================================================================================================================================          
    prop_rotor_bus                           = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    prop_rotor_bus.tag                       = 'prop_rotor_bus'
    prop_rotor_bus.origin                    =  [[2.43775609, 0 , 1.2]]
    prop_rotor_bus.number_of_battery_modules =  2    

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus Battery
    #------------------------------------------------------------------------------------------------------------------------------------ 
    battery_module                                                    = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC() 
    battery_module.tag                                                = 'bus_battery'
    battery_module.electrical_configuration.series                    = 140  
    battery_module.electrical_configuration.parallel                  = 30  
    battery_module.geometrtic_configuration.normal_count              = 168
    battery_module.geometrtic_configuration.parallel_count            = 25
     
    modules_origins = [[0.25 , 0.0, 0.0],[1.5 , 0.0, 0.0]]  # large prop-rotor modules are beneath floor
    for m_i in range(prop_rotor_bus.number_of_battery_modules):
        module =  deepcopy(battery_module)
        module.tag = 'nmc_module_' + str(m_i+1) 
        module.origin = [modules_origins[m_i]]
        prop_rotor_bus.battery_modules.append(module) 
    prop_rotor_bus.initialize_bus_properties()

    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Front Propulsors 
    #------------------------------------------------------------------------------------------------------------------------------------    
     
    # Define Lift Propulsor Container 
    reference_propulsor                                = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
    reference_propulsor.tag                            = 'reference_propulsor'       
              
    # Electronic Speed Controller           
    prop_rotor_esc                                = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller()
    prop_rotor_esc.efficiency                     = 0.95    
    prop_rotor_esc.tag                            = 'prop_rotor_esc_1'  
    prop_rotor_esc.bus_voltage                    = prop_rotor_bus.voltage
    reference_propulsor.electronic_speed_controller         = prop_rotor_esc  
    
    # Lift Rotor Design
    g                                             = 9.81                                    # gravitational acceleration   
    Hover_Load                                    = vehicle.mass_properties.max_takeoff*g *1.1  # hover load   

    prop_rotor                                    = RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor()   
    prop_rotor.tag                                = 'prop_rotor'   
    prop_rotor.tip_radius                         = 1.51
    prop_rotor.hub_radius                         = 0.15 * prop_rotor.tip_radius
    prop_rotor.number_of_blades                   = 4

    prop_rotor.hover.design_altitude              = 40 * Units.feet  
    prop_rotor.hover.design_thrust                = Hover_Load/12
    prop_rotor.hover.design_freestream_velocity   = np.sqrt(prop_rotor.hover.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))
    
    prop_rotor.oei.design_altitude                = 40 * Units.feet  
    prop_rotor.oei.design_thrust                  = Hover_Load/11 
    prop_rotor.oei.design_freestream_velocity     = np.sqrt(prop_rotor.oei.design_thrust/(2*1.2*np.pi*(prop_rotor.tip_radius**2)))
    
    prop_rotor.cruise.design_altitude             = 1500 * Units.feet
    prop_rotor.cruise.design_thrust               = 3150 / 6
    prop_rotor.cruise.design_freestream_velocity  = 130.* Units['mph']      
    
    airfoil                                       = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file                       =  airfoil_path + 'Airfoils' + separator + 'NACA_4412.txt'
    airfoil.polar_files                           = [airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_50000.txt' ,
                                                     airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_100000.txt' ,
                                                     airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_200000.txt' ,
                                                     airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_500000.txt' ,
                                                     airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_1000000.txt',
                                                     airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_3500000.txt',
                                                     airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_5000000.txt',
                                                     airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_7500000.txt' ]
    prop_rotor.append_airfoil(airfoil)                
    prop_rotor.airfoil_polar_stations             = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]    
    #prop_rotor.fidelity                           = "Momentum_Theory"  
    reference_propulsor.rotor                     =  prop_rotor  
    
    #------------------------------------------------------------------------------------------------------------------------------------               
    # Front Rotor Motor  
    #------------------------------------------------------------------------------------------------------------------------------------    
    prop_rotor_motor                         = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    prop_rotor_motor.efficiency              = 0.95
    prop_rotor_motor.nominal_voltage         = prop_rotor_bus.voltage * 0.75 
    prop_rotor_motor.no_load_current         = 0.01      
    reference_propulsor.motor                = prop_rotor_motor
     

    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Nacelle
    #------------------------------------------------------------------------------------------------------------------------------------     
    nacelle                           = RCAIDE.Library.Components.Nacelles.Nacelle() 
    nacelle.length                    = 0.45
    nacelle.diameter                  = 0.3 
    nacelle.orientation_euler_angles  = [0,-90*Units.degrees,0.]   
    nacelle.flow_through              = False  
    reference_propulsor.nacelle       =  nacelle  
     

    if redesign_rotors:
        design_electric_rotor(reference_propulsor)
        save_propulsor(reference_propulsor, os.path.join(local_path, 'lift_rotor_propulsor.res'))
    else:
        regression_prop_rotor_propulsor = deepcopy(reference_propulsor)        
        design_electric_rotor(regression_prop_rotor_propulsor, iterations=2)
        loaded_propulsor = load_propulsor(os.path.join(local_path, 'lift_rotor_propulsor.res'))

        for key,item in reference_propulsor.rotor.items():
            reference_propulsor.rotor[key] = loaded_propulsor.rotor[key] 
        for key,item in reference_propulsor.motor.items():
            reference_propulsor.motor[key] = loaded_propulsor.motor[key]  

    # Front Rotors Locations 
    origins =  [[-0.073, -2.25 ,1.2],  [-0.073, 2.25 ,1.2] , [ -0.073 , -5.7  , 1.2] ,[ -0.073 ,  5.7, 1.2], [ -0.073 , -9.2  , 1.2] ,[ -0.073 , 9.2, 1.2]]
    
    assigned_propulsor_list = []
    for i in range(len(origins)): 
        propulsor_i                                       = deepcopy(reference_propulsor)
        propulsor_i.tag                                   = 'prop_rotor_propulsor_' + str(i + 1)
        propulsor_i.rotor.tag                             = 'prop_rotor_' + str(i + 1) 
        propulsor_i.rotor.origin                          = [origins[i]]  
        propulsor_i.motor.tag                             = 'prop_rotor_motor_' + str(i + 1)   
        propulsor_i.motor.origin                          = [origins[i]]  
        propulsor_i.electronic_speed_controller.tag       = 'prop_rotor_esc_' + str(i + 1)  
        propulsor_i.electronic_speed_controller.origin    = [origins[i]]  
        propulsor_i.nacelle.tag                           = 'prop_rotor_nacelle_' + str(i + 1)  
        propulsor_i.nacelle.origin                        = [origins[i]]   
        network.propulsors.append(propulsor_i) 
        assigned_propulsor_list.append(propulsor_i.tag) 
    prop_rotor_bus.assigned_propulsors = [assigned_propulsor_list]

    # append forward bus
    network.busses.append(prop_rotor_bus)    
    
        
    #==================================================================================================================================== 
    # Rear Bus 
    #====================================================================================================================================          
    lift_rotor_bus                           = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    lift_rotor_bus.tag                       = 'lift_rotor_bus'  
    lift_rotor_bus.number_of_battery_modules =  2
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Bus Battery
    #------------------------------------------------------------------------------------------------------------------------------------ 
    battery_module                                                    = RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC() 
    battery_module.tag                                                = 'lift_bus_battery'
    battery_module.electrical_configuration.series                    = 140   
    battery_module.electrical_configuration.parallel                  = 10  
    battery_module.geometrtic_configuration.normal_count              = 56
    battery_module.geometrtic_configuration.parallel_count            = 25

    modules_origins = [[3, 0.0, 0.0],[3, 0.0, 0.2 ]]  # rear modules are stacked inside cabin
    for m_i in range(lift_rotor_bus.number_of_battery_modules):
        module =  deepcopy(battery_module)
        module.tag = 'nmc_module_' + str(m_i+1) 
        module.origin = [modules_origins[m_i]]
        lift_rotor_bus.battery_modules.append(module) 
    lift_rotor_bus.initialize_bus_properties()
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Lift Propulsors 
    #------------------------------------------------------------------------------------------------------------------------------------    
     
    # Define Lift Propulsor Container 
    lift_propulsor                                        = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()  
              
    # Electronic Speed Controller           
    lift_rotor_esc                                         = RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller() 
    lift_rotor_esc.efficiency                              = 0.95   
    lift_rotor_esc.bus_voltage                             = lift_rotor_bus.voltage
    lift_propulsor.electronic_speed_controller             = lift_rotor_esc 
           
    # Lift Rotor Design              
    lift_rotor                                             = RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor()   
    lift_rotor.tip_radius                                  = 1.51
    lift_rotor.hub_radius                                  = 0.15* lift_rotor.tip_radius 
    lift_rotor.number_of_blades                            = 3

    lift_rotor.hover.design_altitude                       = 40 * Units.feet  
    lift_rotor.hover.design_thrust                         = Hover_Load/12
    lift_rotor.hover.design_freestream_velocity            = np.sqrt(lift_rotor.hover.design_thrust/(2*1.2*np.pi*(lift_rotor.tip_radius**2)))
               
    lift_rotor.oei.design_altitude                         = 40 * Units.feet  
    lift_rotor.oei.design_thrust                           = Hover_Load/11
    lift_rotor.oei.design_freestream_velocity              = np.sqrt(lift_rotor.oei.design_thrust/(2*1.2*np.pi*(lift_rotor.tip_radius**2)))
     
    airfoil                                                = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil.coordinate_file                                = airfoil_path + 'Airfoils' + separator + 'NACA_4412.txt'
    airfoil.polar_files                                    = [airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_50000.txt' ,
                                                             airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_100000.txt' ,
                                                              airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_200000.txt' ,
                                                              airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_500000.txt' ,
                                                              airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_1000000.txt',
                                                              airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_3500000.txt',
                                                              airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_5000000.txt',
                                                              airfoil_path + 'Airfoils' + separator + 'Polars' + separator + 'NACA_4412_polar_Re_7500000.txt' ]
    lift_rotor.append_airfoil(airfoil)                         
    lift_rotor.airfoil_polar_stations                      = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]   
    lift_propulsor.rotor =  lift_rotor          
    
    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Motor  
    #------------------------------------------------------------------------------------------------------------------------------------    
    lift_rotor_motor                                       = RCAIDE.Library.Components.Powertrain.Converters.DC_Motor()
    lift_rotor_motor.efficiency                            = 0.9
    lift_rotor_motor.nominal_voltage                       = lift_rotor_bus.voltage*3/4   
    lift_rotor_motor.no_load_current                       = 0.01      
    lift_propulsor.motor                                   = lift_rotor_motor 

    #------------------------------------------------------------------------------------------------------------------------------------               
    # Lift Rotor Nacelle
    #------------------------------------------------------------------------------------------------------------------------------------     
    nacelle                           = RCAIDE.Library.Components.Nacelles.Nacelle() 
    nacelle.length                    = 0.45
    nacelle.diameter                  = 0.3
    nacelle.orientation_euler_angles  = [0,-90*Units.degrees,0.]    
    nacelle.flow_through              = False     
    lift_propulsor.nacelle            =  nacelle  

    if redesign_rotors:
        design_electric_rotor(lift_propulsor)
        save_propulsor(lift_propulsor, os.path.join(local_path, 'lift_rotor_propulsor.res'))
    else:
        regression_prop_rotor_propulsor = deepcopy(lift_propulsor)        
        design_electric_rotor(regression_prop_rotor_propulsor, iterations=2)
        loaded_propulsor = load_propulsor(os.path.join(local_path, 'lift_rotor_propulsor.res'))

        for key,item in lift_propulsor.rotor.items():
            lift_propulsor.rotor[key] = loaded_propulsor.rotor[key] 
        for key,item in lift_propulsor.motor.items():
            lift_propulsor.motor[key] = loaded_propulsor.motor[key]  
 
    # Front Rotors Locations
    origins =  [[ 4.196, -2.25 ,1.2],  [ 4.196, 2.25 ,1.2] , [ 4.196, -5.7  , 1.2] ,[ 4.196,  5.7, 1.2], [ 4.196, -9.2  , 1.2] ,[  4.196 , 9.2, 1.2]] 
    orientation_euler_angles = [[0,-90*Units.degrees,0.] ,[0,-90*Units.degrees,0.]  ,[0,-90*Units.degrees,0.] , [0,-90*Units.degrees,0.] ,[0,-90*Units.degrees,0.] , [0,-90*Units.degrees,0.]  ]  
    
    assigned_propulsor_list = []
    for i in range(len(origins)): 
        propulsor_i                                       = deepcopy(lift_propulsor)
        propulsor_i.tag                                   = 'lift_rotor_propulsor_' + str(i + 1)
        propulsor_i.rotor.tag                             = 'lift_rotor_' + str(i + 1) 
        propulsor_i.rotor.origin                          = [origins[i]] 
        propulsor_i.rotor.orientation_euler_angle         = orientation_euler_angles[i]
        propulsor_i.motor.tag                             = 'lift_rotor_motor_' + str(i + 1)   
        propulsor_i.motor.origin                          = [origins[i]]  
        propulsor_i.electronic_speed_controller.tag       = 'lift_rotor_esc_' + str(i + 1)  
        propulsor_i.electronic_speed_controller.origin    = [origins[i]]  
        propulsor_i.nacelle.tag                           = 'lift_rotor_nacelle_' + str(i + 1)  
        propulsor_i.nacelle.origin                        = [origins[i]]    
        network.propulsors.append(propulsor_i)  
        assigned_propulsor_list.append(propulsor_i.tag) 
    lift_rotor_bus.assigned_propulsors = [assigned_propulsor_list]

    #------------------------------------------------------------------------------------------------------------------------------------  
    # Additional Bus Loads

    # Avionics                            
    avionics                                                = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
    avionics.power_draw                                     = 10. # Watts  
    avionics.mass_properties.mass                           = 1.0 * Units.kg
    lift_rotor_bus.avionics                                       = avionics    

   
    network.busses.append(lift_rotor_bus)       
        
    # append energy network 
    vehicle.append_energy_network(network)
 
    return vehicle

def configs_setup(vehicle):
    '''
    The configration set up below the scheduling of the nacelle angle and vehicle speed.
    Since one prop_rotor operates at varying flight conditions, one must perscribe  the 
    pitch command of the prop_rotor which us used in the variable pitch model in the analyses
    Note: low pitch at take off & low speeds, high pitch at cruise
    '''
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    configs = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config                                                       = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag                                                   = 'base'     
    configs.append(base_config) 
 
    # ------------------------------------------------------------------
    #   Hover Climb Configuration
    # ------------------------------------------------------------------
    config                                                 = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag                                             = 'vertical_flight'
    vector_angle                                           = 90.0 * Units.degrees    
    bus = config.networks.electric.busses.prop_rotor_bus  
    for prop_tag in  bus.assigned_propulsors[0]: 
        propulsor = config.networks.electric.propulsors[prop_tag]
        propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
    
    configs.append(config)

    # ------------------------------------------------------------------
    #    
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    vector_angle                                      = 75.0  * Units.degrees   
    config.tag                                        = 'vertical_transition'
    bus = config.networks.electric.busses.prop_rotor_bus  
    for prop_tag in  bus.assigned_propulsors[0]: 
        propulsor = config.networks.electric.propulsors[prop_tag]
        propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
        propulsor.rotor.blade_pitch_command   = propulsor.rotor.hover.design_blade_pitch_command * 0.5 
    configs.append(config) 

    # ------------------------------------------------------------------
    #   Hover-to-Cruise Configuration
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag                                        = 'low_speed_climb_transition'
    vector_angle                                      = 85.0  * Units.degrees   
    bus = config.networks.electric.busses.prop_rotor_bus  
    for prop_tag in  bus.assigned_propulsors[0]: 
        propulsor = config.networks.electric.propulsors[prop_tag]
        propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
        propulsor.rotor.blade_pitch_command   =  propulsor.rotor.cruise.design_blade_pitch_command
    configs.append(config) 


    # ------------------------------------------------------------------
    #   Hover-to-Cruise Configuration
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag                                        = 'high_speed_climb_transition'
    vector_angle                                      =  45.0  * Units.degrees   
    bus = config.networks.electric.busses.prop_rotor_bus  
    for prop_tag in  bus.assigned_propulsors[0]: 
        propulsor = config.networks.electric.propulsors[prop_tag]
        propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
        propulsor.rotor.blade_pitch_command   =  propulsor.rotor.cruise.design_blade_pitch_command * 0.5
    configs.append(config) 

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag                                        = 'forward_flight'   
    vector_angle                                      = 0.0 * Units.degrees   
    config.networks.electric.busses.lift_rotor_bus.active  = False   
    bus = config.networks.electric.busses.prop_rotor_bus  
    for prop_tag in  bus.assigned_propulsors[0]: 
        propulsor = config.networks.electric.propulsors[prop_tag]
        propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
        propulsor.rotor.blade_pitch_command   = propulsor.rotor.cruise.design_blade_pitch_command  
    configs.append(config)     
    
    # ------------------------------------------------------------------
    #   
    # ------------------------------------------------------------------ 
    config                                                 = RCAIDE.Library.Components.Configs.Config(vehicle)
    vector_angle                                           = 75.0  * Units.degrees   
    config.tag                                             = 'descent_transition'  
    bus = config.networks.electric.busses.prop_rotor_bus  
    for prop_tag in  bus.assigned_propulsors[0]: 
        propulsor = config.networks.electric.propulsors[prop_tag]
        propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
        propulsor.rotor.blade_pitch_command   =propulsor.rotor.cruise.design_blade_pitch_command *0.5  
    configs.append(config)  


    # ------------------------------------------------------------------
    #   Approach Configuration
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag                                        = 'approach_transition'   
    vector_angle                                      = 75.0  * Units.degrees  
    bus = config.networks.electric.busses.prop_rotor_bus  
    for prop_tag in  bus.assigned_propulsors[0]: 
        propulsor = config.networks.electric.propulsors[prop_tag]
        propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
        propulsor.rotor.blade_pitch_command   =propulsor.rotor.cruise.design_blade_pitch_command *0.5 
    configs.append(config)
    
    
    # ------------------------------------------------------------------
    #   Hover Configuration
    # ------------------------------------------------------------------
    config                                            = RCAIDE.Library.Components.Configs.Config(vehicle)
    config.tag                                        = 'vertical_descent'
    vector_angle                                      = 90.0  * Units.degrees    
    bus = config.networks.electric.busses.prop_rotor_bus  
    for prop_tag in  bus.assigned_propulsors[0]: 
        propulsor = config.networks.electric.propulsors[prop_tag]
        propulsor.rotor.orientation_euler_angles =  [0, vector_angle, 0]
        propulsor.rotor.blade_pitch_command   = propulsor.rotor.hover.design_blade_pitch_command 
    configs.append(config)

    return configs
 

# ----------------------------------------------------------------------
#   Define the Mission
# ---------------------------------------------------------------------- 
def mission_setup(analyses): 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission        = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag    = 'baseline_mission' 
    
    # unpack Segments module
    Segments       = RCAIDE.Framework.Mission.Segments

    # base segment           
    base_segment   = Segments.Segment()
    
    # VSTALL Calculation  
    vehicle_mass   = analyses.base.aerodynamics.vehicle.mass_properties.max_takeoff
    reference_area = analyses.base.aerodynamics.vehicle.reference_area 
    Vstall         = 90 * Units.kts #estimate_stall_speed(vehicle_mass,reference_area,altitude = 0.0,maximum_lift_coefficient = 1.2)      
    '''
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Vertical Climb 
    #------------------------------------------------------------------------------------------------------------------------------------  
    segment        = Segments.Vertical_Flight.Climb(base_segment)
    segment.tag    = "Vertical_Climb"   
    segment.analyses.extend( analyses.vertical_flight )  
    segment.altitude_start                                = 0.0  * Units.ft  
    segment.altitude_end                                  = 50.  * Units.ft   
    segment.initial_battery_state_of_charge               = 1.0 
    segment.climb_rate                                    = 500. * Units['ft/min']   
    segment.true_course                                   = 30 * Units.degree
            
    # define flight dynamics to model  
    segment.flight_dynamics.force_z                        = True   
    segment.flight_dynamics.moment_y                       = True
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6'],
                                                                        ['lift_rotor_propulsor_1','lift_rotor_propulsor_2','lift_rotor_propulsor_3',
                                                                         'lift_rotor_propulsor_4', 'lift_rotor_propulsor_5', 'lift_rotor_propulsor_6']]
       
    mission.append_segment(segment)
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Low-Speed Transition
    #------------------------------------------------------------------------------------------------------------------------------------   

    segment                                               = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                                           = "Low_Speed_Transition"  
    segment.analyses.extend( analyses.vertical_transition)   
    segment.air_speed_end                                 = 35 * Units['mph']     
    segment.acceleration                                  = 0.5
    segment.true_course                                   = 30 * Units.degree

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                        = True  
    segment.flight_dynamics.force_z                        = True  
    segment.flight_dynamics.moment_y                       = True     
    
    # define flight controls
    segment.assigned_control_variables.body_angle.active             = True
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6'],
                                                                        ['lift_rotor_propulsor_1','lift_rotor_propulsor_2','lift_rotor_propulsor_3',
                                                                         'lift_rotor_propulsor_4', 'lift_rotor_propulsor_5', 'lift_rotor_propulsor_6']]
    mission.append_segment(segment) 
     
    #------------------------------------------------------------------------------------------------------------------------------------  
    #   First Climb
    #------------------------------------------------------------------------------------------------------------------------------------  
    segment                                               = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                                           = "Low_Altitude_Climb"   
    segment.analyses.extend( analyses.high_speed_climb_transition )   
    segment.altitude_end                                  = 500. * Units.ft   
    segment.climb_rate                                    = 628.0  * Units['ft/min']
    segment.air_speed_start                               = 10 *  Units.kts
    segment.air_speed_end                                 = 90 *  Units.kts
    segment.true_course                                   = 30 * Units.degree
            
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True 
    segment.flight_dynamics.moment_y                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.body_angle.active             = True
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6'],
                                                                        ['lift_rotor_propulsor_1','lift_rotor_propulsor_2','lift_rotor_propulsor_3',
                                                                         'lift_rotor_propulsor_4', 'lift_rotor_propulsor_5', 'lift_rotor_propulsor_6']]
                
    mission.append_segment(segment)   
  
    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Circular departure pattern 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "Departure_Pattern_Curve"    
    segment.analyses.extend( analyses.forward_flight ) 
    segment.altitude    = 500. * Units.ft
    segment.air_speed   = 90 * Units.kts 
    segment.turn_radius = 3600 * Units.feet  
    segment.true_course = 30 * Units.degree # this is the true couse of the starting value     
    segment.turn_angle  = 90 * Units.degree
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                             = True    
    segment.flight_dynamics.force_z                                             = True    
    segment.flight_dynamics.force_y                                             = True     
    segment.flight_dynamics.moment_y                                            = True 
    segment.flight_dynamics.moment_x                                            = True
    segment.flight_dynamics.moment_z                                            = True 

    # define flight controls              
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                    'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6'] ] 
    segment.assigned_control_variables.body_angle.active                        = True    
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']]   
    segment.assigned_control_variables.aileron_deflection.active                = True    
    segment.assigned_control_variables.aileron_deflection.assigned_surfaces     = [['aileron']] 
    segment.assigned_control_variables.rudder_deflection.active                 = True    
    segment.assigned_control_variables.rudder_deflection.assigned_surfaces      = [['rudder']] 
    segment.assigned_control_variables.bank_angle.active                        = True    
    segment.assigned_control_variables.bank_angle.initial_guess_values          = [[20.0 * Units.degree]]
    
    mission.append_segment(segment)  
  
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Second Climb
    #------------------------------------------------------------------------------------------------------------------------------------  
    segment                                               = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                                           = "High_Altitude_Climb"  
    segment.analyses.extend( analyses.forward_flight)   
    segment.altitude_start                                = 500.0 * Units.ft   
    segment.altitude_end                                  = 1000. * Units.ft   
    segment.climb_rate                                    = 300.  * Units['ft/min']  
    segment.air_speed_end                                 = 110.  * Units['mph']  
    segment.true_course                                   = 90 * Units.degree
 
    # define flight dynamics to model   
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6'] ] 
    segment.assigned_control_variables.body_angle.active             = True                
                 
    mission.append_segment(segment)  
    '''
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Cruise 
    #------------------------------------------------------------------------------------------------------------------------------------  
    segment                                                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                                      = "Cruise"  
    segment.analyses.extend( analyses.forward_flight )                             
    segment.altitude                                                 = 1000.0 * Units.ft  
    segment.air_speed                                                = 110.  * Units['mph']  
    segment.distance                                                 = 40 *Units.nmi 
    segment.true_course                                              = 30 * Units.degree
                                                                     
    # define flight dynamics to model                                
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6'] ] 
    segment.assigned_control_variables.body_angle.active             = True                
         
    mission.append_segment(segment)   
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Descent
    #------------------------------------------------------------------------------------------------------------------------------------   
    segment                                               = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                                           = "Descent"  
    segment.analyses.extend(analyses.forward_flight)  
    segment.altitude_start                                = 1000.0 * Units.ft  
    segment.altitude_end                                  = 500. * Units.ft  
    segment.climb_rate                                    = -500.  * Units['ft/min'] 
    segment.air_speed_end                                 = 90 * Units.kts
    segment.true_course                                   = 30 * Units.degree
            
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6'] ] 
    segment.assigned_control_variables.body_angle.active             = True                
       
    mission.append_segment(segment)  
     
    '''
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Circular approach pattern 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "Approach_Pattern_Curve"      
    segment.analyses.extend( analyses.forward_flight )   
    segment.altitude    = 500. * Units.ft
    segment.air_speed   = 90 * Units.kts 
    segment.turn_radius = 3600 * Units.feet  
    segment.true_course = 30 * Units.degree # this is the true couse of the starting value     
    segment.turn_angle  = 90 * Units.degree
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                             = True    
    segment.flight_dynamics.force_z                                             = True    
    segment.flight_dynamics.force_y                                             = True     
    segment.flight_dynamics.moment_y                                            = True 
    segment.flight_dynamics.moment_x                                            = True
    segment.flight_dynamics.moment_z                                            = True 

    # define flight controls              
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                    'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6'] ] 
    segment.assigned_control_variables.body_angle.active                        = True    
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']]   
    segment.assigned_control_variables.aileron_deflection.active                = True    
    segment.assigned_control_variables.aileron_deflection.assigned_surfaces     = [['aileron']] 
    segment.assigned_control_variables.rudder_deflection.active                 = True    
    segment.assigned_control_variables.rudder_deflection.assigned_surfaces      = [['rudder']] 
    segment.assigned_control_variables.bank_angle.active                        = True    
    segment.assigned_control_variables.bank_angle.initial_guess_values          = [[20.0 * Units.degree]]
    
    mission.append_segment(segment)  
      
                  
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # High-Speed Descending Transition
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                               = Segments.Climb.Constant_Acceleration_Constant_Pitchrate_Constant_Angle(base_segment)
    segment.tag                                           = "High_Speed_Descending_Transition"  
    segment.analyses.extend( analyses.high_speed_climb_transition ) 
    segment.altitude_start                                = 500.0 * Units.ft   
    segment.altitude_end                                  = 50.0 * Units.ft   
    segment.climb_angle                                   = 7.125 * Units.degrees
    segment.acceleration                                  = 0.9574 * Units['m/s/s']    
    segment.pitch_initial                                 = 4.3  * Units.degrees  
    segment.pitch_final                                   = 7. * Units.degrees     
    segment.true_course                                   = 30 * Units.degree

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                        = True  
    segment.flight_dynamics.force_z                        = True  
    segment.flight_dynamics.moment_y                       = True     
    
    # define flight controls
    segment.assigned_control_variables.body_angle.active             = True
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6'],
                                                                        ['lift_rotor_propulsor_1','lift_rotor_propulsor_2','lift_rotor_propulsor_3',
                                                                         'lift_rotor_propulsor_4', 'lift_rotor_propulsor_5', 'lift_rotor_propulsor_6']]
    mission.append_segment(segment)   
    
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Vertical Descent 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                               = Segments.Vertical_Flight.Descent(base_segment)
    segment.tag                                           = "Vertical_Descent" 
    segment.analyses.extend( analyses.vertical_flight)     
    segment.altitude_start                                = 50.0 * Units.ft   
    segment.altitude_end                                  = 0.   * Units.ft  
    segment.descent_rate                                  = 300. * Units['ft/min']
    segment.true_course                                   = 30 * Units.degree
    
    # define flight dynamics to model  
    segment.flight_dynamics.force_z                        = True   
    segment.flight_dynamics.moment_y                       = True  
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6'],
                                                                        ['lift_rotor_propulsor_1','lift_rotor_propulsor_2','lift_rotor_propulsor_3',
                                                                         'lift_rotor_propulsor_4', 'lift_rotor_propulsor_5', 'lift_rotor_propulsor_6']]
       
    mission.append_segment(segment)  
    '''
   
    return mission 


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def complete_mission_setup(analyses ):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment() 
    base_segment.state.numerics.solver.type = 'optimize' #'root_finder' 
    base_segment.state.numerics.number_of_control_points = 6
    #base_segment.state.numerics.solver.max_evaluations = 400
    # # ------------------------------------------------------------------
    # #   First Climb Segment: Constant Speed, Constant Rate
    # # ------------------------------------------------------------------ 
    # segment                                            = Segments.Vertical_Flight.Climb(base_segment)
    # segment.tag                                        = "Vertical_Climb"   
    # segment.analyses.extend(analyses.vertical_climb) 
    # segment.altitude_start                             = 0.0  * Units.ft  
    # segment.altitude_end                               = 50.  * Units.ft   
    # segment.climb_rate                                 = 500. * Units['ft/min'] 
    # segment.initial_battery_state_of_charge            = 1.0 
    # segment.true_course                                = 0   * Units.degree # this is the true couse of the starting value  

    # # define flight dynamics to model  
    # segment.flight_dynamics.force_z                    = True 

    # # define flight controls  
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
    #                                                                      'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    
    # mission.append_segment(segment)    
    # # ------------------------------------------------------------------
    # #  First Transition Segment
    # # ------------------------------------------------------------------ 
    # segment                                               = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    # segment.tag                                           = "Vertical_Transition"  
    # segment.analyses.extend( analyses.vertical_transition)   
    # segment.air_speed_end                                 = 35 * Units['mph']     
    # segment.acceleration                                  = 1.0
    # segment.true_course                                   = 0 * Units.degree  

    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
    #                                                                      'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    # segment.assigned_control_variables.body_angle.active             = True 
    
    # mission.append_segment(segment)
    
    # # ------------------------------------------------------------------
    # #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # # ------------------------------------------------------------------ 
    # segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    # segment.tag                      = "low_speed_climb_transition" 
    # segment.analyses.extend(analyses.low_speed_climb_transition) 
    # segment.climb_rate               = 822. * Units['ft/min'] 
    # segment.air_speed_end            = 100 * Units['mph']   
    # segment.altitude_end             = 500.0 * Units.ft
    # segment.true_course              = 0 * Units.degree  

    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
    #                                                                      'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    # segment.assigned_control_variables.body_angle.active             = True 
                                                                             
    # mission.append_segment(segment)   
    
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    # Circular departure pattern 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    # segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    # segment.tag                                           = "Departure_Pattern_Curve"   
    # segment.analyses.extend( analyses.high_speed_climb_transition )           
    # #segment.state.numerics.solver.type = 'optimize' 
    # segment.air_speed   = 90 * Units['knots']  
    # segment.turn_radius = 3600 * Units.feet  
    # segment.true_course = 0 * Units.degree     
    # segment.turn_angle  = 90 * Units.degree
    # segment.altitude    = 500 * Units.feet
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                                             = True    
    # segment.flight_dynamics.force_z                                             = True    
    # segment.flight_dynamics.force_y                                             = True     
    # segment.flight_dynamics.moment_y                                            = True 
    # segment.flight_dynamics.moment_x                                            = True
    # segment.flight_dynamics.moment_z                                            = True 

    # # define flight controls              
    # segment.assigned_control_variables.throttle.active                          = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors             = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]   
    # segment.assigned_control_variables.body_angle.active                        = True    
    # segment.assigned_control_variables.elevator_deflection.active               = True    
    # segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']] 
    # segment.assigned_control_variables.elevator_deflection.initial_guess_values = [[-4.4 * Units.degree]]
    # segment.assigned_control_variables.aileron_deflection.active                = True    
    # segment.assigned_control_variables.aileron_deflection.assigned_surfaces     = [['aileron']] 
    # segment.assigned_control_variables.rudder_deflection.active                 = True    
    # segment.assigned_control_variables.rudder_deflection.assigned_surfaces      = [['rudder']] 
    # segment.assigned_control_variables.bank_angle.active                        = True    
    # segment.assigned_control_variables.bank_angle.initial_guess_values          = [[20.0 * Units.degree]]
    
    # mission.append_segment(segment)  
     
    # # ------------------------------------------------------------------
    # #  Second Transition Segment
    # # ------------------------------------------------------------------ 
    # segment                           = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    # segment.tag                       = "high_speed_climb_transition"  
    # segment.analyses.extend( analyses.high_speed_climb_transition)   
    # segment.air_speed_end             = 125.  * Units['mph']  
    # segment.acceleration              = 9.81/5 
    # segment.true_course               = 90 * Units.degree  

    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
    #                                                                      'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    # segment.assigned_control_variables.body_angle.active             = True
    # mission.append_segment(segment)
     
    # # ------------------------------------------------------------------
    # #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # # ------------------------------------------------------------------ 
    # segment                           = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    # segment.tag                       = "Climb"  
    # segment.analyses.extend(analyses.cruise) 
    # segment.climb_rate                = 500. * Units['ft/min']
    # segment.air_speed_start           = 90.   * Units.kts 
    # segment.air_speed_end             = 125.  * Units['mph']   
    # segment.altitude_end              = 1000.0 * Units.ft 
    # segment.true_course               = 90 * Units.degree  
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  =[['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
    #                                                                      'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]   
        
    # segment.assigned_control_variables.body_angle.active             = True
    
    # mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   First Cruise Segment: Constant Acceleration, Constant Altitude
    # ------------------------------------------------------------------ 
    segment                          = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                      = "Cruise"  
    segment.analyses.extend(analyses.forward_flight) 
    segment.altitude                 = 1000.0 * Units.ft
    segment.air_speed                = 170.  * Units['mph']   
    segment.distance                 = 90*Units.nmi
    segment.true_course              = 90 * Units.degree  
    #segment.sideslip_angle          = 5.0 * Units.degree
    segment.initial_battery_state_of_charge            = 1.0 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                             = True    
    segment.flight_dynamics.force_z                                             = True    
    # segment.flight_dynamics.force_y                                             = True     
    # segment.flight_dynamics.moment_y                                            = True 
    # segment.flight_dynamics.moment_x                                            = True
    # segment.flight_dynamics.moment_z                                            = True 

    # define flight controls              
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                         'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6'] ] 
    segment.assigned_control_variables.body_angle.active                        = True    
    # segment.assigned_control_variables.elevator_deflection.active               = True    
    # segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']] 
    # segment.assigned_control_variables.aileron_deflection.active                = True    
    # segment.assigned_control_variables.aileron_deflection.assigned_surfaces     = [['aileron']] 
    # segment.assigned_control_variables.rudder_deflection.active                 = True    
    # segment.assigned_control_variables.rudder_deflection.assigned_surfaces      = [['rudder']] 
    # segment.assigned_control_variables.bank_angle.active                        = True    
    mission.append_segment(segment)     
    
    # # ------------------------------------------------------------------
    # #    Descent Segment: Constant Acceleration, Constant Altitude
    # # ------------------------------------------------------------------ 
    # segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    # segment.tag                      = "Descent"  
    # segment.analyses.extend(analyses.cruise)
    # segment.climb_rate               = -300. * Units['ft/min']
    # segment.air_speed_start          = 125.  * Units['mph'] 
    # segment.air_speed_end            = 90 * Units.kts 
    # segment.altitude_start           = 1000.0 * Units.ft
    # segment.altitude_end             = 500.0 * Units.ft
    # segment.true_course              = 90 * Units.degree  

    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                       = True  
    # segment.flight_dynamics.force_z                       = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
    #                                                                      'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    # segment.assigned_control_variables.body_angle.active             = True
    
        
    # mission.append_segment(segment)      
    
    # #------------------------------------------------------------------------------------------------------------------------------------ 
    # # Circular departure pattern 
    # #------------------------------------------------------------------------------------------------------------------------------------ 
    # segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    # segment.tag                                           = "Approach_Pattern_Curve"   
    # segment.analyses.extend( analyses.high_speed_climb_transition )             
    # segment.air_speed   = 90 * Units.kts 
    # segment.turn_radius = 3600 * Units.feet  
    # segment.true_course = 90 * Units.degree     
    # segment.turn_angle  = 90 * Units.degree
    # segment.altitude    = 500 * Units.feet
    
    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x                                             = True    
    # segment.flight_dynamics.force_z                                             = True    
    # segment.flight_dynamics.force_y                                             = True     
    # segment.flight_dynamics.moment_y                                            = True 
    # segment.flight_dynamics.moment_x                                            = True
    # segment.flight_dynamics.moment_z                                            = True 

    # # define flight controls              
    # segment.assigned_control_variables.throttle.active                          = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors             = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
    #                                                                                 'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    # segment.assigned_control_variables.body_angle.active                        = True    
    # segment.assigned_control_variables.elevator_deflection.active               = True    
    # segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']]   
    # segment.assigned_control_variables.aileron_deflection.active                = True    
    # segment.assigned_control_variables.aileron_deflection.assigned_surfaces     = [['aileron']] 
    # segment.assigned_control_variables.rudder_deflection.active                 = True    
    # segment.assigned_control_variables.rudder_deflection.assigned_surfaces      = [['rudder']] 
    # segment.assigned_control_variables.bank_angle.active                        = True    
    # segment.assigned_control_variables.bank_angle.initial_guess_values          = [[20.0 * Units.degree]]
    
    # mission.append_segment(segment)  
     
    # # ------------------------------------------------------------------
    # #  Forth Transition Segment
    # # ------------------------------------------------------------------ 
    # segment                          = Segments.Descent.Linear_Speed_Constant_Rate(base_segment)
    # segment.tag                      = "Approach_Transition"   
    # segment.analyses.extend(analyses.approach_transition)  
    # segment.descent_rate             = 200.  * Units['ft/min'] 
    # segment.air_speed_start          = 90 * Units.kts 
    # segment.air_speed_end            = 300. * Units['ft/min']  
    # segment.altitude_start           = 500 * Units.feet
    # segment.altitude_end             = 50.0 * Units.ft
    # segment.true_course              = 180 * Units.degree  

    # # define flight dynamics to model 
    # segment.flight_dynamics.force_x  = True  
    # segment.flight_dynamics.force_z  = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
    #                                                                      'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    # segment.assigned_control_variables.body_angle.active             = True
        
    # mission.append_segment(segment)
    
    # #------------------------------------------------------------------------------------------------------------------------------------ 
    # # Vertical Descent 
    # #------------------------------------------------------------------------------------------------------------------------------------ 
    # segment                                                         = Segments.Vertical_Flight.Descent(base_segment)
    # segment.tag                                                     = "Vertical_Descent" 
    # segment.analyses.extend( analyses.vertical_descent)                
    # segment.altitude_end                                            = 0.   * Units.ft  
    # segment.descent_rate                                            = 300. * Units['ft/min'] 
    # segment.true_course                                             = 180 * Units.degree  
                  
    # # define flight dynamics to model              
    # segment.flight_dynamics.force_z                                  = True     
    
    # # define flight controls 
    # segment.assigned_control_variables.throttle.active               = True           
    # segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
    #                                                                      'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
            
    # mission.append_segment(segment)      
    
    return mission

def missions_setup(mission): 
 
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions 


def plot_results(results): 
    # Plots fligh conditions 
    plot_flight_conditions(results) 
    
    # Plot arcraft trajectory
    plot_flight_trajectory(results)
    
    # Plot Aerodynamic Coefficients
    plot_aerodynamic_coefficients(results)  
     
    # Plot Aircraft Stability
    plot_longitudinal_stability(results) 
    
    # Plot Aircraft Electronics 
    plot_battery_temperature(results)
    plot_battery_cell_conditions(results) 
    plot_battery_degradation(results) 
    plot_electric_propulsor_efficiencies(results) 
    
    # Plot Propeller Conditions 
    plot_rotor_conditions(results) 
    plot_disc_and_power_loading(results) 
    plot_lateral_stability(results)
      
    return

def save_aircraft_geometry(geometry,filename): 
    pickle_file  = filename + '.pkl'
    with open(pickle_file, 'wb') as file:
        pickle.dump(geometry, file) 
    return 


def load_aircraft_geometry(filename):  
    load_file = filename + '.pkl' 
    with open(load_file, 'rb') as file:
        results = pickle.load(file) 
    return results

def load_propulsor(filename):
    propulsor =  load(filename)
    return propulsor

def save_propulsor(propulsor, filename):
    save(propulsor, filename)
    return

def display_stability_derivatives(segment):
    """This function displays the stability derivatives of the aircraft."""

    # Get the stability derivatives
    stability_derivatives = segment.conditions.static_stability.derivatives

    # Display the stability derivatives
    print(f"CLift_alpha: {stability_derivatives.Clift_alpha[0,0]:.3f}")
    print(f"CY_beta: {stability_derivatives.CY_beta[0,0]:.3f}")
    print(f"CL_beta: {stability_derivatives.CL_beta[0,0]:.4f}")
    print(f"CM_alpha: {stability_derivatives.CM_alpha[0,0]:.3f}")
    print(f"CN_beta: {stability_derivatives.CN_beta[0,0]:.3f}")
    print(f"CL_p: {stability_derivatives.CL_p[0,0]:.5f}")
    print(f"CL_r: {stability_derivatives.CL_r[0,0]:.5f}")
    print(f"CM_q: {stability_derivatives.CM_q[0,0]:.5f}")
    print(f"CN_p: {stability_derivatives.CN_p[0,0]:.5f}")
    print(f"CN_r: {stability_derivatives.CN_r[0,0]:.5f}")
    print(f"CM_delta_e: {stability_derivatives.CM_delta_e[0,0]:.5f}")
    print(f"CL_delta_a: {stability_derivatives.CL_delta_a[0,0]:.5f}")
    print(f"CN_delta_a: {stability_derivatives.CN_delta_a[0,0]:.5f}")
    print(f"CN_delta_r: {stability_derivatives.CN_delta_r[0,0]:.5f}")
    return
 
if __name__ == '__main__': 
    main()    
    plt.show()