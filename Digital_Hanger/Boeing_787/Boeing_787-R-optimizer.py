# RESEARCH/Aircraft/Boeing_787.py
# 
# 
# Created:  Jan 2025, A. Molloy 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units           
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan                import design_turbofan      
from RCAIDE.Library.Methods.Performance.compute_payload_range_diagram     import compute_payload_range_diagram
from RCAIDE.Library.Methods.Performance.compute_noise_certification_data  import compute_noise_certification_data
from RCAIDE.Library.Plots                                                 import *     
import RCAIDE.Framework.External_Interfaces.OpenVSP as openvsp

# python imports 
import numpy as np  
from copy import deepcopy
import matplotlib.pyplot as plt  
import os
import sys
import pickle
from scipy import optimize



# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    wing_reference_area = 395
    fuel_weight = 57500
    engine_diameter = 2.822
    design_thrust = 55000
    MTOW = 227930
    
    # Define the objective function for optimization
    def objective_function(x):
        wing_reference_area, fuel_weight, engine_diameter, design_thrust, MTOW = x
        wing_reference_area = wing_reference_area*1000
        fuel_weight = fuel_weight*100000
        engine_diameter = engine_diameter*10
        design_thrust = design_thrust*10000
        MTOW= MTOW*10000000

        results = nominal_mission(wing_reference_area, fuel_weight, MTOW, engine_diameter, design_thrust)
        fuel_burnt = results.segments[-1].conditions.energy.cumulative_fuel_consumption[-1, 0]/100000
        return fuel_burnt  # Or another objective like range, MTOW, etc.
    
    # Define the constraint: fuel_burnt = fuel_weight*0.15
    def fuel_burn_constraint(x):
        wing_reference_area, fuel_weight, engine_diameter, design_thrust, MTOW = x
        wing_reference_area = wing_reference_area*1000
        fuel_weight = fuel_weight*100000
        engine_diameter = engine_diameter*10
        design_thrust = design_thrust*10000
        MTOW= MTOW*10000000

        results = nominal_mission(wing_reference_area, fuel_weight, MTOW, engine_diameter, design_thrust)
        fuel_burnt = results.segments[-1].conditions.energy.cumulative_fuel_consumption[-1, 0]/100000
        # For equality constraint: g(x) = 0
        return fuel_burnt - fuel_weight*0.15/100000
    
    x0 = [wing_reference_area/1000, fuel_weight/100000, engine_diameter/10, design_thrust/10000, MTOW/10000000]
    bounds = [(250/1000, 400/1000), (30000/100000, 60000/100000), (2/10, 3/10), (40000/10000, 55500/10000), (180000/10000000, (MTOW+1)/10000000)]
    
    # Define all constraints
    cons = [
        {'type': 'eq', 'fun': fuel_burn_constraint}
        # Add other constraints here as needed
    ]
    
    # Run the optimization
    result = optimize.minimize(
        objective_function,
        x0,
        method='SLSQP',
        bounds=bounds,
        constraints=cons,
        options={
            'disp': True,          # Display convergence messages
            'iprint': 2,           # Level of verbosity (2 = most verbose)
            'eps': 0.1,            # Set the step size for finite-difference approximation (default is 1e-8)
            'ftol': 1e-5,          # Precision goal for objective function (default is 1e-6)
            'maxiter': 100,        # Maximum number of iterations
            #'finite_diff_rel_step': 0.1  # Relative step size for finite difference (alternative to eps)
        }
    )
    
    
    # Extract and print results
    print("Optimization successful:", result.success)
    print("Final objective value:", result.fun)
    print("Optimal parameters:")
    print("  Wing area:", result.x[0])
    print("  Fuel weight:", result.x[1])
    print("  Engine diameter:", result.x[2])
    print("  Design thrust:", result.x[3])
    print("  MTOW:", result.x[4])
    
    return result


def nominal_mission(wing_reference_area,fuel_weight, MTOW, engine_diameter,design_thrust):
    print(wing_reference_area,fuel_weight, MTOW, engine_diameter,design_thrust)
    # Step 1 design a vehicle
    save_file_path = os.path.dirname(os.path.abspath(__file__))
    vehicle  = vehicle_setup(wing_reference_area,fuel_weight, MTOW, engine_diameter,design_thrust,save_file_path,resize_aircraft=True)

    # Step 2 create aircraft configuration based on vehicle 
    configs  = configs_setup(vehicle)

    # Step 3 set up analysis
    analyses = analyses_setup(configs)

    # Step 4 set up a flight mission
    mission = mission_setup(analyses)
    missions = missions_setup(mission) 

    # Step 5 execute flight profile
    results = missions.base_mission.evaluate()  
    print('Fuel Consumed',results.segments[-1].conditions.energy.cumulative_fuel_consumption[-1, 0])

    # # Step 6 plot results 
    # plot_mission(results)
    
    

    # # plot vehicle 
    # plot_3d_vehicle(vehicle,
    #                 min_x_axis_limit            = -0,
    #                 max_x_axis_limit            = 60,
    #                 min_y_axis_limit            = -35,
    #                 max_y_axis_limit            = 35,
    #                 min_z_axis_limit            = -35,
    #                 max_z_axis_limit            = 35)          


    return results

def vehicle_setup(wing_reference_area,fuel_weight,MTOW, engine_diameter,design_thrust,save_file_path,resize_aircraft=False, vehicle_name = 'Boeing_787-8') :
    if resize_aircraft:
        # ------------------------------------------------------------------
        #   Initialize the Vehicle
        # ------------------------------------------------------------------      
        vehicle = RCAIDE.Vehicle()
       
        # ################################################# Vehicle-level Properties #################################################   
        vehicle.tag = vehicle_name 
        vehicle.mass_properties.max_takeoff               = MTOW #227930. * Units.kilogram 
        # vehicle.mass_properties.max_zero_fuel             = 161025.0 * Units.kilogram  
        #vehicle.mass_properties.max_fuel                  = fuel_weight*1.4 #101323 * Units.kilogram 
        #vehicle.mass_properties.operating_empty           = 120000 * Units.kilogram          

        vehicle.mass_properties.center_of_gravity         = [[27.0, 0, 0]]
        vehicle.flight_envelope.ultimate_load             = 3.75 
        vehicle.flight_envelope.positive_limit_load       = 2.5  
        vehicle.flight_envelope.negative_limit_load       = 0.75
        vehicle.flight_envelope.design_mach_number        = 0.85  
        vehicle.flight_envelope.design_cruise_altitude    = 35000.0*Units.feet 
        vehicle.flight_envelope.design_range              = 7305.0 * Units.nmi
        vehicle.reference_area                            = wing_reference_area #395.0 * Units['meters**2']    
        vehicle.number_of_passengers                                = 248 # Single class. 242 in dual class (24 business, 21 economy) 
        vehicle.systems.control                           = "fully powered" 
        vehicle.systems.accessories                       = "long range"


        # ------------------------------------------------------------------
        #   Main Wing
        #   Note: The 
        # ------------------------------------------------------------------

        wing                                  = RCAIDE.Library.Components.Wings.Main_Wing()
        wing.tag                              = 'main_wing'
        wing.aspect_ratio                     = 8.61650
        wing.sweeps.quarter_chord             = 35 * Units.deg
        wing.thickness_to_chord               = 0.1
        wing.spans.projected                  =np.sqrt(wing.aspect_ratio * wing_reference_area)
        wing.chords.root                      = 14.0 / 58.138 *  wing.spans.projected
        wing.chords.tip                       = 2.1 / 58.138 *  wing.spans.projected
        wing.taper                            = wing.chords.tip / wing.chords.root
        wing.chords.mean_aerodynamic          = 5.75 / 58.138 *  wing.spans.projected
        wing.areas.reference                  = wing_reference_area
        wing.areas.wetted                     = 2.1*wing_reference_area
        wing.twists.root                      = 0.0 * Units.degrees 
        wing.twists.tip                       = -3.0 * Units.degrees 
        wing.origin                           = [[16.59,0,-0.492]]
        wing.aerodynamic_center               = [0,0,0] 
        wing.vertical                         = False
        wing.dihedral                         = 9.0 * Units.degrees 
        wing.xz_plane_symmetric               = True 
        wing.high_lift                        = True 
        wing.dynamic_pressure_ratio           = 1.0
            
        # Wing Segments
        root_airfoil                          = RCAIDE.Library.Components.Airfoils.Airfoil()
        ospath                                = os.path.abspath(__file__)
        separator                             = os.path.sep
        rel_path                              = os.path.dirname(ospath) + separator  + '..'  + separator 
        root_airfoil.coordinate_file          = rel_path  + 'Airfoils' + separator + 'transonic_wing_root_section_airfoil.txt'
        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'Root'
        segment.percent_span_location         = 0.0
        #segment.twist                         = 4. * Units.deg
        segment.root_chord_percent            = 1. 
        segment.dihedral_outboard             = 8.0 * Units.degrees
        segment.sweeps.quarter_chord          = 27.57 * Units.degrees
        segment.thickness_to_chord            = .1
        segment.append_airfoil(root_airfoil)
        wing.append_segment(segment)

        yehudi_airfoil                        = RCAIDE.Library.Components.Airfoils.Airfoil()
        yehudi_airfoil.coordinate_file        = rel_path+ 'Airfoils' + separator + 'transonic_wing_inboard_section_airfoil.txt'
        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'Yehudi'
        segment.percent_span_location         = 0.345
        #segment.twist                         = 3.0 * Units.deg
        segment.root_chord_percent            = 0.53 
        segment.dihedral_outboard             = 7.0 * Units.degrees
        segment.sweeps.quarter_chord          = 31. * Units.degrees
        segment.thickness_to_chord            = 0.1
        segment.append_airfoil(yehudi_airfoil)
        wing.append_segment(segment)

        tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
        tip_airfoil.coordinate_file           = rel_path + 'Airfoils' + separator + 'transonic_wing_outboard_section_airfoil.txt'
        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'Tip'
        segment.percent_span_location         = 0.95
        #segment.twist                         = 2.0 * Units.degrees
        segment.root_chord_percent            = 0.155 
        segment.dihedral_outboard             = 12.0 * Units.degrees
        segment.sweeps.quarter_chord          = 42.0 * Units.degrees
        segment.thickness_to_chord            = .1
        segment.append_airfoil(tip_airfoil)
        wing.append_segment(segment)

        tip_airfoil                           =  RCAIDE.Library.Components.Airfoils.Airfoil()
        tip_airfoil.coordinate_file           = rel_path + 'Airfoils' + separator + 'transonic_wing_tip_section_airfoil.txt'
        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'Winglet'
        segment.percent_span_location         = 1.00
        segment.twist                         = 0.0 * Units.degrees
        segment.root_chord_percent            = 0.093 
        segment.dihedral_outboard             = 0.0 * Units.degrees
        segment.sweeps.quarter_chord          = 0.0 * Units.degrees
        segment.thickness_to_chord            = .1
        segment.append_airfoil(tip_airfoil)
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
        aileron.deflection            = 0.0 * Units.degrees
        aileron.chord_fraction        = 0.16
        wing.append_control_surface(aileron)
         
        spoiler                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Spoiler()
        spoiler.tag                   = 'spoiler'
        spoiler.span_fraction_start   = 0.3
        spoiler.span_fraction_end     = 0.7
        spoiler.deflection            = 0.0 * Units.degrees
        spoiler.chord_fraction        = 0.05
        wing.append_control_surface(spoiler)        

        vehicle.append_component(wing)

        # ------------------------------------------------------------------
        #  Horizontal Stabilizer
        # ------------------------------------------------------------------

        wing     = RCAIDE.Library.Components.Wings.Horizontal_Tail()
        wing.tag = 'horizontal_stabilizer'

        wing.aspect_ratio            = 5.169
        wing.sweeps.quarter_chord    = 35.0 * Units.deg  
        wing.thickness_to_chord      = 0.1
        wing.taper                   = 0.237
        wing.spans.projected         = 20.11482 * Units.meter
        wing.chords.root             = 6.22 * Units.meter
        wing.chords.tip              = 1.48 * Units.meter
        wing.chords.mean_aerodynamic = 3.853 * Units.meter
        wing.areas.reference         = 78.26 * Units['meters**2']
        wing.areas.exposed           = 66.52 * Units['meters**2']    # Exposed area of the horizontal tail
        wing.areas.wetted            = 136.69 * Units['meters**2']     # Wetted area of the horizontal tail
        wing.twists.root             = 0.0 * Units.degrees
        wing.twists.tip              = 0.0 * Units.degrees 
        wing.origin                  = [[46.961, 0, 1.639]]
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
        segment.dihedral_outboard      = 8.0 * Units.degrees
        segment.sweeps.quarter_chord   = 35.785  * Units.degrees 
        segment.thickness_to_chord     = .07
        wing.append_segment(segment)

        segment                        = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                    = 'tip_segment'
        segment.percent_span_location  = 1.
        segment.twist                  = 0. * Units.deg
        segment.root_chord_percent     = 0.237               
        segment.dihedral_outboard      = 0 * Units.degrees
        segment.sweeps.quarter_chord   = 0 * Units.degrees  
        segment.thickness_to_chord     = .07
        wing.append_segment(segment)
        
                

        # control surfaces -------------------------------------------
        elevator                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator()
        elevator.tag                   = 'elevator'
        elevator.span_fraction_start   = 0.09
        elevator.span_fraction_end     = 0.92
        elevator.deflection            = 0.0  * Units.deg
        elevator.chord_fraction        = 0.3
        wing.append_control_surface(elevator)
        vehicle.append_component(wing)


        # ------------------------------------------------------------------
        #   Vertical Stabilizer
        # ------------------------------------------------------------------

        wing = RCAIDE.Library.Components.Wings.Vertical_Tail()
        wing.tag = 'vertical_stabilizer'

        wing.aspect_ratio            = 1.801
        wing.sweeps.quarter_chord    = 40.0  * Units.deg   
        wing.thickness_to_chord      = 0.1
        wing.taper                   = 0.1

        wing.spans.projected         = 9.77 * Units.meter 
        wing.total_length            = wing.spans.projected 
        
        wing.chords.root             = 8.7 * Units.meter
        wing.chords.tip              = 0.91 * Units.meter
        wing.chords.mean_aerodynamic = 5.91 * Units.meter

        wing.areas.reference         = 53.04 * Units['meters**2']
        wing.areas.wetted            = 111.384 * Units['meters**2']
        
        wing.twists.root             = 0.0 * Units.degrees
        wing.twists.tip              = 0.0 * Units.degrees

        wing.origin                  = [[43.31, 0, 3.115]]
        wing.aerodynamic_center      = [0,0,0]

        wing.vertical                = True
        wing.xz_plane_symmetric      = False
        wing.t_tail                  = False

        wing.dynamic_pressure_ratio  = 1.0

        # Wing Segments
        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'root'
        segment.percent_span_location         = 0.0
        segment.twist                         = 0. * Units.deg
        segment.root_chord_percent            = 1.
        segment.dihedral_outboard             = 0 * Units.degrees
        segment.sweeps.quarter_chord          = 6.97 * Units.degrees  
        segment.thickness_to_chord            = 0.10 
        wing.append_segment(segment)

        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'segment_1'
        segment.percent_span_location         = 0.107
        segment.twist                         = 0. * Units.deg
        segment.root_chord_percent            = 1.031
        segment.dihedral_outboard             = 0. * Units.degrees
        segment.sweeps.quarter_chord          = 63.8274 * Units.degrees   
        segment.thickness_to_chord            = 0.10 
        wing.append_segment(segment)

        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'segment_2'
        segment.percent_span_location         = 0.180
        segment.twist                         = 0. * Units.deg
        segment.root_chord_percent            = 0.815
        segment.dihedral_outboard             = 0.0 * Units.degrees
        segment.sweeps.quarter_chord          = 41.250 * Units.degrees    
        segment.thickness_to_chord            = 0.10  
        wing.append_segment(segment)


        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'segment_3'
        segment.percent_span_location         = 0.94
        segment.twist                         = 0. * Units.deg
        segment.root_chord_percent            = 0.298
        segment.dihedral_outboard             = 0.0 * Units.degrees
        segment.sweeps.quarter_chord          = 69.09 * Units.degrees    
        segment.thickness_to_chord            = 0.10  
        wing.append_segment(segment)


        segment                               = RCAIDE.Library.Components.Wings.Segments.Segment()
        segment.tag                           = 'segment_4'
        segment.percent_span_location         = 1.0
        segment.twist                         = 0. * Units.deg
        segment.root_chord_percent            = 0.105
        segment.dihedral_outboard             = 0.0 * Units.degrees
        segment.sweeps.quarter_chord          = 0.0    
        segment.thickness_to_chord            = 0.10  
        wing.append_segment(segment)
        
    
        # control surfaces -------------------------------------------
        rudder                       = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
        rudder.tag                   = 'rudder'
        rudder.span_fraction_start   = 0.1 
        rudder.span_fraction_end     = 0.95 
        rudder.deflection            = 0 
        rudder.chord_fraction        = 0.33  
        wing.append_control_surface(rudder)    
        
        
                

        # add to vehicle
        vehicle.append_component(wing)
        # ################################################# Fuselage ################################################################ 
            
        fuselage                                    = RCAIDE.Library.Components.Fuselages.Fuselage() 
        fuselage.number_coach_seats                 = vehicle.number_of_passengers 
        fuselage.seats_abreast                      = 9
        fuselage.seat_pitch                         = 0.9     * Units.meter 
        fuselage.fineness.nose                      = 2.
        fuselage.fineness.tail                      = 3. 
        fuselage.lengths.nose                       = 8.48   * Units.meter
        fuselage.lengths.tail                       = 16.55   * Units.meter
        fuselage.lengths.total                      = 56.7 * Units.meter  
        fuselage.lengths.fore_space                 = 1.    * Units.meter
        fuselage.lengths.aft_space                  = 10.    * Units.meter
        fuselage.width                              = 5.9  * Units.meter
        fuselage.heights.maximum                    = 5.9  * Units.meter
        fuselage.effective_diameter                 = 5.9     * Units.meter
        fuselage.areas.side_projected               = fuselage.heights.maximum * fuselage.lengths.total * Units['meters**2'] 
        fuselage.areas.wetted                       = np.pi * fuselage.width/2 * fuselage.lengths.total * Units['meters**2'] 
        fuselage.areas.front_projected              = np.pi * fuselage.width/2      * Units['meters**2']  
        fuselage.differential_pressure              = 5.0e4 * Units.pascal
        fuselage.heights.at_quarter_length          = fuselage.heights.maximum * Units.meter
        fuselage.heights.at_three_quarters_length   = fuselage.heights.maximum * Units.meter
        fuselage.heights.at_wing_root_quarter_chord = fuselage.heights.maximum* Units.meter
        
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
        segment.percent_x_location                  = 0.00410 
        segment.percent_z_location                  = 0.00141 
        segment.height                              = 0.938
        segment.width                               = 1.14
        fuselage.append_segment(segment)   
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_2'   
        segment.percent_x_location                  = 0.00820 
        segment.percent_z_location                  = 0.00256 
        segment.height                              =  1.4
        segment.width                               = 1.47959
        fuselage.append_segment(segment)      
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_3'   
        segment.percent_x_location                  = 0.02029 
        segment.percent_z_location                  = 0.00520 
        segment.height                              = 2.36 
        segment.width                               = 2.32 
        fuselage.append_segment(segment)   

        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_4'   
        segment.percent_x_location                  = 0.02715 	
        segment.percent_z_location                  = 0.00720 
        segment.height                              =  2.79
        segment.width                               = 2.73 
        fuselage.append_segment(segment)   
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_5'   
        segment.percent_x_location                  = 0.04244 
        segment.percent_z_location                  = 0.01194 
        segment.height                              = 3.75 
        segment.width                               = 3.41 
        fuselage.append_segment(segment)     
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_6'   
        segment.percent_x_location                  = 0.05931 
        segment.percent_z_location                  = 0.01588 
        segment.height                              =  4.48
        segment.width                               = 4.06 
        fuselage.append_segment(segment)             
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_7'   
        segment.percent_x_location                  = 0.07737 
        segment.percent_z_location                  = 0.01844 
        segment.height                              = 5.0 
        segment.width                               = 4.598 
        fuselage.append_segment(segment)    
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_8'   
        segment.percent_x_location                  = 0.09544 
        segment.percent_z_location                  = 0.02024 
        segment.height                              = 5.36 
        segment.width                               = 5.05102 
        fuselage.append_segment(segment)   
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_9'     
        segment.percent_x_location                  = 0.1236 
        segment.percent_z_location                  = 0.02180
        segment.height                              = 5.76
        segment.width                               = 5.5289
        fuselage.append_segment(segment)     
            
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_10'     
        segment.percent_x_location                  = 0.15177 
        segment.percent_z_location                  = 0.02305
        segment.height                              = 5.8890
        segment.width                               = 5.9183
        fuselage.append_segment(segment)   
            
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_11'     
        segment.percent_x_location                  = 0.71813 
        segment.percent_z_location                  = 0.02305
        segment.height                              = 5.88980
        segment.width                               = 5.91837
        fuselage.append_segment(segment)    
            
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_12'     
        segment.percent_x_location                  = 0.77176
        segment.percent_z_location                  = 0.02664
        segment.height                              = 5.41224
        segment.width                               = 5.50771
        fuselage.append_segment(segment)             
            
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_13'     
        segment.percent_x_location                  = 0.85221 
        segment.percent_z_location                  = 0.03074
        segment.height                              = 4.27768
        segment.width                               = 4.17826
        fuselage.append_segment(segment)               
            
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_14'     
        segment.percent_x_location                  = 0.93265
        segment.percent_z_location                  = 0.03432
        segment.height                              = 2.38776
        segment.width                               = 2.5
        fuselage.append_segment(segment)
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_15'     
        segment.percent_x_location                  = 0.99488 
        segment.percent_z_location                  = 0.03689
        segment.height                              = 0.74286
        segment.width                               = 1.22145
        fuselage.append_segment(segment)
        
        # Segment                                   
        segment                                     = RCAIDE.Library.Components.Fuselages.Segments.Segment()
        segment.tag                                 = 'segment_16'     
        segment.percent_x_location                  = 1.0 
        segment.percent_z_location                  = 0.04098
        segment.height                              = 0.0
        segment.width                               = 0.0
        fuselage.append_segment(segment)

        # add to vehicle
        vehicle.append_component(fuselage)


        # ################################################# Landing Gear #############################################################   
        # ------------------------------------------------------------------        
        #  Landing Gear
        # Source: https://www.boeing.com/content/dam/boeing/boeingdotcom/commercial/airports/acaps/787.pdf
        # ------------------------------------------------------------------  
        main_gear               = RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear()
        main_gear.tire_diameter = 50.0 * Units.inches
        main_gear.strut_length  = 5.5 * Units.ft 
        main_gear.units         = 2    # Number of main landing gear
        main_gear.wheels        = 4    # Number of wheels on the main landing gear
        vehicle.append_component(main_gear)  

        nose_gear               = RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()       
        nose_gear.tire_diameter = 40. * Units.inches
        nose_gear.units         = 1    # Number of nose landing gear
        nose_gear.wheels        = 2    # Number of wheels on the nose landing gear
        nose_gear.strut_length  = 9.0 * Units.ft 
        vehicle.append_component(nose_gear)


        # ################################################# Energy Network #######################################################          
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
        # Sources: https://www.researchgate.net/publication/320798360_Performance_Analysis_of_Cold_Sections_of_High_BYPASS_Ratio_Turbofan_Aeroengine/figures?lo=1

        #------------------------------------------------------------------------------------------------------------------------------------ 
        # Propulsor: Starboard Propulsor
        #------------------------------------------------------------------------------------------------------------------------------------         
        turbofan1                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()   
        turbofan1.origin                             = [[ 0.0 , 0.0 , 0.0 ]]
        turbofan1.tag                                = 'propulsor_1'    
        turbofan1.engine_length                      = 4.928                      
        turbofan1.engine_diameter                    = engine_diameter                   
        turbofan1.bypass_ratio                       = 9.1                        
        turbofan1.design_altitude                    = 40000*Units.ft             
        turbofan1.design_mach_number                 = 0.7                       
        turbofan1.design_thrust                      = design_thrust
    
        # working fluid                   
        turbofan1.working_fluid                      = RCAIDE.Library.Attributes.Gases.Air() 
        
        # Ram inlet 
        ram                                          = RCAIDE.Library.Components.Powertrain.Converters.Ram()
        ram.tag                                      = 'ram' 
        turbofan1.ram                                = ram 
              
        # inlet nozzle          
        inlet_nozzle                                = RCAIDE.Library.Components.Powertrain.Converters.Compression_Nozzle()
        inlet_nozzle.tag                            = 'inlet nozzle'
        inlet_nozzle.polytropic_efficiency          = 0.97                                       
        inlet_nozzle.pressure_ratio                 = 1
        inlet_nozzle.compressibility_effects        = False
        turbofan1.inlet_nozzle                       = inlet_nozzle
        
        # fan                
        fan                                         = RCAIDE.Library.Components.Powertrain.Converters.Fan()   
        fan.tag                                     = 'fan'
        fan.polytropic_efficiency                   = 0.91                   
        fan.pressure_ratio                          = 1.4                    
        turbofan1.fan                                = fan        
    
        # low pressure compressor    
        low_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
        low_pressure_compressor.tag                   = 'lpc'
        low_pressure_compressor.polytropic_efficiency = 0.91                  
        low_pressure_compressor.pressure_ratio        = 1.3                     
        turbofan1.low_pressure_compressor              = low_pressure_compressor
    
        # high pressure compressor  
        high_pressure_compressor                       = RCAIDE.Library.Components.Powertrain.Converters.Compressor()    
        high_pressure_compressor.tag                   = 'hpc'
        high_pressure_compressor.polytropic_efficiency = 0.91                        
        high_pressure_compressor.pressure_ratio        = 23.9                    
        turbofan1.high_pressure_compressor              = high_pressure_compressor
    
        # low pressure turbine  
        low_pressure_turbine                           = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
        low_pressure_turbine.tag                       ='lpt'
        low_pressure_turbine.mechanical_efficiency     = 0.99                     
        low_pressure_turbine.polytropic_efficiency     = 0.93                     
        turbofan1.low_pressure_turbine                  = low_pressure_turbine
       
        # high pressure turbine     
        high_pressure_turbine                          = RCAIDE.Library.Components.Powertrain.Converters.Turbine()   
        high_pressure_turbine.tag                      ='hpt'
        high_pressure_turbine.mechanical_efficiency    = 0.99                     
        high_pressure_turbine.polytropic_efficiency    = 0.93                     
        turbofan1.high_pressure_turbine                 = high_pressure_turbine 
    
        # combustor  
        combustor                                      = RCAIDE.Library.Components.Powertrain.Converters.Combustor()   
        combustor.tag                                  = 'Comb'
        combustor.efficiency                           = 0.997                    
        combustor.turbine_inlet_temperature            = 1400                  
        combustor.pressure_ratio                       = 0.94                     
        combustor.fuel_data                            = RCAIDE.Library.Attributes.Propellants.Jet_A()  
        turbofan1.combustor                             = combustor
    
        # core nozzle
        core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
        core_nozzle.tag                                = 'core nozzle'
        core_nozzle.polytropic_efficiency              = 0.98                     # CHECKED Ref. [2] Page 9
        core_nozzle.pressure_ratio                     = 0.995 
        turbofan1.core_nozzle                           = core_nozzle
                 
        # fan nozzle             
        fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
        fan_nozzle.tag                                 = 'fan nozzle'
        fan_nozzle.polytropic_efficiency               = 0.98                     # CHECKED Ref. [2] Page 9
        fan_nozzle.pressure_ratio                      = 0.995
        turbofan1.fan_nozzle                            = fan_nozzle 


        # design turbofan
        design_turbofan(turbofan1)

        # Nacelle 
        nacelle                                     = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
        nacelle.diameter                            = 3.556
        nacelle.length                              = 4.9
        nacelle.tag                                 = 'nacelle_1'
        nacelle.inlet_diameter                      = 2.5
        nacelle.origin                              = [[17.818, 10.000,-0.953]] 
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
        turbofan2.origin                           = [[17.818, -10.000,-0.953]]
        turbofan2.nacelle.origin                   = [[17.818, -10.000,-0.953]]
            
        # append propulsor to distribution line 
        net.propulsors.append(turbofan2)

        #------------------------------------------------------------------------------------------------------------------------- 
        #  Energy Source: Fuel Tank
        #------------------------------------------------------------------------------------------------------------------------- 
        # fuel tank
        fuel_tank                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Wing_Fuel_Tank()
        fuel_tank.origin                                 = vehicle.wings.main_wing.origin  
        fuel_tank.fuel                                   = RCAIDE.Library.Attributes.Propellants.Jet_A1()     
        fuel_line.fuel_tanks.append(fuel_tank)

        #------------------------------------------------------------------------------------------------------------------------------------   
        # Assign propulsors to fuel line to network      
        fuel_line.assigned_propulsors =  [['propulsor_1', 'propulsor_2']]

        #------------------------------------------------------------------------------------------------------------------------------------   
        # Append fuel line to fuel line to network      
        net.fuel_lines.append(fuel_line)        

        #------------------------------------------------------------------------------------------------------------------------- 
        # Done ! 
        #-------------------------------------------------------------------------------------------------------------------------      
    
         # Append energy network to aircraft 
        vehicle.append_energy_network(net)   

        # ------------------------------------------------------------------
        #   Vehicle Definition Complete
        # ------------------------------------------------------------------      
        save_aircraft_geometry(vehicle,vehicle.tag,save_file_path) 
        weight_analysis                               = RCAIDE.Framework.Analyses.Weights.Conventional()
        weight_analysis.vehicle                       = vehicle
        weight_analysis.settings.FLOPS.fidelity     = 'Complex' 
        results = weight_analysis.evaluate()   
        vehicle.mass_properties.fuel                       = fuel_weight
        vehicle.mass_properties.payload                   =  55800 * Units.pounds 
        paintweight = 450*Units.kg
        ETOPS = vehicle.number_of_passengers  * 7.7 
        updated_electricalweight = results.empty.systems.electrical*1.67
        vehicle.mass_properties.takeoff                   =  results.operational_items.total + results.empty.total+ vehicle.mass_properties.payload +vehicle.mass_properties.fuel \
                                                            + updated_electricalweight + ETOPS + 56 +  results.empty.structural.landing_gear*0.1 +paintweight
        

    else: 
        vehicle = load_aircraft_geometry(vehicle_name,save_file_path) 

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
     
    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------ 
    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'descent' 
    config.wings['main_wing'].control_surfaces.spoiler.deflection  = 45. * Units.deg    
    configs.append(config) 

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'takeoff'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 30. * Units.deg 
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  3470. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  3470. * Units.rpm 
    config.V2_VS_ratio = 1.21
    configs.append(config)


    # ------------------------------------------------------------------
    #   Cutback Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'cutback'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 20. * Units.deg
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  2780. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  2780. * Units.rpm 
    configs.append(config)   



    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'landing'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  2030. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  2030. * Units.rpm
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    config.Vref_VS_ratio = 1.23
    configs.append(config)   

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------ 

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'short_field_takeoff'    
    config.wings['main_wing'].control_surfaces.flap.deflection  = 20. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg
    config.networks.fuel.propulsors['propulsor_1'].fan.angular_velocity =  3470. * Units.rpm
    config.networks.fuel.propulsors['propulsor_2'].fan.angular_velocity      =  3470. * Units.rpm 
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    config.V2_VS_ratio = 1.21 
    configs.append(config)

    # ------------------------------------------------------------------
    #   Short Field Takeoff Configuration
    # ------------------------------------------------------------------  

    config = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag = 'reverse_thrust'
    config.wings['main_wing'].control_surfaces.flap.deflection  = 30. * Units.deg
    config.wings['main_wing'].control_surfaces.slat.deflection  = 25. * Units.deg 
    config.landing_gears.main_gear.gear_extended    = True
    config.landing_gears.nose_gear.gear_extended    = True  
    configs.append(config)
    
    return configs


# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):
    """This is the baseline set of analyses to be used with this vehicle. Of these, the most
    commonly changed are the weights and aerodynamics methods."""

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle

    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Conventional()
    analyses.append(weights)


    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.number_of_spanwise_vortices   = 40
    aerodynamics.settings.number_of_chordwise_vortices  = 4 
    aerodynamics.settings.fuselage_lift_correction      = 1
    #aerodynamics.settings.trim_drag_correction_factor                        = 1.2
    #aerodynamics.settings.wing_parasite_drag_form_factor                     = 1.5
    #aerodynamics.settings.fuselage_parasite_drag_form_factor                 = 3.0
    aerodynamics.settings.viscous_lift_dependent_drag_factor                 =0.8
    aerodynamics.training.angle_of_attack                     = np.array([-2. , 1E-20 , 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0]) * Units.deg 
    #aerodynamics.training.Mach                                          = np.array([0.1  ,0.3,  0.5,  0.65 , 0.85 , 0.9]) 
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)
    

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    return analyses    



# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.solver.type = 'root_finder'


    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff_Ground_Run" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start                                           = 0.* Units.knots
    segment.velocity_end                                             = 167.0 * Units['knots']
    segment.friction_coefficient                                     = 0.03
    segment.altitude                                                 = 0.0   
    segment.throttle                                                 = 1.0
    mission.append_segment(segment)
     
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Takeoff_Climb" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_end                                             = 35 * Units['ft']
    segment.air_speed_end                                            = 175.0 * Units['knots']
    segment.climb_rate                                               = 250 * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment) 

    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Inital_Climb" 
    segment.analyses.extend( analyses.cutback )  
    segment.altitude_end                                             = 1000  * Units['feet']
    segment.air_speed                                                = 200.0 * Units['knots']
    segment.climb_rate                                               = 1800   * Units['fpm']  
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_1" 
    segment.analyses.extend( analyses.cutback ) 
    segment.altitude_end                                             = 8000   * Units['ft']
    segment.air_speed_end                                            = 300 * Units['knots']
    segment.climb_rate                                               = 1700   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                             = 16000   * Units['ft']
    segment.air_speed                                                = 350 * Units['knots']
    segment.climb_rate                                               = 1300   * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)


    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_3" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                             = 33000   * Units['ft']
    segment.air_speed                                                = 450 * Units['knots']
    segment.climb_rate                                               = 1000   * Units['fpm']  
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_4" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                             = 40000   * Units['ft']
    segment.air_speed                                                = 480 * Units['knots']
    segment.climb_rate                                               = 280   * Units['fpm']  
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)

    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                 = 40000 * Units['ft']  
    segment.air_speed                                                = 450 * Units['knots']
    segment.distance                                                 = 4456.982 * Units.nmi
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.descent ) 
    segment.altitude_end                                             = 10000   * Units.ft
    segment.air_speed                                                = 380 * Units['knots']
    segment.descent_rate                                             = 1850   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag  = "approach" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 2000 * Units.ft
    segment.air_speed                                                = 225.0 * Units['knots']
    segment.descent_rate                                             = 650  * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "final_approach"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 50.0   * Units.ft
    segment.air_speed                                                = 175.0 * Units['knots']
    segment.descent_rate                                             = 600.0   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Fourth Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "level_off_touchdown" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 0  * Units.ft
    segment.air_speed                                                = 160 * Units['knots']
    segment.descent_rate                                             = 300   * Units['fpm']  
           
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)




    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Landing Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "Landing"

    segment.analyses.extend( analyses.reverse_thrust ) 
    segment.velocity_start                                                = 160.0 * Units['knots']
    segment.velocity_end                                                  = 10 * Units.knots 
    segment.friction_coefficient                                          = 0.4
    segment.altitude                                                      = 0.0   
    segment.assigned_control_variables.elapsed_time.active                = True  
    segment.assigned_control_variables.elapsed_time.initial_guess_values  = [[30.]]  
    mission.append_segment(segment)     

    return mission



def approach_mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment() 
 

    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "final_approach"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_start                                           = 120.5   
    segment.altitude_end                                             = 50.0   * Units.ft
    segment.air_speed                                                = 175.0 * Units['knots']
    segment.descent_angle                                            = 3.  * Units.deg                               
           
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Fourth Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "level_off_touchdown" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 0  * Units.ft
    segment.air_speed                                                = 160 * Units['knots']
    segment.descent_angle                                            = 3.  * Units.deg       
           
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment) 
 
    return mission

def takeoff_mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment() 


    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff_Ground_Run" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start                                           = 0.* Units.knots
    segment.velocity_end                                             = 167.0 * Units['knots']
    segment.friction_coefficient                                     = 0.03
    segment.altitude                                                 = 0.0   
    segment.throttle                                                 = 1.0
    mission.append_segment(segment)

     
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Takeoff_Climb" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_end                                             = 35 * Units['ft']
    segment.air_speed_end                                            = 175.0 * Units['knots']
    segment.climb_rate                                               = 250 * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment) 

    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Inital_Climb" 
    segment.analyses.extend( analyses.cutback )  
    segment.altitude_end                                             = 1000  * Units['feet']
    segment.air_speed                                                = 200.0 * Units['knots']
    segment.climb_rate                                               = 1800   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_1" 
    segment.analyses.extend( analyses.cutback ) 
    segment.altitude_end                                             = 8000   * Units['ft']
    segment.air_speed_end                                            = 300 * Units['knots']
    segment.climb_rate                                               = 1700   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)
 

    return mission


def missions_setup(mission):
    """This allows multiple missions to be incorporated if desired, but only one is used here."""

    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)

    return missions

def save_aircraft_geometry(geometry,filename,save_dir): 

    # Create full path for pickle file
    pickle_file = os.path.join(save_dir, filename + '.pkl')

    # Save the file
    with open(pickle_file, 'wb') as file:
        pickle.dump(geometry, file) 
    return


def load_aircraft_geometry(filename,load_dir):
    # Create full path for pickle file
    pickle_file = os.path.join(load_dir, filename +'.pkl')

    # Load the file
    with open(pickle_file, 'rb') as file:
        results = pickle.load(file)
    return results

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------
def plot_mission(results):
    """This function plots the results of the mission analysis and saves those results to 
    png files."""

    # Plot Flight Conditions 
    plot_flight_conditions(results)

    # Plot Aerodynamic Forces 
    plot_aerodynamic_forces(results)

    # Plot Aerodynamic Coefficients 
    plot_aerodynamic_coefficients(results)     

    # Drag Components
    plot_drag_components(results)

    # Plot Altitude, sfc, vehicle weight 
    plot_altitude_sfc_weight(results)

    # Plot Velocities 
    plot_aircraft_velocities(results)  

    plot_propulsor_throttles(results)


    return

if __name__ == '__main__': 
    main()    
    plt.show()