# Regression/scripts/Tests/fuel_tank_volume.py
#
# 
# Created: Mar 2026, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Core                          import Units , Data 
from RCAIDE.Library.Plots                           import *        


# python imports     
import numpy as np  
import sys
import os
import matplotlib.pyplot as plt  


base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Hydrogen_BWB         import vehicle_setup as BWB_vehicle_setup
from Hydrogen_BWB         import configs_setup as BWB_configs_setup

# ----------------------------------------------------------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------------------------------------------------------

def main():

    vehicle = BWB_vehicle_setup()
     # Step 2 create aircraft configuration based on vehicle 
    configs  = BWB_configs_setup(vehicle)
    
    # Step 3 set up analysis 
    analyses = analyses_setup(configs)
    
    # Step 4 set up a flight mission
    mission  = mission_setup(analyses)
    missions = missions_setup(mission) 
    
    # Step 5 execute flight profile
    results = missions.base_mission.evaluate()
    CL_truth = 0.3975726457797412
    CL    = results.segments.cruise.conditions.aerodynamics.coefficients.lift.total[0, 0]

    abs_error = np.abs((CL - CL_truth)) 
    assert abs_error <= 1e-3, ( # a larger tolerence is needed here because we iterate on MTOW and slight variations are expected
        f"CL absolute error too large: {abs_error:.6e} (CL={CL:.6e}, CL_truth={CL_truth:.6e})"
    )

    plot_aircraft_cg_weight_bubbles(results,vehicle,show_figure=False)
    plot_fuel_flow_rates(results)
    plot_fuel_tank_conditions(results)

    for filename in (
        "bwb_hydrogen_test_geometry_description.xlsx",
        "bwb_hydrogen_test_weight_breakdown.xlsx",
    ):
        file_path = os.path.join(base_dir, filename)
        if os.path.exists(file_path):
            os.remove(file_path)

    return

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
        if config.wings['main_wing'].control_surfaces.flap.deflection != 0: 
            analysis.aerodynamics.settings.drag_coefficient_increment =  0.05        
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
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()  
    geometry.settings.update_max_fuel = True
    geometry.settings.compute_fuel_volume = True
    geometry.settings.write_geometry_properties = True
    analyses.append(geometry)
    

    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Hydrogen_BWB()                                                  
    weights.aircraft_type                                                    = 'BWB'
    weights.settings.FLOPS.fidelity                                          = 'Complex' 
    weights.settings.weight_correction_additions.empty.structural.paint      = 464.6384576160517  
    weights.settings.weight_correction_factors.empty.systems.electrical      = 2.67
    weights.settings.weight_correction_factors.empty.systems.hydraulics      = 1.5 
    weights.settings.weight_correction_factors.empty.structural.landing_gear = 1.1 
    weights.settings.weight_correction_factors.empty.systems.control_systems = 1.9   # scaled based on wetted area when compared to 787 
    weights.settings.weight_correction_factors.empty.structural.nacelle      = 0.94   
    weights.settings.weight_correction_factors.empty.structural.empennage    = 0.92   
    weights.settings.write_mass_properties                                   = True 
    weights.settings.run_weights_analysis                                    = True
    weights.settings.iterate_mtow                                            = True
    weights.settings.run_center_of_gravity_analysis                          = True
    weights.settings.run_moments_of_inertia_analysis                         = True
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.settings.drag_reduction_factors.parasite_drag = 0.16
    aerodynamics.settings.store_training_data                  = False
    aerodynamics.training.Mach                                 = np.array([0.1  ,0.3,  0.5,  0.65 , 0.85 , 0.9])
    analyses.append(aerodynamics)
 
    # ------------------------------------------------------------------
    #  Energy
    # ------------------------------------------------------------------
    energy = RCAIDE.Framework.Analyses.Energy.Energy()
    analyses.append(energy)


    # ------------------------------------------------------------------
    #  Stability
    # ------------------------------------------------------------------
    stability = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method()  
    stability.settings.compute_neutral_point = False
    analyses.append(stability)
        

    # ------------------------------------------------------------------
    # Emissions 
    emissions = RCAIDE.Framework.Analyses.Emissions.Emission_Index_Correlation_Method() 
    emissions.settings.use_surrogate     = False                       
    analyses.append(emissions) 

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    return analyses    
    
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

    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "Cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                 = 40000 * Units['ft']  
    segment.mach_number                                              = 0.78
    segment.distance                                                 = 7370 * Units.km  + 626 *Units.nmi  
            
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

if __name__ == '__main__': 
    main()    
