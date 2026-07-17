# RCAIDE/Library/Missions/Common/Pre_Process/mass_properties.py
# 
# 
# Created: Mar 2025, M. Clarke
# Modified: Aug 2025 S. Shekar, Jan 2026 A. Molloy

# ----------------------------------------------------------------------------------------------------------------------
#  RCAIDE
# ---------------------------------------------------------------------------------------------------------------------- 
from copy import deepcopy
import RCAIDE 
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia  import compute_vehicle_moment_of_inertia
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity  import compute_vehicle_center_of_gravity 
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.compute_fuel_mass import compute_fuel_mass
from RCAIDE.Library.Mission.Common.Pre_Process.mass_properties_correction_factors   import apply_correction_factors, apply_component_weights
from RCAIDE.Library.Mission.Common.Pre_Process.mass_properties_report import print_mass_report, write_mass_report   
from scipy.optimize import brentq, minimize_scalar
import numpy as np
import pandas as pd 

# ----------------------------------------------------------------------------------------------------------------------
#  mass_properties
# ----------------------------------------------------------------------------------------------------------------------  
def mass_properties(mission):
    """Calculate and update mass properties for all mission segments.
    
    Performs weight analysis, center of gravity computation, and moment of inertia 
    calculations for each mission segment. Handles multiple analysis scenarios including
    user-defined weights, MTOW-based calculations, and iterative weight convergence.
    
    Parameters
    ----------
    mission : RCAIDE.Framework.Mission
        Mission object containing segments with weight analysis requirements.
        
    Raises
    ------
    AttributeError
        If max_takeoff weight is not defined.
    AssertionError
        If payload exceeds max_payload or fuel exceeds max_fuel.
        
    Notes
    -----
    The function operates in three main modes:
    
    1. **Pre-defined weights**: Uses existing takeoff weight if provided
    2. **Simple MTOW**: Falls back to MTOW if aircraft_type undefined
    3. **Full analysis**: Performs complete weight buildup with iterations, if the 
          setting update_takeoff_weight is True then it updates the aircraft takeoff weight 
          with the new one based on the buildup. otherwise the takeoff weight is not adjusted
    
    To complete a weight breakdown the methods require an aircraft to have the following
    defined: MTOW, payload and/or fuel weight, aircraft method type (included in the weights 
    analysis type most times), max fuel and max zero fuel weights or neither. If
    the user desires to have the takeoff weight calculated by the weight breakdown
    used for further analyses then specify under the weight analysis settings 
    "update_takeoff_weight == True".

    Algorithm Flow
    ~~~~~~~~~~~~~~
    
    .. code-block:: text
    
        For each segment:
        ├── Check if weights analysis exists
        ├── Validate MTOW defined (required)
        ├── If aircraft_type undefined → Use MTOW for takeoff weight
        ├── Else if no payload/fuel → Use MTOW  
        ├── Else → Perform weight analysis:
        │   ├── Check payload/fuel limits
        │   ├── If max_fuel/max_zero_fuel undefined:
        │   │   └── Iterate to convergence (max 100 iterations)
        │   │       ├── Initial guess from regression
        │   │       ├── Evaluate weights
        │   │       ├── Apply corrections
        │   │       └── Check convergence (<10 kg residual)
        │   ├── Single evaluation
        |   └── Apply correction factors if specified
        ├── Update takeoff weight if requested
        ├── Update CG if requested
        ├── Update MOI if requested
        └── Copy vehicle to aerodynamics
    
    Weight Equations
    ~~~~~~~~~~~~~~~~
    
    .. math::
    
        W_{takeoff} = W_{OEW} + W_{payload} + W_{fuel}
        
        W_{OEW} = W_{empty} + W_{operational}
        
        W_{max\\_zero\\_fuel} = W_{OEW} + W_{max\\_payload}
        
        W_{max\\_fuel} = W_{MTOW} - W_{OEW} - W_{min\\_payload}
    
    Initial Regression Estimates
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    
    When max_fuel or max_zero_fuel undefined:
    
    .. math::
    
        W_{max\\_fuel}^{(0)} = 0.477 \\cdot W_{MTOW} - 13455
        
        W_{max\\_zero\\_fuel}^{(0)} = 0.6269 \\cdot W_{MTOW} + 20505
    """
 
    for i ,  segment in enumerate(mission.segments):
        if segment.analyses.weights == None:
            raise AssertionError('Define weights analysis method')
        else: 
            if i ==0 or segment.analyses.geometry.settings.unique_geometry:
                mass_properties_preprocess_routine(segment, i) 
            else: 
                analyses         = segment.analyses
                weights_analysis = analyses.weights 
                vehicle_1 = deepcopy(mission.segments[i-1].analyses.vehicle)
                segment.analyses.vehicle.mass_properties = vehicle_1.mass_properties
                weights_analysis.settings.iterate_mtow = False
                mass_properties_preprocess_routine(segment, i) 
                
    return 

def mass_properties_preprocess_routine(segment, i = 0):
    analyses         = segment.analyses
    weights_analysis = analyses.weights 
    
    # ---------------------------------------------------------------------------------------------------------------------------
    # STEP 1: Compute fuel mass and max fuel mass for the vehicle based on the fuel tanks defined in the vehicle
    # ---------------------------------------------------------------------------------------------------------------------------    
    update_max_fuel_mass = weights_analysis.settings.update_max_fuel_mass                       
    update_fuel_mass     = weights_analysis.settings.update_fuel_mass   
    compute_fuel_mass(analyses.vehicle, update_fuel_mass, update_max_fuel_mass)    
    
    # ---------------------------------------------------------------------------------------------------------------------------
    # STEP 2:  Pre-checks for weights analysis 
    # ---------------------------------------------------------------------------------------------------------------------------      
    if analyses.vehicle.mass_properties.max_takeoff == None:
        # For all weights analysis a maximum take off weight needs to be defined by the user
        raise AttributeError("Max Takeoff Weight for aircraft not defined")
    # orig_takeoff_weight = analyses.vehicle.mass_properties.takeoff
    
    if weights_analysis.aircraft_type == None:
        # If an aircraft type is not defined analysis cannot be performed, and the maximum take off weight will be assumed to be the takeoff weight 
        print('\n Warning: Weight Analysis type not defined')
        if analyses.vehicle.mass_properties.takeoff == None:
            print('\n Using Maximum Takeoff Weight')
            analyses.vehicle.mass_properties.takeoff = analyses.vehicle.mass_properties.max_takeoff  
    
    elif analyses.vehicle.mass_properties.takeoff == None and analyses.vehicle.mass_properties.payload == None and analyses.vehicle.mass_properties.fuel == None:
        # Without either fuel on board or payload on board, takeoff weight cannot be computed thus the takeoff weight is assumed to be max takeoff 
        print('Warning: Payload or fuel weight not defined; assuming takeoff weight is MTOW')
        analyses.vehicle.mass_properties.takeoff = analyses.vehicle.mass_properties.max_takeoff
    elif weights_analysis.settings.run_weights_analysis:
    
        # ---------------------------------------------------------------------------------------------------------------------------
        # STEP 3: Run weights analysis 
        # ---------------------------------------------------------------------------------------------------------------------------         
        if i == 0 and analyses.vehicle.mass_properties.payload > analyses.vehicle.mass_properties.max_payload:
            print('Warning: Prescribed payload weight is greater than maximum payload weight')
        if weights_analysis.settings.iterate_mtow:
            solve_for_mtow(analyses, weights_analysis, i)
            
        else:
            if analyses.vehicle.mass_properties.max_zero_fuel == None:
                # Before proceeding to the weight buildups, the buildups need either the max fuel capacity or the max zero fuel to compute OEW 
                if i == 0:
                    print('\n Warning: Max Fuel or Max Zero Fuel not defined. Iterating to find these values.')
                    iterate_max_fuel_and_max_zero_fuel(analyses)  
            
            _ = weights_analysis.evaluate(analyses.vehicle) 

            if i == 0 and analyses.vehicle.mass_properties.payload > analyses.vehicle.mass_properties.max_payload:
                print('Warning: Computed payload weight is greater than maximum payload weight')        
            
            # Compute OEW
            if weights_analysis.settings.overwrite_operating_empty_weight:
                analyses.vehicle.mass_properties.operating_empty = analyses.vehicle.mass_properties.weight_breakdown.empty.total
                            
            # Apply correction factors  if any
            apply_correction_factors(analyses)
            if i == 0:
                apply_component_weights(analyses)
 

        if (analyses.vehicle.mass_properties.fuel  == 0 or analyses.vehicle.mass_properties.fuel is None) and weights_analysis.propulsion_architecture != 'Electric':
            ('Fuel Weight for the mission is not defned. Filling up the airplace till max takeoff weight')     
            analyses.vehicle.mass_properties.fuel     = analyses.vehicle.mass_properties.max_takeoff- (analyses.vehicle.mass_properties.operating_empty + analyses.vehicle.mass_properties.payload) 

        # Compute takeoff weight and max zero fuel weight 
        if analyses.vehicle.mass_properties.takeoff == None:
            analyses.vehicle.mass_properties.takeoff = analyses.vehicle.mass_properties.operating_empty  + analyses.vehicle.mass_properties.payload+ analyses.vehicle.mass_properties.fuel    
       
        analyses.vehicle.mass_properties.max_zero_fuel = analyses.vehicle.mass_properties.operating_empty\
                                                                    + analyses.vehicle.mass_properties.max_payload 
        
        # ---------------------------------------------------------------------------------------------------------------------------
        # STEP 4: Print weight statements and apply weight factors  
        # --------------------------------------------------------------------------------------------------------------------------- 
        if i == 0: 
            if weights_analysis.print_weight_analysis_report and type(weights_analysis) != RCAIDE.Framework.Analyses.Weights.Weights:  
                print_mass_report(analyses)  
            if weights_analysis.settings.write_mass_properties:
                excel_filename = write_mass_report(analyses)     
    
    # ---------------------------------------------------------------------------------------------------------------------------     
    #  STEP 5: Compute Center of Gravity   
    # --------------------------------------------------------------------------------------------------------------------------- 
    if weights_analysis.settings.run_center_of_gravity_analysis: 
        centre_of_gravity_df = pd.DataFrame(columns=["Component", "Mass (kg)", "CG x (m)", "CG y (m)", "CG z (m)" ])
        if i != 0:
            verbose_flag = False
        else:
            verbose_flag = weights_analysis.print_weight_analysis_report
        _ ,_, _, centre_of_gravity_df = compute_vehicle_center_of_gravity(analyses.vehicle,centre_of_gravity_df,
                                                overwrite_center_of_gravity =  weights_analysis.settings.run_center_of_gravity_analysis ,
                                                segment=segment,
                                                verbose=verbose_flag)  

        if i==0 and weights_analysis.settings.write_mass_properties:
            # Centre of Gravity sheet
            with pd.ExcelWriter(excel_filename, engine="openpyxl",mode="a",if_sheet_exists="replace") as writer:
                centre_of_gravity_df.to_excel(writer,sheet_name="Centre of Gravity",index=False)
            print(f"CG breakdown written to Excel:\n  {excel_filename}")

        analyses.vehicle.mass_properties.center_of_gravity_breakdown = centre_of_gravity_df  

    # ---------------------------------------------------------------------------------------------------------------------------         
    # STEP 6: Compute Moment of Inertia 
    # --------------------------------------------------------------------------------------------------------------------------- 
    if weights_analysis.settings.run_moments_of_inertia_analysis:
        moment_of_inertia_df = pd.DataFrame(columns=["Component", "Mass (kg)","Ixx (kg·m²)","Iyy (kg·m²)","Izz (kg·m²)","Ixy (kg·m²)","Ixz (kg·m²)","Iyz (kg·m²)", ])
        overwrite_MOI = False
        tensor = analyses.vehicle.mass_properties.moments_of_inertia.tensor
        if np.all(tensor == 0):
            overwrite_MOI = True
        if i != 0:
            verbose_flag = False
        else:
            verbose_flag = weights_analysis.print_weight_analysis_report
        _ ,moment_of_inertia_df = compute_vehicle_moment_of_inertia(analyses.vehicle,moment_of_inertia_df,
                                            overwrite_moment_of_intertia = overwrite_MOI,
                                            segment=segment,
                                            verbose=verbose_flag) 
        if i==0 and weights_analysis.settings.write_mass_properties:
            # Moment of Inertia sheet
            with pd.ExcelWriter(excel_filename, engine="openpyxl",mode="a",if_sheet_exists="replace") as writer:
                moment_of_inertia_df.to_excel(writer,sheet_name="Moment of Inertia",index=False)
            print(f"MOI breakdown written to Excel:\n  {excel_filename}") 
 

def iterate_max_fuel_and_max_zero_fuel(analyses, max_iterations=100):
    # Inital guess for max fuel and max zero fuel based on regressional analysis which use max takeoff weight of the aircraft
    compute_max_fuel = False
    if analyses.vehicle.mass_properties.max_fuel == None:
        analyses.vehicle.mass_properties.max_fuel =  0.477*analyses.vehicle.mass_properties.max_takeoff -13455
        compute_max_fuel = True
    analyses.vehicle.mass_properties.max_zero_fuel = 0.6269*analyses.vehicle.mass_properties.max_takeoff + 20505
    
    iteration = 0

    # Convergence loop
    while iteration < max_iterations:                               
        # Run weights analysis ! 
        _ = analyses.weights.evaluate(analyses.vehicle)
        
        # Compute OEW
        if analyses.weights.settings.overwrite_operating_empty_weight:
            analyses.vehicle.mass_properties.operating_empty = analyses.vehicle.mass_properties.weight_breakdown.empty.total
                        
        # Apply Correction Factors if any
        apply_correction_factors(analyses)
        apply_component_weights(analyses)

        analyses.vehicle.mass_properties.takeoff         = analyses.vehicle.mass_properties.operating_empty + analyses.vehicle.mass_properties.payload + analyses.vehicle.mass_properties.fuel                    
        mew_max_zero_fuel                                = analyses.vehicle.mass_properties.operating_empty + analyses.vehicle.mass_properties.max_payload
        residual_max_zero_fuel                           = abs(mew_max_zero_fuel - analyses.vehicle.mass_properties.max_zero_fuel)
        analyses.vehicle.mass_properties.max_zero_fuel   = mew_max_zero_fuel
        
        residual_max_fuel = 0
        if compute_max_fuel:
            new_max_fuel       = analyses.vehicle.mass_properties.max_takeoff - analyses.vehicle.mass_properties.operating_empty - analyses.vehicle.mass_properties.min_payload
            fuel_density       = next((ft.fuel.density for network in analyses.vehicle.networks for fl in network.fuel_lines for ft in fl.fuel_tanks), None)
            if fuel_density is not None:
                max_fuel_by_volume = analyses.vehicle.volume_properties.max_fuel * fuel_density
                new_max_fuel       = min(new_max_fuel, max_fuel_by_volume)
            residual_max_fuel  = abs(new_max_fuel - analyses.vehicle.mass_properties.max_fuel)
            analyses.vehicle.mass_properties.max_fuel = new_max_fuel
        
        iteration += 1
        if residual_max_fuel < 1 and residual_max_zero_fuel <1:
            break
        else:
            analyses.vehicle.mass_properties.max_zero_fuel += residual_max_zero_fuel * 0.1
            if compute_max_fuel:
                analyses.vehicle.mass_properties.max_fuel      += residual_max_fuel * 0.1

    return


def solve_for_mtow(analyses, weights_analysis, i):
    """Solves for MTOW that satisfies the target capacity fraction.

    The MTOW capacity fraction is MTOW / (OEW + Max Fuel + Max Payload).
    Uses brentq for superlinear convergence instead of fixed-point iteration.
    """

    target_fraction    = weights_analysis.settings.mtow_capacity_fraction
    max_zero_fuel_flag = analyses.vehicle.mass_properties.max_zero_fuel is None

    if max_zero_fuel_flag and i == 0:
        print('\n Warning: Max Fuel or Max Zero Fuel not defined. Iterating to find these values.')

    def _mtow_residual(mtow):
        analyses.vehicle.mass_properties.max_takeoff = mtow
        if max_zero_fuel_flag:
            analyses.vehicle.mass_properties.max_zero_fuel = None
            iterate_max_fuel_and_max_zero_fuel(analyses)
        _ = weights_analysis.evaluate(analyses.vehicle)
        analyses.vehicle.mass_properties.operating_empty = analyses.vehicle.mass_properties.weight_breakdown.empty.total
        apply_correction_factors(analyses)
        if i == 0:
            apply_component_weights(analyses)
        oew         = analyses.vehicle.mass_properties.operating_empty
        max_fuel    = analyses.vehicle.mass_properties.max_fuel
        max_payload = analyses.vehicle.mass_properties.max_payload
        return target_fraction - mtow / (oew + max_fuel + max_payload)

    mtow_0 = analyses.vehicle.mass_properties.max_takeoff
    try:
        mtow_converged = brentq(_mtow_residual, 0.5 * mtow_0, 1.5 * mtow_0, xtol=1.0)
    except ValueError:
        res = minimize_scalar(lambda m: _mtow_residual(m)**2,
                              bounds=(0.5 * mtow_0, 1.5 * mtow_0), method='bounded')
        mtow_converged = float(res.x)

    # Final evaluation to leave vehicle in correct state
    _mtow_residual(mtow_converged)
    analyses.vehicle.mass_properties.max_takeoff = mtow_converged

    return