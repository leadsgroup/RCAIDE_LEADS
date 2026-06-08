# rcaide_import_export_test.py
# 
# Created:  June 2026, M. Clarke  
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units ,  Data
from RCAIDE.Library.Plots             import *       

# python imports
import numpy as np
import sys
import os

# local imports 
base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Boeing_737    import vehicle_setup as vehicle_setup
from Boeing_737    import configs_setup as configs_setup 
 

#from RCAIDE.Library.Methods.Performance.compute_load_and_trim_diagram        import compute_load_and_trim_diagram
 
from RCAIDE.Input_Output import export, import_data
from copy import deepcopy
import matplotlib.pyplot as plt
import shutil

# ----------------------------------------------------------------------
#   Round-trip comparison
# ----------------------------------------------------------------------

_CMP_SKIP = frozenset({
    # Internal RCAIDE bookkeeping — never serialized
    '_component_root_map', '_energy_network_root_map', '_base', '_diff',
    # Back-references — excluded from export by design
    'vehicle',
    # Segment runtime state — excluded from export by design
    'analyses', 'state', 'conditions',
    # Analysis process trees — contain bound methods, not serializable
    'process',
})


def _walk_compare(orig, imp, path, mismatches, rtol=1e-6):
    """Recurse into two RCAIDE Data trees; collect (path, orig, imp) for each mismatch."""
    # Skip callables and type objects — these are never serialized
    if callable(orig) or isinstance(orig, type):
        return

    orig_is_map = hasattr(orig, 'keys')
    imp_is_map  = hasattr(imp,  'keys')

    if orig_is_map and imp_is_map:
        for k in orig.keys():
            if k in _CMP_SKIP:
                continue
            child_path = path + [str(k)]
            if k not in imp.keys():
                mismatches.append((child_path, orig[k], '<missing>'))
                continue
            _walk_compare(orig[k], imp[k], child_path, mismatches, rtol)
        return

    if isinstance(orig, np.ndarray) or isinstance(imp, np.ndarray):
        try:
            if not np.allclose(orig, imp, rtol=rtol, equal_nan=True):
                mismatches.append((path, orig, imp))
        except Exception:
            if not np.array_equal(orig, imp):
                mismatches.append((path, orig, imp))
        return

    if isinstance(orig, float) or isinstance(imp, float):
        try:
            if not np.isclose(float(orig), float(imp), rtol=rtol):
                mismatches.append((path, orig, imp))
        except Exception:
            if orig != imp:
                mismatches.append((path, orig, imp))
        return

    if orig != imp:
        mismatches.append((path, orig, imp))


def _report(label, mismatches):
    if not mismatches:
        print(f"  [PASS] {label}")
    else:
        print(f"  [FAIL] {label}  ({len(mismatches)} mismatch(es))")
        for path, orig, imp in mismatches[:5]:
            key = ' > '.join(path)
            print(f"         {key}")
            print(f"           orig     : {orig!r}")
            print(f"           imported : {imp!r}")
        if len(mismatches) > 5:
            print(f"         ... and {len(mismatches) - 5} more")


def compare_rcaide_data(vehicle, configs, analyses, missions, imported):
    """
    Compare original study objects (lines 30-40) against the round-trip result
    from import_rcaide_data.  Prints PASS/FAIL for each section and lists the
    first few mismatched leaf paths.
    """
    print("\n=== import_rcaide_data round-trip check ===")

    # Vehicle
    v_mm = []
    _walk_compare(vehicle, imported.vehicle, ['vehicle'], v_mm)
    _report('vehicle', v_mm)

    # Configurations
    for tag, cfg in configs.items():
        c_mm = []
        if tag not in imported.configurations.keys():
            print(f"  [FAIL] config '{tag}'  (<missing in imported>)")
            continue
        imp_cfg = imported.configurations[tag]
        _walk_compare(cfg, imp_cfg, ['configs', str(tag)], c_mm)
        _report(f"config '{tag}'", c_mm)

    # Analyses
    for tag, av in analyses.items():
        a_mm = []
        if tag not in imported.analyses.keys():
            print(f"  [FAIL] analyses '{tag}'  (<missing in imported>)")
            continue
        imp_av = imported.analyses[tag]
        _walk_compare(av, imp_av, ['analyses', str(tag)], a_mm)
        _report(f"analyses '{tag}'", a_mm)

    # Missions — compare segment tags and key numerical leaves
    for m_tag, mission in missions.items():
        if isinstance(mission, str):
            continue
        if m_tag not in imported.missions.keys():
            print(f"  [FAIL] mission '{m_tag}'  (<missing in imported>)")
            continue
        imp_mission = imported.missions[m_tag]
        orig_segs = list(mission.segments.keys())   if hasattr(mission, 'segments') else []
        imp_segs  = list(imp_mission.segments.keys()) if hasattr(imp_mission, 'segments') else []
        if orig_segs != imp_segs:
            print(f"  [FAIL] mission '{m_tag}' segment order/tags differ")
            print(f"         orig     : {orig_segs}")
            print(f"         imported : {imp_segs}")
            continue
        m_mm = []
        for seg_tag in orig_segs:
            orig_seg = mission.segments[seg_tag]
            imp_seg  = imp_mission.segments[seg_tag]
            _walk_compare(orig_seg, imp_seg,
                          ['missions', str(m_tag), 'segments', str(seg_tag)], m_mm)
        _report(f"mission '{m_tag}'", m_mm)

    print("===========================================\n")


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    
    # Step 1 design a vehicle
    vehicle  = vehicle_setup() 
      
    # Step 2 create aircraft configuration based on vehicle 
    configs  = configs_setup(vehicle)
    
    # Step 3 set up analysis
    analyses = analyses_setup(configs)
    
    # Step 4 set up a flight mission
    mission  = mission_setup(analyses)
    missions = missions_setup(mission) 
    

    _script_dir = os.path.dirname(os.path.abspath(__file__))
    _test_file  = os.path.join(_script_dir, 'RCAIDE_JSON_Test')

    export(vehicle=vehicle,
           configurations=configs,
           analyses=analyses,
           missions=missions,
           filename=_test_file)

    RCAIDE_JSON_Test = import_data(_test_file)
    compare_rcaide_data(vehicle, configs, analyses, missions, RCAIDE_JSON_Test) 

    
    # Step 5 execute flight profile
    results_original = missions.base_mission.evaluate()  
    results_loaded   = RCAIDE_JSON_Test.missions.base_mission.evaluate()  

    # Compare results 
    cruise_CL_original       = results_original.segments.cruise.conditions.aerodynamics.coefficients.lift.total[2][0]
    cruise_CL_loaded        = results_loaded.segments.cruise.conditions.aerodynamics.coefficients.lift.total[2][0] 
    
    print(f"Original cruise CL: {cruise_CL_original:.6f}")
    print(f"Loaded cruise CL:   {cruise_CL_loaded:.6f}")
    delta_CL = cruise_CL_loaded - cruise_CL_original
    print(f"Difference in cruise CL: {delta_CL:.6e}")

    # if different is more than 1e-6, consider it a failure 
    assert(np.abs(delta_CL)<1e-6)

    return 
    
 
# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in list(configs.items()):
        analysis = base_analysis(config)
        analyses[tag] = analysis
 
    return analyses


def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    analyses.vehicle =  vehicle
    
    # ------------------------------------------------------------------
    #  geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry() 
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Weights
    weights                                          = RCAIDE.Framework.Analyses.Weights.Conventional_Transport() 
    analyses.append(weights)
 
    #  Aerodynamics Analysis
    aerodynamics                                        = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()   
    aerodynamics.settings.number_of_spanwise_vortices   = 40
    aerodynamics.settings.number_of_chordwise_vortices  = 2 
    analyses.append(aerodynamics)
  
    #  Energy
    energy                                           = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)
 
    #  Planet Analysis
    planet                                           = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere                                       = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    # done!
    return analyses 
   
# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses): 

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'base_mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points = 4  
 
 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Cruise Segment 3 : Constant Speed Constant Altitude
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                                = 11. * Units.km    
    segment.air_speed                                               = 450 *Units.knots
    segment.distance                                                = 500 * Units.km   
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                 = True  
    segment.flight_dynamics.force_z                                 = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active              = True           
    segment.assigned_control_variables.throttle.assigned_propulsors = [['propulsor_1','propulsor_2']]
    segment.assigned_control_variables.throttle.initial_guess       = [[0.5]]
        
    mission.append_segment(segment)   
     
    return mission
 

def missions_setup(mission): 
 
    missions     = RCAIDE.Framework.Mission.Missions()
    missions.append(mission)
 
    return missions  


if __name__ == '__main__': 
    main()    
