# RCAIDE/Methods/Aerodynamics/Common/Drag/parasite_drag_total.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
import numpy as np 
 
# package imports
import numpy as np  

# ----------------------------------------------------------------------------------------------------------------------  
#  Total Parasite Drag 
# ----------------------------------------------------------------------------------------------------------------------   
def parasite_total(state,settings,geometry):
    """Sums up the parasite drags from all Components 

    Assumptions:
        None

    Source:
        None

    Args:
        state                                                (dict): flight conditions       [-]
        settings                                             (dict): analyses settings       [-]
        geometry                                             (dict): aircraft data stucture  [-]
        conditions.aerodynamics.coefficients.drag.
          parasite[wing.tag(s)].parasite_drag       (numpy.ndarray): wing parasite drags     [Unitless]
          parasite[fuselage.tag(s)].parasite_drag   (numpy.ndarray): fuselage parasite drags [Unitless]
          parasite[boom.tag(s)].parasite_drag       (numpy.ndarray): boom parasite drags     [Unitless]
          parasite[nacelle.tag(s)].parasite_drag    (numpy.ndarray): nacelle parasite drags  [Unitless]
          parasite[pylon.tag(s)].parasite_drag      (numpy.ndarray): pylon parasite drags    [Unitless]


    Returns:
        None 
    """ 
    # unpack
    conditions             = state.conditions 
    vehicle_reference_area = geometry.reference_area
    
    #compute parasite drag total
    total_parasite_drag = 0.0
    
    # renormalize parasite drag from wings using reference area of aircraft 
    for wing in geometry.wings:
        wing_parasite_drag = conditions.aerodynamics.coefficients.drag.parasite[wing.tag].total 
        conditions.aerodynamics.coefficients.drag.parasite[wing.tag].total  = wing_parasite_drag * wing.areas.reference/vehicle_reference_area
        total_parasite_drag += wing_parasite_drag * wing.areas.reference/vehicle_reference_area
 
    # renormalize parasite drag from fuselages using reference area of aircraft 
    for fuselage in geometry.fuselages:
        fuselage_parasite_drag = conditions.aerodynamics.coefficients.drag.parasite[fuselage.tag].total 
        conditions.aerodynamics.coefficients.drag.parasite[fuselage.tag].total = fuselage_parasite_drag * fuselage.areas.front_projected/vehicle_reference_area
        total_parasite_drag += fuselage_parasite_drag * fuselage.areas.front_projected/vehicle_reference_area
    
    # renormalize parasite drag from nacelles and pylons using reference area of aircraft  
    for network in  geometry.networks: 
        for propulsor in network.propulsors:  
            if 'nacelle' in propulsor:
                nacelle       = propulsor.nacelle
                ref_area      = np.pi * nacelle.diameter * nacelle.length 
                nacelle_parasite_drag = conditions.aerodynamics.coefficients.drag.parasite[nacelle.tag].total * (1+ settings.pylon_parasite_drag_factor) 
                conditions.aerodynamics.coefficients.drag.parasite[nacelle.tag].total  = nacelle_parasite_drag * nacelle_parasite_drag/vehicle_reference_area
                total_parasite_drag += nacelle_parasite_drag * ref_area/vehicle_reference_area
                
    state.conditions.aerodynamics.coefficients.drag.parasite.total = total_parasite_drag * (1 -  settings.drag_reduction_factors.parasite_drag)

    return 