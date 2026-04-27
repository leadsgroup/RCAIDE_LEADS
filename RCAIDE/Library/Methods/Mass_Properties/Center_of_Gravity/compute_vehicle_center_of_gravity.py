# RCAIDE/Library/Methods/Mass_Properties/Center_of_Gravity/compute_vehicle_center_of_gravity.py 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports      
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity.compute_component_center_of_gravity import compute_component_center_of_gravity 

# package imports 
import numpy as np  
import pandas as pd

# ----------------------------------------------------------------------------------------------------------------------
#  Computer Aircraft Center of Gravity
# ----------------------------------------------------------------------------------------------------------------------   
def compute_vehicle_center_of_gravity(vehicle,centre_of_gravity_df, overwrite_center_of_gravity=True,segment=None,verbose=True,include_payload = True, include_fuel = True): 
    ''' Computes the moment of inertia of aircraft 
    
    Source:
    Simplified Mass and Inertial Estimates for Aircraft with Components of Constant Density
    Moulton, B. C., and Hunsaker, D. F., “Simplified Mass and Inertial Estimates for Aircraft with Components 
    of Constant Density,” AIAA SCITECH 2023 Forum, January 2023, AIAA-2023-2432 DOI: 10.2514/
    6.2023-2432
    
    
    Assumtions:
    Assumes simplified shapes 
    
    Inputs:
    vehicle           - vehicle data structure           [m]
    
    Outputs:
    I                 - mass moment of inertia matrix    [kg-m^2]
    
    '''
    if verbose:
        print("\n\n=== COMPONENT CENTER OF GRAVITY BREAKDOWN REPORT ===" )    
        print("Component \t \t \t Mass \t \t C.G. Location [[x,y,z]]" )    
     
    # --------------------------------------------------------------------------------------
    # Center of Gravity at Operating Empty Weight 
    # --------------------------------------------------------------------------------------
    OEW_moment      = np.array([[0.0,0.0,0.0]])
    OEW_mass        = np.array([0.0])
    for key in vehicle.keys():
        item = vehicle[key]  
        OEW_mass,OEW_moment = compute_component_center_of_gravity(centre_of_gravity_df,item,vehicle,OEW_mass,OEW_moment,None,False,False,False)    
    
    # center of gravity
    OEW_CG = OEW_moment / OEW_mass 
    OEW_mass_percentage = (OEW_mass[0] / vehicle.mass_properties.operating_empty) * 100 
     
    # --------------------------------------------------------------------------------------    
    # Mission Center of Gravity 
    # --------------------------------------------------------------------------------------
    mission_moment = np.array([[0.0,0.0,0.0]])
    dummy_pd = pd.DataFrame(columns=[
        "Component",
        "Mass (kg)",
        "CG x (m)",
        "CG y (m)",
        "CG z (m)"
        ])
    mission_mass   = np.array([0.0])                
    for key in vehicle.keys():
        item = vehicle[key]  
        mission_mass,mission_moment = compute_component_center_of_gravity(dummy_pd,item,vehicle,mission_mass,mission_moment,segment,verbose,include_payload,include_fuel)    
    
    # print center of gravity 
    CG = mission_moment /mission_mass 
    centre_of_gravity_df = centre_of_gravity_df[centre_of_gravity_df["Mass (kg)"] != 0].reset_index(drop=True)
    if verbose:
        print('\n*************** Center of Gravity *************** ')
        print('OEW Center of Gravity            : ', OEW_CG) 
        print('% Mass used in OEW CG calculation: ', round(OEW_mass_percentage,2), '%')  
        print('Mission Center of Gravity        : ', CG)   
    centre_of_gravity_df.loc[len(centre_of_gravity_df)] = [
                'Operating_Empty',
                round(OEW_mass[0], 2),
                OEW_CG[0][0],
                OEW_CG[0][1],
                OEW_CG[0][2],
            ]
                  
    if segment != None:         
        ones_row  = segment.state.ones_row  
        segment.state.conditions.weights.vehicle.global_center_of_gravity = CG * ones_row(1)
            
    if overwrite_center_of_gravity and (mission_mass != 0.0): 
        vehicle.mass_properties.center_of_gravity = CG.tolist()
        vehicle.mass_properties.operating_empty_center_of_gravity = OEW_CG.tolist()
        
    return vehicle.mass_properties.center_of_gravity, mission_mass, mission_moment, centre_of_gravity_df 
