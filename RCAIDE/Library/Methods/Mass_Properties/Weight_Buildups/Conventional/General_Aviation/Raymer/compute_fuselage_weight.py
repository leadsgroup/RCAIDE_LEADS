# RCAIDE/Library/Methods/Weights/Correlation_Buildups/General_Aviation/compute_fuselage_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE 
from RCAIDE.Framework.Core import  Units
import  numpy as  np

# ----------------------------------------------------------------------------------------------------------------------
# Fuselage Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_fuselage_weight(fuselage, vehicle, l_ht):
    """ 
        Calculate the weight of a fuselage for a GA aircraft
        
        Source: Raymer: Aircraft Design, a Conceptual Approach (pages 460-461 in 4th edition)
        
        Inputs:
            S_f - fuselage wetted area                          [meters**2]
            Nult - ultimate load of the aircraft                [dimensionless]]
            TOW - maximum takeoff weight of the aircraft        [kilograms]
            w_fus - width of the fuselage                       [meters]
            h_fus - height of the fuselage                      [meters]
            l_fus - length of the fuselage                      [meters]
            l_ht  - length of tail arm                          [meters]
            q_c    - dynamic pressure at cruise                 [Pascals] 
            V_fuse - volume of pressurized cabin                [meters**3]
            diff_p_fus - Maximum fuselage pressure differential [Pascals]
           

            
        Outputs:
            weight - weight of the fuselage [kilograms]
            
         Assumptions:
            fuselage for a general aviation type aircraft
    """ 
    S_fus       = fuselage.areas.wetted
    diff_p_fus  = fuselage.differential_pressure # accounts for pressurization
    w_fus       = fuselage.width
    h_fus       = fuselage.heights.maximum 
    l_fus       = fuselage.lengths.total-fuselage.lengths.tail  
    V_fuse      = fuselage.mass_properties.volume 
    num_seats   = fuselage.number_coach_seats  

    Nult       = vehicle.flight_envelope.ultimate_load 
    TOW        = vehicle.mass_properties.max_takeoff
    q_c        = vehicle.design_dynamic_pressure
    
    # take average as diameter
    d_fus    = (h_fus+w_fus)/2.
    
    # obtained from http://homepage.ntlworld.com/marc.barbour/structures.html
    d_str    = .025*d_fus+1.*Units.inches
    
    diff_p   = diff_p_fus / (Units.force_pound / Units.ft**2.) # Convert Pascals to lbs/ square ft 
    tail_arm = np.abs(l_ht)/Units.ft 
    weight   = TOW / Units.lb    # Convert kg to lbs
    area     = S_fus / (Units.ft**2.) # Convert square meters to square ft 
    q        = q_c /(Units.force_pound / Units.ft**2.)

    # Calculate weight of wing for traditional aircraft vertical tail without rudder
    fuselage_weight = .052*(area**1.086)*((Nult*weight)**.177)*(tail_arm**(-.051))*((l_fus/d_str)**(-.072))*(q**.241)+11.9*((V_fuse* diff_p)**.271)

    weight = fuselage_weight*Units.lbs #convert to kg
    return weight