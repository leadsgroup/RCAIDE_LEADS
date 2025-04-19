# RCAIDE/Library/Methods/Weights/Correlation_Buildups/General_Aviation/compute_main_wing_weight.py
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
# Main Wing Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_main_wing_weight(wing, vehicle, m_fuel):      
    """ 
        Calculate the weight of the main wing of an aircraft     

        Source:
            Raymer, Aircraft Design: A Conceptual Approach (pg 460 in 4th edition)
        
        Inputs:
            S_wing- area of the main wing                                              [meters**2]
            m_fuel - predicted weight of fuel in the wing                              [kilograms]
            AR_w -aspect ratio of main wing                                            [dimensionless]
            sweep_w - quarter chord sweep of the main wing                             [radians]
            q_c - dynamic pressure at cruise                                           [Pascals]
            taper_w - taper ratio of wing                                              [dimensionless]
            t_c_w -thickness to chord ratio of wing                                    [dimensionless]
            Nult - ultimate load of the aircraft                                       [dimensionless]
            TOW - maximum takeoff weight of the aircraft                               [kilograms]
   
        Outputs:
            output - a dictionary with outputs:
                W_main_wing - weight of the vertical fin portion of the vertical tail [kilograms]
    """     
    # unpack inputs 

    S_wing     = vehicle.reference_area
    b          = wing.spans.projected
    AR_w       = (b**2.)/S_wing
    taper_w    = wing.taper
    t_c_w      = wing.thickness_to_chord
    sweep_w    = wing.sweeps.quarter_chord 
    q_c        = vehicle.design_dynamic_pressure
    Nult       = vehicle.flight_envelope.ultimate_load 
    TOW        = vehicle.mass_properties.max_takeoff
    
    W_0  = TOW / Units.lb # Convert kg to lbs
    S_w  = S_wing / (Units.ft**2) # Convert from meters squared to ft squared  
    W_fw = m_fuel/Units.lbs #convert from kg to lbs
    q    = q_c /(Units.lbs/(Units.ft**2.))

    # Calculate weight of wing for traditional aircraft vertical tail without rudder
    weight_English = .036 * (S_w**.758)*(W_fw**.0035)*((AR_w/(np.cos(sweep_w)**2))**.6)*(q**.006)*(taper_w**.04)*((100.*t_c_w/np.cos(sweep_w))**(-.3))*((Nult*W_0)**.49)
    
    # packup outputs    
    weight =  weight_English * Units.lbs # Convert from lbs to kg

    return weight