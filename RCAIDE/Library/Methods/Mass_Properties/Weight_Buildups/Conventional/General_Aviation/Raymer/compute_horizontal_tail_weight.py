# RCAIDE/Library/Methods/Weights/Correlation_Buildups/General_Aviation/compute_horizontal_tail_weight.py
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
# Horizontal Tail Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_horizontal_tail_weight(wing, vehicle):       
    """ 
        Calculate the weight of the horizontal tail for a General Aviation-type aircraft
        
        Source:
            Aircraft Design: A Conceptual Approach by Raymer (pg 460 in 4th edition)
        
      
        Inputs:
            S_h  = trapezoidal area of horizontal tail           [meters**2]
            Ar_h = aspect ratio of horizontal tail               [dimensionless]
            sweep_h = quarter chord sweep of the horizontal tail [radians]
            q_c  = dynamic pressure at cruise                    [Pascals]
            
            
        Outputs:
            weight - weight of the horizontal tail               [kilograms]
            
        Assumptions:
            calculated weight of the horizontal tail including the elevator
            Assume that the elevator is 25% of the horizontal tail
        
    """
    # unpack inputs
    S_h                = wing.areas.reference
    b_h                = wing.spans.projected
    AR_h               = (b_h**2.)/S_h
    taper_h            = wing.spans.projected
    sweep_h            = wing.sweeps.quarter_chord 
    t_c_h              = wing.thickness_to_chord
    q_c                = vehicle.design_dynamic_pressure
    Nult               = vehicle.flight_envelope.ultimate_load 
    TOW                = vehicle.mass_properties.max_takeoff

    W_0   = TOW / Units.lb # Convert kg to lbs
    S_ht  = S_h/ Units.ft**2 # Convert from meters squared to ft squared  
    q     = q_c /(Units.force_pound / Units.ft**2.)

    #Calculate weight of wing for traditional aircraft horizontal tail
    weight_English = .016*((Nult*W_0)**.414)*(q**.168)*(S_ht**.896)*((100.*t_c_h/np.cos(sweep_h))**(-.12))*((AR_h/(np.cos(sweep_h)**2))**.043)*(taper_h**(-.02))
    weight         = weight_English * Units.lbs # Convert from lbs to kg

    return weight