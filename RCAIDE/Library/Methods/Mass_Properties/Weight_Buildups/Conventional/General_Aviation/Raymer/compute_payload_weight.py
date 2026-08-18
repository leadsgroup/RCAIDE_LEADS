# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Conventional/General_Aviation/Raymer/compute_payload_weight.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data, Units 

# ---------------------------------------------------------------------------------------------------------------------- 
# Payload
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_payload_weight(vehicle, W_passenger=195 * Units.lbs, W_baggage=30 * Units.lbs):
    """ Calculate the weight of the payload and the resulting fuel mass
    
    Assumptions:
        based on FAA guidelines for weight of passengers
        
    Source: 
        N/A
        
    Inputs:
        TOW -                                                              [kilograms]
        W_empty - Operating empty weight of the aircraft                  [kilograms]
        num_pax - number of passengers on the aircraft                     [dimensionless]
        W_cargo - weight of cargo being carried on the aircraft           [kilogram]
        W_passenger - weight of each passenger on the aircraft            [kilogram]
        W_baggage - weight of the baggage for each passenger              [kilogram]
    
    Outputs:
        output - a data dictionary with fields:
            payload - weight of the passengers plus baggage and paid cargo [kilograms]
            pax - weight of all the passengers                             [kilogram]
            bag - weight of all the baggage                                [kilogram]
            fuel - weight of the fuel carried                              [kilogram]
            empty - operating empty weight of the aircraft                 [kilograms]
               
    Properties Used:
        N/A
    """

    # process
    num_pax    = vehicle.number_of_passengers
    W_pax      = W_passenger * num_pax
    W_bag      = W_baggage * num_pax  
            
    if vehicle.mass_properties.cargo == 0:
        vehicle.mass_properties.cargo = W_bag 
        
    if vehicle.mass_properties.payload == 0: 
        vehicle.mass_properties.payload  = W_pax + W_bag  
        
    ##-------------------------------------------------------------------------------   
    # Cabin
    ##------------------------------------------------------------------------------- 
    for fuselage in vehicle.fuselages:
        for cabin in fuselage.cabins:  
            cabin.mass_properties.mass = W_pax* (cabin.number_of_passengers / vehicle.number_of_passengers )              
    for wing in vehicle.wings:
        if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            for cabin in wing.cabins:  
                cabin.mass_properties.mass = W_pax * (cabin.number_of_passengers / vehicle.number_of_passengers )
                
    # packup outputs
    output              = Data()
    output.total        = vehicle.mass_properties.payload
    output.passengers   = W_pax
    output.baggage      = W_bag
    output.cargo        = vehicle.mass_properties.cargo

    return output