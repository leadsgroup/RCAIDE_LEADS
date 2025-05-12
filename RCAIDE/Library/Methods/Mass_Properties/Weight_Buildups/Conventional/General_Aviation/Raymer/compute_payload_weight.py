# RCAIDE/Library/Methods/Weights/Correlation_Buildups/Common/compute_payload_weight.py
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
    num_pax    = vehicle.passengers
    W_pax      = W_passenger * num_pax
    W_bag      = W_baggage * num_pax
    if vehicle.mass_properties.payload == 0:
        vehicle.mass_properties.payload  = W_pax + W_bag
        W_cargo =  0
    else:
        W_cargo = vehicle.mass_properties.payload - W_pax - W_bag
        
    # -----------------------------------------------------------------------      
    # Cargo
    # -----------------------------------------------------------------------
    # check if cargo bays defined in aircraft, if none, define one 
    if len(vehicle.cargo_bays) == None:
        cargo_bay =  RCAIDE.Library.Components.Cargo_Bays.Cargo_Bay()
        vehicle.cargo_bays.append(cargo_bay) 
             
    total_volume =  0
    for cargo_bay in vehicle.cargo_bays:
        total_volume += (cargo_bay.length * cargo_bay.width * cargo_bay.height)
    
    for cargo_bay in vehicle.cargo_bays:
        cargo_bay_volume = (cargo_bay.length * cargo_bay.width * cargo_bay.height)
        cargo_bay.mass_properties.mass         = (W_cargo + W_bag) * (cargo_bay_volume / total_volume)
        cargo_bay.baggage.mass_properties.mass = W_bag * (cargo_bay_volume / total_volume)
        cargo_bay.cargo.mass_properties.mass   = W_cargo * (cargo_bay_volume / total_volume)

    # packup outputs
    output              = Data()
    output.total        = vehicle.mass_properties.payload
    output.passengers   = W_pax
    output.baggage      = W_bag
    output.cargo        = W_cargo

    return output