# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Conventional/Common/compute_payload_weight.py
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
    """
    Computes the total payload weight including passengers, baggage, and cargo based on 
    FAA standard weights and aircraft configuration.

    Parameters
    ----------
    vehicle : Vehicle
        The vehicle instance containing:
            - passengers : int
                Number of passengers 
                Mass of cargo [kg]
    W_passenger : float, optional
        Standard passenger weight [kg], default 195 lbs
    W_baggage : float, optional
        Standard baggage weight per passenger [kg], default 30 lbs

    Returns
    -------
    output : Data
        Container with payload breakdown:
            - total : float
                Total payload weight [kg]
            - passengers : float
                Total passenger weight [kg]
            - baggage : float
                Total baggage weight [kg]
            - cargo : float
                Bulk cargo weight [kg]

    Notes
    -----
    Uses FAA standard weights for passengers and baggage in commercial operations.

    **Major Assumptions**
        * Standard passenger weights
        * Fixed baggage allowance per passenger
        * Uniform passenger distribution
        * No special cargo requirements
        * No seasonal weight variations

    **Theory**
    Total payload weight is computed as:
    .. math::
        W_{payload} = n_{pax}(W_{pax} + W_{bag}) + W_{cargo}

    where:
        * n_pax = number of passengers
        * W_pax = standard passenger weight
        * W_bag = standard baggage allowance
        * W_cargo = bulk cargo weight

    See Also
    --------
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional
    """
 
    num_pax    = vehicle.number_of_passengers
    W_pax      = W_passenger * num_pax
    W_bag      = W_baggage * num_pax
    
    if W_pax + W_bag + vehicle.mass_properties.cargo> vehicle.mass_properties.payload:
        print('Sum of Cargo and Number of passengers defined will result in excess payload than defined.')

    if W_pax + W_bag > vehicle.mass_properties.payload:
        print('Number of passengers defined will result in excess payload than defined.')
        vehicle.mass_properties.cargo = 0
    else:
        if vehicle.mass_properties.cargo == 0 and vehicle.mass_properties.payload != 0:
            vehicle.mass_properties.cargo = vehicle.mass_properties.payload - W_pax - W_bag  
    
    #-------------------------------------------------------------------------------   
    # Cargo
    #-------------------------------------------------------------------------------         
    
    total_volume =  0
    for cargo_bay in vehicle.cargo_bays:
        total_volume += (cargo_bay.length * cargo_bay.width * cargo_bay.height) 
    for cargo_bay in vehicle.cargo_bays:
        cargo_bay_volume   = (cargo_bay.length * cargo_bay.width * cargo_bay.height)  
        if cargo_bay.mass_properties.mass == 0: 
            cargo_bay.mass_properties.mass   = (vehicle.mass_properties.cargo+W_bag) * (cargo_bay_volume / total_volume) # WE put the bags in cargo because thats where they go
        
    #-------------------------------------------------------------------------------   
    # Paylpad 
    #-------------------------------------------------------------------------------                
    if vehicle.mass_properties.payload == 0: 
        vehicle.mass_properties.payload  = W_pax + W_bag + vehicle.mass_properties.cargo
            
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