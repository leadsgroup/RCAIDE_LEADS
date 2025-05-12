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

    # process
    num_pax    = vehicle.passengers
    W_pax      = W_passenger * num_pax
    W_bag      = W_baggage * num_pax
    if vehicle.mass_properties.payload == 0:
        vehicle.mass_properties.payload  = W_pax + W_bag
        W_cargo = 0
    else:
        W_cargo = vehicle.mass_properties.payload - W_pax - W_bag
     
    # check if cargo bays defined in aircraft, if none, define one 
    if len(vehicle.cargo_bays) == 0:
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