# RCAIDE/Library/Methods/Weights/Correlation_Buildups/Common/compute_payload_weight.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------
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
            - mass_properties.cargo : float
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
        vehicle.mass_properties.payload  = W_pax + W_bag + vehicle.mass_properties.cargo
    else:
        vehicle.mass_properties.cargo = vehicle.mass_properties.payload - W_pax - W_bag

    # packup outputs
    output              = Data()
    output.total        = vehicle.mass_properties.payload
    output.passengers   = W_pax
    output.baggage      = W_bag
    output.cargo        = vehicle.mass_properties.cargo

    return output