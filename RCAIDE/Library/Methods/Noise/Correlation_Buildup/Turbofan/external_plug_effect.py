# RCAIDE/Methods/Noise/Correlation_Buildup/Engine/external_plug_effect.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Framework.Core  import Data

# Python package imports   
import numpy as np   

# ----------------------------------------------------------------------------------------------------------------------     
#  External Plug Effect
# ----------------------------------------------------------------------------------------------------------------------       
def external_plug_effect(Velocity_primary, Velocity_secondary, Velocity_mixed, Diameter_primary,
                         Diameter_secondary, Diameter_mixed, Plug_diameter, sound_ambient, theta_p, theta_s, theta_m):
    """
    This function calculates the adjustments, in decibels, to be added to the predicted jet noise levels due to external plugs in coaxial jets.

    Parameters
    ----------
    Velocity_primary : float
        Velocity of the primary jet [m/s].
    Velocity_secondary : float
        Velocity of the secondary jet [m/s].
    Velocity_mixed : float
        Velocity of the mixed jet [m/s].
    Diameter_primary : float
        Diameter of the primary jet [m].
    Diameter_secondary : float
        Diameter of the secondary jet [m].
    Diameter_mixed : float
        Diameter of the mixed jet [m].
    Plug_diameter : float
        Diameter of the external plug [m].
    sound_ambient : float
        Ambient sound level [dB].
    theta_p : float
        Angle for the primary jet [rad].
    theta_s : float
        Angle for the secondary jet [rad].
    theta_m : float
        Angle for the mixed jet [rad].

    Returns
    -------
    jet_plug_effects : Data
        Contains the noise adjustments for each jet component.
            - PG_p : float
                Adjustment for the primary jet [dB].
            - PG_s : float
                Adjustment for the secondary jet [dB].
            - PG_m : float
                Adjustment for the mixed jet [dB].

    Notes
    -----
    The function assumes that the external plug effect is significant for coaxial jets and calculates the noise adjustments accordingly.

    **Definitions**

    'PG'
        Plug Gain, the adjustment in decibels due to the presence of an external plug.

    References
    ----------
    [1] SAE ARP876D: Gas Turbine Jet Exhaust Noise Prediction (original)
    [2] de Almeida, Odenir. "Semi-empirical methods for coaxial jet noise prediction." (2008). (adapted)
    """

    # Primary jet
    PG_p = 0.1*(Velocity_primary/sound_ambient)*(10-(18*theta_p/np.pi))*Plug_diameter/Diameter_primary
    
    # Secondary jet
    PG_s = 0.1*(Velocity_secondary/sound_ambient)*(6-(18*theta_s/np.pi))*Plug_diameter/Diameter_secondary
    
    # Mixed jet
    PG_m = 0.1*(Velocity_primary*Velocity_mixed/(sound_ambient**2))*(9-(18*theta_m/np.pi))*Plug_diameter/Diameter_mixed
    
    # Pack Results 
    jet_plug_effects = Data()
    jet_plug_effects.PG_p = PG_p
    jet_plug_effects.PG_s = PG_s
    jet_plug_effects.PG_m = PG_m 

    return jet_plug_effects
