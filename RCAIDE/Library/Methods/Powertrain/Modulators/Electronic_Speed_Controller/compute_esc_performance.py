# RCAIDE/Methods/Energy/Propulsors/Modulators/compute_esc_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
# compute_electric_rotor_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_voltage_out_from_throttle(esc,esc_conditions,conditions):
    """
    Computes the output voltage of the Electronic Speed Controller (ESC) based on the 
    throttle setting. Implements a linear voltage modulation model where output voltage 
    is proportionally controlled by the throttle position.

    Parameters
    ----------
    esc : RCAIDE.Library.Components.Energy.Modulators.Electronic_Speed_Controller
        The electronic speed controller component
    esc_conditions : RCAIDE.Framework.Mission.Common.Conditions
        ESC-specific operating conditions
            - throttle : float
                Power modulation setting [0-1]
            - inputs : Conditions
                Input parameters
                    - voltage : float
                        Input voltage [V]
            - outputs : Conditions
                Output parameters
                    - voltage : float
                        Output voltage [V]
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions (not directly used but maintained for API consistency)

    Returns
    -------
    None

    Notes
    -----
    The function performs these operations:
        1. Clamps throttle between 0 and 1
        2. Computes output voltage as a linear function of throttle
        3. Updates ESC conditions with new values

    **Major Assumptions**
        * Linear relationship between throttle and output voltage
        * No voltage drop across the ESC
        * Instantaneous throttle response
        * Perfect voltage regulation
        * Throttle values outside [0,1] are clamped

    **Definitions**
      
    'Voltage Modulation'
        Process of controlling output voltage through throttle position
    """
    
    eta        = esc_conditions.throttle * 1.0
    
    # Negative throttle is bad
    eta[eta<=0.0] = 0.0
    
    # Cap the throttle
    eta[eta>=1.0] = 1.0
    
    # Pack the output
    esc_conditions.outputs.voltage  =eta*esc_conditions.inputs.voltage
    esc_conditions.throttle         = eta 
    
    return
 

# ----------------------------------------------------------------------------------------------------------------------
# compute_current_in_from_throttle
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_current_in_from_throttle(esc,esc_conditions,conditions):
    """ The current going into the speed controller
    
        Assumptions:
            The ESC draws current.
        
        Inputs:
            esc_conditions.inputs.currentout [amps]
           
        Outputs:
            outputs.currentin      [amps]
        
        Properties Used:
            esc.efficiency - [0-1] efficiency of the ESC
           
    """
    
    # Unpack, don't modify the throttle
    eta        = esc_conditions.throttle
    eff        = esc.efficiency
    currentout = esc_conditions.outputs.current 
    currentin  = currentout*eta/eff # The inclusion of eta satisfies a power balance: p_in = p_out/eff
    
    # Pack 
    esc_conditions.inputs.current   = currentin
    esc_conditions.inputs.power     = esc_conditions.inputs.voltage *currentin
    
    return