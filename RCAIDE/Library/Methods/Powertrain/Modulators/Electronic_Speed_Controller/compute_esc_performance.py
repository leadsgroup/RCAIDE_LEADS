# RCAIDE/Methods/Energy/Propulsors/Modulators/compute_esc_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
# compute_electric_rotor_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_voltage_out_from_throttle(esc,esc_conditions,conditions):
    """ The voltage out of the electronic speed controller
    
        Assumptions:
        The ESC's output voltage is linearly related to throttle setting

        Source:
        N/A

        Inputs:
        conditions.energy.throttle     [0-1] 
        esc_conditions.inputs.voltage            [volts]

        Outputs:
        voltsout                       [volts]
        esc_conditions.outputs.voltageout        [volts]

        Properties Used:
        None
       
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