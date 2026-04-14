# RCAIDE/Methods/Powertrain/Distributors/Electrical_Bus/calculate_electrical_cable_loss.py
#
# Created:  Apr 2026, S. Sharma

# Python Imports
import numpy as np

""" 
Note: This Function is used under:
RCAIDE/Methods/Powertrain/Distributors/Electrical_Bus/compute_electrical_bus_conditions.py

"""

def calculate_electrical_cable_loss(Electrical_Bus):
    """
    Used During Mission Solver:
    Calculates the actual power loss, voltage drop, and source power required during 
    a specific mission segment based on the fixed resistance of a pre-sized cable.

    Parameters
    ----------
    P_req_watts : float
        The instantaneous power demanded at the destination (e.g., cabin/motor) in Watts
        
    V_source : float
        The voltage provided at the source (e.g., generator/battery) in Volts
        
    R_total_ohms : float
        The fixed total electrical resistance of the cable line in Ohms

    Returns
    -------
    dict
        Dictionary containing the mission segment electrical states:
        
        - Mission_Current_Amps : float
            The actual current drawn through the line to meet destination power demands
        - Destination_Voltage : float
            The actual voltage at the destination after line losses
        - Power_Loss_Watts : float
            Energy lost as heat due to line resistance (I^2 * R)
        - Required_Source_Power_Watts : float
            Total power the source must generate to overcome losses and meet the destination demand

    Notes
    -----
    This solver uses the roots of a quadratic equation (R*I^2 - V*I + P = 0) to determine 
    the exact current draw. If the requested power exceeds the physical transmission 
    limits of the cable resistance, the discriminant becomes negative, indicating 
    a voltage collapse scenario.
    """
    P_req_watts = Electrical_Bus.mission_power
    V_source = Electrical_Bus.source_voltage
    R_total_ohms = Electrical_Bus.cable_resistance
    
    # Solve the quadratic equation: R_total * I^2 - V_source * I + P_req = 0
    a = R_total_ohms
    b = -V_source
    c = P_req_watts
    
    discriminant = b**2 - 4*a*c
    if discriminant < 0:
        raise ValueError("Voltage Collapse: Cable resistance is too high to deliver the requested power.")
        
    # The smaller root (the standard operating current)
    I_mission = (-b - np.sqrt(discriminant)) / (2*a)
    
    # Calculate Power Loss (I^2 * R)
    P_loss_watts = (I_mission**2) * R_total_ohms
    
    # Total power the engine/generator must output
    P_source_total_watts = P_req_watts + P_loss_watts
    
    # Voltage at the destination (eg: cabin)
    V_destination = V_source - (I_mission * R_total_ohms)
    
    cable_data = {
        'Mission_Current_Amps': I_mission,
        'Destination_Voltage': V_destination,
        'Power_Loss_Watts': P_loss_watts,
        'Required_Source_Power_Watts': P_source_total_watts
    }
    
    return cable_data