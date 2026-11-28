# RCAIDE/Library/Methods/Powertrain/Converters/DC_generator/design_optimal_generator.py
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE

# python imports  
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  design generator 
# ----------------------------------------------------------------------------------------------------------------------     
def design_optimal_generator(generator, minimum_resistance=0.001):
    """
    Sizes a generator: the speed constant and resistance that give the design power at the design
    angular velocity and nominal voltage with the specified efficiency.
    
    Parameters
    ----------
    generator : RCAIDE.Library.Components.Powertrain.Converters.Generator
        Generator component with the following attributes:
            - no_load_current : float
                No-load current [A]
            - nominal_voltage : float
                Nominal voltage [V]
            - angular_velocity : float
                Angular velocity [radians/s]
            - efficiency : float
                Efficiency [unitless]
            - design_torque : float
                Design torque [N·m]
            - gearbox : Data
                Gearbox component
                    - gear_ratio : float
                        Gear ratio [unitless]
            - design_angular_velocity : float
                Design angular velocity [radians/s]
            - design_power : float
                Design power [W]
    minimum_resistance : float, optional
        Resistance given to a generator sized without a design load [ohms]. Default is 0.001.
    
    Returns
    -------
    generator : RCAIDE.Library.Components.Powertrain.Converters.Generator
        Generator with updated attributes:
            - speed_constant : float
                Speed constant [unitless]
            - resistance : float
                Resistance [ohms]
    
    Notes
    -----
    The two design conditions fix the two unknowns, so the solution is direct.
    
    **Major Assumptions**
        * The generator follows a DC generator model
    
    **Theory**
    The generator model uses the relationships of Drela's DC machine model [1]:
        - Current: :math:`I = (V - \\omega/Kv)/R`
        - Power: :math:`P = \\omega\\cdot (I - I_0)/Kv`
        - Efficiency: :math:`\\eta = (1 - I_0/I)\\cdot(\\omega/(V\\cdot Kv)) = P/(V I)`
    
    so that
        - :math:`I = P/(\\eta V)`
        - :math:`Kv = \\omega (I - I_0)/P`
        - :math:`R = (V - \\omega/Kv)/I`
    
    where V is the nominal voltage, ω the angular velocity of the generator, Kv the speed constant,
    R the resistance, I₀ the no-load current, P the power and η the efficiency.
    
    This solution exists only if the no-load current is below :math:`(1 - \\eta) I`, since the no-load
    loss alone must be smaller than the total loss the efficiency allows. Otherwise the generator is sized
    so that the design point is its most efficient one. From Eq. (5) of [1], the efficiency at a given
    voltage peaks where :math:`I^2 R = I_0 V`, and is then :math:`(1 - I_0/I)^2`. With the design power this
    gives :math:`(I - I_0)^2 = (P/V) I`, then :math:`R = I_0 V/I^2` and :math:`Kv = \\omega (I - I_0)/P`.
    
    A generator with no design power, or a negative one, is given a speed constant that reaches the design
    speed at the nominal voltage, and the minimum resistance.
    
    References
    ----------
    [1] Drela, M., "DC Motor / Propeller Matching," Lab 5 Lecture Notes, MIT, 3 Mar. 2005.
        https://web.mit.edu/drela/Public/web/qprop/motorprop.pdf
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Generator.compute_generator_performance
    """
    
    if type(generator) == RCAIDE.Library.Components.Powertrain.Converters.Generator and generator.voltage_type != 'DC':
        raise Exception('function only supports low-fidelity (DC) generator')
    
    # design properties of the generator 
    io     = generator.no_load_current
    v      = generator.nominal_voltage  
    G      = generator.gearbox.gear_ratio      
    omega  = generator.design_angular_velocity /G     
    etam   = generator.efficiency 
    P      = generator.design_power
    
    current = P / (etam * v)
    if P <= 0:
        generator.speed_constant = omega / v
        generator.resistance     = minimum_resistance
    elif io < (1 - etam) * current:
        generator.speed_constant = omega * (current - io) / P
        generator.resistance     = (v - omega / generator.speed_constant) / current
    else:
        # the efficiency cannot be met: make the design point the generator's most efficient one
        c                        = P / v
        current                  = ((2 * io + c) + np.sqrt((2 * io + c)**2 - 4 * io**2)) / 2
        generator.resistance     = io * v / current**2
        generator.speed_constant = omega * (current - io) / P
        print('\n Generator ' + generator.tag + ' cannot reach an efficiency of ' + str(etam) + ' with a no-load current of ' + str(io)
              + ' A. Sized for its peak efficiency of ' + str((1 - io / current)**2) + ' at the design point')
    
    return generator  
