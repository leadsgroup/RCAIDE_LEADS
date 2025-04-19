# RCAIDE/Library/Methods/Weights/Correlation_Buildups/FLOPS/ccompute_propulsion_system_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE
import  RCAIDE 
from RCAIDE.Framework.Core    import Units ,  Data

# python imports 
import  numpy as  np
 
# ----------------------------------------------------------------------------------------------------------------------
#  Propulsion Systems Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_propulsion_system_weight(vehicle,network):
    """
    Calculate the weight of the complete propulsion system using FLOPS methodology.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure containing:
            - networks : list
                List of all propulsion networks
            - design_mach_number : float
                Design cruise Mach number
            - mass_properties.max_zero_fuel : float
                Maximum zero fuel weight [kg]
    network : RCAIDE.Network()
        Network data structure

    Returns
    -------
    output : Data()
        Data structure containing:
            - W_prop : float
                Total propulsion system weight [kg]
            - W_thrust_reverser : float
                Thrust reverser weight [kg]
            - W_starter : float
                Starter engine weight [kg]
            - W_engine_controls : float
                Engine controls weight [kg]
            - W_fuel_system : float
                Fuel system weight [kg]
            - W_nacelle : float
                Nacelle weight [kg]
            - W_engine : float
                Dry engine weight [kg]
            - number_of_engines : int
                Total number of engines
            - number_of_fuel_tanks : int
                Total number of fuel tanks

    Notes
    -----
    Calculates weights for all propulsion system components based on engine type
    and configuration.
    
    **Major Assumptions**
        * No mixed propulsion (either all piston or all turbine)
        * Piston engines have no thrust reversers, engine controls, or starters
        * All engines of same type are identical
        * All nacelles are identical
    """
     
    JNENG =  0 
    PNENG =  0
    WENG  = 0.0
    number_of_tanks =  0
    for network in  vehicle.networks:
        for propulsor in network.propulsors:
            if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) or  isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet):
                WENG          += compute_turbine_engine_weight(vehicle,propulsor)
                JNENG  += 1 

            if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.ICE_Propeller) or  isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Constant_Speed_ICE_Propeller):
                WENG          += compute_piston_engine_weight(propulsor)
                PNENG  += 1 
                
        for fuel_line in network.fuel_lines:
            for _ in fuel_line.fuel_tanks:
                number_of_tanks +=  1
    
    WTHR = 0.0
    WEC = 0.0
    WSTART = 0.0
    WNAC = 0.0
    for network in  vehicle.networks:
        for propulsor in network.propulsors:
            if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) or  isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet):
                if 'nacelle' in propulsor:
                    ref_nacelle =  propulsor.nacelle   
                    WNAC = compute_nacelle_weight(propulsor,ref_nacelle,JNENG)
                WTHR = compute_thrust_reverser_weight(propulsor,JNENG)
                WEC, WSTART = compute_misc_propulsion_system_weight(vehicle,propulsor,ref_nacelle,JNENG )
    
    NENG = JNENG + PNENG
                  
    WFSYS           = compute_fuel_system_weight(vehicle, NENG)
    
    WPRO            = NENG * WENG + WFSYS

    output                      = Data()
    output.W_prop               = WPRO
    output.W_thrust_reverser    = WTHR
    output.W_starter            = WSTART
    output.W_engine_controls    = WEC
    output.W_fuel_system        = WFSYS
    output.W_nacelle            = WNAC
    output.W_engine             = WENG
    output.number_of_engines    = NENG 
    output.number_of_fuel_tanks = number_of_tanks  
    return output

def compute_piston_engine_weight(ref_propulsor):
    """
    Calculate the dry engine weight for piston engines based on power.

    Parameters
    ----------
    ref_propulsor : RCAIDE.Component()
        Propulsor data structure containing:
            - engine.sea_level_power : float
                Sea level power of engine [kW]

    Returns
    -------
    WENG : float
        Dry engine weight [kg]

    Notes
    -----
    Uses linear regression based on real engine data.
    
    **Major Assumptions**
        * Linear relationship between engine weight and power
        * Valid for engines between 48 and 313 kW (64 to 420 hp)
    
    **Theory**

    .. math::
        W_{eng} = 0.8953 * P_{SL} + 19.121

    Where:
        - W_{eng} is engine weight [kg]
        - P_{SL} is sea level power [kW]

    References
    ----------
    [1] Based on 26 GA aircraft engines from Rotax, Lycoming, and Continental
    """
  
    WENG = 0.8953*(ref_propulsor.engine.sea_level_power/1000) + 19.121
    return WENG

def compute_turbine_engine_weight(vehicle, ref_propulsor):
    """
    Calculate the dry engine weight for turbine engines using FLOPS methodology.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure
    ref_propulsor : RCAIDE.Component()
        Propulsor data structure containing:
            - sealevel_static_thrust : float
                Sea level static thrust [N]

    Returns
    -------
    WENG : float
        Dry engine weight [kg]

    Notes
    -----
    Uses FLOPS weight estimation method for turbine engines.
    
    **Major Assumptions**
        * Engine weight scaling parameter is 1.15
        * Engine inlet weight scaling exponent is 1
        * Baseline inlet and nozzle weights are 0 lbs
    
    **Theory**

    .. math::
        W_{eng} = \\frac{T_{SLS}}{5.5} * (\\frac{T}{T_{SLS}})^{1.15}

    Where:
        - W_{eng} is engine weight [lb]
        - T_{SLS} is sea level static thrust [lb]
        - T is rated thrust [lb]
    """
    EEXP = 1.15
    EINL = 1
    ENOZ = 1
    THRSO = ref_propulsor.sealevel_static_thrust * 1 / Units.lbf
    THRUST = THRSO
    WENGB = THRSO / 5.5
    WINLB = 0 / Units.lbs
    WNOZB = 0 / Units.lbs
    WENGP = WENGB * (THRUST / THRSO) ** EEXP
    WINL = WINLB * (THRUST / THRSO) ** EINL
    WNOZ = WNOZB * (THRUST / THRSO) ** ENOZ
    WENG = WENGP + WINL + WNOZ
    return WENG * Units.lbs

def compute_nacelle_weight(ref_propulsor,ref_nacelle,NENG):
    """
    Calculate the nacelle weight using FLOPS methodology.

    Parameters
    ----------
    ref_propulsor : RCAIDE.Component()
        Propulsor data structure containing:
            - sealevel_static_thrust : float
                Sea level static thrust [N]
    ref_nacelle : RCAIDE.Component()
        Nacelle data structure containing:
            - diameter : float
                Nacelle diameter [m]
            - length : float
                Nacelle length [m]
    NENG : int
        Number of engines

    Returns
    -------
    WNAC : float
        Nacelle weight [kg]

    Notes
    -----
    Uses FLOPS weight estimation method for engine nacelles.
    
    **Major Assumptions**
        * All nacelles are identical
        * Number of nacelles equals number of engines
    
    **Theory**

    .. math::
        W_{nac} = 0.25 * N * D * L * T^{0.36}

    Where:
        - W_{nac} is nacelle weight [lb]
        - N is number of nacelles
        - D is nacelle diameter [ft]
        - L is nacelle length [ft]
        - T is sea level static thrust [lb]
    """ 
    TNAC   = NENG + 0.5 * (NENG - 2 * np.floor(NENG / 2.))
    DNAC   = ref_nacelle.diameter / Units.ft
    XNAC   = ref_nacelle.length / Units.ft
    FTHRST = ref_propulsor.sealevel_static_thrust * 1 / Units.lbf
    WNAC   = 0.25 * TNAC * DNAC * XNAC * FTHRST ** 0.36
    return WNAC * Units.lbs


def compute_thrust_reverser_weight(ref_propulsor,NENG):
    """
    Calculate the thrust reverser weight using FLOPS methodology.

    Parameters
    ----------
    ref_propulsor : RCAIDE.Component()
        Propulsor data structure containing:
            - sealevel_static_thrust : float
                Sea level static thrust [N]
    NENG : int
        Number of engines

    Returns
    -------
    WTHR : float
        Thrust reverser weight [kg]

    Notes
    -----
    Uses FLOPS weight estimation method for thrust reversers.
    
    **Theory**

    .. math::
        W_{tr} = 0.034 * T * N

    Where:
        - W_{tr} is thrust reverser weight [lb]
        - T is sea level static thrust [lb]
        - N is number of engines
    """ 
    TNAC = NENG + 1. / 2 * (NENG - 2 * np.floor(NENG / 2.))
    THRUST = ref_propulsor.sealevel_static_thrust * 1 / Units.lbf
    WTHR = 0.034 * THRUST * TNAC
    return WTHR * Units.lbs


def compute_misc_propulsion_system_weight(vehicle,ref_propulsor,ref_nacelle,NENG ):
    """
    Calculate miscellaneous propulsion system weights using FLOPS methodology.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure containing:
            - design_mach_number : float
                Design cruise Mach number
    ref_propulsor : RCAIDE.Component()
        Propulsor data structure containing:
            - sealevel_static_thrust : float
                Sea level static thrust [N]
    ref_nacelle : RCAIDE.Component()
        Nacelle data structure containing:
            - diameter : float
                Nacelle diameter [m]
    NENG : int
        Number of engines

    Returns
    -------
    WEC : float
        Engine control system weight [kg]
    WSTART : float
        Starter engine weight [kg]

    Notes
    -----
    Calculates electrical control system and starter engine weights.
    
    **Theory**

    Engine controls:
    .. math::
        W_{ec} = 0.26 * N * T^{0.5}

    Starter:
    .. math::
        W_{st} = 11.0 * N * M^{0.32} * D^{1.6}

    Where:
        - W_{ec} is engine controls weight [lb]
        - W_{st} is starter weight [lb]
        - N is number of engines
        - T is sea level static thrust [lb]
        - M is design Mach number
        - D is nacelle diameter [ft]
    """ 
    THRUST  = ref_propulsor.sealevel_static_thrust * 1 / Units.lbf
    WEC     = 0.26 * NENG * THRUST ** 0.5
    FNAC    = ref_nacelle.diameter / Units.ft
    VMAX    = vehicle.flight_envelope.design_mach_number
    WSTART  = 11.0 * NENG * VMAX ** 0.32 * FNAC ** 1.6
    return WEC * Units.lbs, WSTART * Units.lbs

 
def compute_fuel_system_weight(vehicle, NENG):
    """
    Calculate the weight of the general aviation aircraft fuel system using FLOPS methodology.

    Parameters
    ----------
    vehicle : RCAIDE.Vehicle()
        Vehicle data structure containing:
            - mass_properties.max_fuel : float
                Maximum fuel capacity [kg]
    NENG : int
        Number of engines

    Returns
    -------
    fuel_system_weight : float
        Weight of the complete fuel system [kg]

    Notes
    -----
    The function implements the FLOPS weight estimation method for aircraft fuel systems.
    The calculation accounts for total fuel capacity and number of engines.
    
    **Major Assumptions**
        * Conventional fuel tank and distribution system
        * System includes tanks, plumbing, pumps, and associated hardware
        * Weight scales with total fuel capacity and number of engines
    
    **Theory**

    The FLOPS fuel system weight estimation follows:

    .. math::
        W_{fs} = 1.07 * W_{f}^{0.58} * N_{eng}^{0.43}

    Where:
        - W_{fs} is fuel system weight [lb]
        - W_{f} is maximum fuel weight [lb]
        - N_{eng} is number of engines

    References
    ----------
    [1] NASA. (1979). The Flight Optimization System Weights Estimation Method. 
        NASA Technical Report.
    """
    FMXTOT = vehicle.mass_properties.max_fuel / Units.lbs
    WFSYS = 1.07 * FMXTOT ** 0.58 * NENG ** 0.43
    return WFSYS * Units.lbs