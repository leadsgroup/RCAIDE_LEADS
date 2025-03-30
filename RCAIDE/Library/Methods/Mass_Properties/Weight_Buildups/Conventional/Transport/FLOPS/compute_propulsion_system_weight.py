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
def compute_propulsion_system_weight(vehicle,ref_propulsor):
    """
    Computes the complete propulsion system weight using NASA FLOPS weight estimation 
    method. Includes engines, nacelles, thrust reversers, and associated systems.

    Parameters
    ----------
    vehicle : Vehicle
        The vehicle instance containing:
            - networks : list
                Propulsion systems with:
                    - propulsors : list
                        Engine data
                    - fuel_lines : list
                        Fuel system data with fuel tanks
            - design_mach_number : float
                Design cruise Mach number
            - mass_properties.max_zero_fuel : float
                Maximum zero fuel weight [kg]
            - systems.accessories : str
                Aircraft type ('short-range', 'commuter', 'medium-range', 
                'long-range', 'sst', 'cargo')
    ref_propulsor : Propulsor
        Reference engine containing:
            - sealevel_static_thrust : float
                Sea level static thrust [N]
            - nacelle : Nacelle
                Nacelle geometry with:
                    - diameter : float
                        Maximum diameter [m]
                    - length : float
                        Total length [m]

    Returns
    -------
    output : Data
        Container with propulsion weight breakdown:
            - W_prop : float
                Total propulsion system weight [kg]
            - W_engine : float
                Dry engine weight [kg]
            - W_thrust_reverser : float
                Thrust reverser weight [kg]
            - W_starter : float
                Starter system weight [kg]
            - W_engine_controls : float
                Engine controls weight [kg]
            - W_fuel_system : float
                Fuel system weight [kg]
            - W_nacelle : float
                Nacelle weight [kg]
            - number_of_engines : int
                Total engine count
            - number_of_fuel_tanks : int
                Total fuel tank count

    Notes
    -----
    Uses FLOPS correlations developed from transport aircraft database.

    **Major Assumptions**
        * Engines have a thrust to weight ratio of 5.5
        * All nacelles are identical
        * Number of nacelles equals number of engines
        * Number of thrust reversers equals the number of engines unless there is an odd number of engines in which case it is N - 1

    **Theory**
    Engine weight is computed using:
    .. math::
        W_{eng} = THRUST/5.5

    Nacelle weight is computed using:
    .. math::
        W_{nac} = 0.25N_{nac}D_{nac}L_{nac}T^{0.36}

    Thrust reverser weight is computed using:
    .. math::
        W_{rev} = 0.034T N_{nac}

    where:
        * W_base = baseline engine weight
        * T = sea level static thrust
        * N_nac = number of nacelles
        * D_nac = nacelle diameter
        * L_nac = nacelle length

    References
    ----------
    [1] NASA Flight Optimization System (FLOPS)
    """
     
    NENG =  0 
    number_of_tanks =  0
    ref_nacelle =  None
    for network in  vehicle.networks:
        for propulsor in network.propulsors:
            if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan) or  isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet):
                ref_propulsor = propulsor  
                NENG  += 1 
            if 'nacelle' in propulsor:
                ref_nacelle =  propulsor.nacelle   
        for fuel_line in network.fuel_lines:
            for _ in fuel_line.fuel_tanks:
                number_of_tanks +=  1
                  
    if ref_nacelle is not None:
        WNAC            = compute_nacelle_weight(ref_propulsor,ref_nacelle,NENG ) 
    WFSYS           = compute_fuel_system_weight(vehicle, NENG)
    WENG            = compute_engine_weight(vehicle,ref_propulsor)
    WEC, WSTART     = compute_misc_propulsion_system_weight(vehicle,ref_propulsor,ref_nacelle,NENG)
    WTHR            = compute_thrust_reverser_weight(ref_propulsor,NENG)
    WPRO            = NENG * WENG + WFSYS + WEC + WSTART + WTHR + WNAC

    output                      = Data()
    output.W_prop               = WPRO
    output.W_thrust_reverser    = WTHR
    output.W_starter            = WSTART
    output.W_engine_controls    = WEC
    output.W_fuel_system        = WFSYS
    output.W_nacelle            = WNAC
    output.W_engine             = WENG * NENG
    output.number_of_engines    = NENG 
    output.number_of_fuel_tanks = number_of_tanks  
    return output


def compute_nacelle_weight(ref_propulsor,ref_nacelle,NENG):
    """ Calculates the nacelle weight based on the FLOPS method
    
        Assumptions:
            1) All nacelles are identical
            2) The number of nacelles is the same as the number of engines 

        Source:
            The Flight Optimization System Weight Estimation Method

        Inputs:
            ref_propulsor    - data dictionary for the specific network that is being estimated [dimensionless]
                -.number_of_engines: number of engines
                -.engine_lenght: total length of engine                                  [m]
                -.sealevel_static_thrust: sealevel static thrust of engine               [N]
            nacelle.             
                -.diameter: diameter of nacelle                                          [m]
            WENG    - dry engine weight                                                  [kg]
             
             
        Outputs:             
            WNAC: nacelle weight                                                         [kg]

        Properties Used:
            N/A
    """ 
    TNAC   = NENG + 0.5 * (NENG - 2 * np.floor(NENG / 2.))
    DNAC   = ref_nacelle.diameter / Units.ft
    XNAC   = ref_nacelle.length / Units.ft
    FTHRST = ref_propulsor.sealevel_static_thrust * 1 / Units.lbf
    WNAC   = 0.25 * TNAC * DNAC * XNAC * FTHRST ** 0.36
    return WNAC * Units.lbs


def compute_thrust_reverser_weight(ref_propulsor,NENG):
    """ Calculates the weight of the thrust reversers of the aircraft
    
        Assumptions:

        Source:
            The Flight Optimization System Weight Estimation Method

        Inputs:
            ref_propulsor    - data dictionary for the specific network that is being estimated [dimensionless]
                -.number_of_engines: number of engines
                -.sealevel_static_thrust: sealevel static thrust of engine  [N]

        Outputs:
            WTHR: Thrust reversers weight                                   [kg]

        Properties Used:
            N/A
    """ 
    TNAC = NENG + 1. / 2 * (NENG - 2 * np.floor(NENG / 2.))
    THRUST = ref_propulsor.sealevel_static_thrust * 1 / Units.lbf
    WTHR = 0.034 * THRUST * TNAC
    return WTHR * Units.lbs


def compute_misc_propulsion_system_weight(vehicle,ref_propulsor,ref_nacelle,NENG ):
    """ Calculates the miscellaneous engine weight based on the FLOPS method, electrical control system weight
        and starter engine weight
        
        Assumptions:
            1) All nacelles are identical
            2) The number of nacelles is the same as the number of engines 

        Source:
            The Flight Optimization System Weight Estimation Method

        Inputs:
            vehicle - data dictionary with vehicle properties                            [dimensionless]
                 -.design_mach_number: design mach number
            ref_propulsor    - data dictionary for the specific network that is being estimated [dimensionless]
                -.number_of_engines: number of engines
                -.sealevel_static_thrust: sealevel static thrust of engine               [N]
            nacelle              
                -.diameter: diameter of nacelle                                          [m]
              
        Outputs:              
            WEC: electrical engine control system weight                                 [kg]
            WSTART: starter engine weight                                                [kg]

        Properties Used:
            N/A
    """ 
    THRUST  = ref_propulsor.sealevel_static_thrust * 1 / Units.lbf
    WEC     = 0.26 * NENG * THRUST ** 0.5
    FNAC    = ref_nacelle.diameter / Units.ft
    VMAX    = vehicle.flight_envelope.design_mach_number
    WSTART  = 11.0 * NENG * VMAX ** 0.32 * FNAC ** 1.6
    return WEC * Units.lbs, WSTART * Units.lbs

 
def compute_fuel_system_weight(vehicle, NENG):
    """ Calculates the weight of the fuel system based on the FLOPS method
        Assumptions:

        Source:
            The Flight Optimization System Weight Estimation Method

        Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
                -.design_mach_number: design mach number
                -   [kg]

        Outputs:
            WFSYS: Fuel system weight                                       [kg]

        Properties Used:
            N/A
    """
    VMAX = vehicle.flight_envelope.design_mach_number
    FMXTOT = vehicle.mass_properties.max_fuel / Units.lbs
    WFSYS = 1.07 * FMXTOT ** 0.58 * NENG ** 0.43 * VMAX ** 0.34
    return WFSYS * Units.lbs


def compute_engine_weight(vehicle, ref_propulsor):
    """ Calculates the dry engine weight based on the FLOPS method
        Assumptions:
            Rated thrust per scaled engine and rated thurst for baseline are the same
            Engine weight scaling parameter is 1.15
            Enginge inlet weight scaling exponent is 1
            Baseline inlet weight is 0 lbs as in example files FLOPS
            Baseline nozzle weight is 0 lbs as in example files FLOPS

        Source:
            The Flight Optimization System Weight Estimation Method

        Inputs:
            vehicle - data dictionary with vehicle properties                   [dimensionless]
                -.systems.accessories: type of aircraft (short-range, commuter
                                                        medium-range, long-range,
                                                        sst, cargo)
            ref_propulsor    - data dictionary for the specific network that is being estimated [dimensionless]
                -.sealevel_static_thrust: sealevel static thrust of engine  [N]

        Outputs:
            WENG: dry engine weight                                         [kg]

        Properties Used:
            N/A
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