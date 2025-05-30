 # generate_V_n_diagram.py
#
# Created:  Nov 2018, S. Karpuk
# Modified:

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE Imports
import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Framework.Core import Units 
from RCAIDE.Framework.Mission.Common  import Results  

# package imports
import numpy as np
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------------------------------------------------- 
#  Compute a V-n diagram
# ---------------------------------------------------------------------------------------------------------------------- 
def generate_V_n_diagram(vehicle,analyses,altitude,delta_ISA):
    
    """
    Computes a V-n (velocity-load factor) diagram for an aircraft according to FAR requirements.

    Parameters
    ----------
    vehicle : Vehicle
        The vehicle instance containing:
            - reference_area : float
                Wing reference area [m²]
            - maximum_lift_coefficient : float
                Maximum lift coefficient
            - minimum_lift_coefficient : float
                Minimum lift coefficient
            - chords.mean_aerodynamic : float
                Mean aerodynamic chord [m]
            - flight_envelope : Data
                Container with:
                    - FAR_part_number : str
                        '23' or '25' for certification category
                    - category : str
                        For Part 23: 'normal', 'utility', 'acrobatic', or 'commuter'
                    - design_mach_number : float
                        Cruise Mach number
                    - positive_limit_load : float
                        Positive load factor limit
                    - negative_limit_load : float
                        Negative load factor limit
    analyses : Analyses
        Container with atmosphere and aerodynamic analyses
    altitude : float
        Analysis altitude [m]
    delta_ISA : float
        Temperature offset from ISA conditions [K]

    Returns
    -------
    V_n_data : Data
        Container of V-n diagram data including:
            - limit_loads : Data
                Positive and negative load factor limits
            - airspeeds : Data
                Critical airspeeds (Vs, Va, Vb, Vc, Vd)
            - gust_load_factors : Data
                Load factors from gust conditions
            - load_factors : Data
                Complete set of load factors vs velocity

    Notes
    -----
    Computes the following critical speeds and conditions:
        * Vs1: Stall speed
        * Va: Maneuvering speed
        * Vb: Design speed for maximum gust intensity
        * Vc: Design cruise speed
        * Vd: Design diving speed

    **Major Assumptions**
        * Quasi-steady aerodynamics
        * Linear lift curve slope
        * Rigid aircraft structure
        * Standard atmosphere modified by altitude and delta_ISA

    **Theory**
    Load factor limits are determined by:

    .. math::
        n = \\frac{L}{W} = \\frac{\\rho V^2 S C_L}{2W}

    Gust loads are computed using:

    .. math::
        \\Delta n = \\frac{K_g U_de V a C_{L_\\alpha}}{498 W/S}

    References
    ----------
    [1] FAR Part 23: https://www.ecfr.gov/current/title-14/part-23
    [2] FAR Part 25: https://www.ecfr.gov/current/title-14/part-25
    [3] Gudmundsson, S. (2022). General Aviation Aircraft Design: Applied Methods and procedures. Elsevier. 
    """
    
    weight =  vehicle.mass_properties.max_takeoff
 
    # ----------------------------------------------
    # Unpack
    # ---------------------------------------------- 
    FAR_part_number = vehicle.flight_envelope.FAR_part_number
    atmo            = analyses.atmosphere
    Mc              = vehicle.flight_envelope.design_mach_number

    for wing in vehicle.wings: 
        reference_area  = vehicle.reference_area 
        Cmac            = wing.chords.mean_aerodynamic 
        pos_limit_load  = vehicle.flight_envelope.positive_limit_load
        neg_limit_load  = vehicle.flight_envelope.positive_limit_load

    category_tag = vehicle.flight_envelope.category
    
    # ----------------------------------------------
    # Computing atmospheric conditions
    # ----------------------------------------------
    atmo_values       = atmo.compute_values(altitude,delta_ISA)
    SL_atmo_values    = atmo.compute_values(0,delta_ISA)
    conditions        = Results()

    rho               = atmo_values.density
    sea_level_rho     = SL_atmo_values.density
    sea_level_gravity = atmo.planet.sea_level_gravity
    Vc                = Mc * atmo_values.speed_of_sound
    
    # ------------------------------
    # Computing lift-curve slope
    # ------------------------------ 
    results =  evalaute_aircraft(vehicle,altitude,Vc) 
    CLa     =  results.segments.cruise.conditions.static_stability.derivatives.Clift_alpha[0, 0] 

    # -----------------------------------------------------------
    # Determining vehicle minimum and maximum lift coefficients
    # -----------------------------------------------------------
    if vehicle.flight_envelope.maximum_lift_coefficient != None:
        maximum_lift_coefficient = vehicle.flight_envelope.maximum_lift_coefficient
    else:
        raise ValueError("Maximum lift coefficient not specified.")

    if vehicle.flight_envelope.minimum_lift_coefficient != None:
        minimum_lift_coefficient = vehicle.flight_envelope.minimum_lift_coefficient
    else: 
        raise ValueError("Minimum lift coefficient not specified.")
             
    # -----------------------------------------------------------------------------
    # Convert all terms to English (Used for FAR) and remove elements from arrays
    # -----------------------------------------------------------------------------
    altitude          = altitude / Units.ft
    rho               = rho[0,0] / Units['slug/ft**3']
    sea_level_rho     = sea_level_rho[0,0] / Units['slug/ft**3']
    density_ratio     = (rho/sea_level_rho)**0.5
    sea_level_gravity = sea_level_gravity / Units['ft/s**2']
    weight            = weight / Units['slug'] * sea_level_gravity
    reference_area    = reference_area / Units['ft**2']
    Cmac              = Cmac / Units.ft
    wing_loading      = weight / reference_area
    Vc                = Vc / Units['ft/s']
    
    load_factors_pos    = np.zeros(shape=(5));
    load_factors_neg    = np.zeros(shape=(5));
    load_factors_pos[1] =  1;
    load_factors_neg[1] = -1;
    airspeeds_pos       = np.zeros(shape=(5));
    airspeeds_neg       = np.zeros(shape=(5))
        
    # --------------------------------------------------
    # Establish limit maneuver load factors n+ and n- 
    # -------------------------------------------------- 
    # CFR Part 25
    # Positive and negative limits
    if FAR_part_number == '25':
        # Positive limit 
        load_factors_pos[2] = 2.1 + 24000 / (weight + 10000)
        if load_factors_pos[2] < 2.5: 
            load_factors_pos[2] = 2.5
        elif load_factors_pos[2] < pos_limit_load:
            load_factors_pos[2] = pos_limit_load
        elif load_factors_pos[2] > 3.8: 
            load_factors_pos[2] = 3.8

        # Negative limit 
        load_factors_neg[2] = neg_limit_load
        if load_factors_neg[2] > -1: 
            load_factors_neg[2] = -1

    elif FAR_part_number == '23':
        if category_tag == 'normal' or category_tag == 'commuter':
            # Positive limit 
            load_factors_pos[2] = 2.1 + 24000 / (weight + 10000)
            if load_factors_pos[2] < 2.5: 
                load_factors_pos[2] = 2.5
            elif load_factors_pos[2] < pos_limit_load:
                load_factors_pos[2] = pos_limit_load
            elif load_factors_pos[2] > 3.8: 
                load_factors_pos[2] = 3.8

            # Negative limit 
            load_factors_neg[2] = -0.4 * load_factors_pos[2]
            
        elif category_tag == 'utility':
            # Positive limit 
            load_factors_pos[2] =  pos_limit_load
            if load_factors_pos[2] < 4.4: 
                load_factors_pos[2] = 4.4
                 
            # Negative limit 
            load_factors_neg[2] = -0.4 * load_factors_pos[2]
            
        elif category_tag == 'acrobatic':
            # Positive limit
            load_factors_pos[2] = pos_limit_load
            if load_factors_pos[2] < 6.0: 
                load_factors_pos[2] = 6.0

            # Negative limit    
            load_factors_neg[2] = -0.5 * load_factors_pos[2]
            
        else:
            raise ValueError("Check the category_tag input. The parameter was not found")
       
    else:
        raise ValueError("Check the FARflag input. The parameter was not found")

    # Check input of the limit load
    if abs(neg_limit_load) > abs(load_factors_neg[2]):
        load_factors_neg[2] = -neg_limit_load

    #----------------------------------------
    # Generate a V-n diagram data structure
    #----------------------------------------
    V_n_data                          = Data()
    V_n_data.limit_loads              = Data()
    V_n_data.limit_loads.dive         = Data()
    V_n_data.load_factors             = Data()
    V_n_data.gust_load_factors        = Data()
    V_n_data.Vb_load_factor           = Data()
    V_n_data.airspeeds                = Data()
    V_n_data.Vs1                      = Data()
    V_n_data.Va                       = Data()
    V_n_data.Vb                       = Data()
    V_n_data.load_factors.positive    = load_factors_pos
    V_n_data.load_factors.negative    = load_factors_neg
    V_n_data.airspeeds.positive       = airspeeds_pos
    V_n_data.airspeeds.negative       = airspeeds_neg
    V_n_data.Vc                       = Vc
    V_n_data.weight                   = weight
    V_n_data.wing_loading             = wing_loading
    V_n_data.altitude                 = altitude
    V_n_data.density                  = rho
    V_n_data.density_ratio            = density_ratio
    V_n_data.reference_area           = reference_area
    V_n_data.maximum_lift_coefficient = maximum_lift_coefficient
    V_n_data.minimum_lift_coefficient = minimum_lift_coefficient
    V_n_data.positive_limit_load      = load_factors_pos[2]
    V_n_data.negative_limit_load      = load_factors_neg[2]
    
    # --------------------------------------------------
    # Computing critical speeds (Va, Vc, Vb, Vd, Vs1)
    # -------------------------------------------------- 
    
    # Calculate Stall and Maneuver speeds 
    stall_maneuver_speeds(V_n_data)  
        
    # convert speeds to KEAS for future calculations 
    convert_keas(V_n_data)

    # unpack modified airspeeds 
    airspeeds_pos = V_n_data.airspeeds.positive
    airspeeds_neg = V_n_data.airspeeds.negative
    Vc            = V_n_data.Vc
    Va_pos        = V_n_data.Va.positive 
    Va_neg        = V_n_data.Va.negative
    Va_pos        = V_n_data.Va.positive 
    Va_neg        = V_n_data.Va.negative 
 
    if Va_neg > Vc and Va_neg > Va_pos: 
        Vc = 1.15 * Va_neg
    elif Va_pos > Vc and Va_neg < Va_pos: 
        Vc = 1.15 * Va_pos
    
    # Gust speeds between Vb and Vc (EAS) and minimum Vc
    miu = 2 * wing_loading / (rho * Cmac * CLa * sea_level_gravity)
    Kg  = 0.88 * miu / (5.3 + miu)
          
    if FAR_part_number == '25':
        if altitude < 15000:
            Uref_cruise = (-0.0008 * altitude + 56)
            Uref_rough  = Uref_cruise
            Uref_dive   = 0.5 * Uref_cruise
        else:
            Uref_cruise = (-0.0005142 * altitude + 51.7133)
            Uref_rough  = Uref_cruise
            Uref_dive   = 0.5 * Uref_cruise
            
        # Minimum Cruise speed Vc_min
        coefs = [1, -Uref_cruise * (2.64 + (Kg * CLa * airspeeds_pos[1]**2)/(498 * wing_loading)), 1.72424 * Uref_cruise**2 - airspeeds_pos[1]**2]
        Vc1   = max(np.roots(coefs))
        
    elif FAR_part_number == '23':           
        if altitude < 20000:
            Uref_cruise = 50;
            Uref_dive   = 25;
        else:
            Uref_cruise = (-0.0008333 * altitude + 66.67);
            Uref_dive = (-0.0004167 * altitude + 33.334);

        if category_tag == 'commuter':
            if altitude < 20000:
                Uref_rough = 66
            else:
                Uref_rough = -0.000933 * altitude + 84.667
        else:
            Uref_rough = Uref_cruise
                       
        # Minimum Cruise speed Vc_min
        if category_tag == 'acrobatic':
            Vc1 = 36 * wing_loading**0.5
            if wing_loading >= 20:
                Vc1 = (-0.0925 * wing_loading + 37.85) * wing_loading **0.5
        else:
            Vc1 = 33 * wing_loading**0.5
            if wing_loading >= 20:       
                Vc1 = (-0.055 * wing_loading + 34.1) * wing_loading **0.5

    # checking input Cruise speed 
    if Vc1 > Vc: 
        Vc = Vc1

    # Dive speed 
    if FAR_part_number == '25':        
        airspeeds_pos[4] = 1.25 * Vc
    elif FAR_part_number == '23':
        if category_tag == 'acrobatic':
            airspeeds_pos[4] = 1.55 * Vc1
            if wing_loading > 20:
                airspeeds_pos[4] = (-0.0025 * wing_loading + 1.6) * Vc1
        elif category_tag == 'utility':
            airspeeds_pos[4] = 1.5 * Vc1
            if wing_loading > 20:
                airspeeds_pos[4] = (-0.001875 * wing_loading + 1.5375) * Vc1           
        else:
            airspeeds_pos[4] = 1.4 * Vc1
            if wing_loading > 20:
                airspeeds_pos[4] = (-0.000625 * wing_loading + 1.4125) * Vc1 
        if airspeeds_pos[4] < 1.15 * Vc: 
            airspeeds_pos[4] = 1.15 * Vc
   
    Vd               = airspeeds_pos[4]
    airspeeds_pos[3] = airspeeds_pos[4]
    airspeeds_neg[3] = Vc
    airspeeds_neg[4] = airspeeds_pos[4]
    
    # complete initial load factors
    load_factors_pos[4] = 0
    load_factors_neg[4] = 0
    if category_tag == 'acrobatic' or category_tag == 'utility':
        load_factors_neg[4] = -1
        
    load_factors_pos[3] = load_factors_pos[2]
    load_factors_neg[3] = load_factors_neg[2]

    # add parameters to the data structure
    V_n_data.load_factors.positive    = load_factors_pos
    V_n_data.load_factors.negative    = load_factors_neg
    V_n_data.airspeeds.positive       = airspeeds_pos
    V_n_data.airspeeds.negative       = airspeeds_neg
    V_n_data.Vd                       = Vd
    V_n_data.Vc                       = Vc

    #------------------------
    # Create Stall lines
    #------------------------ 
    Num_of_points = 20                                            # number of points for the stall line
    upper_bound = 2;
    lower_bound = 1;
    stall_line(V_n_data, upper_bound, lower_bound, Num_of_points, 1)
    stall_line(V_n_data, upper_bound, lower_bound, Num_of_points, 2)

    # ----------------------------------------------
    # Determine Gust loads
    # ---------------------------------------------- 
    V_n_data.gust_data           = Data()
    V_n_data.gust_data.airspeeds = Data()
    
    V_n_data.gust_data.airspeeds.rough_gust  = Uref_rough
    V_n_data.gust_data.airspeeds.cruise_gust = Uref_cruise
    V_n_data.gust_data.airspeeds.dive_gust   = Uref_dive
    
    gust_loads(category_tag, V_n_data, Kg, CLa, Num_of_points, FAR_part_number, 1)
    gust_loads(category_tag, V_n_data, Kg, CLa, Num_of_points, FAR_part_number, 2)

    #----------------------------------------------------------------
    # Finalize the load factors for acrobatic and utility aircraft
    #----------------------------------------------------------------
    if category_tag == 'acrobatic' or category_tag == 'utility':
        V_n_data.airspeeds.negative    = np.append(V_n_data.airspeeds.negative, Vd)
        V_n_data.load_factors.negative = np.append(V_n_data.load_factors.negative, 0)

    # ----------------------------------------------
    # Post-processing the V-n diagram
    # ---------------------------------------------- 
    V_n_data.positive_limit_load = max(V_n_data.load_factors.positive)
    V_n_data.negative_limit_load = min(V_n_data.load_factors.negative) 
    post_processing(category_tag, Uref_rough, Uref_cruise, Uref_dive, V_n_data, vehicle) 
    return V_n_data

      
def evalaute_aircraft(vehicle,altitude,Vc): 
    
    # Set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = analyses_setup(configs)

    # mission analyses
    mission  = base_mission_setup(analyses,altitude,Vc) 

    # create mission instances (for multiple types of missions)
    missions = missions_setup(mission) 

    # mission analysis 
    results = missions.base_mission.evaluate() 

    return results
 
def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):

       # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses        = RCAIDE.Framework.Analyses.Vehicle() 

    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------
    weights         = RCAIDE.Framework.Analyses.Weights.Conventional()
    weights.aircraft_type = 'General_Aviation'
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    # ------------------------------------------------------------------
    aerodynamics                                      = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle                              = vehicle  
    aerodynamics.settings.use_surrogate               = False 
    aerodynamics.settings.trim_aircraft               = False 
    analyses.append(aerodynamics)
    
    
    # ------------------------------------------------------------------
    #  Energy
    # ------------------------------------------------------------------
    energy     = RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle = vehicle  
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    # ------------------------------------------------------------------
    planet     = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    # ------------------------------------------------------------------
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    # done!
    return analyses

  
def configs_setup(vehicle): 
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------ 
    configs = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config                                                       = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag                                                   = 'base'     
    configs.append(base_config)  
    return configs
 
def base_mission_setup(analyses,altitude,Vc):   
    '''
    This sets up the nominal cruise of the aircraft
    '''
     
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'base_mission'
  
    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments

    #   Cruise Segment: constant Speed, constant altitude 
    segment                           = Segments.Untrimmed.Untrimmed()
    segment.analyses.extend( analyses.base )   
    segment.tag                       = "cruise" 
    segment.altitude                  = altitude
    segment.air_speed                 = Vc

    segment.flight_dynamics.force_x   = True    
    segment.flight_dynamics.force_z   = True    
    segment.flight_dynamics.force_y   = True     
    segment.flight_dynamics.moment_y  = True 
    segment.flight_dynamics.moment_x  = True
    segment.flight_dynamics.moment_z  = True
    
    mission.append_segment(segment)     
    
    return mission

def missions_setup(mission): 
 
    missions     = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions  


#------------------------------------------------------------------------------------------------------
# USEFUL FUNCTIONS
#------------------------------------------------------------------------------------------------------

def stall_maneuver_speeds(V_n_data):

    """ Computes stall and maneuver speeds for positive and negative halves of the the V-n diagram

    Source:
    S. Gudmundsson "General Aviation Aircraft Design: Applied Methods and Procedures", Butterworth-Heinemann; 1 edition

    Inputs:
    V_n_data.
        airspeeds.positive                      [kts]
            negative                            [kts]
        weight                                  [lb]
        density                                 [slug/ft**3]
        density_ratio                           [Unitless]
        reference_area                          [ft**2]
        maximum_lift_coefficient                [Unitless]
        minimum_lift_coefficient                [Unitless]
        load_factors.positive                   [Unitless]
        load_factors.negative                   [Unitless]

    Outputs:
    V_n_data.
        airspeeds.positive                      [kts]
            negative                            [kts]
        Vs1.positive                            [kts]       
            negative                            [kts]
        Va.positive                             [kts]
            negative                            [kts]
        load_factors.positive                   [Unitless]
            negative                            [Unitless]                 

    Properties Used:
    N/A

    Description:   
    """     

    # Unpack
    airspeeds_pos    = V_n_data.airspeeds.positive
    airspeeds_neg    = V_n_data.airspeeds.negative
    weight           = V_n_data.weight
    rho              = V_n_data.density
    reference_area   = V_n_data.reference_area
    max_lift_coef    = V_n_data.maximum_lift_coefficient
    min_lift_coef    = V_n_data.minimum_lift_coefficient
    load_factors_pos = V_n_data.load_factors.positive
    load_factors_neg = V_n_data.load_factors.negative
    
    # Stall speeds
    airspeeds_pos[1] = (2 * weight / (rho * reference_area * max_lift_coef)) ** 0.5
    airspeeds_neg[1] = (2 * weight / (rho * reference_area * abs(min_lift_coef))) ** 0.5
    airspeeds_pos[0] = airspeeds_pos[1]
    airspeeds_neg[0] = airspeeds_neg[1]

    # Maneuver speeds
    airspeeds_pos[2] = airspeeds_pos[1] * load_factors_pos[2] ** 0.5
    airspeeds_neg[2] = (2 * weight * abs(load_factors_neg[2]) / (rho * reference_area * \
                                                                 abs(min_lift_coef))) ** 0.5
    
    # Pack
    V_n_data.airspeeds.positive           = airspeeds_pos
    V_n_data.airspeeds.negative           = airspeeds_neg
    V_n_data.load_factors.positive        = load_factors_pos
    V_n_data.load_factors.negative        = load_factors_neg
    V_n_data.Vs1.positive                 = airspeeds_pos[1]
    V_n_data.Vs1.negative                 = airspeeds_neg[1]
    V_n_data.Va.positive                  = airspeeds_pos[2]
    V_n_data.Va.negative                  = airspeeds_neg[2]
    
#------------------------------------------------------------------------------------------------------------

def stall_line(V_n_data, upper_bound, lower_bound, Num_of_points, sign_flag):
    
    """ Calculates Stall lines of positive and negative halves of the V-n diagram

    Source:
    S. Gudmundsson "General Aviation Aircraft Design: Applied Methods and Procedures", Butterworth-Heinemann; 1 edition

    Inputs:
    V_n_data.
        airspeeds.positive                  [kts]
            negative                        [kts]
        load_factors.positive               [Unitless]
            negative                        [Unitless]
        weight                              [lb]
        density                             [slug/ft**3]
        density_ratio                       [Unitless]
        lift_coefficient                    [Unitless]
        reference_area                      [ft**2]
    lower_bound                             [Unitless]
    Num_of_points                           [Unitless]
    sign_flag                               [Unitless]

    Outputs:
    V_n_data.
        load_factors.positive               [Unitless]
            negative                        [Unitless]
        airspeeds.positive                  [kts]
            negative                        [kts]

    Properties Used:
    N/A

    Description:   
    """  
    # Unpack
    weight          = V_n_data.weight
    reference_area  = V_n_data.reference_area
    density_ratio   = V_n_data.density_ratio
    rho             = V_n_data.density
    
    if sign_flag == 1:
        load_factors     = V_n_data.load_factors.positive
        airspeeds        = V_n_data.airspeeds.positive
        lift_coefficient = V_n_data.maximum_lift_coefficient

    elif sign_flag == 2:
        load_factors     = V_n_data.load_factors.negative
        airspeeds        = V_n_data.airspeeds.negative
        lift_coefficient = V_n_data.minimum_lift_coefficient
    
    delta = (airspeeds[upper_bound] - airspeeds[lower_bound]) / (Num_of_points + 1)     # Step size
    for i in range(Num_of_points):       
        coef      = lower_bound + i + 1
        airspeeds = np.concatenate((airspeeds[:coef], [airspeeds[lower_bound] + (i + 1) * delta], airspeeds[coef:]))
        Vtas      = airspeeds[coef] / density_ratio * Units.knots / Units['ft/s']      
        if load_factors[1] > 0:
            nl = 0.5 * rho * Vtas**2 * reference_area * lift_coefficient / weight
        else:
            nl = -0.5 * rho * Vtas**2 * reference_area * abs(lift_coefficient) / weight
            
        load_factors = np.concatenate((load_factors[:coef], [nl], load_factors[coef:]))

    # Pack
    if sign_flag == 1:
        V_n_data.load_factors.positive     = load_factors
        V_n_data.airspeeds.positive        = airspeeds

    elif sign_flag == 2:
        V_n_data.load_factors.negative     = load_factors
        V_n_data.airspeeds.negative        = airspeeds
    
    return 
#--------------------------------------------------------------------------------------------------------------

def gust_loads(category_tag, V_n_data, Kg, CLa, Num_of_points, FAR_part_number, sign_flag):

    """ Calculates airspeeds and load factors for gust loads and modifies the V-n diagram

    Source:
    S. Gudmundsson "General Aviation Aircraft Design: Applied Methods and Procedures", Butterworth-Heinemann; 1 edition

    Inputs:
    V_n_data.
        airspeeds.positive                      [kts]
            negative                            [kts]
        load_factors.positive                   [Unitless]
            negative                            [Unitless]
        positive_limit_load                    [Unitless]
            negative                            [Unitless]
        weight                                  [lb]
        wing_loading                            [lb/ft**2]
        reference_area                          [ft**2]
        density                                 [slug/ft**3]
        density_ratio                           [Unitless]
        lift_coefficient                        [Unitless]
        Vc                                      [kts]
        Vd                                      [kts]
    Uref_rough                                  [ft/s]
    Uref_cruise                                 [ft/s]
    Uref_dive                                   [ft/s]
    Num_of_points                               [Unitless]
    sign_flag                                   [Unitless]
  
    
    Outputs:
    V_n_data.
        airspeeds.positive                      [kts]
            negative                            [kts]
        load_factors.positive                   [Unitless]
            .negative                           [Unitless]
        gust_load_factors.positive              [Unitless]
            negative                            [Unitless]

    Properties Used:
    N/A

    Description:
    The function calculates the gust-related load factors at critical speeds (Va, Vc, Vd). Then, if the load factors exceed
    the standart diagram limits, the diagram is modified to include the new limit loads.
    For more details, refer to S. Gudmundsson "General Aviation Aircraft Design: Applied Methods and Procedures"
    """

    # Unpack
    weight          = V_n_data.weight
    wing_loading    = V_n_data.wing_loading
    reference_area  = V_n_data.reference_area
    density         = V_n_data.density
    density_ratio   = V_n_data.density_ratio
    Vc              = V_n_data.Vc
    Vd              = V_n_data.Vd
    
    if sign_flag == 1:
        airspeeds        = V_n_data.airspeeds.positive
        load_factors     = V_n_data.load_factors.positive
        lift_coefficient = V_n_data.maximum_lift_coefficient
        limit_load       = V_n_data.positive_limit_load
        Uref_rough       = V_n_data.gust_data.airspeeds.rough_gust
        Uref_cruise      = V_n_data.gust_data.airspeeds.cruise_gust
        Uref_dive        = V_n_data.gust_data.airspeeds.dive_gust

    elif sign_flag == 2:
        airspeeds        = V_n_data.airspeeds.negative
        load_factors     = V_n_data.load_factors.negative
        lift_coefficient = V_n_data.minimum_lift_coefficient
        limit_load       = V_n_data.negative_limit_load
        Uref_rough       = -V_n_data.gust_data.airspeeds.rough_gust
        Uref_cruise      = -V_n_data.gust_data.airspeeds.cruise_gust
        Uref_dive        = -V_n_data.gust_data.airspeeds.dive_gust
        

    gust_load_factors    = np.zeros(shape=(4));    
    gust_load_factors[0] = 1;
    
    # Cruise speed Gust loads at Va and Vc
    gust_load_factors[1] = 1 + Kg * CLa * airspeeds[Num_of_points+2] * Uref_rough / (498 * wing_loading)
    gust_load_factors[2] = 1 + Kg * CLa * Vc * Uref_cruise/(498 * wing_loading)

    # Intersection between cruise gust load and Va
    if abs(gust_load_factors[1]) > abs(limit_load):
        sea_level_rho      = density / density_ratio**2
        coefs              = [709.486 * sea_level_rho * lift_coefficient, -Kg * Uref_rough * CLa, -498 * wing_loading]
        V_inters           = max(np.roots(coefs))
        load_factor_inters = 1 + Kg * CLa * V_inters * Uref_rough / (498 * wing_loading)

        airspeeds    = np.concatenate((airspeeds[:(Num_of_points + 3)], [V_inters], airspeeds[(Num_of_points + 3):]))
        load_factors = np.concatenate((load_factors[:(Num_of_points + 3)], [load_factor_inters], load_factors[(Num_of_points + 3):]))

        # Pack
        if sign_flag == 1:
            V_n_data.airspeeds.positive          = airspeeds
            V_n_data.load_factors.positive       = load_factors
            V_n_data.gust_load_factors.positive  = gust_load_factors
            V_n_data.Vb_load_factor.positive     = load_factor_inters
            V_n_data.Vb.positive                 = V_inters           
            
        if sign_flag == 2:
            V_n_data.airspeeds.negative          = airspeeds
            V_n_data.load_factors.negative       = load_factors
            V_n_data.gust_load_factors.negative  = gust_load_factors
            V_n_data.Vb_load_factor.negative     = load_factor_inters
            V_n_data.Vb.negative                 = V_inters
        
        # continue stall lines
        Num_of_points_ext       = 5;
        upper_bound             = Num_of_points+3;
        lower_bound             = Num_of_points+2;
        stall_line(V_n_data, upper_bound, lower_bound, Num_of_points_ext, sign_flag)        
        Num_of_points = Num_of_points + Num_of_points_ext + 1

        # Unpack
        if sign_flag == 1:
            airspeeds        = V_n_data.airspeeds.positive
            load_factors     = V_n_data.load_factors.positive
            lift_coefficient = V_n_data.maximum_lift_coefficient

        elif sign_flag == 2:
            airspeeds        = V_n_data.airspeeds.negative
            load_factors     = V_n_data.load_factors.negative
            lift_coefficient = V_n_data.minimum_lift_coefficient

        
        # insert the cruise speed Vc in the positive load factor line
        if load_factors[1] > 0:
            airspeeds    = np.concatenate((airspeeds[:(Num_of_points + 3)], [Vc], airspeeds[(Num_of_points + 3):]))
            load_factors = np.concatenate((load_factors[:(Num_of_points + 3)], [gust_load_factors[2]], \
                                           load_factors[(Num_of_points + 3):]))

    # Intersection between cruise gust load and maximum load at Vc
    elif abs(gust_load_factors[2]) > abs(limit_load):
        V_inters      = 498 * (load_factors[Num_of_points + 2] - 1) * wing_loading / (Kg * Uref_cruise * CLa)
        airspeeds     = np.concatenate((airspeeds[:(Num_of_points + 3)], [V_inters], airspeeds[(Num_of_points + 3):]))
        load_factors  = np.concatenate((load_factors[:(Num_of_points + 3)], [limit_load], \
                                       load_factors[(Num_of_points + 3):]))
        Num_of_points = Num_of_points + 1

        if load_factors[1] > 0:
            airspeeds    = np.concatenate((airspeeds[:(Num_of_points + 3)], [Vc], airspeeds[(Num_of_points + 3):]))
            load_factors = np.concatenate((load_factors[:(Num_of_points + 3)], [gust_load_factors[2]], \
                                           load_factors[(Num_of_points + 3):]))
        else:
            load_factors[len(airspeeds)-2] = gust_load_factors[2]

        
    # Dive speed Gust loads Vd
    gust_load_factors[3] = 1 + Kg * CLa * Vd * Uref_dive / (498 * wing_loading)

    # Resolve the upper half of the dive section
    if load_factors[1] > 0: 
        if abs(gust_load_factors[2]) > abs(limit_load):   
            if abs(gust_load_factors[3]) > abs(limit_load):
                load_factors[len(load_factors) - 2] = gust_load_factors[3]               
            else:
                airspeeds, load_factors = gust_dive_speed_intersection(category_tag, load_factors, gust_load_factors, airspeeds, \
                                                                       len(load_factors)-2, Vc, Vd)

    # Resolve the lower half of the dive section
    else:
        if gust_load_factors[3] < load_factors[len(load_factors) - 1]:
            airspeeds    = np.concatenate((airspeeds[:(len(load_factors) - 1)], [airspeeds[(len(load_factors) - 1)]], \
                                           airspeeds[(len(load_factors) - 1):]))
            load_factors = np.concatenate((load_factors[:(len(load_factors) - 1)], [gust_load_factors[3]], \
                                           load_factors[(len(load_factors) - 1):]))
            
            if abs(gust_load_factors[2]) > abs(limit_load):
                load_factors[len(load_factors) - 2] = gust_load_factors[3]
            else:
                airspeeds, load_factors = gust_dive_speed_intersection(category_tag, load_factors, gust_load_factors, airspeeds, \
                                                                       len(load_factors)-2, Vc, Vd)

            V_n_data.limit_loads.dive.negative = load_factors[len(load_factors) - 2]
        else:
            V_n_data.limit_loads.dive.negative = load_factors[len(load_factors) - 1]

    # gusts load extension for gust lines at Vd
    gust_load_factors = np.append(gust_load_factors, 1 + Kg * CLa * (1.05 * Vd) * Uref_cruise/(498 * wing_loading))
    gust_load_factors = np.append(gust_load_factors, 1 + Kg * CLa * (1.05 * Vd) * Uref_dive/(498 * wing_loading))

    # guts load extension for gust lines at Vb and Vc for a rough gust
    if category_tag == 'commuter':
        gust_load_factors = np.append(gust_load_factors, 1 + Kg * CLa * (1.05 * Vd) * Uref_rough/(498 * wing_loading))
    else:
        gust_load_factors = np.append(gust_load_factors, 0)
      
    # Pack
    if sign_flag == 1:
        V_n_data.airspeeds.positive          = airspeeds
        V_n_data.load_factors.positive       = load_factors
        V_n_data.gust_load_factors.positive  = gust_load_factors
        V_n_data.limit_loads.dive.positive   = load_factors[len(load_factors) - 2]
        
    if sign_flag == 2:
        V_n_data.airspeeds.negative          = airspeeds
        V_n_data.load_factors.negative       = load_factors
        V_n_data.gust_load_factors.negative  = gust_load_factors
    
    return 
#------------------------------------------------------------------------------------------------------------------------

def gust_dive_speed_intersection(category_tag, load_factors, gust_load_factors, airspeeds, element_num, Vc, Vd):

    """ Calculates intersection between the general V-n diagram and the gust load for Vd

    Source:
    S. Gudmundsson "General Aviation Aircraft Design: Applied Methods and Procedures", Butterworth-Heinemann; 1 edition

    Inputs:
    load_factors                                [Unitless]
    gust_load_factors                           [Unitless]
    airspeeds                                   [kts]
    Vc                                          [kts]
    Vd                                          [kts]
    element_num                                 [Unitless]
    
    Outputs:
    airspeeds                                    [kts]
    load_factors                                 [Unitless]

    Properties Used:
    N/A

    Description:
    A specific function for CFR FAR Part 25 regulations. For negative loads, the general curve must go linear from a specific
    load factor at Vc to zero at Vd. Gust loads may put a non-zero load factor at Vd, so an intersection between the old and
    the new curves is required
    """
    
    if load_factors[1] > 0:
        V_inters = (load_factors[element_num] - gust_load_factors[2]) * (Vd - Vc) \
                    / (gust_load_factors[3] - gust_load_factors[2]) + Vc
        load     = load_factors[element_num]
    else:
        if category_tag == 'acrobatic':          
            V_inters = ((gust_load_factors[3] + 1) * Vc + (min(load_factors) - gust_load_factors[2]) * Vd) / \
                       (gust_load_factors[3] - gust_load_factors[2] + min(load_factors) + 1)
            load     = (min(load_factors) + 1) / (Vc - Vd) * V_inters - ((min(load_factors) + 1) / (Vc - Vd) * Vd + 1)
        else:
            V_inters = (gust_load_factors[3] * Vc + (min(load_factors) - gust_load_factors[2]) * Vd) / \
                       (gust_load_factors[3] - gust_load_factors[2] + min(load_factors))
            load     = min(load_factors) * (V_inters - Vd) / (Vc - Vd)


    airspeeds    = np.concatenate((airspeeds[:(element_num)], [V_inters], airspeeds[(element_num):]))
    load_factors = np.concatenate((load_factors[:(element_num)], [load], \
                                        load_factors[(element_num):]))    

    return airspeeds, load_factors
#--------------------------------------------------------------------------------------------------------------------------

def post_processing(category_tag, Uref_rough, Uref_cruise, Uref_dive, V_n_data, vehicle):

    """ Plot graph, save the final figure, and create results output file

    Source:

    Inputs:
    V_n_data.
        airspeeds.positive                  [kts]
        airspeeds.negative                  [kts]
        Vc                                  [kts]
        Vd                                  [kts]
        Vs1.positive                        [kts]
            negative                        [kts]
        Va.positive                         [kts]
            negative                        [kts]
        load_factors.positive               [Unitless]
            negative                        [Unitless]
        gust_load_factors.positive          [Unitless]
            negative                        [Unitless]
        weight                              [lb]
        altitude                            [ft]
    vehicle._base.tag                       [Unitless]
    Uref_rough                              [ft/s]
    Uref_cruise                             [ft/s]
    Uref_dive                               [ft/s]

    Outputs:

    Properties Used:
    N/A

    Description:
    """

    # Unpack
    load_factors_pos        = V_n_data.load_factors.positive
    load_factors_neg        = V_n_data.load_factors.negative
    airspeeds_pos           = V_n_data.airspeeds.positive
    airspeeds_neg           = V_n_data.airspeeds.negative
    Vc                      = V_n_data.Vc
    Vd                      = V_n_data.Vd
    Vs1_pos                 = V_n_data.Vs1.positive
    Vs1_neg                 = V_n_data.Vs1.negative
    Va_pos                  = V_n_data.Va.positive
    Va_neg                  = V_n_data.Va.negative
    gust_load_factors_pos   = V_n_data.gust_load_factors.positive
    gust_load_factors_neg   = V_n_data.gust_load_factors.negative
    weight                  = V_n_data.weight
    altitude                = V_n_data.altitude

    #-----------------------------
    # Plotting the V-n diagram
    #-----------------------------
    fig, ax = plt.subplots()
    ax.fill(airspeeds_pos, load_factors_pos, c='b', alpha=0.3)
    ax.fill(airspeeds_neg, load_factors_neg, c='b', alpha=0.3)
    ax.plot(airspeeds_pos, load_factors_pos, c='b')
    ax.plot(airspeeds_neg, load_factors_neg, c='b')

    # Plotting gust lines
    ax.plot([0, Vc,1.05*Vd],[1,gust_load_factors_pos[2],gust_load_factors_pos[len(gust_load_factors_pos)-3]],'--', c='r', label = ('Gust ' + str(round(Uref_cruise)) + 'fps'))
    ax.plot([0, Vd,1.05*Vd],[1,gust_load_factors_pos[3],gust_load_factors_pos[len(gust_load_factors_pos)-2]],'--', c='g', label = ('Gust ' + str(round(Uref_dive)) + 'fps'))
    ax.plot([0, Vc,1.05*Vd],[1,gust_load_factors_neg[2],gust_load_factors_neg[len(gust_load_factors_neg)-3]],'--', c='r')
    ax.plot([0, Vd,1.05*Vd],[1,gust_load_factors_neg[3],gust_load_factors_neg[len(gust_load_factors_neg)-2]],'--', c='g')

    if category_tag == 'commuter':
        ax.plot([0, 1.05*Vd],[1,gust_load_factors_pos[len(gust_load_factors_pos)-1]],'--', c='m', label = ('Gust ' + str(round(Uref_rough)) + 'fps'))
        ax.plot([0, 1.05*Vd],[1,gust_load_factors_neg[len(gust_load_factors_neg)-1]],'--', c='m')

    # Formating the plot
    ax.set_xlabel('Airspeed, KEAS')
    ax.set_ylabel('Load Factor')
    ax.set_title(vehicle.tag + '  Weight=' + str(round(weight)) + 'lb  ' + ' Altitude=' + str(round(altitude)) + 'ft ')
    ax.legend()
    ax.grid() 

    #---------------------------------
    # Creating results output file
    #---------------------------------
    fres = open("V_n_diagram_results_" + vehicle.tag +".dat","w")
    fres.write('V-n diagram summary\n')
    fres.write('-------------------\n')
    fres.write('Aircraft: ' + vehicle.tag + '\n')
    fres.write('category: ' + vehicle.flight_envelope.category + '\n')
    fres.write('FAR certification: Part ' +  vehicle.flight_envelope.FAR_part_number  + '\n')
    fres.write('Weight = ' + str(round(weight)) + ' lb\n')
    fres.write('Altitude = ' + str(round(altitude)) + ' ft\n')
    fres.write('---------------------------------------------------------------\n\n')
    fres.write('Airspeeds: \n')
    fres.write('    Positive stall speed (Vs1)   = ' + str(round(Vs1_pos,1)) + ' KEAS\n')
    fres.write('    Negative stall speed (Vs1)   = ' + str(round(Vs1_neg,1)) + ' KEAS\n')
    fres.write('    Positive maneuver speed (Va) = ' + str(round(Va_pos,1))  + ' KEAS\n')
    fres.write('    Negative maneuver speed (Va) = ' + str(round(Va_neg,1))  + ' KEAS\n')
    fres.write('    Cruise speed (Vc)            = ' + str(round(Vc,1))      + ' KEAS\n')
    fres.write('    Dive speed (Vd)              = ' + str(round(Vd,1))      + ' KEAS\n')
    fres.write('Load factors: \n')
    fres.write('    Positive limit load factor (n+) = ' + str(round(max(load_factors_pos),2)) + '\n')
    fres.write('    Negative limit load factor (n-) = ' + str(round(min(load_factors_neg),2)) + '\n')
    fres.write('    Positive load factor at Vd      = ' + str(round(V_n_data.limit_loads.dive.positive,2)) + '\n')
    fres.write('    Negative load factor at Vd      = ' + str(round(V_n_data.limit_loads.dive.negative,2)) + '\n')
   
    return
#------------------------------------------------------------------------------------------------------------------------

def convert_keas(V_n_data):

    """ Convert speed to KEAS

    Source:

    Inputs:
    V_n_data.
        airspeeds.positive              [ft/s]
            negative                    [ft/s]
        Vc                              [ft/s]
        Va.positive                     [ft/s]
        Va.negative                     [ft/s]
        Vs1.negative                    [ft/s]
        Vs1.positive                    [ft/s]
        density_ratio                   [Unitless]

    Outputs:
    V_n_data.              
        airspeeds.positive              [kts]
            negative                    [kts]
        Vc                              [kts]
        Va.positive                     [kts]
        Va.negative                     [kts]
        Vs1.negative                    [kts]
        Vs1.positive                    [kts]

    Properties Used:
    N/A

    Description:
    """

    # Unpack
    airspeeds_pos = V_n_data.airspeeds.positive
    airspeeds_neg = V_n_data.airspeeds.negative
    density_ratio = V_n_data.density_ratio
    Vc            = V_n_data.Vc
    Va_pos        = V_n_data.Va.positive
    Va_neg        = V_n_data.Va.negative
    Vs1_neg       = V_n_data.Vs1.negative
    Vs1_pos       = V_n_data.Vs1.positive
    
    airspeeds_pos = airspeeds_pos * Units['ft/s'] / Units.knots * density_ratio
    airspeeds_neg = airspeeds_neg * Units['ft/s'] / Units.knots * density_ratio

    Vs1_pos       = Vs1_pos * Units['ft/s'] / Units.knots * density_ratio
    Vs1_neg       = Vs1_neg * Units['ft/s'] / Units.knots * density_ratio
    Va_pos        = Va_pos * Units['ft/s'] / Units.knots * density_ratio
    Va_neg        = Va_neg * Units['ft/s'] / Units.knots * density_ratio
    Vc            = Vc[0] * Units['ft/s'] / Units.knots * density_ratio
    Vc            = Vc[0]
    
    # Pack
    V_n_data.airspeeds.positive = airspeeds_pos
    V_n_data.airspeeds.negative = airspeeds_neg
    V_n_data.Vc                 = Vc
    V_n_data.Va.positive        = Va_pos
    V_n_data.Va.negative        = Va_neg
    V_n_data.Vs1.negative       = Vs1_neg
    V_n_data.Vs1.positive       = Vs1_pos
    
    
    return