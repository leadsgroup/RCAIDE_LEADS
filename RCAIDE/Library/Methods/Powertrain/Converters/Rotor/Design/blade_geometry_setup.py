# RCAIDE/Methods/Energy/Propulsors/Rotor_Design/optimization_setup.py
# 
# 
# Created:  Jul 2023, M. Clarke 


# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  

# RCAIDE Imports 
import RCAIDE   
from RCAIDE.Library.Methods.Geometry.Airfoil  import compute_airfoil_properties, compute_naca_4series, import_airfoil_geometry

# Python package imports   
import numpy as np  
    
# ----------------------------------------------------------------------------------------------------------------------  
#  Blade Geometry Setup 
# ----------------------------------------------------------------------------------------------------------------------    
def blade_geometry_setup(rotor, number_of_stations):
    """
    Defines a configuration for prop-rotor blade optimization.
    
    Parameters
    ----------
    rotor : RCAIDE.Library.Components.Powertrain.Converters.Rotor
        Rotor component with the following attributes:
            - number_of_blades : int
                Number of blades on the rotor
            - tip_radius : float
                Tip radius of the rotor [m]
            - hub_radius : float
                Hub radius of the rotor [m]
            - hover : Data
                Hover conditions
                    - design_thrust : float, optional
                        Design thrust at hover [N]
                    - design_power : float, optional
                        Design power at hover [W]
                    - design_freestream_velocity : float
                        Freestream velocity at hover [m/s]
                    - design_altitude : float
                        Altitude at hover [m]
            - oei : Data
                One engine inoperative conditions
                    - design_freestream_velocity : float, optional
                        Freestream velocity at OEI [m/s]
                    - design_altitude : float, optional
                        Altitude at OEI [m]
                    - design_thrust : float, optional
                        Design thrust at OEI [N]
                    - design_power : float, optional
                        Design power at OEI [W]
            - airfoils : dict
                Dictionary of airfoil objects
            - airfoil_polar_stations : list
                List of airfoil section indices for each station
    number_of_stations : int
        Number of radial stations for blade discretization
    
    Returns
    -------
    configs : RCAIDE.Library.Components.Configs.Config.Container
        Configuration container with the following configs:
            - hover : Config
                Hover configuration
            - oei : Config
                One engine inoperative configuration
            - cruise : Config (only for prop-rotors)
                Cruise configuration
    
    Notes
    -----
    This function prepares the necessary configurations for rotor blade optimization by:
        1. Unpacking rotor geometry parameters
        2. Validating design requirements (thrust or power must be specified)
        3. Processing airfoil data (importing geometry and computing polars if needed)
        4. Setting up the radial distribution of stations
        5. Extracting thickness-to-chord ratios from airfoils
        6. Setting default OEI conditions if not specified
        7. Creating vehicle and network configurations for analysis
    
    The function creates separate configurations for hover, OEI, and cruise (for prop-rotors)
    conditions, which are used during the optimization process to evaluate the rotor
    performance at different operating points.
    
    **Major Assumptions**
        * Either design thrust or design power must be specified (not both)
        * If OEI conditions are not specified, they default to hover conditions with 10% increase
        * Rotor orientation is set to [0.0, π/2, 0.0] (horizontal rotor)
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Design.optimization_setup
    RCAIDE.Library.Methods.Geometry.Airfoil.compute_airfoil_properties
    RCAIDE.Library.Methods.Geometry.Airfoil.compute_naca_4series
    RCAIDE.Library.Methods.Geometry.Airfoil.import_airfoil_geometry
    """    
    
    # Unpack prop-rotor geometry  
    N                     = number_of_stations       
    B                     = rotor.number_of_blades    
    R                     = rotor.tip_radius
    Rh                    = rotor.hub_radius 
    design_thrust_hover   = rotor.hover.design_thrust
    design_power_hover    = rotor.hover.design_power
    chi0                  = Rh/R  
    chi                   = np.linspace(chi0,1,N+1)  
    chi                   = chi[0:N]
    airfoils              = rotor.airfoils      
    a_loc                 = rotor.airfoil_polar_stations  
    
    # determine target values 
    if (design_thrust_hover == None) and (design_power_hover== None):
        raise AssertionError('Specify either design thrust or design power at hover!') 
    elif (design_thrust_hover!= None) and (design_power_hover!= None):
        raise AssertionError('Specify either design thrust or design power at hover!')     
    num_airfoils = len(airfoils.keys())
    if num_airfoils>0:
        if len(a_loc) != N:
            raise AssertionError('\nDimension of airfoil sections must be equal to number of stations on propeller') 
        
        for _,airfoil in enumerate(airfoils):  
            if airfoil.geometry == None: # first, if airfoil geometry data not defined, import from geoemtry files
                if type(airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil: # check if naca 4 series of airfoil from datafile
                    airfoil.geometry = compute_naca_4series(airfoil.NACA_4_Series_code,airfoil.number_of_points)
                else:
                    airfoil.geometry = import_airfoil_geometry(airfoil.coordinate_file,airfoil.number_of_points) 
    
            if airfoil.polars == None: # compute airfoil polars for airfoils
                airfoil.polars = compute_airfoil_properties(airfoil.geometry, airfoil_polar_files= airfoil.polar_files) 
                     
    # thickness to chord         
    t_c           = np.zeros(N)    
    if num_airfoils>0:
        for j,airfoil in enumerate(airfoils): 
            a_geo         = airfoil.geometry
            locs          = np.where(np.array(a_loc) == j ) 
            t_c[locs]     = a_geo.thickness_to_chord 
            
    # append additional prop-rotor  properties for optimization  
    rotor.number_of_blades             = int(B)  
    rotor.thickness_to_chord           = t_c
    rotor.radius_distribution          = chi*R  
    
    # set oei conditions if they are not set
    if rotor.oei.design_freestream_velocity == None: 
        rotor.oei.design_freestream_velocity = rotor.hover.design_freestream_velocity
    if rotor.oei.design_altitude == None:
        rotor.oei.design_altitude = rotor.hover.design_altitude
    if design_thrust_hover == None:
        if rotor.oei.design_power == None: 
            rotor.oei.design_power = rotor.hover.design_power*1.1
    elif  design_power_hover == None:
        if rotor.oei.design_thrust == None: 
            rotor.oei.design_thrust = rotor.hover.design_thrust*1.1
    
    vehicle                            = RCAIDE.Vehicle()  
    net                                = RCAIDE.Framework.Networks.Electric() 
    propulsor                          = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor()
    propulsor.rotor                    = rotor 
    net.propulsors.append(propulsor) 
    vehicle.append_energy_network(net)
    
    configs                             = RCAIDE.Library.Components.Configs.Config.Container()
    base_config                         = RCAIDE.Library.Components.Configs.Config(vehicle) 
    
    config                              = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                          = 'hover' 
    config.networks.electric.propulsors.electric_rotor.rotor.orientation_euler_angles = [0.0,np.pi/2,0.0]    
    configs.append(config)        

    config                              = RCAIDE.Library.Components.Configs.Config(base_config)
    config.tag                          = 'oei' 
    config.networks.electric.propulsors.electric_rotor.rotor.orientation_euler_angles = [0.0,np.pi/2,0.0]    
    configs.append(config)       
    
    if type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor:  
        design_thrust_cruise  = rotor.cruise.design_thrust 
        design_power_cruise   = rotor.cruise.design_power      
        if (design_thrust_cruise == None) and (design_power_cruise== None):
            raise AssertionError('Specify either design thrust or design power at cruise!') 
        elif (design_thrust_cruise!= None) and (design_power_cruise!= None):
            raise AssertionError('Specify either design thrust or design power at cruise!') 
        
        config                          = RCAIDE.Library.Components.Configs.Config(base_config)
        config.tag                      = 'cruise'
        config.networks.electric.propulsors.electric_rotor.rotor.orientation_euler_angles = [0.0,np.pi/2,0.0] 
        configs.append(config)
    return configs 