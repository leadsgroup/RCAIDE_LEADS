## @ingroup DCode-C2_Aeroacoustics-CasalinoTest
# Casalino_Rotor.py
#
# Created:  Feb 2022, R. Erhard

import RCAIDE
from RCAIDE.Framework.Core import Units
#from SUAVE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_properties import compute_airfoil_properties #compute_airfoil_polars \
     #import compute_airfoil_polars
     
from DCode.Common.MFRotors.Methods.Airfoil.compute_airfoil_properties                      import compute_airfoil_properties     
from RCAIDE.Library.Methods.Geometry.Airfoil.import_airfoil_geometry                       import import_airfoil_geometry

#from SUAVE.Plots.Performance.Airfoil_Plots import plot_airfoil_aerodynamic_coefficients
from SUAVE.Analyses.Propulsion.Rotor_Wake_Fidelity_One import Rotor_Wake_Fidelity_One
from SUAVE.Analyses.Propulsion.Rotor_Wake_Fidelity_Two import Rotor_Wake_Fidelity_Two

from DCode.Common.MFRotors.Components import Propeller, Airfoil

import pylab as plt
import numpy as np
import os


def vehicle_setup(plots=False):
    print("Setting up vehicle...")
    #-----------------------------------------------------------------
    #   Vehicle Initialization:
    #-----------------------------------------------------------------
    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Casalino_Propeller_Vehicle'    
    
    # ------------------------------------------------------------------
    #   Propulsion Properties
    # ------------------------------------------------------------------
    net                          = SUAVE.Components.Energy.Networks.Battery_Propeller()
    net.tag                      = 'battery_propeller'
    net.number_of_engines        = 1
    
    # Use time-accurate propeller model
    print("\tGenerating rotor geometry...")
    prop = Casalino_Rotor(plot_figures=plots) 
    
    # set propeller origin and defaults
    prop.origin                    = np.array([[0., 0., 0.]])
    prop.inputs.y_axis_rotation    = 0.0
    prop.use_2d_analysis           = True
    prop.number_azimuthal_stations = 20
    prop.rotation = -1
    
    net.propellers.append(prop)
    vehicle.append_component(net)
    
    
    # setup atmospheric analyses
    analyses   = SUAVE.Analyses.Vehicle()
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)      
    
    return vehicle, analyses


def Casalino_Rotor(fidelity, plot_figures):
    """
    Rotor geometry definition, as described in Casalino's paper, "Definition of a benchmark
    for low Reynolds number propeller aeroacoustics," 2021.
    
    """

    # --------------------------------------------------------------------------------------------------
    # Propeller Distribution Data Extracted from Paper
    # --------------------------------------------------------------------------------------------------
    r_R_input = np.linspace(0.3, 0.99, 17)
    r_R, c_R, beta_deg = plot_rotor_distributions(r_R_input, plot_figures=plot_figures)
    
    # --------------------------------------------------------------------------------------------------
    # Generate Propeller Geometry
    # --------------------------------------------------------------------------------------------------
    prop = Propeller()
    if fidelity == 1:
        prop.Wake = Rotor_Wake_Fidelity_One()
    elif fidelity == 2:
        prop.Wake = Rotor_Wake_Fidelity_Two()
        
    prop.tag = "propeller"
    prop.number_of_blades           = 2
    prop.tip_radius                 = 0.1500 * Units.meter
    prop.hub_radius                 = r_R_input[0]* prop.tip_radius #0.0125 * Units.meter
    prop.twist_distribution         = beta_deg * Units.deg
    prop.chord_distribution         = c_R * prop.tip_radius
    prop.radius_distribution        = r_R * prop.tip_radius
    prop.thickness_to_chord         = np.ones_like(c_R) * 0.12     
    prop.max_thickness_distribution = prop.thickness_to_chord * prop.chord_distribution
    
    # note original section parameters
    
    prop.number_azimuthal_stations = 91
    prop.number_radial_stations    = len(prop.radius_distribution)    
    #prop.airfoil_polar_stations    = prop.number_radial_stations * [0] #list(map(int,np.zeros(len(prop.radius_distribution))))
    
    airfoils_path         = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Airfoils/")
    polars_path           = os.path.join(os.path.dirname(os.path.abspath(__file__)), "Airfoils/Polars/")
    
    airfoil_1 = Airfoil()
    airfoil_1.tag = "NACA_4412"
    airfoil_1.coordinate_file = f"{airfoils_path}/NACA_4412.txt"
    airfoil_1.polar_files = [f"{polars_path}/NACA_4412_polar_Re_100000.txt",
                           f"{polars_path}/NACA_4412_polar_Re_200000.txt",
                           f"{polars_path}/NACA_4412_polar_Re_500000.txt",
                           f"{polars_path}/NACA_4412_polar_Re_1000000.txt"]
    airfoil_1.geometry = import_airfoil_geometry(airfoil_1.coordinate_file, airfoil_1.number_of_points)
    airfoil_1.polars = compute_airfoil_properties(airfoil_1.geometry, airfoil_1.polar_files)
    prop.append_airfoil(airfoil_1)
    
    prop.airfoil_polar_stations = np.zeros(len(prop.radius_distribution))
    prop.airfoil_polar_stations = list(prop.airfoil_polar_stations.astype(int))
    
    # get thickness distribution
    prop.finalize_rotor_geometry()
    
    # plot surrogate results
    if plot_figures:
        plot_airfoil_aerodynamic_coefficients(prop.airfoil_geometry, prop.airfoil_polars, line_color='m-',use_surrogate=True)    
    return prop


def plot_rotor_distributions(r_R_input=None, plot_figures=True):
    '''
    Data extracted from Casalino's paper, Figure 4, for blade chord and twist distributions.
    Third order polynomial fit to data used.
       
    '''
    #-----------------------------------------------------------------------------------------------
    # Data digitized from paper
    #-----------------------------------------------------------------------------------------------
    r_R1 = np.array([0.1903,0.2166,0.2545,0.2997,0.3377,0.3916,0.4383,0.4821,0.5273,0.5857,
                     0.6382,0.6994,0.7563,0.8103,0.8789,0.9241,0.9970 ])
    c_R = np.array([0.1783,0.1912,0.2067,0.2195,0.2263,0.2297,0.2283,0.2243,0.2168,0.2040,
                    0.1891,0.1716,0.1527,0.1344,0.1094,0.0939,0.0672 ])
    r_R2 = np.array([0.1903,0.2005,0.2122,0.2326,0.2603,0.2881,0.3202,0.3581,0.3975,0.4485,
                     0.5040,0.5594,0.6192,0.7038,0.7724,0.8249,0.8803,0.9387,0.9970,])
    beta = np.array([43.101,43.775,43.640,42.157,39.325,36.494,33.528,30.764,28.134,25.438,
                     22.876,20.719,19.101,16.808,15.325,14.382,13.438,12.629,12.224])    
    #-----------------------------------------------------------------------------------------------    
    # Set new radius distribution
    #-----------------------------------------------------------------------------------------------
    if r_R_input is None:
        r_R = np.linspace(r_R1[0],r_R1[-1],100) 
    else:
        r_R = r_R_input
        
    #-----------------------------------------------------------------------------------------------    
    # Create polynomial fit for distributions from data
    #-----------------------------------------------------------------------------------------------
    c_R_fit  = np.poly1d(np.polyfit(r_R1, c_R, 3))
    c_R_new  = c_R_fit(r_R)

    beta_fit  = np.poly1d(np.polyfit(r_R2, beta, 3))
    beta_new  = beta_fit(r_R)     

    #-----------------------------------------------------------------------------------------------    
    # Plot distributions
    #-----------------------------------------------------------------------------------------------
    if plot_figures:
        fig, ax1 = plt.subplots()
        ax2      = ax1.twinx()
        
        ax1.plot(r_R1, c_R, 'ro')
        ax1.plot(r_R,c_R_new,'r-',label='Chord distribution')
        ax2.plot(r_R2, beta, 'bo')
        ax2.plot(r_R,beta_new,'b-',label='Twist distribution')
    
        ax1.set_xlabel('r/R [-]')    
        ax1.set_ylabel('c/R [-]')
        ax2.set_ylabel('Twist [deg]')
        
        ax1.set_ylim([0.05, 0.3])
        ax2.set_ylim([0, 50])
        fig.legend()
        plt.show()

    #-----------------------------------------------------------------------------------------------    
    # Return distributions
    #-----------------------------------------------------------------------------------------------    
    return r_R, c_R_new, beta_new