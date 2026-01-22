# Imports
import RCAIDE
from RCAIDE.Core import Units, Data   
from RCAIDE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_properties import compute_airfoil_properties
from RCAIDE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry    import import_airfoil_geometry    
from scipy.interpolate import interp1d
import os
import numpy as np 
import os 

# design propeller 
def design_SR1_prop():
    prop                            = RCAIDE.Library.Components.Powertrain.Converters.Propeller()
    prop.inputs                     = Data()
    prop.inputs.blade_pitch_command       = 0 
    prop.inputs.y_axis_rotation     = 0.
    prop.tag                        = 'SR1_Propeller'  
    prop.tip_radius                 = 0.631/2
    prop.hub_radius                 = prop.tip_radius * 0.2
    prop.number_of_blades           = 8
    prop.thrust_angle               = 0.0    
    t_b_vs_x_data                   = np.array([[0.21964793500338532, 0.2098578199052132],[0.23015572105619508, 0.1956398104265402],[0.24217332430602556, 0.1791469194312795],
                                                [0.24884224779959374, 0.1524170616113743],[0.26987813134732574, 0.1234123222748814],[0.30509817197020994, 0.0972511848341231],
                                                [0.3457684495599187, 0.07848341232227485],[0.4121123899796886, 0.06085308056872032],[0.501252538930264, 0.044928909952606566],
                                                [0.581577522004062, 0.03582938388625587],[0.6646377792823288, 0.03014218009478667],[0.761943127962085, 0.025592417061611333],
                                                [0.8477792823290451, 0.02218009478672980],[0.9306770480704127, 0.02104265402843597],[0.9992687880839539, 0.02047393364928904]])
                      
    b_D_vs_x_data                   = np.array([[0.2178943805010154, 0.149478672985782],[0.29787406905890323, 0.14976303317535],
                                              [0.3635680433310766, 0.150047393364928],[0.4464861205145563, 0.149194312796208],[0.5207718348002708, 0.149194312796208],
                                              [0.6065267433987811, 0.148625592417061],[0.6494041976980365, 0.148341232227488],[0.692362897765741, 0.1469194312796208],
                                              [0.7410765064319564, 0.144928909952606],[0.7841367637102232, 0.142085308056872],[0.8272376438727149, 0.138672985781990],
                                              [0.85895057549086, 0.13469194312796207],[0.8936425186188215, 0.129004739336492],[0.9168449559918752, 0.124170616113744],
                                              [0.934414353419093, 0.1181990521327014],[0.9491469194312794, 0.111943127962085],[0.9638388625592418, 0.106255924170616],
                                              [0.9815098171970211, 0.098862559241706],[0.9934664861205142, 0.091469194312796],[0.9967095463777929, 0.086066350710900]])
                    
                    
    delta_beta_vs_x                 = np. array([[0.2127623561272849, 20.530805687203788],[0.2630467163168584, 18.938388625592413],[0.309004739336493, 17.573459715639807],[0.36941096817874053, 15.29857819905213],
                                                [0.42696005416384564, 13.02369668246445],[0.490264048747461, 10.521327014218006],[0.5477928232904534, 8.360189573459714],
                                                [0.6226269465132024, 5.2890995260663445],[0.685870006770481, 3.127962085308056],[0.7664590385917398, -0.170616113744074],
                                                [0.8240487474610698, -2.672985781990526],[0.9161272850372373, -6.312796208530806],[1.0024306025727827, -9.611374407582943]])
    beta                            = 10
    dim                             = 30
    new_radius_distribution         = np.linspace(0.22,0.98,dim)
    func_twist_distribution         = interp1d(delta_beta_vs_x[:,0], delta_beta_vs_x [:,1], kind='cubic')
    func_chord_distribution         = interp1d(b_D_vs_x_data[:,0], b_D_vs_x_data[:,1]*2*prop.tip_radius   , kind='cubic')
    func_radius_distribution        = interp1d(b_D_vs_x_data[:,0], b_D_vs_x_data[:,0] , kind='cubic')
    func_max_thickness_distribution = interp1d(t_b_vs_x_data[:,0], t_b_vs_x_data[:,1], kind='cubic')  
    
    prop.twist_distribution         = (func_twist_distribution(new_radius_distribution) + beta) * Units.degrees     
    prop.chord_distribution         = func_chord_distribution(new_radius_distribution)         
    prop.radius_distribution        = func_radius_distribution(new_radius_distribution)        
    prop.max_thickness_distribution = func_max_thickness_distribution(new_radius_distribution)*prop.chord_distribution
    prop.thickness_to_chord         = prop.max_thickness_distribution/prop.chord_distribution 
    ospath                          = os.path.abspath(__file__)
    separator                       = os.path.sep
    rel_path                        = os.path.dirname(ospath) + separator   
     
    airfoil_1                       = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil_1.coordinate_file       =  rel_path +'..' + separator + 'Airfoils' + separator + 'NACA_65_215.txt'
    airfoil_1.polar_files           = [rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_65_215_polar_Re_50000.txt',rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_65_215_polar_Re_100000.txt',
                                        rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_65_215_polar_Re_200000.txt',rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_65_215_polar_Re_500000.txt',
                                        rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_65_215_polar_Re_1000000.txt']
    airfoil_1.geometry              = import_airfoil_geometry(airfoil_1.coordinate_file,airfoil_1.number_of_points)
    airfoil_1.polars                = compute_airfoil_properties(airfoil_1.geometry,airfoil_1.polar_files)
    prop.append_airfoil(airfoil_1)  

    airfoil_2                       = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil_2.coordinate_file       = rel_path +'..' + separator + 'Airfoils' + separator + 'NACA_15.txt'
    airfoil_2.polar_files           = [rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_15_polar_Re_50000.txt',
                                        rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_15_polar_Re_100000.txt',rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_15_polar_Re_200000.txt',
                                        rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_15_polar_Re_500000.txt',rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'NACA_15_polar_Re_1000000.txt']
    airfoil_2.geometry              = import_airfoil_geometry(airfoil_2.coordinate_file,airfoil_2.number_of_points)
    airfoil_2.polars                = compute_airfoil_properties(airfoil_2.geometry,airfoil_2.polar_files)
    prop.append_airfoil(airfoil_2)      
     
    airfoil_polar_stations          = np.zeros(dim) 
    n                               = len(prop.twist_distribution)  
    airfoil_polar_stations[round(n*0.50):] = 1    
    prop.airfoil_polar_stations     = list(airfoil_polar_stations.astype(int) )  
    prop.mid_chord_alignment        = prop.radius_distribution*np.tan(23*Units.degrees)  
    
    return prop
