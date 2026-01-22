# Imports
import RCAIDE
from RCAIDE.Core import Units, Data  
from RCAIDE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.compute_airfoil_properties import compute_airfoil_properties
from RCAIDE.Methods.Geometry.Two_Dimensional.Cross_Section.Airfoil.import_airfoil_geometry    import import_airfoil_geometry    
from scipy.interpolate import interp1d 
import os 
import numpy as np   

# design propeller 
def design_DJI_9_4x5_prop():        
    prop                            = RCAIDE.Library.Components.Powertrain.Converters.Rotor()
    prop.inputs                     = Data() 
    prop.inputs.blade_pitch_command       = 0 
    prop.inputs.y_axis_rotation     = 0.
    prop.tag                        = 'DJI_9_4x5_Propeller'
    prop.tip_radius                 = 4.75*Units.inches
    prop.hub_radius                 = prop.tip_radius*0.15
    prop.number_of_blades           = 2  
    prop.thrust_angle               = 0. 
    dimensionless_radius_chord      = np.array([[0.08, 0.15231660784126677              ],[0.10113994085662492, 0.1733749880759324],[0.12981970809882665, 0.1956295907660020],[0.14898883907278448, 0.2166908327768767],[0.17289897929981873, 0.2365415434513020],
                                                [0.19914623676428495, 0.2515701612133930],[0.22058571019746248, 0.2593756558237145],[0.2443575312410568, 0.2617566536296861 ],[0.2704712391490984, 0.25991796241533904],[0.29180578078794234, 0.2544705714013163],
                                                [0.31311647429171036, 0.2460111609272154],[0.3415577601831536, 0.23814556901650286],[0.36285891443289137, 0.2284813507583706],[0.3865401125631976, 0.21941667461604497],[0.4054803014404272, 0.21156252981016876],
                                                [0.4339168177048554, 0.20309453400744057],[0.4576075550891919, 0.19523466564914618],[0.4813078317275588, 0.18857960507488308],[0.5073833826194789, 0.1819216827244109 ],[0.5310836592578458, 0.1752666221501478 ],
                                                [0.5524182008966898, 0.16981923113612513],[0.5737432032815033, 0.16316703233807112],[0.5998187541734235, 0.15650910998759893],[0.6211580654392825, 0.15166412286559186],[0.6448678813316797, 0.14621387007536008],
                                                [0.6685776972240769, 0.14076361728512823],[0.6922875131164743, 0.13531336449489648],[0.7160068682629017, 0.13106791948869592],[0.739721453782314, 0.1262200705904798  ],[0.7634408089287417, 0.12197462558427927],
                                                [0.787155394448154, 0.11712677668606311 ],[0.8108795192215967, 0.11348373557187824],[0.8369789182485929, 0.10983783268148428],[0.8607030430220357, 0.10619479156729938],[0.8796813889153867, 0.10315987789754835],
                                                [0.908151292568921, 0.09890870933892965 ],[0.9295001430888103, 0.0952685300009539 ],[0.9532242678622531, 0.09162548888676902],[0.97692454450062, 0.08497042831250592  ],[0.99, 0.06927644758179907              ]])
                                   
    dimensionless_radius_twist      = np.array([[0.08, 17.635270541082164                ],[0.09523809523809512, 18.7374749498998   ],[0.12142857142857144, 19.338677354709418 ],[0.1452380952380954, 19.68937875751503   ],[0.16666666666666674, 19.789579158316634 ],
                                                [0.1952380952380952, 19.839679358717433  ],[0.21666666666666679, 19.488977955911825 ],[0.23809523809523814, 19.138276553106213 ],[0.2666666666666666, 18.637274549098198  ],[0.28809523809523796, 18.18637274549098  ],
                                                [0.3119047619047619, 17.635270541082164  ],[0.33571428571428563, 16.983967935871743 ],[0.3595238095238096, 16.482965931863728  ],[0.38095238095238093, 16.032064128256515 ],[0.40714285714285703, 15.480961923847694 ],
                                                [0.43095238095238075, 15.080160320641282 ],[0.4523809523809521, 14.579158316633267  ],[0.47857142857142865, 14.028056112224448 ],[0.5023809523809524, 13.577154308617235  ],[0.5238095238095237, 13.076152304609218  ],
                                                [0.5476190476190474, 12.625250501002004  ],[0.5714285714285712, 12.17434869739479   ],[0.5952380952380953, 11.723446893787575  ],[0.6214285714285714, 11.172344689378757  ],[0.6428571428571428, 10.721442885771543  ],
                                                [0.6642857142857141, 10.220440881763526  ],[0.6904761904761907, 9.76953907815631    ],[0.7119047619047616, 9.268537074148298   ],[0.7357142857142858, 8.867735470941884   ],[0.7595238095238095, 8.466933867735474   ],
                                                [0.7857142857142856, 8.016032064128257   ],[0.8095238095238098, 7.565130260521041   ],[0.8309523809523811, 7.264529058116231   ],[0.8547619047619048, 7.064128256513026   ],[0.8785714285714286, 6.813627254509019   ],
                                                [0.9023809523809523, 6.6633266533066156  ],[0.926190476190476, 6.462925851703407    ],[0.9476190476190474, 6.4128256513026045  ],[0.9761904761904758, 6.212424849699399   ],[0.99, 5.7                 ]])

    r_R                             = dimensionless_radius_chord[:,0]
    b_R                             = dimensionless_radius_chord[:,1] 
 
    dim = 20
    r_R_data                        = np.array([0.239,0.275,0.367,0.449,0.5,0.55,
                                                0.6,0.65,0.7,0.75,0.8,0.85,0.9,0.95,0.99])
    t_b_data                        = np.array([0.122,0.105,0.077,0.061,0.055,0.049,0.045,0.041,0.038
                                                ,0.035,0.033,0.031,0.029,0.027,0.026])  
    b_D_data                        = np.array([0.14485,0.14587,0.1481,
                                                0.1499,0.15061,0.15058,0.14981,0.14831,0.1468,0.14529,0.14268,
                                                0.13764,0.12896,0.11304,0.085])    
    
    new_radius_distribution         = np.linspace(0.239,0.98,dim) 
    func_max_thickness_distribution = interp1d(r_R_data, t_b_data*b_D_data*2*prop.tip_radius, kind='cubic')   
    
    beta                            = dimensionless_radius_twist[:,1]  
    prop.twist_distribution         = beta[::2]*Units.degrees
    prop.chord_distribution         = b_R[::2]*prop.tip_radius   
    prop.radius_distribution        = r_R[::2]*prop.tip_radius    
    prop.max_thickness_distribution = func_max_thickness_distribution(new_radius_distribution)  
    prop.thickness_to_chord         = prop.max_thickness_distribution/prop.chord_distribution  

    ospath                          = os.path.abspath(__file__)
    separator                       = os.path.sep
    rel_path                        = os.path.dirname(ospath) + separator   
     
    airfoil_1                       = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil_1.coordinate_file       = rel_path +'..' + separator + 'Airfoils' + separator + 'E63.txt'
    airfoil_1.polar_files           = [rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'E63_polar_Re_50000.txt'     ,rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'E63_polar_Re_100000.txt',
                                        rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'E63_polar_Re_200000.txt'    ,rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'E63_polar_Re_500000.txt',
                                        rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'E63_polar_Re_1000000.txt']
    airfoil_1.geometry              = import_airfoil_geometry(airfoil_1.coordinate_file,airfoil_1.number_of_points)
    airfoil_1.polars                = compute_airfoil_properties(airfoil_1.geometry,airfoil_1.polar_files)
    prop.append_airfoil(airfoil_1)  

    airfoil_2                       = RCAIDE.Library.Components.Airfoils.Airfoil()   
    airfoil_2.coordinate_file       = rel_path +'..' + separator + 'Airfoils' + separator + 'Clark_y.txt'
    airfoil_2.polar_files           = [ rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_50000.txt',
                                        rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_100000.txt',rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_200000.txt',
                                        rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_500000.txt',rel_path +'..' + separator + 'Airfoils' + separator + 'Polars' + separator +'Clark_y_polar_Re_1000000.txt']
    airfoil_2.geometry              = import_airfoil_geometry(airfoil_2.coordinate_file,airfoil_2.number_of_points)
    airfoil_2.polars                = compute_airfoil_properties(airfoil_2.geometry,airfoil_2.polar_files)
    prop.append_airfoil(airfoil_2)      
     
    airfoil_polar_stations          =  np.zeros(len(prop.thickness_to_chord)) 
    prop.airfoil_polar_stations     = list(airfoil_polar_stations.astype(int) ) 
    prop.mid_chord_alignment        = np.zeros_like(prop.chord_distribution) #  prop.chord_distribution/4. - prop.chord_distribution[0]/4.    
    
    return prop