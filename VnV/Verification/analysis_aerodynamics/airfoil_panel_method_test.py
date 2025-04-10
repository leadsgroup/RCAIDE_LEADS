# airfoil_panel_method_test.py
#
# Created: Dec 2023, M. Clarke  
 
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE Imports 
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Aerodynamics.Airfoil_Panel_Method     import airfoil_analysis 
from RCAIDE.Library.Methods.Geometry.Airfoil                      import compute_naca_4series
from RCAIDE.Library.Methods.Geometry.Airfoil                      import import_airfoil_geometry
from RCAIDE.Library.Plots import * 

# Python imports
import os 
import numpy as np
import matplotlib.pyplot as plt    
import matplotlib.cm as cm

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():   
    single_airfoil()
    multi_airfoil()
    high_AoA_test()
    
    return 
    
def single_airfoil():
    # -----------------------------------------------
    # Batch analysis of single airfoil - NACA 4412 
    # -----------------------------------------------
    AoA_deg              = np.linspace(-5,10,16)
    Re_vals              = np.atleast_2d(np.ones(len(AoA_deg)))*1E5 
    AoA_rad              = np.atleast_2d(AoA_deg*Units.degrees)   
    airfoil_file_1       = '4412'
    airfoil_geometry_1   = compute_naca_4series(airfoil_file_1,npoints = 201)
    airfoil_properties_1 = airfoil_analysis(airfoil_geometry_1,AoA_rad,Re_vals) 
     
    # get line colors for plots 
    line_colors   = cm.viridis(np.linspace(0,1,16))

    fig_1   = plt.figure("Chordwise Lift Distribution")
    fig_2   = plt.figure("Chordwise Drag Distribution") 
    fig_1.set_size_inches(7,5)
    fig_2.set_size_inches(7,5) 
    axis_1 = fig_1.add_subplot(1,1,1)    
    axis_2 = fig_2.add_subplot(1,1,1)
    
    # distribution plots
    for i in range(16):
        axis_1.plot(airfoil_geometry_1.x_upper_surface[:-1],airfoil_properties_1.fL[:,i,0], color = line_colors[i] ,  label = 'AOA =' +  str(AoA_deg[i]) + 'degrees') 
        axis_1.set_ylabel("fL")
        axis_1.set_xlabel("c") 
        axis_1.set_ylim([-1, 1])
    
    for i in range(16):
        axis_2.plot(airfoil_geometry_1.x_upper_surface[:-1],airfoil_properties_1.fD[:,i,0], color = line_colors[i] ,  label = 'AOA =' +  str(AoA_deg[i]) + 'degrees') 
        axis_2.set_ylabel("fD")
        axis_2.set_xlabel("c")
        axis_2.set_ylim([-1, 1])
    
    # Plots    
    plot_airfoil_surface_forces(airfoil_properties_1)   
    plot_airfoil_polars(airfoil_properties_1)
    plot_airfoil_boundary_layer_properties(airfoil_properties_1,show_legend = True ) 
    
    # Verification  
    cl_invisc_true        = 1.0675160492711429
    cd_invisc_true        = 7.638304328309864e-05
    cd_visc_true          = 0.014083014418361904
    cm_invisc_true        = -0.11066949071512713 
    
    print('\nThis is for single airfoil')
    diff_CL = np.abs((airfoil_properties_1.cl_invisc[0,10] - cl_invisc_true)/cl_invisc_true)    
    print('\ncl_invisc difference')
    print(diff_CL)
    assert diff_CL < 1e-6 
    
    diff_CD_invisc = np.abs((airfoil_properties_1.cd_invisc[0,10] - cd_invisc_true)/cd_invisc_true)  
    print('\ncd_invisc difference')
    print(diff_CD_invisc)
    assert diff_CD_invisc  < 1e-6

    diff_CD_visc = np.abs((airfoil_properties_1.cd_visc[0,10] - cd_visc_true)/cd_visc_true)  
    print('\ncd_invisc difference')
    print(diff_CD_visc)
    assert diff_CD_visc  < 1e-6    

    diff_CM = np.abs((airfoil_properties_1.cm_invisc[0,10] - cm_invisc_true)/cm_invisc_true) 
    print('\ncm_invisc difference')
    print(diff_CM)
    assert diff_CM < 1e-6
    
    
    # Abbot Validation
    Cl_abbot = 0.906
    Cd_abbot = 0.0064
    Cm_abbot = -0.0885
    
    CL_invisc_abb_diff = np.abs((airfoil_properties_1.cl_invisc[0,10] - Cl_abbot)/Cl_abbot) 
    CD_invisc_abb_diff = np.abs((airfoil_properties_1.cd_invisc[0,10] - Cd_abbot)/Cd_abbot)
    CD_visc_abb_diff   = np.abs((airfoil_properties_1.cd_visc[0,10] - Cd_abbot)/Cd_abbot)
    CM_invisc_abb_diff = np.abs((airfoil_properties_1.cm_invisc[0,10] - Cm_abbot)/Cm_abbot)
    print('\nValidation against Abbot data')
    print('invisc Cl-Cl_abbot/Cl_abbot',CL_invisc_abb_diff*100,'%\n') 
    print('invisc Cd-Cd_abbot/Cd_abbot',CD_invisc_abb_diff*100,'%\n')
    print('visc Cd-Cd_abbot/Cd_abbot',CD_visc_abb_diff*100,'%\n')
    print('invisc Cm-Cm_abbot/Cm_abbot',CM_invisc_abb_diff*100,'%\n')
    
    
    # CFD Validation
    Cl_CFD = 1.086832
    Cd_CFD = 0.013989972
    Cm_CFD = -0.64460867
    
    CL_invisc_CFD_diff = np.abs((airfoil_properties_1.cl_invisc[0,10] - Cl_CFD)/Cl_CFD) 
    CD_invisc_CFD_diff = np.abs((airfoil_properties_1.cd_invisc[0,10] - Cd_CFD)/Cd_CFD)
    CD_visc_CFD_diff = np.abs((airfoil_properties_1.cd_visc[0,10] - Cd_CFD)/Cd_CFD)
    CM_invisc_CFD_diff = np.abs((airfoil_properties_1.cm_invisc[0,10] - Cm_CFD)/Cm_CFD)
    print('\nValidation against CFD data')
    print('invisc Cl-Cl_CFD/Cl_CFD',CL_invisc_CFD_diff*100,'%\n') 
    print('invisc Cd-Cd_CFD/Cd_CFD',CD_invisc_CFD_diff*100,'%\n')
    print('visc Cd-Cd_CFD/Cd_CFD',CD_visc_CFD_diff*100,'%\n')
    print('invisc Cm-Cm_CFD/Cm_CFD',CM_invisc_CFD_diff*100,'%\n')
    
    AoA_abbot_cl_cd = np.array([-4.1552,-3.2336,-2.128,-1.207,-0.1014,0,1.0038,2.6623,4.8738,6.4392,7.8209,9.6644])
    cl_abbot  = np.array([-0.0267,0.0573,0.1693,0.2811,0.393,0.404973127035831,0.5235,0.6914,0.906,1.1017,1.2602,1.4096])
    cd_abbot  = np.array([0.0066,0.0066,0.0063,0.0063,0.0062,0.0062,0.0062,0.0062,0.0064,0.008,0.0106,0.0123])
    
    AOA_abbot_cm = np.array([-4.0694,-1.4798,0,0.7399,3.1445,5.7341,8.1387])
    cm_abbot  = np.array([-0.0916,-0.0913,-0.0895666666666667,-0.0887,-0.0885,-0.0882,-0.081])
    
    cl_xfoil = np.array([-0.072,0.034,0.152,0.261,0.375,0.485,0.582,0.704,0.814,0.924,1.021,1.126,1.219,1.3,1.363,1.43])
    cd_xfoil = np.array([0.008,0.008,0.007,0.007,0.007,0.007,0.006,0.006,0.006,0.007,0.008,0.009,0.01,0.013,0.015,0.017])
    cm_xfoil = np.array([-0.105,-0.105,-0.104,-0.104,-0.104,-0.103,-0.101,-0.103,-0.102,-0.101,-0.1,-0.099,-0.095,-0.09,-0.084,-0.076])
    
    Final_CL_diff_invisc = np.abs(np.sum((airfoil_properties_1.cl_invisc[0,:] - cl_xfoil)/cl_xfoil))/16
    print('\nNormalised mean absolute error with respect to xfoil in CL inviscid',Final_CL_diff_invisc*100,'%\n') 
    Final_CD_diff_invisc = np.abs(np.sum((airfoil_properties_1.cd_invisc[0,:] - cd_xfoil)/cd_xfoil))/16
    print('Normalised mean absolute error with respect to xfoil in CD inviscid',Final_CD_diff_invisc*100,'%\n')
    Final_CD_diff_visc = np.abs(np.sum((airfoil_properties_1.cd_visc[0,:] - cd_xfoil)/cd_xfoil))/16
    print('Normalised mean absolute error with respect to xfoil in CD viscous',Final_CD_diff_visc*100,'%\n')
    Final_CM_diff_invisc = np.abs(np.sum((airfoil_properties_1.cm_invisc[0,:] - cm_xfoil)/cm_xfoil))/16
    print('Normalised mean absolute error with respect to xfoil in CM inviscid',Final_CM_diff_invisc*100,'%\n') 
    
    fig_3 = plt.figure()
    fig_3.set_size_inches(6,6)
    axis_3_1 = fig_3.add_subplot(3,1,1) 
    axis_3_2 = fig_3.add_subplot(3,1,2) 
    axis_3_3 = fig_3.add_subplot(3,1,3) 
    
    axis_3_1.plot(AoA_deg,airfoil_properties_1.cl_invisc[0,:],'-ko', label='cl invisc')
    axis_3_1.set_ylabel('CL inviscid')
    axis_3_1.plot(AoA_abbot_cl_cd,cl_abbot, label='abbot')
    axis_3_1.plot(AoA_deg,cl_xfoil,label='xfoil')
    axis_3_1.legend(loc="upper right") 
    axis_3_2.plot(AoA_deg,airfoil_properties_1.cd_invisc[0,:],'-ko', label='cd invisc')
    axis_3_2.set_ylabel('CD inviscid')
    axis_3_2.plot(AoA_abbot_cl_cd,cd_abbot,label='abbot')
    axis_3_2.plot(AoA_deg,cd_xfoil,label='xfoil')
    axis_3_2.legend(loc="upper right")  
    axis_3_3.plot(AoA_deg,airfoil_properties_1.cm_invisc[0,:],'-ko', label='cm invisc')
    axis_3_3.set_ylabel('CM inviscid')
    axis_3_3.plot(AOA_abbot_cm,cm_abbot,label='abbot')
    axis_3_3.plot(AoA_deg,cm_xfoil,label='xfoil')
    axis_3_3.legend(loc="upper right")
    axis_3_3.set_xlabel('AoA') 
       
    return 
    
def multi_airfoil():
    # -----------------------------------------------
    # Single Condition Analysis of multiple airfoils  
    # ----------------------------------------------- 
    ospath                = os.path.abspath(__file__)
    separator             = os.path.sep 
    rel_path              = ospath.split('analysis_aerodynamics' + separator + 'airfoil_panel_method_test.py')[0] + '..' + separator + 'Vehicles' + separator + 'Airfoils' + separator 
    Re_vals               = np.array([[1E5, 1E5, 1E5, 1E5, 1E5, 1E5],[2E5, 2E5, 2E5, 2E5, 2E5, 2E5]])
    AoA_vals              = np.array([[0,1,2,3,4,5],[0,1,2,3,4,5]])*Units.degrees  
    airfoil_file_2        = rel_path + 'NACA_4412.txt'     
    airfoil_geometry_2    = import_airfoil_geometry(airfoil_file_2,npoints = 201)      
    airfoil_properties_2  = airfoil_analysis(airfoil_geometry_2,AoA_vals,Re_vals)     
       
    True_cl_invisc = np.array([0.45707213, 0.56697577, 0.6767239 , 0.78628253, 0.8956177 , 1.0046955 ])
    True_cd_invisc = np.array([ 5.62827861e-05, -9.72045393e-05, -2.40818475e-04, -3.73629564e-04, -4.94741048e-04, -6.03291489e-04])
    True_cm_invisc = np.array([-0.09711004, -0.09645007, -0.09581086, -0.0951932 , -0.09459784, -0.0940255 ])
    
    print('\n\nThis is for multi airfoil')
    print('\ninvisc CL difference')
    print(np.sum(np.abs((airfoil_properties_2.cl_invisc[0]  - True_cl_invisc)/True_cl_invisc)))
    assert np.sum(np.abs((airfoil_properties_2.cl_invisc[0]   - True_cl_invisc)/True_cl_invisc)) < 1e-5 
    
    print('\nvisc CD difference') 
    print(np.sum(np.abs((airfoil_properties_2.cd_invisc[0]  - True_cd_invisc)/True_cd_invisc)))
    assert np.sum(np.abs((airfoil_properties_2.cd_invisc[0]   - True_cd_invisc)/True_cd_invisc)) < 1e-5
    
    print('\ninvisc CM difference') 
    print(np.sum(np.abs((airfoil_properties_2.cm_invisc[0]  - True_cm_invisc)/True_cm_invisc)))
    assert np.sum(np.abs((airfoil_properties_2.cm_invisc[0]   - True_cm_invisc)/True_cm_invisc)) < 1e-5  
    return

def high_AoA_test():
    
    # This test confirms if AoAs above 80 degrees are neglected inside airfoil analysis

    AoA_deg              = np.array([0,20,40,60,85,100])
    Re_vals              = np.atleast_2d(np.ones(len(AoA_deg)))*1E6 
    AoA_rad              = np.atleast_2d(AoA_deg*Units.degrees)
    airfoil_file_1       = '4412'
    airfoil_geometry_1   = compute_naca_4series(airfoil_file_1,npoints = 201)
    airfoil_properties_1 = airfoil_analysis(airfoil_geometry_1,AoA_rad,Re_vals)
    
    fig_1 = plt.figure()
    fig_1.set_size_inches(6,6)
    axis_1 = fig_1.add_subplot(1,1,1)
    axis_1.plot(AoA_deg,airfoil_properties_1.cl_invisc[0,:],'-ko', label='cl invisc')
    axis_1.set_ylabel('CL inviscid')
    axis_1.set_xlabel('AoA')

    fig_2 = plt.figure()
    fig_2.set_size_inches(6,6)
    axis_2 = fig_2.add_subplot(1,1,1)
    axis_2.plot(AoA_deg,airfoil_properties_1.cd_visc[0,:],'-ko', label='cd visc')
    axis_2.set_ylabel('CD viscous')
    axis_2.set_xlabel('AoA')

    return


if __name__ == '__main__': 
    main() 
    plt.show()