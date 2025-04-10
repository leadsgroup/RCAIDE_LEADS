# airfoil_panel_method_convergence.py
#
# Created: May 2023, Niranjan Nanjappa 
 
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE Imports 
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Aerodynamics.Airfoil_Panel_Method import airfoil_analysis 
from RCAIDE.Library.Methods.Geometry.Airfoil                  import compute_naca_4series 
from RCAIDE.Library.Plots import * 

# Python imports
import os 
import numpy as np
import matplotlib.pyplot as plt    
 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():   
    single_airfoil()
    return

def single_airfoil():
    # -----------------------------------------------
    # Batch analysis of single airfoil - NACA 4412 
    # -----------------------------------------------
    AoA_deg              = np.linspace(-5,10,16)
    Re_vals              = np.atleast_2d(np.ones(len(AoA_deg)))*1E5 
    AoA_rad              = np.atleast_2d(AoA_deg*Units.degrees)   
    airfoil_file         = '4412'
    npoints1             = 101
    npoints2             = 201
    npoints3             = 301
    npoints4             = 401
    npoints5             = 501
    
    # npoints vector
    npoints = np.zeros(5)
    for i in range(5):
        npoints[i] = 101 + 100*i
    
    airfoil_geometry_1   = compute_naca_4series(airfoil_file,npoints1)
    airfoil_properties_1 = airfoil_analysis(airfoil_geometry_1,AoA_rad,Re_vals)
    
    airfoil_geometry_2   = compute_naca_4series(airfoil_file,npoints2)
    airfoil_properties_2 = airfoil_analysis(airfoil_geometry_2,AoA_rad,Re_vals)
    
    airfoil_geometry_3   = compute_naca_4series(airfoil_file,npoints3)
    airfoil_properties_3 = airfoil_analysis(airfoil_geometry_3,AoA_rad,Re_vals)
    
    airfoil_geometry_4   = compute_naca_4series(airfoil_file,npoints4)
    airfoil_properties_4 = airfoil_analysis(airfoil_geometry_4,AoA_rad,Re_vals)
    
    airfoil_geometry_5   = compute_naca_4series(airfoil_file,npoints5)
    airfoil_properties_5 = airfoil_analysis(airfoil_geometry_5,AoA_rad,Re_vals)  
    
    # ------------------------------------------------
    # Print Convergence
    # ------------------------------------------------
    
    # Convergence at 0 deg AOA
    print('\n\n\nCl inviscid at 0 deg AOA')
    print(npoints1,'panels :', airfoil_properties_1.cl_invisc[0,5])
    print(npoints2,'panels :', airfoil_properties_2.cl_invisc[0,5])
    print(npoints3,'panels :', airfoil_properties_3.cl_invisc[0,5])
    print(npoints4,'panels :', airfoil_properties_4.cl_invisc[0,5])
    print(npoints5,'panels :', airfoil_properties_5.cl_invisc[0,5])
    
    print('\nCd inviscid at 0 deg AOA')
    print(npoints1,'panels :', airfoil_properties_1.cd_invisc[0,5])
    print(npoints2,'panels :', airfoil_properties_2.cd_invisc[0,5])
    print(npoints3,'panels :', airfoil_properties_3.cd_invisc[0,5])
    print(npoints4,'panels :', airfoil_properties_4.cd_invisc[0,5])
    print(npoints5,'panels :', airfoil_properties_5.cd_invisc[0,5])
    
    print('\nCd viscous at 0 deg AOA')
    print(npoints1,'panels :', airfoil_properties_1.cd_visc[0,5])
    print(npoints2,'panels :', airfoil_properties_2.cd_visc[0,5])
    print(npoints3,'panels :', airfoil_properties_3.cd_visc[0,5])
    print(npoints4,'panels :', airfoil_properties_4.cd_visc[0,5])
    print(npoints5,'panels :', airfoil_properties_5.cd_visc[0,5])
    
    print('\nCm inviscid at 0 deg AOA')
    print(npoints1,'panels :', airfoil_properties_1.cm_invisc[0,5])
    print(npoints2,'panels :', airfoil_properties_2.cm_invisc[0,5])
    print(npoints3,'panels :', airfoil_properties_3.cm_invisc[0,5])
    print(npoints4,'panels :', airfoil_properties_4.cm_invisc[0,5])
    print(npoints5,'panels :', airfoil_properties_5.cm_invisc[0,5])
    
    
    # Convergence at 5 deg AOA
    print('\n\n\nCl inviscid at 5 deg AOA')
    print(npoints1,'panels :', airfoil_properties_1.cl_invisc[0,10])
    print(npoints2,'panels :', airfoil_properties_2.cl_invisc[0,10])
    print(npoints3,'panels :', airfoil_properties_3.cl_invisc[0,10])
    print(npoints4,'panels :', airfoil_properties_4.cl_invisc[0,10])
    print(npoints5,'panels :', airfoil_properties_5.cl_invisc[0,10])
    
    print('\nCd inviscid at 5 deg AOA')
    print(npoints1,'panels :', airfoil_properties_1.cd_invisc[0,10])
    print(npoints2,'panels :', airfoil_properties_2.cd_invisc[0,10])
    print(npoints3,'panels :', airfoil_properties_3.cd_invisc[0,10])
    print(npoints4,'panels :', airfoil_properties_4.cd_invisc[0,10])
    print(npoints5,'panels :', airfoil_properties_5.cd_invisc[0,10])
    
    print('\nCd viscous at 5 deg AOA')
    print(npoints1,'panels :', airfoil_properties_1.cd_visc[0,10])
    print(npoints2,'panels :', airfoil_properties_2.cd_visc[0,10])
    print(npoints3,'panels :', airfoil_properties_3.cd_visc[0,10])
    print(npoints4,'panels :', airfoil_properties_4.cd_visc[0,10])
    print(npoints5,'panels :', airfoil_properties_5.cd_visc[0,10])
    
    print('\nCm inviscid at 5 deg AOA')
    print(npoints1,'panels :', airfoil_properties_1.cm_invisc[0,10])
    print(npoints2,'panels :', airfoil_properties_2.cm_invisc[0,10])
    print(npoints3,'panels :', airfoil_properties_3.cm_invisc[0,10])
    print(npoints4,'panels :', airfoil_properties_4.cm_invisc[0,10])
    print(npoints5,'panels :', airfoil_properties_5.cm_invisc[0,10])
    
    
    # Convergence of net difference
    # Cl inviscid
    conv_diff_cl_invisc = np.zeros(4)
    conv_diff_cl_invisc[0] = np.average((airfoil_properties_2.cl_invisc - airfoil_properties_1.cl_invisc)/airfoil_properties_1.cl_invisc, axis=1)
    conv_diff_cl_invisc[1] = np.average((airfoil_properties_3.cl_invisc - airfoil_properties_2.cl_invisc)/airfoil_properties_1.cl_invisc, axis=1)
    conv_diff_cl_invisc[2] = np.average((airfoil_properties_4.cl_invisc - airfoil_properties_3.cl_invisc)/airfoil_properties_1.cl_invisc, axis=1)
    conv_diff_cl_invisc[3] = np.average((airfoil_properties_5.cl_invisc - airfoil_properties_4.cl_invisc)/airfoil_properties_1.cl_invisc, axis=1)
    
    # Cd inviscid
    conv_diff_cd_invisc = np.zeros(4)
    conv_diff_cd_invisc[0] = np.average((airfoil_properties_2.cd_invisc - airfoil_properties_1.cd_invisc)/airfoil_properties_1.cd_invisc, axis=1)
    conv_diff_cd_invisc[1] = np.average((airfoil_properties_3.cd_invisc - airfoil_properties_2.cd_invisc)/airfoil_properties_1.cd_invisc, axis=1)
    conv_diff_cd_invisc[2] = np.average((airfoil_properties_4.cd_invisc - airfoil_properties_3.cd_invisc)/airfoil_properties_1.cd_invisc, axis=1)
    conv_diff_cd_invisc[3] = np.average((airfoil_properties_5.cd_invisc - airfoil_properties_4.cd_invisc)/airfoil_properties_1.cd_invisc, axis=1)
    
    # Cd viscous
    conv_diff_cd_visc = np.zeros(4)
    conv_diff_cd_visc[0] = np.average((airfoil_properties_2.cd_visc - airfoil_properties_1.cd_visc)/airfoil_properties_1.cd_visc, axis=1)
    conv_diff_cd_visc[1] = np.average((airfoil_properties_3.cd_visc - airfoil_properties_2.cd_visc)/airfoil_properties_1.cd_visc, axis=1)
    conv_diff_cd_visc[2] = np.average((airfoil_properties_4.cd_visc - airfoil_properties_3.cd_visc)/airfoil_properties_1.cd_visc, axis=1)
    conv_diff_cd_visc[3] = np.average((airfoil_properties_5.cd_visc - airfoil_properties_4.cd_visc)/airfoil_properties_1.cd_visc, axis=1)
    
    # Cm inviscid
    conv_diff_cm_invisc = np.zeros(4)
    conv_diff_cm_invisc[0] = np.average((airfoil_properties_2.cm_invisc - airfoil_properties_1.cm_invisc)/airfoil_properties_1.cm_invisc, axis=1)
    conv_diff_cm_invisc[1] = np.average((airfoil_properties_3.cm_invisc - airfoil_properties_2.cm_invisc)/airfoil_properties_1.cm_invisc, axis=1)
    conv_diff_cm_invisc[2] = np.average((airfoil_properties_4.cm_invisc - airfoil_properties_3.cm_invisc)/airfoil_properties_1.cm_invisc, axis=1)
    conv_diff_cm_invisc[3] = np.average((airfoil_properties_5.cm_invisc - airfoil_properties_4.cm_invisc)/airfoil_properties_1.cm_invisc, axis=1)
    
    print('\n\n\nConvergence of Difference')
    print('\nCl inviscid')
    print(npoints2, '-', npoints1, 'panels :', conv_diff_cl_invisc[0])
    print(npoints3, '-', npoints2, 'panels :', conv_diff_cl_invisc[1])
    print(npoints4, '-', npoints3, 'panels :', conv_diff_cl_invisc[2])
    print(npoints5, '-', npoints4, 'panels :', conv_diff_cl_invisc[3])
    
    print('\nCd inviscid')
    print(npoints2, '-', npoints1, 'panels :', conv_diff_cd_invisc[0])
    print(npoints3, '-', npoints2, 'panels :', conv_diff_cd_invisc[1])
    print(npoints4, '-', npoints3, 'panels :', conv_diff_cd_invisc[2])
    print(npoints5, '-', npoints4, 'panels :', conv_diff_cd_invisc[3])
    
    print('\nCd viscous')
    print(npoints2, '-', npoints1, 'panels :', conv_diff_cd_visc[0])
    print(npoints3, '-', npoints2, 'panels :', conv_diff_cd_visc[1])
    print(npoints4, '-', npoints3, 'panels :', conv_diff_cd_visc[2])
    print(npoints5, '-', npoints4, 'panels :', conv_diff_cd_visc[3])
    
    print('\nCm inviscid')
    print(npoints2, '-', npoints1, 'panels :', conv_diff_cm_invisc[0])
    print(npoints3, '-', npoints2, 'panels :', conv_diff_cm_invisc[1])
    print(npoints4, '-', npoints3, 'panels :', conv_diff_cm_invisc[2])
    print(npoints5, '-', npoints4, 'panels :', conv_diff_cm_invisc[3]) 
    
    
    # ------------------------------------------------
    # Plot Convergence
    # ------------------------------------------------
    
    # Convergence at 0 deg AOA
    cl_invisc_0_plt = np.zeros(5)
    cl_invisc_0_plt[0] = airfoil_properties_1.cl_invisc[0,5]
    cl_invisc_0_plt[1] = airfoil_properties_2.cl_invisc[0,5]
    cl_invisc_0_plt[2] = airfoil_properties_3.cl_invisc[0,5]
    cl_invisc_0_plt[3] = airfoil_properties_4.cl_invisc[0,5]
    cl_invisc_0_plt[4] = airfoil_properties_5.cl_invisc[0,5]
    
    cd_invisc_0_plt = np.zeros(5)
    cd_invisc_0_plt[0] = airfoil_properties_1.cd_invisc[0,5]
    cd_invisc_0_plt[1] = airfoil_properties_2.cd_invisc[0,5]
    cd_invisc_0_plt[2] = airfoil_properties_3.cd_invisc[0,5]
    cd_invisc_0_plt[3] = airfoil_properties_4.cd_invisc[0,5]
    cd_invisc_0_plt[4] = airfoil_properties_5.cd_invisc[0,5]
    
    cd_visc_0_plt = np.zeros(5)
    cd_visc_0_plt[0] = airfoil_properties_1.cd_visc[0,5]
    cd_visc_0_plt[1] = airfoil_properties_2.cd_visc[0,5]
    cd_visc_0_plt[2] = airfoil_properties_3.cd_visc[0,5]
    cd_visc_0_plt[3] = airfoil_properties_4.cd_visc[0,5]
    cd_visc_0_plt[4] = airfoil_properties_5.cd_visc[0,5]
    
    cm_invisc_0_plt = np.zeros(5)
    cm_invisc_0_plt[0] = airfoil_properties_1.cm_invisc[0,5]
    cm_invisc_0_plt[1] = airfoil_properties_2.cm_invisc[0,5]
    cm_invisc_0_plt[2] = airfoil_properties_3.cm_invisc[0,5]
    cm_invisc_0_plt[3] = airfoil_properties_4.cm_invisc[0,5]
    cm_invisc_0_plt[4] = airfoil_properties_5.cm_invisc[0,5]
    
    fig0  = plt.figure('AOA 0 degrees convergence') 
    fig0.set_size_inches(11,7)
    fig0.suptitle('0 degrees AOA')
    axis1  = fig0.add_subplot(2,2,1)  
    axis2  = fig0.add_subplot(2,2,2) 
    axis3  = fig0.add_subplot(2,2,3) 
    axis4  = fig0.add_subplot(2,2,4) 
    axis1.set_ylabel(r'CL inviscid')           
    axis2.set_ylabel(r'CD inviscid')         
    axis3.set_ylabel(r'CD viscous')         
    axis4.set_ylabel(r'CM inviscid')
    axis1.plot(npoints, cl_invisc_0_plt)  
    axis2.plot(npoints, cd_invisc_0_plt)
    axis3.plot(npoints, cd_visc_0_plt)
    axis4.plot(npoints, cm_invisc_0_plt)

    
    # Convergence at 5 deg AOA
    cl_invisc_5_plt = np.zeros(5)
    cl_invisc_5_plt[0] = airfoil_properties_1.cl_invisc[0,10]
    cl_invisc_5_plt[1] = airfoil_properties_2.cl_invisc[0,10]
    cl_invisc_5_plt[2] = airfoil_properties_3.cl_invisc[0,10]
    cl_invisc_5_plt[3] = airfoil_properties_4.cl_invisc[0,10]
    cl_invisc_5_plt[4] = airfoil_properties_5.cl_invisc[0,10]
    
    cd_invisc_5_plt = np.zeros(5)
    cd_invisc_5_plt[0] = airfoil_properties_1.cd_invisc[0,10]
    cd_invisc_5_plt[1] = airfoil_properties_2.cd_invisc[0,10]
    cd_invisc_5_plt[2] = airfoil_properties_3.cd_invisc[0,10]
    cd_invisc_5_plt[3] = airfoil_properties_4.cd_invisc[0,10]
    cd_invisc_5_plt[4] = airfoil_properties_5.cd_invisc[0,10]
    
    cd_visc_5_plt = np.zeros(5)
    cd_visc_5_plt[0] = airfoil_properties_1.cd_visc[0,10]
    cd_visc_5_plt[1] = airfoil_properties_2.cd_visc[0,10]
    cd_visc_5_plt[2] = airfoil_properties_3.cd_visc[0,10]
    cd_visc_5_plt[3] = airfoil_properties_4.cd_visc[0,10]
    cd_visc_5_plt[4] = airfoil_properties_5.cd_visc[0,10]
    
    cm_invisc_5_plt = np.zeros(5)
    cm_invisc_5_plt[0] = airfoil_properties_1.cm_invisc[0,10]
    cm_invisc_5_plt[1] = airfoil_properties_2.cm_invisc[0,10]
    cm_invisc_5_plt[2] = airfoil_properties_3.cm_invisc[0,10]
    cm_invisc_5_plt[3] = airfoil_properties_4.cm_invisc[0,10]
    cm_invisc_5_plt[4] = airfoil_properties_5.cm_invisc[0,10]
    
    fig5 = plt.figure('AOA 5 degrees convergence') 
    fig5.set_size_inches(11,7)
    fig5.suptitle('5 degrees AOA')
    axis1  = fig5.add_subplot(2,2,1)  
    axis2  = fig5.add_subplot(2,2,2) 
    axis3  = fig5.add_subplot(2,2,3) 
    axis4  = fig5.add_subplot(2,2,4) 
    axis1.set_ylabel(r'CL inviscid')           
    axis2.set_ylabel(r'CD inviscid')         
    axis3.set_ylabel(r'CD viscous')         
    axis4.set_ylabel(r'CM inviscid')
    axis1.plot(npoints, cl_invisc_5_plt)  
    axis2.plot(npoints, cd_invisc_5_plt)
    axis3.plot(npoints, cd_visc_5_plt)
    axis4.plot(npoints, cm_invisc_5_plt)
    
    # Convergence of differences
    nums = np.zeros(4)
    for i in range(4):
        nums[i] = 1 + i
    
    fig_diff = plt.figure('Difference Convergence')
    fig_diff.set_size_inches(11,7)
    fig_diff.suptitle('Convergence of Differences')
    axis1  = fig_diff.add_subplot(2,2,1)  
    axis2  = fig_diff.add_subplot(2,2,2) 
    axis3  = fig_diff.add_subplot(2,2,3) 
    axis4  = fig_diff.add_subplot(2,2,4) 
    axis1.set_ylabel(r'CL inviscid')           
    axis2.set_ylabel(r'CD inviscid')         
    axis3.set_ylabel(r'CD viscous')         
    axis4.set_ylabel(r'CM inviscid')
    axis1.plot(nums, conv_diff_cl_invisc)  
    axis2.plot(nums, conv_diff_cd_invisc)  
    axis3.plot(nums, conv_diff_cd_visc)  
    axis4.plot(nums, conv_diff_cm_invisc)  

    return
        
    
    


if __name__ == '__main__': 
    main() 
    plt.show()