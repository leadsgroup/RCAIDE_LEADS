# test_take_off_field_length.py
#
# Created: Dec 2024, M Clarke   

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# SUave Imports
import RCAIDE
from RCAIDE.Framework.Core   import Data,Units 
from RCAIDE.Library.Methods.Performance.estimate_take_off_field_length import estimate_take_off_field_length

# package imports
import numpy as np
import pylab as plt 
import sys
import os
import numpy as np
from  copy import  deepcopy

# import vehicle file
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Embraer_190 import vehicle_setup, configs_setup

# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def main():

    # ----------------------------------------------------------------------
    #   Main
    # ----------------------------------------------------------------------    
    vehicle = vehicle_setup()
    configs = configs_setup(vehicle)
    
    # --- Takeoff Configuration ---
    configuration = configs.takeoff
    configuration.wings['main_wing'].flaps_angle =  20. * Units.deg
    configuration.wings['main_wing'].slats_angle  = 25. * Units.deg
    
    # V2_V2_ratio may be informed by user. If not, use default value (1.2)
    configuration.V2_VS_ratio = 1.21
    analyses = RCAIDE.Framework.Analyses.Analysis.Container()
    analyses = base_analysis(vehicle)
    analyses.aerodynamics.settings.maximum_lift_coefficient_factor = 0.90

    # CLmax for a given configuration may be informed by user
    # configuration.maximum_lift_coefficient = 2.XX 
    w_vec                = np.linspace(40000.,52000.,10)
    engines              = (2,3,4)
    takeoff_field_length = np.zeros((len(w_vec),len(engines)))
    second_seg_clb_grad  = np.zeros((len(w_vec),len(engines)))
    
    compute_clb_grad = 1 # flag for Second segment climb estimation
    
    for network in  configuration.networks:
        for propulsor in  network.propulsors: 
            baseline_propulsor = deepcopy(propulsor)
            
            # delete propulsor 
            del network.propulsors[propulsor.tag] 

        for fuel_line in  network.fuel_lines: 
            fuel_line.assigned_propulsors = []           
    
    for id_eng,engine_number in enumerate(engines):
        propulsor_list = []
        # append propulsors 
        for network in  configuration.networks:
            for i in  range(engine_number):
                propulsor =  deepcopy(baseline_propulsor)
                propulsor.tag = 'propulsor_' +  str(i+1)
                network.propulsors.append(propulsor)
                propulsor_list.append(propulsor.tag) 
                
            for fuel_line in  network.fuel_lines:  
                fuel_line.assigned_propulsors =  [propulsor_list]
                
        for id_w,weight in enumerate(w_vec):
            configuration.mass_properties.takeoff = weight
            takeoff_field_length[id_w,id_eng],second_seg_clb_grad[id_w,id_eng] =  estimate_take_off_field_length(configuration,analyses,compute_2nd_seg_climb = True)
    
        # delete propulsors again 
        for propulsor in  network.propulsors: 
            baseline_propulsor = deepcopy(propulsor) 
            del network.propulsors[propulsor.tag] 
    
        for fuel_line in  network.fuel_lines: 
            fuel_line.assigned_propulsors = []                       
        
    truth_TOFL =  np.array([[ 794.00351564,  532.99654905,  387.20956395],
                            [ 831.53328381,  555.74516789,  403.57880791],
                            [ 870.48366427,  579.29577615,  420.50868415],
                            [ 910.87305651,  603.65380605,  438.00150906],
                            [ 952.72044414,  628.82486228,  456.05967252],
                            [ 996.04539486,  654.81472196,  474.68563797],
                            [1040.86806051,  681.62933469,  493.88194238],
                            [1087.20917702,  709.2748225 ,  513.65119627],
                            [1135.09006444,  737.75747988,  533.99608366],
                            [1184.53262692,  767.08377378,  554.91936214]])
    
    print(' takeoff_field_length = ',  takeoff_field_length)
    print(' second_seg_clb_grad  = ', second_seg_clb_grad)                      
                             
    truth_clb_grad =  np.array([[0.27638379, 0.63708282, 0.99902958],
                                [0.26479683, 0.61392533, 0.96422233],
                                [0.25392382, 0.59219893, 0.93157065],
                                [0.2437006 , 0.57177482, 0.90088018],
                                [0.23407048, 0.55253917, 0.87197923],
                                [0.22498318, 0.53439108, 0.8447156 ],
                                [0.21639395, 0.51724072, 0.81895389],
                                [0.20826284, 0.50100793, 0.79457321],
                                [0.20055404, 0.48562089, 0.7714653 ],
                                [0.19323541, 0.47101509, 0.74953296]])


    TOFL_error = np.max(np.abs(truth_TOFL-takeoff_field_length)/truth_TOFL)                           
    GRAD_error = np.max(np.abs(truth_clb_grad-second_seg_clb_grad)/truth_clb_grad)
    
    print('Maximum Take OFF Field Length Error= %.4e' % TOFL_error)
    print('Second Segment Climb Gradient Error= %.4e' % GRAD_error)    
    
    import pylab as plt
    title = "TOFL vs W"
    plt.figure(1); 
    plt.plot(w_vec,takeoff_field_length[:,0], 'k-', label = '2 Engines')
    plt.plot(w_vec,takeoff_field_length[:,1], 'r-', label = '3 Engines')
    plt.plot(w_vec,takeoff_field_length[:,2], 'b-', label = '4 Engines')

    plt.title(title); plt.grid(True)
    plt.plot(w_vec,truth_TOFL[:,0], 'k--o', label = '2 Engines [truth]')
    plt.plot(w_vec,truth_TOFL[:,1], 'r--o', label = '3 Engines [truth]')
    plt.plot(w_vec,truth_TOFL[:,2], 'b--o', label = '4 Engines [truth]')
    legend = plt.legend(loc='lower right')
    plt.xlabel('Weight (kg)')
    plt.ylabel('Takeoff field length (m)')    
    
    title = "2nd Segment Climb Gradient vs W"
    plt.figure(2); 
    plt.plot(w_vec,second_seg_clb_grad[:,0], 'k-', label = '2 Engines')
    plt.plot(w_vec,second_seg_clb_grad[:,1], 'r-', label = '3 Engines')
    plt.plot(w_vec,second_seg_clb_grad[:,2], 'b-', label = '4 Engines')

    plt.title(title); plt.grid(True)
    plt.plot(w_vec,truth_clb_grad[:,0], 'k--o', label = '2 Engines [truth]')
    plt.plot(w_vec,truth_clb_grad[:,1], 'r--o', label = '3 Engines [truth]')
    plt.plot(w_vec,truth_clb_grad[:,2], 'b--o', label = '4 Engines [truth]')
    legend = plt.legend(loc='lower right')
    plt.xlabel('Weight (kg)')
    plt.ylabel('Second Segment Climb Gradient (%)')    
    
    assert( TOFL_error   < 1e-6 )
    assert( GRAD_error   < 1e-6 )

    return 
    
def base_analysis(vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()  
   
    #  Aerodynamics Analysis
    aerodynamics         = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.vehicle = vehicle 
    analyses.append(aerodynamics)
    
    # ------------------------------------------------------------------
    #  Energy Analysis
    energy         = RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle = vehicle 
    analyses.append(energy)
    
    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)    
    
    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)     
    
    # done!
    return analyses     

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()
    plt.show()