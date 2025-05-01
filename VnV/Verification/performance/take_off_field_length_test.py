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
from RCAIDE.Library.Methods.Geometry.Planform import wing_planform

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
    for wing in vehicle.wings: 
         wing_planform(wing,overwrite_reference =  True) 
         if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
            vehicle.reference_area = wing.areas.reference
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
        
    truth_TOFL =  np.array([[ 803.25375954,  538.60880784,  391.24943957],
                            [ 841.45864363,  561.75203029,  407.89855969],
                            [ 881.11269222,  585.7120746 ,  425.11829625],
                            [ 922.23492584,  610.49455634,  442.9110438 ],
                            [ 964.84496881,  636.10526937,  461.27927291],
                            [1008.96304932,  662.55018582,  480.22553016],
                            [1054.60999936,  689.83545608,  499.75243815],
                            [1101.80725475,  717.96740883,  519.8626955 ],
                            [1150.57685514,  746.95255101,  540.55907684],
                            [1200.941444  ,  776.79756783,  561.84443282]])
    
    print(' takeoff_field_length = ',  takeoff_field_length)
    print(' second_seg_clb_grad  = ', second_seg_clb_grad)                      
                             
    truth_clb_grad =  np.array([[0.27737533, 0.63810888, 1.00006962],
                                [0.26578696, 0.61494665, 0.96525562],
                                [0.25491284, 0.59321615, 0.932598  ],
                                [0.24468879, 0.57278846, 0.90190228],
                                [0.23505808, 0.55354972, 0.87299671],
                                [0.2259704 , 0.53539894, 0.84572902],
                                [0.21738097, 0.51824628, 0.81996372],
                                [0.20924981, 0.50201151, 0.79557988],
                                [0.2015411 , 0.48662277, 0.7724692 ],
                                [0.19422268, 0.47201553, 0.75053441]])


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