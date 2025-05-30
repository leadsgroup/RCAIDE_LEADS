# RCAIDE/Methods/Noise/Correlation_Buildup/Engine/jet_installation_effect.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
 
# Python package imports   
import numpy as np   

# ----------------------------------------------------------------------------------------------------------------------     
#  Jet Installation Effect
# ----------------------------------------------------------------------------------------------------------------------      
def jet_installation_effect(Xe,Ye,Ce,theta_s,Diameter_mixed):
    """This calculates the installation effect, in decibels, to be added to the predicted secondary jet noise level.
    
    Assumptions:
        N/A

    Source:
        SAE ARP876D: Gas Turbine Jet Exhaust Noise Prediction

    Inputs:
        Ce = wing chord length at the engine location - as figure 7.3 of the SAE ARP 876D                    [m]                          
        Xe = fan exit location downstream of the leading edge (Xe<Ce) - as figure 7.3 of the SAE ARP 876D    [m] 
        Ye = separation distance from the wing chord line to nozzle lip - as figure 7.3 of the SAE ARP 876D  [m] 
        theta_s                                                                                              [rad]
        Diameter_mixed                                                                                       [m] 

    Outputs:
        INST_s          [-]

    Properties Used: 
        N/A 
    """ 
    # Instalation effect
    INST_s = np.array([0.5*((Ce-Xe)**2/(Ce*Diameter_mixed))*(np.exp(-Ye/Diameter_mixed)*((1.8*theta_s/np.pi))-0.6)**2])  
    INST_s[INST_s>2.5]=2.5 

    return INST_s