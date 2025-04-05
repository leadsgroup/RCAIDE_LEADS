# RCAIDE/Library/Methods/Weights/Correlation_Buildups/Propulsion/compute_jet_engine_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE 
import RCAIDE
from RCAIDE.Framework.Core import  Units, Data
import numpy as np
 

def compute_propulsion_system_weight(network):
    W_energy_network_total = 0
    number_of_jet_engines = 0
    number_of_piston_engines = 0
    for propulsor in network.propulsors: # Check this 
            if isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan):
                number_of_jet_engines += 1
                W_engine_jet            = compute_jet_engine_weight(propulsor)
                W_propulsion            = integrated_propulsion_jet(W_engine_jet) 
                propulsor.mass_properties.mass = W_propulsion
                W_energy_network_total  += W_propulsion                
            elif isinstance(propulsor, RCAIDE.Library.Components.Powertrain.Propulsors.Internal_Combustion_Engine):    
                number_of_piston_engines += 1
                W_engine_piston          = compute_piston_engine_weight(propulsor)
                W_propulsion             = integrated_propulsion_piston(W_engine_piston) 
                propulsor.mass_properties.mass = W_propulsion
                W_energy_network_total  += W_propulsion
            elif type(propulsor) ==  RCAIDE.Library.Components.Powertrain.Propulsors.Turboprop:      
                number_of_piston_engines += 1
                W_turboprop          = compute_turboprop_engine_weight(propulsor)
                W_propulsion             = integrated_propulsion_piston(W_turboprop) 
                propulsor.mass_properties.mass = W_propulsion
                W_energy_network_total  += W_propulsion     
    
    output = Data()
    output.W_prop = W_propulsion
    output.number_of_engines = number_of_jet_engines + number_of_piston_engines
    return output   
                

# ----------------------------------------------------------------------------------------------------------------------
#  Jet Engine Weight 
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_jet_engine_weight(propulsor):
    """ Calculate the weight of the dry engine  
    
    Assumptions:
            calculated engine weight from a correlation of engines
    
    Source: 
            N/A
            
    Inputs:
            thrust_sls - sea level static thrust of a single engine [Newtons]
    
    Outputs:
            weight - weight of the dry engine                       [kilograms]
        
    Properties Used:
            N/A
    """     
    # setup
    thrust_sls    =  propulsor.sealevel_static_thrust 
    thrust_sls_en = thrust_sls / Units.force_pound # Convert N to lbs force  
    BPR = propulsor.bypass_ratio

    WENG   = 0.084 *  (propulsor.sealevel_static_thrust/Units.lbf)**1.1 * np.exp(-0.045*BPR) * Units.lbs # Raymer 3rd Edition eq. 10.4 
    
    return WENG
 
# ----------------------------------------------------------------------------------------------------------------------
#  Motor Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_motor_weight(motor):
    """ Calculate the weight of motor using NASA correlations 
             
    
    Inputs:
            torque- maximum torque the motor can deliver safely      [N-m]
            kwt2
            xwt
            
    Outputs:
            mass- mass of the motor                                [kilograms]
        
    Properties Used:
            N/A
    """
    
    torque =  motor.design_torque
    mass  = -2E-7 * (torque ** 2) +  0.0117 * torque +  34.124
     
    return mass 

# ----------------------------------------------------------------------------------------------------------------------
#  Piston Engine Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_piston_engine_weight(propulsor):
    """ Calculate the weight of an piston engine  
        weight correlation; weight=kwt2*(max_power**xwt)
        Inputs:
                max_power- maximum power the motor can deliver safely [Watts]
                kwt2
                xwt
                
        Outputs:
                weight- weight of the motor [kilograms]
        
        Source: Raymer, Aircraft Design, a Conceptual Approach         
    """    
    bhp    = propulsor.engine.sea_level_power/Units.horsepower
    weight = 5.22*((bhp)**0.780)  # weight in lbs.
    mass   = weight*Units.lbs
    return mass

# ----------------------------------------------------------------------------------------------------------------------
#  Turboprop Engine Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_turboprop_engine_weight(propulsor):
    """ Calculate the weight of an turboprop engine  
        weight correlation; weight=kwt2*(max_power**xwt)
        Inputs:
                max_power- maximum power the motor can deliver safely [Watts]
                kwt2
                xwt
                
        Outputs:
                weight- weight of the motor [kilograms]
        
        Source: Raymer, Aircraft Design, a Conceptual Approach         
    """    
    bhp    = propulsor.design_power / Units.horsepower
    weight = 1.67*((bhp)**0.803)  # weight in lbs.
    mass   = weight*Units.lbs
    return mass

# ----------------------------------------------------------------------------------------------------------------------
#  Integrated Propulsion Weight 
# ----------------------------------------------------------------------------------------------------------------------
def integrated_propulsion_jet(jet_engine_weight, engine_W_factor = 1.6):
    """ Calculate the weight of the entire propulsion system 
    
    Assumptions:
            The propulsion system is a fixed 60% greater than the dry engine alone. 
            The propulsion system includes the engines, engine exhaust, reverser, starting,
            controls, lubricating, and fuel systems. The nacelle and pylon weight are also
            part of this calculation.           
            
    Source: 
            N/A
            
    Inputs:
            compute_jet_engine_weight - dry weight of the engine                                             [kilograms]
            num_eng - total number of engines on the aircraft                                 [dimensionless]
            engine_W_factor - weight increase factor for entire integrated propulsion system [dimensionless]
    
    Outputs:
            weight - weight of the full propulsion system                                     [kilograms]
        
    Properties Used:
            N/A
    """   
    
    weight = jet_engine_weight * engine_W_factor
    
    return weight
    
# ----------------------------------------------------------------------------------------------------------------------
#  Integrated Propulsion Weight 
# ----------------------------------------------------------------------------------------------------------------------
def integrated_propulsion_piston(engine_weight):
    """ 
        Calculate the weight of the entire propulsion system        

        Source:
                Source: Raymer, Aircraft Design, a Conceptual Approach        
                
        Inputs:
                piston_engine_weight - dry weight of a single engine                                     [kilograms]
                num_eng - total number of engines on the aircraft                                 [dimensionless]
                engine_W_factor - weight increase factor for entire integrated propulsion system [dimensionless]
        
        Outputs:
                weight - weight of the full propulsion system [kilograms]
    """     
    engine_dry = engine_weight/Units.lbs
    weight     = 2.575 * (engine_dry**0.922)
    mass       = weight*Units.lbs #convert to kg

    return mass