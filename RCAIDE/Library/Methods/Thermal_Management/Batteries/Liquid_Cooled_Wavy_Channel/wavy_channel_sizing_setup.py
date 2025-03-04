# RCAIDE/Library/Methods/Thermal_Management/Batteries/Liquid_Cooled_Wavy_Channel/wavy_channel_sizing_setup.py


# Created: Apr 2024 S. Shekar 
#
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE Imports
import RCAIDE
from RCAIDE.Framework.Analyses.Process   import Process  

# Python package imports  
import numpy as np  

# ----------------------------------------------------------------------------------------------------------------------
#  Sizing Setup for Wavy Channel
# ----------------------------------------------------------------------------------------------------------------------
def wavy_channel_sizing_setup():
    """ Creates a process to modify and analyze the heat acquistion system
          
          Inputs:  
             None
             
          Outputs:   
             procedure - optimization methodology                                       
              
          Assumptions: 
             N/A 
        
          Source:
             None
    """            

    # size the base config
    procedure = Process()

    # modify battery thermal management system
    procedure.modify_HAS = modify_wavy_channel_HAS

    # post process the results
    procedure.post_process = post_process

    return procedure


def modify_wavy_channel_HAS(nexus): 
    """ Modifies geometry of Wavy Channel  
          
          Inputs:  
             nexus     - RCAIDE optmization framework with Wavy Channel geometry data structure [None]
              
          Outputs:   
             procedure - optimization methodology                                       
              
          Assumptions: 
             N/A 
        
          Source:
             None
    """        
    battery_list  = list(nexus.hrs_configurations.optimized.networks.electric.busses.bus.battery_modules.keys())
    battery       = nexus.hrs_configurations.optimized.networks.electric.busses.bus.battery_modules[battery_list[0]]
    has_opt       = nexus.hrs_configurations.optimized.networks.electric.coolant_lines.coolant_line.battery_modules[battery.tag].thermal_management_system.heat_acquisition_system

    # ------------------------------------------------------------------------------------------------------------------------
    # Unpack paramters  
    # ------------------------------------------------------------------------------------------------------------------------
    
    # Coolant 
    m_coolant = has_opt.coolant_flow_rate 
    coolant   = has_opt.coolant
    T_i       = has_opt.coolant_inlet_temperature  
    
    # Battery 
    d_cell    = battery.cell.diameter                    
    h_cell    = battery.cell.height                      
    A_cell    = np.pi*d_cell*h_cell    
    T_bat     = has_opt.design_battery_operating_temperature
    Q_module  = has_opt.design_heat_removed 
    
    # Channel 
    channel          = has_opt.channel
    b                = has_opt.channel_side_thickness                
    d                = has_opt.channel_width                         
    theta            = has_opt.channel_contact_angle    
    c                = battery.cell.height 
    a                = has_opt.channel_top_thickness    
    AR               = d/c    
    channel_density  = channel.density
    k_chan           = channel.thermal_conductivity  

    # Pump
    n_pump           = 0.7 # Replce with Pump Class


    # Considering a lumped mass model  
    N_cells = battery.electrical_configuration.parallel*battery.electrical_configuration.series
    
    # Contact Surface area of the channel 
    A_chan = 2*N_cells*(theta)*A_cell 
    
    # Update Spacing of the battery pack
    new_normal_spacing  = 2 * (d_cell + d) * np.sin(theta/2)
    new_parllel_spacing  = (d_cell + d) * np.cos(theta/2)
    
    #Length of Channel   
    L_chan  = (battery.geometrtic_configuration.normal_count*new_normal_spacing)*battery.geometrtic_configuration.parallel_count
    L_extra  = 3*d_cell
    L_chan   = (N_cells*d_cell)+L_extra 

    # Hydraulic diameter    
    dh   = (4*c*d)/(2*(c+d))   

    # Thermophysical Properties of Coolant 
    rho  = coolant.compute_density(T_i)
    mu   = coolant.compute_absolute_viscosity(T_i)  
    cp   = coolant.compute_cp(T_i)
    Pr   = coolant.compute_prandtl_number(T_i) 
    k    = coolant.compute_thermal_conductivity(T_i)   

    #calculate the velocity of the fluid in the channel 
    v=rho*c*d*m_coolant

    # calculate the Reynolds Number 
    Re=(rho*dh*v)/mu

    # fanning friction factor (eq 32),  Nusselt Number (eq 12)   
    if Re< 2300:
        f= 24*(1-(1.3553*AR)+(1.9467*(AR**2))-(1.7012*(AR**3))+(0.9564*(AR**4))-(0.2537*(AR**5)))/Re
        Nu = 8.235*(1-(2.0421*AR)+(3.0853*(AR**2))-(2.4765*(AR**3))+(1.0578*(AR**4))-(0.1861*(AR**5)))   
    elif Re>=2300:
        f= (0.0791*(Re**(-0.25)))*(1.8075-0.1125*AR)
        Nu = ((f/2)*(Re-1000)*Pr)/(1+(12.7*((f/2)**0.5)*(Pr**(2/3)-1)))   

    # Calculate the pressure drop in the channel 
    dp     = 2*f*rho*v*v*L_chan/dh	

    # Calculate the Power consumed
    Power   = RCAIDE.Library.Components.Thermal_Management.Accessories.Pump.compute_power_consumed(dp, rho, m_coolant, n_pump) 

    # Mass calculations - Channel  
    rho_line        = channel_density*(2*a*((2*b)+d)+(2*b*c))
    mass_channel    = rho_line*L_chan

    # Mass calculations - Liquid 
    mass_liquid  = rho*c*d 
    total_mass   = mass_channel+mass_liquid 

    # heat transfer coefficient of the channeled coolant (eq 11)
    h = k*Nu/dh

    # Overall Heat Transfer Coeffipackcient from battery surface to the coolant fluid (eq 10)
    U_total = 1/((1/h)+(b/k_chan))

    # Calculate NTU
    NTU = U_total*A_chan/(m_coolant*cp)
    
    # Effectiveness of the Channel 
    eff_HAS = 1 - np.exp(-NTU)

    # Calculate Outlet Temparture To ( eq 8)
    T_o = ((T_bat-T_i)*(1-np.exp(-NTU)))+T_i

    # Calculate the Log mean temperature 
    T_lm = ((T_bat-T_i)-(T_bat-T_o))/(np.log((T_bat-T_i)/(T_bat-T_o)))

    # Calculated Heat Convected 
    Q_convec = U_total*A_chan*T_lm*eff_HAS   

    # line level properties for optmization 
    has_opt.mass_properties.mass       = total_mass
    has_opt.design_power_draw          = Power
    has_opt.heat_removed               = Q_convec 
    has_opt.heat_generated             = Q_module 
    has_opt.coolant_outlet_temperature = T_o
    has_opt.coolant_pressure_drop      = dp  
    has_opt.surface_area_channel       = A_chan
    has_opt.heat_transfer_efficiency   = eff_HAS
    has_opt.battery_parllel_spacing     = new_parllel_spacing
    has_opt.battery_series_spacing      = new_normal_spacing

    return nexus     
# ----------------------------------------------------------------------
#   Post Process Results to give back to the optimizer
# ----------------------------------------------------------------------   
def post_process(nexus):
    """ Post processes the outputs obtained from the optimizer  
          
          Inputs:  
             nexus     - RCAIDE optmization framework with Wavy Channel geometry data structure [None]
              
          Outputs:   
             nexus     - Objective Function calculation 
              
              
          Assumptions: 
            Factors were applied to mass and power to scale it for the optimizer in lines 218, 219.
        
          Source:
             None
    """            
    
    summary              = nexus.summary  
    battery_list         = list(nexus.hrs_configurations.optimized.networks.electric.coolant_lines.coolant_line.battery_modules.keys())
    battery              = nexus.hrs_configurations.optimized.networks.electric.coolant_lines.coolant_line.battery_modules[battery_list[0]]
    has_opt              = battery.thermal_management_system.heat_acquisition_system 
    Q_line_rem           = has_opt.heat_removed
    Q_line_gen           = has_opt.heat_generated  
    new_normal_spacing   = has_opt.battery_series_spacing   
    new_parallel_spacing = has_opt.battery_parllel_spacing
    channel_width        = has_opt.channel_width        
    channel_thickness    = has_opt.channel_side_thickness 

    # calculate objective 
    spacing                      = new_normal_spacing + new_parallel_spacing
    spacing                      = spacing*100
    total_mass                   = has_opt.mass_properties.mass/100
    Power                        = has_opt.design_power_draw * 10
    summary.mass_power_objective =  (spacing**2+Power**2 + total_mass**2)**(0.5)  

    # calculate heat constraint  
    summary.heat_energy_constraint  = Q_line_rem - Q_line_gen  
    summary.thickness_constraint    = channel_width-(2*channel_thickness)

    return nexus     