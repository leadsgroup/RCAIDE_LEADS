# RCAIDE/Methods/Energy/Thermal_Management/Batteries/Heat_Acquisition_Systems/wavy_channel_rating_model.py
# 
# 
# Created:  Apr 2024, M. Clarke, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import Data   

# python package imports 
import numpy as np  

# ----------------------------------------------------------------------------------------------------------------------
#  Wavy Channel Rating Model
# ----------------------------------------------------------------------------------------------------------------------
def  wavy_channel_rating_model(HAS,battery,bus,coolant_line,Q_heat_gen,T_cell,state,delta_t,t_idx):
    """ Computes the net heat removed by a wavy channel heat acquisition system.

    Assumptions:
    1) Battery pack cell heat transfer can be modelled as a cooling columns in a cross-flow
    2) Isothermal battery cell - the temperature at the center of the cell is the same at 
    the surface of the cell

    Source: 
    Zhao, C., Clarke, M., Kellermann H., Verstraete D.,
    “Design of a Liquid Cooling System for Lithium-Ion Battery Packs for eVTOL Aircraft" 

    Inputs:  
              T_current                 (pack temperature)           [Kelvin]
              T_cell                    (battery cell temperature)   [Kelvin] 
              heat_transfer_efficiency                               [unitless]
          HAS.
              channel_side_thickness                                 [meter]
              channel_width                                          [meter]
              channel_contact_angle                                  [degree]
              channel_top_thickness                                  [meter]
              channel                   (Properties of channel)     [unitless]
              coolant                   (Properties of coolant)    [unitless]
              coolant_flow_rate                                       [kg/s]
              
              
      
      Outputs:
             T_cell                     (Updated battery cell temperature) [Kelvin]
  
    Properties Used:
    None 
    """        
 
    # Inlet Properties from mission solver.
    for reservoir in  coolant_line.reservoirs:
        T_inlet = state.conditions.energy[coolant_line.tag][reservoir.tag].coolant_temperature[t_idx, 0]  
    #turndown_ratio           = battery_conditions.thermal_management_system.HAS.percent_operation[t_idx,0] 
    T_cell                   = state.conditions.energy[bus.tag].battery_modules[battery.tag].cell.temperature[t_idx, 0]  
    heat_transfer_efficiency = HAS.heat_transfer_efficiency   

    # Coolant Properties
    opt_coolant                 = compute_coolant_properties(HAS,T_inlet,state,delta_t,t_idx)
    m_coolant                   = opt_coolant.flowrate#*turndown_ratio 
    rho                         = opt_coolant.inlet_density    
    mu                          = opt_coolant.inlet_visc       
    cp                          = opt_coolant.inlet_Cp         
    Pr                          = opt_coolant.inlet_Pr         
    k                           = opt_coolant.inlet_thermal_cond
    
    # Battery Properties
    d_cell                      = battery.cell.diameter                    
    h_cell                      = battery.cell.height                      
    A_cell                      = np.pi*d_cell*h_cell 
    N_cells_geometric_config    = battery.geometrtic_configuration.parallel_count*battery.geometrtic_configuration.normal_count  
    cell_mass                   = battery.cell.mass 
    Nn_module_cells             = battery.electrical_configuration.series            
    Np_module_cells             = battery.electrical_configuration.parallel
    number_of_cells_in_module   = Nn_module_cells*Np_module_cells   
    Q_module                    = Q_heat_gen*number_of_cells_in_module
    Cp_bat                      = battery.cell.specific_heat_capacity
    
    
    # Channel Properties
    b         = HAS.channel_side_thickness                
    d         = HAS.channel_width                         
    theta     = HAS.channel_contact_angle    
    c         = battery.cell.height  
    channel   = HAS.channel
    AR        = d/c    
    k_chan    = channel.thermal_conductivity    
    n_pump    = 0.7 
    
    # Contact Surface area of the channel 
    A_chan   = 2*N_cells_geometric_config*(theta)*A_cell  

    #Length of Channel   
    L_extra  =  battery.geometrtic_configuration.parallel_count*d_cell
    L_chan   = (N_cells_geometric_config*d_cell)+L_extra 

    # Hydraulic diameter    
    dh   = (4*c*d)/(2*(c+d))   
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
    
    # heat transfer coefficient of the channeled coolant (eq 11)
    h = k*Nu/dh

    # Overall Heat Transfer Coefficient from battery surface to the coolant fluid (eq 10)
    U_total = 1/((1/h)+(b/k_chan))

    # Calculate NTU
    NTU = U_total*A_chan/(m_coolant*cp)
    
    # Effectiveness of the Channel 
    eff_HAS = 1 - np.exp(-NTU) 
    
    if T_inlet  < T_cell:
    
        # Calculate Outlet Temparture To ( eq 8)
        T_o = ((T_cell-T_inlet)*(1-np.exp(-NTU)))+T_inlet   

        # Calculate the Log mean temperature 
        T_lm = ((T_cell-T_inlet)-(T_cell-T_o))/(np.log((T_cell-T_inlet)/(T_cell-T_o)))
        
        # Calculated Heat Convected 
        Q_convec = U_total*A_chan*T_lm*eff_HAS
        
        # check the wavy channel effectiveness
        heat_transfer_efficiency      = (T_o - T_inlet) / (T_cell - T_inlet)
        
        # Calculate the Power consumed
        Power   = RCAIDE.Library.Components.Thermal_Management.Accessories.Pump.compute_power_consumed(dp, rho, m_coolant, n_pump) 
        
        # Update temperature of Battery Pack
        P_net                   = Q_module - Q_convec 
    
    elif T_inlet  > T_cell:
        # Reverse Heat Transfer
        
        # Calculate Outlet Temparture To ( eq 8)
        T_o =  T_inlet - ((T_inlet - T_cell)*(1-np.exp(-NTU)))   
    
        # Calculate the Log mean temperature 
        T_lm = ((T_inlet - T_cell)-(T_o - T_cell))/(np.log((T_inlet - T_cell)/(T_o - T_cell)))
            
        # Calculated Heat Convected 
        Q_convec = U_total*A_chan*T_lm*eff_HAS     
            
        # check the wavy channel effectiveness
        heat_transfer_efficiency      = (T_o - T_inlet) / (T_cell - T_inlet)
            
        # Calculate the Power consumed
        Power   = RCAIDE.Library.Components.Thermal_Management.Accessories.Pump.compute_power_consumed(dp, rho, m_coolant, n_pump) 
            
        # Update temperature of Battery Pack
        P_net                   = Q_convec + Q_module
        
    elif T_inlet  == T_cell:
        # When battery temperature is equal to the battery temperature 
        P_net = 0
        Q_convec = 0
        T_o = T_inlet
        Power   = RCAIDE.Library.Components.Thermal_Management.Accessories.Pump.compute_power_consumed(dp, rho, m_coolant, n_pump)         
        
        
    dT_dt                   = P_net/(cell_mass*N_cells_geometric_config*Cp_bat)
    T_cell_new              = T_cell + dT_dt*delta_t 
   
    state.conditions.energy[coolant_line.tag][HAS.tag].heat_removed[t_idx+1]               = Q_convec
    state.conditions.energy[coolant_line.tag][HAS.tag].outlet_coolant_temperature[t_idx+1] = T_o
    state.conditions.energy[coolant_line.tag][HAS.tag].coolant_mass_flow_rate[t_idx+1]     = m_coolant
    state.conditions.energy[coolant_line.tag][HAS.tag].effectiveness[t_idx+1]              = heat_transfer_efficiency
    state.conditions.energy[coolant_line.tag][HAS.tag].power[t_idx+1]                      = Power
    
    if not state.conditions.energy.recharging:
        state.conditions.energy[bus.tag].power_draw[t_idx+1]                                  += Power 
 
    return T_cell_new

def compute_coolant_properties(HAS,T_inlet,state,delta_t,t_idx):
    
    coolant    = HAS.coolant  
    m_coolant  = HAS.coolant_flow_rate 
    rho        = coolant.compute_density(T_inlet) # Keep track of units
    mu         = coolant.compute_absolute_viscosity(T_inlet)  
    cp         = coolant.compute_cp(T_inlet)
    Pr         = coolant.compute_prandtl_number(T_inlet) 
    k          = coolant.compute_thermal_conductivity(T_inlet)         
        
    opt_coolant_properties = Data(flowrate             =m_coolant,
                                  inlet_temperature    =T_inlet,
                                  inlet_density        =rho,
                                  inlet_visc           =mu,
                                  inlet_Cp             =cp,
                                  inlet_Pr             =Pr,
                                  inlet_thermal_cond   =k)


    return opt_coolant_properties