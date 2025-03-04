# RCAIDE/Library/Components/Powertrain/Modulators/Solar_Logic.py
#  
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
from RCAIDE.Library.Components import Component
 
# ----------------------------------------------------------------------------------------------------------------------
#  Solar_Logic
# ----------------------------------------------------------------------------------------------------------------------  
class Solar_Logic(Component):
    """
    Class for managing solar power extraction using Maximum Power Point Tracking
    
    Attributes
    ----------
    MPPT_efficiency : float
        Efficiency of the Maximum Power Point Tracking system (default: 0.0)
        
    system_voltage : float
        Operating voltage of the electrical system (default: 0.0)

    Notes
    -----
    The Solar Logic component manages the complex power flow in solar aircraft,
    including:
    - Maximum power point tracking for solar panels
    - System voltage

    **Definitions**

    'Maximum Power Point Tracking (MPPT)'
        Control strategy that adjusts solar panel voltage to extract maximum
        available power under varying conditions

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Solar_Panels
        Solar panel components
    RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules
        Battery storage components
    """
    
    def __defaults__(self):
        """
        Sets default values for solar logic attributes
        
        Notes
        -----
        Initializes MPPT efficiency and system voltage to zero. These should be
        set to appropriate values based on the specific system configuration.
        """         
        
        self.MPPT_efficiency = 0.0
        self.system_voltage  = 0.0
    
    #def voltage(self):
        #""" The system voltage
        
            #Assumptions:
                #this function practically does nothing
                    
            #Source:
            #N/A
            
            #Inputs:
                #self.system_voltage         [volts]
               
            #Outputs:
                #self.outputs.system_voltage [volts]
                
            #Properties Used:
            #None               
        #"""
        #volts = self.system_voltage
        
        #self.outputs.system_voltage = volts
        
        #return volts

    #def logic(self,conditions,numerics):
        #""" The power being sent to the battery
        
            #Assumptions:
                #the system voltage is constant
                #the maximum power point is at a constant voltage
                
            #Source:
            #N/A
            
            #Inputs:
                #self.inputs:
                    #powerin
                    #pavionics
                    #ppayload
                    #currentesc
                #numerics.time.integrate

            #Outputs:
                #self.outputs:
                    #current
                    #power_in
                    #energy_transfer
                    
            #Properties Used:
                #self.MPPT_efficiency

        #"""
        ##Unpack
        #pin         = self.inputs.powerin[:,0,None]
        #pavionics   = self.inputs.pavionics
        #ppayload    = self.inputs.ppayload
        #esccurrent  = self.inputs.currentesc
        #volts       = self.voltage()
        #I           = numerics.time.integrate
        
        #pavail = pin*self.MPPT_efficiency
        
        #plevel = pavail -pavionics -ppayload - volts*esccurrent
        
        ## Integrate the plevel over time to assess the energy consumption
        ## or energy storage
        #e = np.dot(I,plevel)
        
        ## Send or take power out of the battery, Pack up
        #self.outputs.current         = (plevel/volts)
        #self.outputs.power_in        = plevel
        #self.outputs.energy_transfer = e
        
        
        #return 