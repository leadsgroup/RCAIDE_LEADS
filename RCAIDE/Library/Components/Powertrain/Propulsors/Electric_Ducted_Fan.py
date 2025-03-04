# RCAIDE/Library/Components/Propulsors/Electric_Ducted_Fan.py
# 
# 
# Created:  Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports 
from .   import Propulsor  
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Ducted_Fan_Propulsor.append_electric_ducted_fan_conditions           import append_electric_ducted_fan_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Ducted_Fan_Propulsor.unpack_electric_ducted_fan_unknowns             import unpack_electric_ducted_fan_unknowns
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Ducted_Fan_Propulsor.pack_electric_ducted_fan_residuals              import pack_electric_ducted_fan_residuals 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Ducted_Fan_Propulsor.compute_electric_ducted_fan_performance         import compute_electric_ducted_fan_performance, reuse_stored_electric_ducted_fan_data
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Ducted_Fan_Propulsor.append_electric_ducted_fan_residual_and_unknown import append_electric_ducted_fan_residual_and_unknown

# ----------------------------------------------------------------------
#  Electric Ducted Fan Component
# ----------------------------------------------------------------------
class Electric_Ducted_Fan(Propulsor):
    """
    A propulsion system class that combines an electric motor with a ducted fan.
    
    Attributes
    ----------
    tag : str
        Identifier for the propulsion system, defaults to 'electric_ducted_fan'
    
    motor : None or Motor
        The electric motor component that provides rotational power
        
    ducted_fan : None or DuctedFan
        The ducted fan component that generates thrust
        
    electronic_speed_controller : None or ESC
        The electronic speed controller that regulates power to the motor
    
    Notes
    -----
    This class models an electric propulsion system where an electric motor drives 
    a ducted fan to generate thrust. The system includes an electronic speed controller 
    (ESC) to regulate power delivery from the electrical system to the motor.
    
    The ducted fan configuration provides several advantages over open propellers:
    - Higher static thrust efficiency
    - Reduced tip losses
    - Lower noise emission
    - Improved safety through shrouding
    
    **Definitions**

    'Ducted Fan'
        A propulsion device consisting of a fan enclosed within a duct/shroud 
        that improves thrust efficiency
    
    'Electronic Speed Controller (ESC)'
        Device that controls the speed of the electric motor by regulating 
        power delivery based on input commands
    
    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Propulsors.Propulsor
    """
    def __defaults__(self):    
        # setting the default values
        self.tag                          = 'electric_ducted_fan'    
        self.motor                        = None
        self.ducted_fan                   = None 
        self.electronic_speed_controller  = None

    def append_operating_conditions(self,segment):
        append_electric_ducted_fan_conditions(self,segment)
        return 

    def unpack_propulsor_unknowns(self,segment):  
        unpack_electric_ducted_fan_unknowns(self,segment)
        return 

    def pack_propulsor_residuals(self,segment): 
        pack_electric_ducted_fan_residuals(self,segment)
        return        

    def append_propulsor_unknowns_and_residuals(self,segment):
        append_electric_ducted_fan_residual_and_unknown(self,segment)
        return 
    
    def compute_performance(self,state,voltage,center_of_gravity = [[0, 0, 0]]):
        thrust,moment,power,stored_results_flag,stored_propulsor_tag =  compute_electric_ducted_fan_performance(self,state,voltage,center_of_gravity)
        return thrust,moment,power,stored_results_flag,stored_propulsor_tag
    
    def reuse_stored_data(EDF,state,bus,stored_propulsor_tag,center_of_gravity = [[0, 0, 0]]):
        thrust,moment,power = reuse_stored_electric_ducted_fan_data(EDF,state,bus,stored_propulsor_tag,center_of_gravity)
        return thrust,moment,power
