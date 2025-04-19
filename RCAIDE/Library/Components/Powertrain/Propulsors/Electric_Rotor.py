# RCAIDE/Library/Components/Propulsors/Electric_Rotor.py
#  
# 
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports 
from .   import Propulsor 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor_Propulsor.unpack_electric_rotor_unknowns             import unpack_electric_rotor_unknowns
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor_Propulsor.pack_electric_rotor_residuals              import pack_electric_rotor_residuals
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor_Propulsor.append_electric_rotor_conditions           import append_electric_rotor_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor_Propulsor.compute_electric_rotor_performance         import compute_electric_rotor_performance, reuse_stored_electric_rotor_data
from RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor_Propulsor.append_electric_rotor_residual_and_unknown import append_electric_rotor_residual_and_unknown

# ---------------------------------------------------------------------------------------------------------------------- 
#  Electric_Rotor
# ----------------------------------------------------------------------------------------------------------------------  
class Electric_Rotor(Propulsor):
    """
    A propulsion system class that combines an electric motor with a rotor for vertical and forward flight.
    
    Attributes
    ----------
    tag : str
        Identifier for the propulsion system, defaults to 'electric_rotor'
    
    motor : None or Motor
        The electric motor component that provides rotational power
        
    rotor : None or Rotor
        The rotor component that generates lift and thrust
        
    electronic_speed_controller : None or ESC
        The electronic speed controller that regulates power to the motor
        
    active_crypgenic_tanks_tanks : None or list
        Collection of active cryogenoc tanks. Default is None.
    
    Notes
    -----
    This class models an electric propulsion system where an electric motor drives 
    a rotor to generate lift and thrust. The system includes an electronic speed 
    controller (ESC) to regulate power delivery from the electrical system to the motor.
    
    The class provides methods for:
    - Computing rotor performance (thrust, moment, power)
    - Managing operating conditions
    - Handling system states and residuals
    - Reusing stored performance data for computational efficiency
    
    This propulsor type is commonly used in:
    - Multicopters
    - eVTOL aircraft
    - Hybrid helicopters
    
    **Definitions**

    'Rotor'
        A rotating assembly of airfoils (blades) that generates lift and thrust 
        through aerodynamic forces
        
    'Electronic Speed Controller (ESC)'
        Device that controls the speed of the electric motor by regulating 
        power delivery based on input commands
    
    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Propulsors.Propulsor
    RCAIDE.Library.Components.Energy
    """
    def __defaults__(self):    
        # setting the default values
        self.tag                           = 'electric_rotor'    
        self.motor                         = None
        self.rotor                         = None 
        self.electronic_speed_controller   = None  
        self.active_crypgenic_tanks_tanks  = None 

    def append_operating_conditions(self,segment):
        """
        Appends operating conditions to the segment.
        """
        append_electric_rotor_conditions(self,segment)
        return
    
    def append_propulsor_unknowns_and_residuals(self,segment):
        """
        Appends propulsor unknowns and residuals to the segment.
        """
        append_electric_rotor_residual_and_unknown(self,segment)
        return

    def unpack_propulsor_unknowns(self,segment):  
        """
        Unpacks propulsor unknowns from the segment.
        """
        unpack_electric_rotor_unknowns(self,segment)
        return 

    def pack_propulsor_residuals(self,segment): 
        """
        Packs propulsor residuals into the segment.
        """
        pack_electric_rotor_residuals(self,segment)
        return    
    
    def compute_performance(self,state,voltage,center_of_gravity = [[0, 0, 0]]):
        """
        Computes propulsor performance including thrust, moment, and power. 
        """
        thrust,moment,power,stored_results_flag,stored_propulsor_tag =  compute_electric_rotor_performance(self,state,voltage,center_of_gravity)
        return thrust,moment,power,stored_results_flag,stored_propulsor_tag
    
    def reuse_stored_data(electric_rotor,state,network,stored_propulsor_tag,center_of_gravity = [[0, 0, 0]]):
        """
        Reuses stored propulsor data for performance calculations.
        """
        thrust,moment,power = reuse_stored_electric_rotor_data(electric_rotor,state,network,stored_propulsor_tag,center_of_gravity)
        return thrust,moment,power