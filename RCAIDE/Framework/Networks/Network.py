# RCAIDE/Framework/Networks/Network.py
# (c) Copyright 2023 Aerospace Research Community LLC
#
# Created:  Jul 2024, RCAIDE Team
# Modified: Aug 2023, E. Botero


# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Components import Component

# ----------------------------------------------------------------------------------------------------------------------
#  NETWORK
# ---------------------------------------------------------------------------------------------------------------------- 
class Network(Component):
    """ The top-level network class.
    """
    def __defaults__(self):
        """ This sets the default attributes for the network.

            Assumptions:
                None

            Source:
                None 
        """
        self.tag                          = 'network'
        self.propulsors                   = Container()  
        self.busses                       = Container()
        self.coolant_lines                = Container()
        self.fuel_lines                   = Container() 
        self.identical_propulsors         = True
        
# ----------------------------------------------------------------------
#  Component Container
# ---------------------------------------------------------------------- 
class Container(Component.Container):
    """ The Network container class 
    """
    def evaluate(self,state,center_of_gravity):
        """ This is used to evaluate the thrust produced by the network.

            Assumptions:  
                If multiple networks are attached their performances will be summed

            Source:
                None

            Args:
                State (dict): flight conditions 

            Returns:
                results (dict): Results of the evaluate method 
        """ 
        for net in self.values(): 
            net.evaluate(state,center_of_gravity)  
        return   

# ----------------------------------------------------------------------
#  Handle Linking
# ----------------------------------------------------------------------
Network.Container = Container