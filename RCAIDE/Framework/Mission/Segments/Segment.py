# RCAIDE/Framework/Mission/Segment/Segment.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE Imports
from RCAIDE.Framework.Core import Data
from RCAIDE.Framework.Analyses                    import Analysis, Settings, Process   
from RCAIDE.Framework.Mission.Common     import State 

# ----------------------------------------------------------------------------------------------------------------------
#  ANALYSES
# ----------------------------------------------------------------------------------------------------------------------  
class Segment(Analysis):
    """ 
    """    
    
    def __defaults__(self):
        """This sets the default values.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            None
    
            Outputs:
            None
    
            Properties Used:
            None
        """          
        
        self.settings                      = Settings() 
        self.state                         = State() 
        self.analyses                      = Analysis.Container() 
        self.process                       = Process() 
        self.process.initialize            = Process()
          
        self.process.converge              = Process()
        self.process.iterate               = Process()
        self.process.iterate.unknowns      = Process()
        self.process.iterate.initials      = Process()
        self.process.iterate.conditions    = Process()
        self.process.iterate.residuals     = Process()
        self.process.post_process          = Process()  
        
        self.conditions = self.state.conditions 
        
        return
    
    def initialize(self):
        """ This executes the initialize process
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            State  [Data()]
    
            Outputs:
            None
    
            Properties Used:
            None
        """        
        self.process.initialize(self)
        return
    
    def converge(self,state):
        """ This executes the converge process
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            State  [Data()]
    
            Outputs:
            None
    
            Properties Used:
            None
        """             
        self.process.converge(self,state)    
    
    def iterate(self):
        """ This executes the iterate process
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            State  [Data()]
    
            Outputs:
            None
    
            Properties Used:
            None
        """        
        self.process.iterate(self)
        return
    
    def post_process(self):
        """ This executes post processing
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            State  [Data()]
    
            Outputs:
            None
    
            Properties Used:
            None
        """         
        self.process.post_process(self)
        return
    
    def evaluate(self,state=None):
        """ This executes the entire process
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            State  [Data()]
    
            Outputs:
            State  [Data()]
    
            Properties Used:
            None
        """          
        if state is None:
            state = self.state
        self.process(self)
        return self
    
    def flight_dynamics_and_controls(self): 
        self.flight_dynamics                                             = Data()
        self.flight_dynamics.force_x                                     = False 
        self.flight_dynamics.force_y                                     = False 
        self.flight_dynamics.force_z                                     = False 
        self.flight_dynamics.moment_x                                    = False 
        self.flight_dynamics.moment_y                                    = False 
        self.flight_dynamics.moment_z                                    = False    
        
        self.assigned_control_variables                                              = Data() 
        
        self.assigned_control_variables.body_angle                                   = Data()
        self.assigned_control_variables.body_angle.active                            = False               
        self.assigned_control_variables.body_angle.initial_guess_values              = None

        self.assigned_control_variables.bank_angle                                   = Data()
        self.assigned_control_variables.bank_angle.active                            = False 
        self.assigned_control_variables.bank_angle.initial_guess_values              = None

        self.assigned_control_variables.wind_angle                                   = Data()
        self.assigned_control_variables.wind_angle.active                            = False             
        self.assigned_control_variables.wind_angle.initial_guess_values              = None   

        self.assigned_control_variables.elapsed_time                                 = Data()
        self.assigned_control_variables.elapsed_time.active                          = False                 
        self.assigned_control_variables.elapsed_time.initial_guess_values            = None  
    
        self.assigned_control_variables.velocity                                     = Data()
        self.assigned_control_variables.velocity.active                              = False                  
        self.assigned_control_variables.velocity.initial_guess_values                = None
        
        self.assigned_control_variables.acceleration                                 = Data()
        self.assigned_control_variables.acceleration.active                          = False                 
        self.assigned_control_variables.acceleration.initial_guess_values            = None
        
        self.assigned_control_variables.altitude                                     = Data()
        self.assigned_control_variables.altitude.active                              = False                 
        self.assigned_control_variables.altitude.initial_guess_values                = None
    
        self.assigned_control_variables.throttle                                     = Data() 
        self.assigned_control_variables.throttle.active                              = False                
        self.assigned_control_variables.throttle.assigned_propulsors                 = None       
        self.assigned_control_variables.throttle.initial_guess_values                = None
    
        self.assigned_control_variables.elevator_deflection                          = Data() 
        self.assigned_control_variables.elevator_deflection.active                   = False      
        self.assigned_control_variables.elevator_deflection.assigned_surfaces        = None 
        self.assigned_control_variables.elevator_deflection.initial_guess_values     = None

        self.assigned_control_variables.rudder_deflection                            = Data()
        self.assigned_control_variables.rudder_deflection.active                     = False
        self.assigned_control_variables.rudder_deflection.assigned_surfaces          = None 
        self.assigned_control_variables.rudder_deflection.initial_guess_values       = None

        self.assigned_control_variables.flap_deflection                              = Data() 
        self.assigned_control_variables.flap_deflection.active                       = False          
        self.assigned_control_variables.flap_deflection.assigned_surfaces            = None 
        self.assigned_control_variables.flap_deflection.initial_guess_values         = None

        self.assigned_control_variables.slat_deflection                              = Data() 
        self.assigned_control_variables.slat_deflection.active                       = False          
        self.assigned_control_variables.slat_deflection.assigned_surfaces            = None 
        self.assigned_control_variables.slat_deflection.initial_guess_values         = None            
    
        self.assigned_control_variables.aileron_deflection                           = Data() 
        self.assigned_control_variables.aileron_deflection.active                    = False      
        self.assigned_control_variables.aileron_deflection.assigned_surfaces         = None 
        self.assigned_control_variables.aileron_deflection.initial_guess_values      = None
    
        self.assigned_control_variables.thrust_vector_angle                          = Data() 
        self.assigned_control_variables.thrust_vector_angle.active                   = False        
        self.assigned_control_variables.thrust_vector_angle.assigned_propulsors      = None 
        self.assigned_control_variables.thrust_vector_angle.initial_guess_values     = None
        
        return     
           
# ----------------------------------------------------------------------
#  Container
# ----------------------------------------------------------------------

class Container(Segment):
    """ A container for the segment
    
        Assumptions:
        None
        
        Source:
        None
    """    
    
    def __defaults__(self):
        """This sets the default values.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            None
    
            Outputs:
            None
    
            Properties Used:
            None
        """          
                
        self.segments = Process()
        
        self.state = State.Container()
        
    def append_segment(self,segment):
        """ Add a SubSegment
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            segment  [Segment()]
    
            Outputs:
            None
    
            Properties Used:
            None
        """          
        self.segments.append(segment)
        return    
        
Segment.Container = Container