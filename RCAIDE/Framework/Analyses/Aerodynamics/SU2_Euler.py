# RCAIDE/Framework/Analyses/Aerodynamics/SU2_Euler.py
#  
# Created:  Aug 2025, W. Zheng

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports   
from RCAIDE.Framework.Core                                           import Data, Units
from RCAIDE.Framework.Analyses                                       import Process 
from RCAIDE.Library.Methods.Aerodynamics                             import Common
from RCAIDE.Framework.Analyses.Common.Process_Geometry               import Process_Geometry  
from RCAIDE.Library.Methods.Aerodynamics.SU2                         import *   
from RCAIDE.Framework.External_Interfaces.OpenVSP.export_vsp_vehicle import export_vsp_vehicle 
from RCAIDE.Framework.External_Interfaces.OpenVSP.run_vsp_mesh       import run_vsp_mesh 
from RCAIDE.Framework.External_Interfaces.GMSH.write_SU2_file        import write_SU2_file
from .Aerodynamics                                                   import Aerodynamics 

# package imports 
import numpy as np 
import os
import pickle

# ----------------------------------------------------------------------------------------------------------------------
#  SU2_Euler
# ---------------------------------------------------------------------------------------------------------------------- 
class SU2_Euler(Aerodynamics):
    """This is a

     Assumptions:
     Stall effects are negligible 
 
     Source:
     N/A
 
     Inputs:
     None
 
     Outputs:
     None
 
     Properties Used:
     N/A 
    """      
    
    def __defaults__(self):
        """This sets the default values and methods for the analysis.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        None

        Properties Used:
        N/A
        """          
        self.tag                                                    = 'SU2_Euler'
        self.vehicle                                                = Data()  
        self.process                                                = Process()
        self.process.initialize                                     = Process()

        # settings              
        self.settings.vsp_filename                                  = None
        self.settings.stl_filename                                  = None
        self.settings.SU2_filename                                  = None
        self.settings.SU2_config_filename                           = None
        self.settings.half_mesh_flag                                = False # Avoid using this flag. Keep as False.
        self.settings.number_of_processors                          = 8
        self.settings.vsp_mesh_growth_ratio                         = 1.2
        self.settings.vsp_mesh_growth_limiting_flag                 = False
        self.settings.run_new_SU2_sim                               = False
        self.settings.CFD_file_name                                 = None
        self.settings.maxedge                                       = 0.5
        self.settings.minedge                                       = 0.025
        self.settings.farfield_scale                                = 25
        self.settings.trim_aircraft                                 = False
        
        
        # conditions table, used for surrogate model training
        self.training                                               = Data() 
        self.training.angle_of_attack                               = np.array([ -5, -3, 0, 3, 5, 7, 10 ]) * Units.deg 
        self.training.Mach                                          = np.array([0.1, 0.3, 0.5, 0.65 , 0.85 , 0.9]) 
        self.training.temperature                                   = 285
        self.training.pressure                                      = 101325            
           
        self.reference_values                                       = Data()
        self.reference_values.S_ref                                 = 0
        self.reference_values.c_ref                                 = 0
        self.reference_values.b_ref                                 = 0
        self.reference_values.X_ref                                 = 0
        self.reference_values.Y_ref                                 = 0
        self.reference_values.Z_ref                                 = 0 
                          
        # surrogoate models                 
        self.surrogates                                             = Data() 

        # build the evaluation process
        compute                                                     = Process() 
        compute.lift                                                = Process()  
        compute.lift.inviscid_wings                                 = evaluate_surrogate 
        compute.lift.fuselage                                       = Common.Lift.fuselage_correction
        compute.drag                                                = Process()
        compute.drag.parasite                                       = Process()
        compute.drag.parasite.wings                                 = Process_Geometry('wings')
        compute.drag.parasite.wings.wing                            = Common.Drag.parasite_drag_wing 
        compute.drag.parasite.fuselages                             = Process_Geometry('fuselages')
        compute.drag.parasite.fuselages.fuselage                    = Common.Drag.parasite_drag_fuselage
        compute.drag.parasite.booms                                 = Process_Geometry('booms')
        compute.drag.parasite.booms.boom                            = Common.Drag.parasite_drag_fuselage 
        compute.drag.parasite.nacelles                              = Common.Drag.parasite_drag_nacelle
        compute.drag.parasite.pylons                                = Common.Drag.parasite_drag_pylon
        compute.drag.parasite.total                                 = Common.Drag.parasite_total
        compute.drag.induced                                        = Common.Drag.induced_drag 
        compute.drag.cooling                                        = Common.Drag.cooling_drag        
        compute.drag.compressibility                                = Common.Drag.compressibility_drag 
        compute.drag.miscellaneous                                  = Common.Drag.miscellaneous_drag 
        compute.drag.form                                           = Common.Drag.form_drag  
        compute.drag.wave                                           = Common.Drag.wave_drag
        compute.drag.trim                                           = Common.Drag.trim_drag 
        compute.drag.total                                          = Common.Drag.total_drag
        self.process.compute                                        = compute
        

    def initialize(self):
        use_surrogate   = self.settings.use_surrogate   
        # If we are using the surrogate
        vehicle = self.vehicle
        if use_surrogate == True: 
            #  training data
            if not os.path.exists(self.filename):
                for network in vehicle.networks:
                    for propulsor in network.propulsors:
                        propulsor.clear()
                export_vsp_vehicle(vehicle, vehicle.tag)
        
                vsp_filename        =  vehicle.tag +  '.vsp3'
                stl_filename        =  vehicle.tag +  '.stl'  
                SU2_filename        =  vehicle.tag +  '.su2'  
                SU2_config_filename =  vehicle.tag +  '.cfg'       
                run_vsp_mesh(vehicle,vsp_filename, self.settings.minedge,self.settings.maxedge, sym=False,farfield_scale = self.settings.farfield_scale,farfield=True,source=False)
                write_SU2_file(stl_filename, SU2_filename)
                
                self.settings.vsp_filename        = vsp_filename
                self.settings.stl_filename        = stl_filename
                self.settings.SU2_filename        = SU2_filename
                self.settings.SU2_config_filename = SU2_config_filename
    
                train_SU2_surrogates(self)
                if self.settings.store_training_data:
                    with open(self.filename, 'wb') as file:
                        pickle.dump(self.training, file)
            else:
                with open(self.filename, 'rb') as file:
                    self.training = pickle.load(file)
                print(r""" 
                [INFO] Aerodynamic training data loaded. Delete the file and rerun to regenerate. """)
            # build surrogate
            build_SU2_surrogates(self)
            
        return 
    
         
    def evaluate(self,state):
        """The default evaluate function.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        results   <RCAIDE data class>

        Properties Used:
        self.settings
        self.vehicle
        """          
        settings = self.settings
        vehicle  = self.vehicle 
        results  = self.process.compute(state,settings,vehicle)
        
        return results