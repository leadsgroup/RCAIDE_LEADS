# RCAIDE/Library/Components/Propulsors/Turboprop.py 
#
#
# Created:  Mar 2024, M. Clarke
# Modified: May 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports   
from .                     import Propulsor
from RCAIDE.Framework.Core import Data , Units
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.append_turboprop_conditions    import append_turboprop_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.compute_turboprop_performance  import compute_turboprop_performance, reuse_stored_turboprop_data 
 
# python imports 
import numpy as np
# ---------------------------------------------------------------------------------------------------------------------- 
#  Turboprop
# ---------------------------------------------------------------------------------------------------------------------- 
class Turboprop(Propulsor):
    """
    A turboprop propulsion system model that simulates the performance of a turboprop engine.

    Attributes
    ----------
    tag : str
        Identifier for the turboprop engine. Default is 'turboprop'.
    
    nacelle : Component
        Nacelle component of the engine. Default is None.
        
    compressor : Component
        Compressor component of the engine. Default is None.
        
    turbine : Component
        Turbine component of the engine. Default is None.
        
    combustor : Component
        Combustor component of the engine. Default is None. 
        
    diameter : float
        Diameter of the engine [m]. Default is 0.0.
        
    length : float
        Length of the engine [m]. Default is 0.0.
        
    height : float
        Engine centerline height above the ground plane [m]. Default is 0.5.
        
    design_isa_deviation : float
        ISA temperature deviation at design point [K]. Default is 0.0.
        
    specific_fuel_consumption_reduction_factor : float
        Specific fuel consumption adjustment factor (Less than 1 is a reduction). Default is 0.0.
        
    design_altitude : float
        Design altitude of the engine [m]. Default is 0.0. 
        
    gearbox.efficiency : float
        Design point gearbox efficiency. Default is 0.0.
        
    design_mach_number : float
        Design Mach number. Default is 0.0.
        
    compressor_nondimensional_massflow : float
        Non-dimensional mass flow through the compressor. Default is 0.0.
        
    reference_temperature : float
        Reference temperature for calculations [K]. Default is 288.15.
        
    reference_pressure : float
        Reference pressure for calculations [Pa]. Default is 101325.0.

    design_power : float
        Design-point shaft power delivered by the free (low-pressure)
        turbine to the propeller, through the gearbox [W]. This is what
        actually powers the propeller -- the free turbine extracts this much
        work from the gas path (via `external_shaft.work_done`, the same
        mechanism `Turbofan`'s IDG/motor offtake uses), leaving little
        thrust to come from the core nozzle itself, which is the correct
        physical picture for a turboprop. Default is 0.0, which reproduces
        the previous (physically incomplete) behavior of a free turbine that
        does no work at all -- existing vehicles must set this explicitly to
        get a physically meaningful split between propeller and core thrust.

    design_power_offtake : float
        Design-point shaft power extracted from (or, via `integrated_drive_
        motor`, added to) the *gas-generator* spool (compressor +
        `high_pressure_turbine`) by `integrated_drive_generator`/
        `integrated_drive_motor` [W] -- an accessory tap (fuel pumps,
        aircraft electrics), distinct from `design_power` above (which is
        the free turbine's propeller power, on a mechanically separate
        shaft). Same convention as `Turbofan.design_power_offtake`. Default
        is 0.0.

    design_shaft_work_specific : float
        The gas-generator accessory offtake (`design_power_offtake` above),
        as *specific* work [J/kg core flow] at the converged design point --
        set by `design_turboprop`. Zero with no `integrated_drive_generator`/
        `integrated_drive_motor`. Default is 0.0.

    offdesign_matching : Data, optional
        If set (as `Data(design_constants=..., reference_point=...)` from
        `design_turboprop_offdesign_matching`), `compute_turboprop_performance`
        uses live off-design component matching
        (`Turboprop_OffDesign_Matching.solve_turboprop_offdesign_robust`)
        instead of the analytical cycle model. Default is None.

    Notes
    -----
    The Turboprop class inherits from the Propulsor base class and implements
    methods for computing turboprop engine performance. A turboprop engine uses
    a gas turbine core to drive a propeller through a reduction gearbox, combining
    the efficiency of a propeller at low speeds with the power of a turbine engine.

    **Definitions**

    'ISA'
        International Standard Atmosphere - standard atmospheric model

    'Mach number'
        Ratio of flow velocity to the local speed of sound

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Propulsors.Propulsor
    RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan
    RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet
    """ 
    def __defaults__(self):    
        # setting the default values
        self.tag                                        = 'turboprop'
        self.domain                                     = 'chemical'
        self.nacelle                                    = None 
        self.compressor                                 = None  
        self.turbine                                    = None  
        self.combustor                                  = None       
        self.diameter                                   = 0.0      
        self.length                                     = 0.0   
        self.propeller                                  = None
        self.height                                     = 0.0      
        self.design_isa_deviation                       = 0.0
        self.design_altitude                            = 0.0 
        self.gearbox                                    = Data()
        self.specific_fuel_consumption_reduction_factor = -3.875 
        self.gearbox.gear_ratio                         = 1.0
        self.gearbox.efficiency                         = 0.0  
        self.design_mach_number                         = None 
        self.design_freestream_velocity                 = None
        self.compressor_nondimensional_massflow         = 0.0
        self.reference_temperature                      = 288.15
        self.reference_pressure                         = 1.0*Units.atmosphere
        self.integrated_drive_generator                 = None
        self.integrated_drive_motor                     = None
        self.design_power                               = 0.0
        self.design_power_offtake                       = 0.0
        self.design_shaft_work_specific                 = 0.0
        self.offdesign_matching                         = None    # see docstring

    def append_operating_conditions(self,segment):
        """
        Appends operating conditions of the segment.
        """
        append_turboprop_conditions(self,segment)
        return
    
    def unpack_unknowns(self,segment):
        return 

    def pack_residuals(self,segment): 
        return

    def append_unknowns_and_residuals(self,segment):
        return    
    
    def compute_performance(self,state,network=None,center_of_gravity = [[0, 0, 0]]):
        """
        Computes turboprop performance including thrust, moment, and power.
        """
        inputs, outputs, stored_results_flag, stored_propulsor_tag =  compute_turboprop_performance(self,state,center_of_gravity)
        return inputs, outputs, stored_results_flag, stored_propulsor_tag
    
    def reuse_stored_data(turboprop,state,network,stored_propulsor_tag = None,center_of_gravity = [[0, 0, 0]]):
        """
        Reuses stored turboprop data for performance calculations.
        """ 
        inputs, outputs  = reuse_stored_turboprop_data(turboprop,state,network,stored_propulsor_tag,center_of_gravity)
        return inputs, outputs 
