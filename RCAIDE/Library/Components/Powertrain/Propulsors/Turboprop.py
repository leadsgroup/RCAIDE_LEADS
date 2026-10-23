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
        turbine to the propeller, through the gearbox [W]. An output, not an
        input: it is `design_turboprop`'s result once `low_pressure_turbine.
        pressure_ratio` (the actual design input -- see `Turbine.
        pressure_ratio`) and the design mass flow rate are both known,
        reported here for visibility/downstream use. Default is 0.0 before
        `design_turboprop` runs.

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

    design_thrust : float
        Design-point thrust [N]. Solved by `design_turboprop` when only `rated_takeoff_power` is given.
        Default is 0.0.

    rated_takeoff_power : float, optional
        Rated sea-level static takeoff shaft power [W], at the propeller shaft (the engine type
        certificate's maximum take-off power). When set (> 0) without `design_thrust`,
        `design_turboprop` sizes the engine to it and solves `design_thrust`; when set together with
        `design_thrust`, it solves `rated_takeoff_temperature_ratio` so that the sea-level static shaft power
        equals it. Default is 0.0.

    rated_takeoff_temperature_ratio : float
        Combustor exit temperature at the takeoff rating divided by its design-point value, set by
        `design_turboprop` (1.0 without a takeoff rating). Throttle is a fraction of the takeoff rating,
        so throttle 1 runs at this ratio times the design-point combustor exit temperature. Default is 1.0.

    maximum_climb_throttle : float, optional
        Throttle of the maximum climb rating. None (default) takes the design point,
        1/rated_takeoff_temperature_ratio, as maximum climb; set it for an engine whose design point is
        not maximum climb (e.g. sea-level static).

    takeoff_combustor_exit_temperature_ratio : float
        Combustor exit temperature at the takeoff rating divided by its design-point value, used when
        sizing the engine to `rated_takeoff_power`. Default is 1.0: turboprops are flat rated, so at
        sea level in a standard atmosphere the takeoff power is limited by the gearbox torque rather
        than by the turbine temperature (the PW127M maximum take-off power of 2051 kW is held up to
        39 C, EASA TCDS IM.E.041).

    propeller_polytropic_efficiency : float
        Polytropic efficiency of the propeller as an actuator disk (Cantwell, AA283, Ch. 6), set by
        `design_turboprop` so that the propeller efficiency equals `propeller.design_efficiency` at
        the design point; the propeller thrust at other conditions follows from the shaft power,
        flight speed, density and disk area. Default is None (before design).

    offdesign_matching : Data, optional
        Off-design matching model (`Data(design_constants=..., reference_point=..., idle_fallback=...)`
        from `build_turboprop_offdesign_matching`), attached by `design_turboprop`; when set,
        `compute_turboprop_performance` uses live off-design component matching
        (`Turboprop_OffDesign_Matching.solve_turboprop_offdesign_robust`) instead of the analytical
        cycle model. Default is None.

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
        self.design_thrust                              = 0.0
        self.rated_takeoff_power                        = 0.0     # see docstring
        self.rated_takeoff_temperature_ratio            = 1.0     # see docstring
        self.takeoff_combustor_exit_temperature_ratio   = 1.0     # see docstring
        self.maximum_climb_throttle                     = None    # see docstring
        self.propeller_polytropic_efficiency            = None    # see docstring
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
