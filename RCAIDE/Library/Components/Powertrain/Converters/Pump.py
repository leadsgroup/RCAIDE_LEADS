# RCAIDE/Library/Components/Powertrain/Converters/Pump.py
# 
# Created:  Jan 2026, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from .Converter  import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Pump import compute_pump_performance, append_pump_conditions
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity.compute_cylinder_center_of_gravity  import compute_cylinder_center_of_gravity
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_cylinder_moment_of_inertia  import compute_cylinder_moment_of_inertia
 
# ----------------------------------------------------------------------------------------------------------------------
# Pump
# ----------------------------------------------------------------------------------------------------------------------            
class Pump(Converter):
    """ 
    """  
    def __defaults__(self): 
        """
        Sets default values for the system attributes.
        """         
        self.tag                     = 'pump'
        self.diameter                = 0.0
        self.length                  = 0.0
        self.power_density           = 0.0
        self.working_fluid           = None
        self.pump_efficiency         = 1.0
        self.turbine_efficiency      = 1.0
        self.design_mass_flow_rate   = 0.0
        self.design_power_rating     = 0.0 # W/(kg/s)
        self.design_inlet_pressure   = 0.0 
        self.design_outlet_pressure  = 0.0
        self.design_hydraulic_power  = 0.0 
        self.casting_and_mount_factor= 2.0 

 
    def compute_performance(self,state,fuel_line = None,bus = None):
        """
        Computes Turboelectric_Generator performance including power.
        """
        P_mech,P_elec,stored_results_flag,stored_propulsor_tag =  compute_pump_performance(self,state,fuel_line, bus)
        return P_mech,P_elec,stored_results_flag,stored_propulsor_tag 
    
    def append_operating_conditions(self,segment,energy_conditions,noise_conditions=None): 
        """
        Adds operating conditions for the avionics system to a mission segment.

        Parameters
        ----------
        segment : Data
            Mission segment to which conditions are being added
        bus : Data
            Electrical bus supplying power to the avionics
        """
        append_pump_conditions(self,segment,energy_conditions) 
        return  

    def compute_moments_of_inertia(self,vehicle,center_of_gravity=[[0, 0, 0]]): 
        """
        Computes the moment of inertia tensor for the motor.

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.center_of_gravity.compute_fuselage_center_of_gravity
            Implementation of the center of gravity calculation
        """
        _ , _ = compute_cylinder_moment_of_inertia(self,outer_length=self.length,outer_radius=self.diameter/2,center_of_gravity= center_of_gravity) 
        return
    

    def compute_center_of_gravity(self,vehicle): 
        """
        Computes the center of gravity for the motor.

        Parameters
        ----------
        center_of_gravity : list, optional
            Reference point coordinates for moment calculation, defaults to [[0, 0, 0]] 

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.moments_of_inertia.compute_fuselage_moment_of_inertia
            Implementation of the moment of inertia calculation
        """
        _  = compute_cylinder_center_of_gravity(self, length=self.length) 
        return
        