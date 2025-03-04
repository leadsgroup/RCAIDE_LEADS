# RCAIDE/Library/Compoments/Powertrain/Sources/Fuel_Cells/Generic_Fuel_Cell.py
# 
# 
# Created:  Jan 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                                     import Units, Data
from RCAIDE.Library.Components                                 import Component    
from RCAIDE.Library.Attributes.Gases                           import Air  
from RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Larminie_Model.compute_fuel_cell_performance import *
from RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Larminie_Model.append_fuel_cell_conditions   import *

# ----------------------------------------------------------------------------------------------------------------------
#  Generic_Fuel_Cell
# ----------------------------------------------------------------------------------------------------------------------    
class Generic_Fuel_Cell_Stack(Component):
    """This is a fuel cell component.
    
    Assumptions:
    None

    Source:
    None
    """    
    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
        None

        Source:
        Some default values come from a Nissan 2011 fuel cell

        Inputs:
        None

        Outputs:
        None

        Properties Used:
        None
        """           
        self.tag                                        = 'fuel_cell'     
        self.mass_properties.mass                       = 1.0
        self.energy_density                             = 0.0
        self.current_energy                             = 0.0
        self.current_capacitor_charge                   = 0.0
        self.capacity                                   = 0.0
            
        self.length                                     = 0.0
        self.width                                      = 0.0
        self.height                                     = 0.0
        self.volume_packaging_factor                    = 1.05 
                 
        self.orientation_euler_angles                   = [0.,0.,0.]  # vector of angles defining default orientation of rotor        
                     
        self.fuel_cell                                  = Data()  
        self.fuel_cell.propellant                       = RCAIDE.Library.Attributes.Propellants.Gaseous_Hydrogen()
        self.fuel_cell.oxidizer                         = Air()
        self.fuel_cell.efficiency                       = .65                                 # normal fuel cell operating efficiency at sea level
        self.fuel_cell.specific_power                   = 2080                                # specific power of fuel cell [W/kg]; default is Nissan 2011 level
        self.fuel_cell.mass_density                     = 1203.208556                         # take default as specs from Nissan 2011 fuel cell      
        self.fuel_cell.volume                           = 0.0
        self.fuel_cell.max_power                        = 0.0 
        self.fuel_cell.length                           = 0.02
        self.fuel_cell.width                            = 0.05
        self.fuel_cell.height                           = 0.1 
        self.fuel_cell.rated_current_density            = 1.0
        self.fuel_cell.rated_power_density              = 1.0 
        
        self.fuel_cell.interface_area                   = 875.*(Units.cm**2.)                  # area of the fuel cell interface
        self.fuel_cell.r                                = (2.45E-4) *(1000*(Units.cm**2))      # area specific resistance [k-Ohm-cm^2]
        self.fuel_cell.Eoc                              = .931                                 # effective activation energy (V)
        self.fuel_cell.A1                               = .03                                  # slope of the Tafel line (models activation losses) (V)
        self.fuel_cell.m                                = 1.05E-4                              # constant in mass-transfer overvoltage equation (V)
        self.fuel_cell.n                                = 8E-3                                 # constant in mass-transfer overvoltage equation
        self.fuel_cell.ideal_voltage                    = 1.48
        self.fuel_cell.wall_thickness                   = .0022224                             # thickness of cell wall in meters  
        self.fuel_cell.cell_density                     =1988.                                 # cell density in kg/m^3
        self.fuel_cell.porosity_coefficient             =.6                                    # porosity coefficient  

        self.electrical_configuration                   = Data()
        self.electrical_configuration.series            = 1
        self.electrical_configuration.parallel          = 1   
        
        self.geometrtic_configuration                   = Data() 
        self.geometrtic_configuration.normal_count      = 1
        self.geometrtic_configuration.parallel_count    = 1
        self.geometrtic_configuration.normal_spacing    = 0.02
        self.geometrtic_configuration.stacking_rows     = 3
        self.geometrtic_configuration.parallel_spacing  = 0.02
        
         
    def energy_calc(self,state,bus,coolant_lines, t_idx, delta_t): 
        """Computes the state of the NMC battery cell.
           
        Assumptions:
            None
            
        Source:
            None
    
        Args:
            self               : battery        [unitless]
            state              : temperature    [K]
            bus                : pressure       [Pa]
            discharge (boolean): discharge flag [unitless]
            
        Returns: 
            None
        """                  
        
        stored_results_flag, stored_battery_tag = compute_fuel_cell_performance(self,state,bus,coolant_lines, t_idx,delta_t) 
        
        return stored_results_flag, stored_battery_tag 

    def append_operating_conditions(self,segment,bus):  
        append_fuel_cell_conditions(self,segment,bus)  
        return
    
    def append_fuel_cell_segment_conditions(self,bus, conditions, segment):
        append_fuel_cell_segment_conditions(self,bus, conditions, segment)
        return 

    def reuse_stored_data(self,state,bus,stored_results_flag, stored_fuel_cell_tag):
        reuse_stored_fuel_cell_data(self,state,bus,stored_results_flag, stored_fuel_cell_tag)
        return     
    