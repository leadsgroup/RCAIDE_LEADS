# RCAIDE/Library/Components/Powertrain/Distributors/Fuel_Line.py 
# 
# Created:  Jul 2023, M. Clarke 
# Modified: Sep. 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
import RCAIDE
from   RCAIDE.Framework.Core                                    import Data
from   .Distributor                                             import Distributor   
from   RCAIDE.Library.Methods.Powertrain.Distributors.Fuel_Line import *

# ----------------------------------------------------------------------------------------------------------------------
#  Fuel Line
# ---------------------------------------------------------------------------------------------------------------------- 
class Cryogenic_Fuel_Line(Distributor):
    """
    """ 
    
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """          
        self.tag                           = 'cryogenic_fuel_line'  
        self.active                        = True 
        self.domain                        = 'chemical'
        self.efficiency                    = 1.0
        self.inner_diameter                = 0.015
        self.pressure                      = 150000.0  # Pa
        
    def append_operating_conditions(self, segment):
        """
        Append operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing operating conditions
        """
        append_fuel_line_conditions(self, segment)
        return

        
    def append_segment_conditions(self, segment):
        """
        Append segment-specific conditions to the bus
        
        Parameters
        ----------
        conditions : Data
            Container for segment conditions
        segment : Segment
            Flight segment data
        """
        append_fuel_line_segment_conditions(self, segment)
        return   

    def compute_performance(self, state):

        # Lookup table: inner diameter (inches) → dry weight (kg/m)
        dry_mass_lookup = {
            0.50:  4.46,
            1.00:  6.67,
            1.50:  8.12,
            2.00:  9.03,
            3.00:  15.35,
            4.00:  18.83,
            6.00:  37.14,
            8.00:  52.55,
            10.00: 70.16,
            12.00: 91.52,
        }

        lh2_heat_leak_lookup = {
            0.50:  0.29,
            1.00:  0.42,
            1.50:  0.54,
            2.00:  0.71,
            3.00:  1.01,
            4.00:  1.28,
            6.00:  1.87,
            8.00:  2.41,
            10.00: 3.00,
            12.00: 3.50,
        }

        # Flow rate lookup (liters per minute)
        flow_rate_lpm_lookup = {
            0.50:   29,
            1.00:   112,
            1.50:   323,
            2.00:   602,
            3.00:   1669,
            4.00:   3369,
            6.00:   9149,
            8.00:   18484,
            10.00:  33084,
            12.00:  51860,
        }

        # Convert inner diameter from meters → inches
        inner_d_in = self.inner_diameter / 0.0254

        # Find nearest available size
        nearest_size = min(dry_mass_lookup.keys(), key=lambda x: abs(x - inner_d_in))

        # Assign dry mass (kg/m)
        self.dry_mass_per_m = dry_mass_lookup[nearest_size]

        # Assign LH2 heat leak (W/m)
        self.lh2_heat_leak_per_m = lh2_heat_leak_lookup[nearest_size]

        # Assign recommended maximum flow rate (liters per minute)
        self.flow_rate_lpm = flow_rate_lpm_lookup[nearest_size]

        # Build inputs/outputs same as before
        inputs  = Data()
        outputs = Data()

        inputs.power.mechanical  = state.conditions.energy.distributors[self.tag].inputs.power.mechanical
        inputs.power.electrical  = state.conditions.energy.distributors[self.tag].inputs.power.electrical
        inputs.power.chemical    = state.conditions.energy.distributors[self.tag].inputs.power.chemical  
        inputs.power.pneumatic   = state.conditions.energy.distributors[self.tag].inputs.power.pneumatic 
        inputs.power.hydraulic   = state.conditions.energy.distributors[self.tag].inputs.power.hydraulic 
        inputs.power.thermal     = state.conditions.energy.distributors[self.tag].inputs.power.thermal  

        outputs.power.mechanical = state.conditions.energy.distributors[self.tag].outputs.power.mechanical
        outputs.power.electrical = state.conditions.energy.distributors[self.tag].outputs.power.electrical
        outputs.power.chemical   = state.conditions.energy.distributors[self.tag].outputs.power.chemical  
        outputs.power.pneumatic  = state.conditions.energy.distributors[self.tag].outputs.power.pneumatic 
        outputs.power.hydraulic  = state.conditions.energy.distributors[self.tag].outputs.power.hydraulic 
        outputs.power.thermal    = state.conditions.energy.distributors[self.tag].outputs.power.thermal  

        return inputs, outputs