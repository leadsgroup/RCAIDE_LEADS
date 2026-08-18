# RCAIDE/Library/Components/Thermal_Management/Reservoirs/Reservoir.py
# 
# Created:  Mar 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Components.Powertrain.Sources.Source                            import Source   
from RCAIDE.Library.Attributes.Coolants.Glycol_Water                                import Glycol_Water
from RCAIDE.Library.Attributes.Materials.Polyetherimide                             import Polyetherimide
from RCAIDE.Library.Methods.Powertrain.Sources.Reservoirs.Reservoir_Tank            import compute_reservoir_performance, append_reservoir_conditions, append_reservoir_segment_conditions, append_reservoir_unknowns_and_residuals
from RCAIDE.Library.Plots.Thermal_Management.plot_reservoir_conditions              import plot_reservoir_conditions

# ----------------------------------------------------------------------
#  Reservoir
# ---------------------------------------------------------------------- 
class Reservoir(Source):
    """
    A class representing a coolant storage reservoir for thermal management systems.

    Attributes
    ----------
    tag : str
        Unique identifier for the reservoir, defaults to 'coolant_reservoir'
        
    material : Material
        Reservoir construction material, defaults to Polyetherimide()
        
    coolant : Coolant
        Fluid stored in reservoir, defaults to Glycol_Water()
        
    length : float
        Reservoir length in meters, defaults to 0.3
        
    width : float
        Reservoir width in meters, defaults to 0.3
        
    height : float
        Reservoir height in meters, defaults to 0.3
        
    thickness : float
        Wall thickness in meters, defaults to 5e-3
        
    surface_area : float
        Total surface area in square meters, calculated from dimensions
        
    volume : float
        Internal volume in cubic meters, calculated from dimensions
        
    mass_properties.mass : float
        Total mass including coolant, calculated from volume and coolant density

    Notes
    -----
    The reservoir serves as a thermal buffer and coolant storage system. It provides:
    
    * Coolant storage and thermal mass
    * Temperature stabilization through mixing
    * Volume compensation for thermal expansion
    * Air separation and pressure control

    **Definitions**

    'Mixing Temperature'
        Temperature resulting from combining flows of different temperatures
        
    'Surface Area'
        Total area for heat transfer with environment

    See Also
    --------
    RCAIDE.Library.Components.Thermal_Management.Heat_Exchangers.Cross_Flow_Heat_Exchanger
        Heat exchanger that interfaces with reservoir
    RCAIDE.Library.Components.Thermal_Management.Accessories.Pump
        Pump for circulating coolant from reservoir
    """

    def __defaults__(self):
        """
        Sets default values for the reservoir attributes.
        """
        self.tag                  = 'coolant_reservoir'
        self.material             = Polyetherimide()
        self.coolant              = Glycol_Water()
        self.length               = 0.3
        self.width                = 0.3
        self.height               = 0.3
        self.thickness            = 5e-3
        self.surface_area         = 2*(self.length*self.width + self.width*self.height +  self.length*self.height)
        self.volume               = self.length*self.width*self.height
        self.mass_properties.mass = self.coolant.density*self.volume
        return
    
    def append_operating_conditions(self, segment, coolant_line):
        """
        Adds operating conditions for the reservoir to a mission segment.

        Parameters
        ----------
        segment : Data
            Mission segment to which conditions are being added
        coolant_line : Data
            Cooling system flow path information
        """
        append_reservoir_conditions(self, segment, coolant_line)
        return
    
    def append_segment_conditions(self, segment, coolant_line):
        """
        Adds specific segment conditions to the reservoir analysis.

        Parameters
        ----------
        segment : Data
            Mission segment being analyzed
        coolant_line : Data
            Cooling system flow path information
        conditions : Data
            Operating conditions for the segment
        """
        append_reservoir_segment_conditions(self, segment, coolant_line)
        return    

    def append_unknowns_and_residuals(self, coolant_line, segment):
        """
        Registers the reservoir's coolant temperature unknown and residual.

        Parameters
        ----------
        coolant_line : Data
            Cooling system flow path information
        segment : Data
            Mission segment containing state conditions
        """
        append_reservoir_unknowns_and_residuals(self, coolant_line, segment)
        return

    def unpack_unknowns(self, coolant_line, segment):
        return

    def pack_residuals(self, coolant_line, segment):
        return

    def compute_performance(self, coolant_line, state, network):
        """
        Computes the reservoir's coolant temperature residual.

        Parameters
        ----------
        coolant_line : Data
            Cooling system flow path information
        state : Data
            Current system state
        network : Data
            The powertrain network
        """
        compute_reservoir_performance(self, coolant_line, state, network)
        return
    
    def plot_operating_conditions(self, results, coolant_line, save_filename, save_figure, 
                                show_legend, file_type, width, height):
        """
        Creates visualization plots of the reservoir operating conditions.

        Parameters
        ----------
        results : Data
            Simulation results data
        coolant_line : Data
            Cooling system flow path information
        save_filename : str
            Path for saving the plot
        save_figure : bool
            Flag to save the figure
        show_legend : bool
            Flag to display plot legend
        file_type : str
            Output file format
        width : float
            Plot width
        height : float
            Plot height
        """
        plot_reservoir_conditions(self, results, coolant_line, save_figure,
                                show_legend, save_filename, file_type, width, height)
        return    
