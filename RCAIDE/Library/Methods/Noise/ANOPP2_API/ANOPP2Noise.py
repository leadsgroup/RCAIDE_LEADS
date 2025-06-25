## @ingroup DCode-C2_Aeroacoustics-ANOPP2_Python
# ANOPP2Noise.py
#
# Created:  Mar 2023, R. Erhard
# Modified: Jun 2025, Niranjan Nanjappa

# RCAIDE Imports
from RCAIDE.Library.Methods.Noise.ANOPP2_API.ANOPP2_api   import *
from RCAIDE.Framework.Core                                import Data, Units

# Python Imports
import numpy as np

class ANOPP2Noise(Data):
    """
    This is an interface between SUAVE and ANOPP2.

    Assumptions:
    None

    Source:
    None
    """
    def __defaults__(self):

        #------------------------------------------------------------------------------------------
        # Tags for the Data Structures and Functional Modules.  These are set by the _create and
        # _load routines of their respective APIs (Atmosphere API for intAtmosphereTag, etc.).
        #------------------------------------------------------------------------------------------        
        self.intAtmosphereTag   = A2_IK (0) # Tag for atmospheric conditions.
        self.intFlightPathTag   = A2_IK (0) # Tag for flight characteristics: position and velocity.
        self.intObserverTag     = A2_IK (0) # Tag for observer locations and noise from predictions.
        self.intFormulationTag  = A2_IK (0) # Tag for noise predictions.  (ie. F1, F1A, G0, G1, G1A, and V1A)
        self.intBladeLineTags   = A2_IK (0) # Tag for 
        
        # Array of tag values used to access the results from all the
        # functional modules.  There is one array for each functional module.
        self.intResultTags     = POINTER(A2_IK)()
        self.nResultTags       = A2_IK (0)         # Number of tags returned.        
        self.intTotalResultTag = A2_IK (0)         # Tag associated with the total noise.
        
        # atmospheric conditions
        self.fltSpeedOfSound    = A2_RK (0)
        self.fltAmbientDensity  = A2_RK (0)
        self.intSuccess         = A2_IK (0)   
        
        # rotor specific conditions
        self.fltRpm     = 0
        self.fltPeriod  = 0
        self.fltOmega   = 0
        self.nBlades    = 0
        self.nAzimuthStations = 0

    def initialize(self, rotor, conditions):
        """
        This initializes the aerodynamics, atmosphere, flight path, and observer modules. 

        """
        # Extract rotor operation parameters
        self.fltRpm     = rotor.inputs.omega[0][0] / Units.rpm
        self.fltPeriod  = 60. / self.fltRpm  
        self.fltOmega   = 2 * np.pi / self.fltPeriod
        self.nBlades    = rotor.number_of_blades
        self.nAzimuthStations = rotor.number_azimuthal_stations   
        self.nRadialStations  = len(rotor.radius_distribution)
        
        # Extract rotor geometry parameters
        # Lifting line: x axis is the chord direction, y axis is the thickness, and z is the span.
        self.fltChordLengths = rotor.chord_distribution
        self.fltCrossSectionalAreas = rotor.cross_sectional_areas
        self.fltSpanLineCoordinates = rotor.radius_distribution
        self.fltChordLineCoordinates = np.zeros_like(rotor.radius_distribution)
        self.fltThicknessLineCoordinates = np.zeros_like(rotor.radius_distribution)        
        
        # reset blade tags for number of blades
        self.intBladeLineTags = []
        for iBlade in range(self.nBlades):
            self.intBladeLineTags.append(A2_IK (0))
            
        #==========================================================================================
        # Step 0: Run aerodynamics (if rotor outputs are not provided)
        #==========================================================================================
        if np.size(rotor.outputs)==0:
            print("\nRunning rotor spin function to get aerodynamic inputs for CF1A...")
            rotor.spin(conditions)

        #==========================================================================================
        # Step 1: Initialize all of ANOPP2
        # The ANOPP2 API must be initialized before any of the functions can be executed.
        #==========================================================================================
        ANOPP2.a2py_exec_init_api ()

        #==========================================================================================
        # Step 2: Create Data Structures
        # This step is to create all the data structures that are needed to do the F1A 
        # computation.  The data structures include the atmosphere, a flight path, rotor blade
        # geometries, and an observer.
        #==========================================================================================
        # setup data structures
        self.setup_atmosphere()
        self.setup_flight_path()
        self.setup_observer(rotor)

        return

    def setup_atmosphere(self):
        #------------------------------------------------------------------------------------------
        # Initialize parameters for defining conditions on the lifting line
        #------------------------------------------------------------------------------------------
        fltGround         = (int (3) * A2_RK)()     # Ground Position
        fltZero           = A2_RK (0)

        # Create the atmosphere using the catalog in ANOPP2's Atmosphere Data Structure (AADS)
        # to define a uniform atmosphere at sea level.  The result of this call is an integer 
        # that is used to refer to that atmosphere data structure.
        self.intSuccess =                     \
        ANOPP2.a2py_atm_load \
      (pointer (self.intAtmosphereTag), a2_atm_sea_level, create_string_buffer (b""))

        # Set the position of the query to 0.0, 0.0, 0.0
        fltGround[0] = A2_RK (0)
        fltGround[1] = A2_RK (0)
        fltGround[2] = A2_RK (0)

        # Grab the speed of sound at sea level so we can use it to calculate the period.
        self.intSuccess =                                   \
        ANOPP2.a2py_atm_get_speed_of_sound \
      (self.intAtmosphereTag, fltGround, fltZero, pointer (self.fltSpeedOfSound))

        # Also grab the ambient density.
        self.intSuccess =                                    \
        ANOPP2.a2py_atm_get_ambient_density \
      (self.intAtmosphereTag, fltGround, fltZero, pointer (self.fltAmbientDensity))

        return  

    def setup_flight_path(self):
        # The flight path is trivial, no motion of the vehicle with respect to the ground.  Use
        # the catalog in ANOPP2's Flight Path Data Structure (AFPDS) to load a trivial flight
        # path.  A trivial flight path includes no movement.
        self.intSuccess =                                                     \
        ANOPP2.a2py_fp_load                                  \
      (pointer (self.intFlightPathTag), a2_fp_trivial, self.intAtmosphereTag, \
         create_string_buffer (b""))
        return

    def setup_observer(self,rotor):
        # ------------------------------------------------------------------------------------
        # Observer creation
        # ------------------------------------------------------------------------------------
        R = rotor.tip_radius
        obsX_R = 0.
        obsY_R = 8.
        obsZ_R = 0.

        # Create an observer point cloud and add observer points.  
        # The result tags pointer will be returned as null 
        # (since we are loading a point cloud from the observer catalog and not a restart file).
        # First use ANOPP2's Observer Data Structure (AODS) to create a point cloud that includes
        # no points.  We will add a point afterward.

        # Initialize observer position
        fltObserverPosition = (int (3) * A2_RK)()

        self.intSuccess =                                                                   \
        ANOPP2.a2py_obs_load                                               \
      (pointer (self.intObserverTag), a2_obs_point_cloud, create_string_buffer (b""), \
         pointer (self.nResultTags), pointer (self.intResultTags))

        # Set the position of the observer to (0.0, 10.0, 0.0) * R
        fltObserverPosition[0] = obsX_R * R
        fltObserverPosition[1] = obsY_R * R
        fltObserverPosition[2] = obsZ_R * R

        # Define an observer location in the plane of the rotor 10R away.
        self.intSuccess = ANOPP2.a2py_obs_new_node (self.intObserverTag, fltObserverPosition)

        return   
