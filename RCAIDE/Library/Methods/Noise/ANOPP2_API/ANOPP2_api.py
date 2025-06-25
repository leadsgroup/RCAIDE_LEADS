#!/usr/bin/env python
#
# The ANOPP2_api Module for ANOPP2.
#
# Import Modules:
import math

from ctypes import *
import os
base = os.path.dirname(__file__)
ANOPP2 = CDLL(base+'/libANOPP2.dylib', RTLD_GLOBAL)

#
# Define Basic Constants used by the API.
#
false = c_bool(0)
true = c_bool(1)

#------------------------------------------------------------------------------------------
# These are ANOPP2's type definitions.  4 types are defined: integer, real, enumerator,
# and logical.
#------------------------------------------------------------------------------------------

# This is the standard double precision real type.
A2_RK = c_double

# This is the standard integer type.
A2_IK = c_int

# This is the standard enumerator type.
A2_EK = c_int

# This is the standard logical type.
A2_LK = c_bool

# This is the standard character type.
A2_CK = c_char

# This is the standard memory type.
A2_BK = c_char



#!/usr/bin/env python
# -----------------------------------------------------------------------------------------
# This file is the interface file for the Python subroutines in the ANOPP2
# Application Programming Interface (API).  This file should be copied to your local
# directory and an "include 'ANOPP2.api.py'" must be present in your
# program.  See anyone of the demonstrators provided with this API in the Demos
# directory.  For explanation of how to call and the theory behind each function, 
# please see the API manual provided with ANOPP2.
# -----------------------------------------------------------------------------------------
# @file ANOPP2.api.py
# @author The ANOPP2 Development Team
# @version 1.0.0
# -----------------------------------------------------------------------------------------



# =========================================================================================
# First part of this section of the contains interfaces into the available ANOPP2 API 
# functions.
# =========================================================================================



# -----------------------------------------------------------------------------------------
# This function initializes the ANOPP2 API by setting internal variables and function
# parameters that must exist before any other call to the API can be made.
# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
# This routine performs all the unit tests within the Command Executive.  An integer
# is returned that equals the number of fails in the system.  Zero indicates no
# failures.
# -----------------------------------------------------------------------------------------
# @result
#       A success integer that is equal to the number of failed asserts.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_exec_unit_test.restype = A2_IK



# -----------------------------------------------------------------------------------------
# This function call creates a Functional Module provided a set of input tags. The order
# and what tags are in the list is dependent on the Functional Module being created.  
# see Documentation for more information on what specific tags are included.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is the tag that will be returned to the user after the object has
#        been created.
# @param strConfigurationFile
#        This is a file that contains the inputs required for the object to be
#        created. This is typically a namelist file.
# @param nInputs
#        This is the number of inputs provided to this routine
# @param intInputTags
#        These are the tags of the inputs provided to the Functional Module
# @param intObserverTag
#        This is the tag associated with the observer data structure where the
#        results of the functional module will be cast.
# @param nResults
#        These are the number of results provided by executing the Functional Module.
# @param intResultTags
#        These are tags associated to the results provided by the Functional Module
# @result
#        An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_exec_create_functional_module.restype = A2_IK
ANOPP2.a2py_exec_create_functional_module.argtypes =                    \
  [POINTER(A2_IK), POINTER(A2_CK), A2_IK, POINTER(A2_IK), POINTER(A2_IK), POINTER \
    (A2_IK), POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This routine executes a single functional module.  The functional module must be a time
# series functional module.  The atmosphere and flight path must be provided as well.
# -----------------------------------------------------------------------------------------
# @param intFunctionalModuleTag
#        This tag is associated to the functional module that is to be executed.
# @param intAtmosphereTag
#        This is the tag associated to the atmosphere.
# @param intFlightPathTag
#        This is the tag associated to the flight path.
# @result
#        An integer that is 0 if everything has occurred as expected.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_exec_execute_functional_module.restype = A2_IK
ANOPP2.a2py_exec_execute_functional_module.argtypes = [A2_IK, A2_IK, A2_IK]



# -----------------------------------------------------------------------------------------
# This function takes in an atmosphere tag, flight path tag, number of time steps in the
# Functional Module, the maximum number of time steps per Functional Module, an array of 
# waypoint times the size of which is the number of time steps, and a 2-dimensional
# array of predition tags.  The size of the first dimension of Functional Module tags 
# is equal to the number of time steps, and the size of the second dimension is equal
# to the maximum number of Functional Modules.  This function will create a mission in
# the ANOPP2 API. The mission class includes a list of Functional Module per time step.
# -----------------------------------------------------------------------------------------
# @param intMissionTag
#        This function returns a mission which can be accessed by this tag value.
# @param strConfigurationFile
#        This is a file that the mission uses for settings.
# @param intAtmisphereTag
#        A tag which is associated to the atmosphere that will be used to generate the
#        aircraft specific atmospheric properties.
# @param intFlightPathTag
#        A tag which is associated to the aircraft's flight path for this mission.
# @param nTimeSteps
#        The number of time steps for this Functional Module.
# @param nMaximumSingleTimeFunctionalModules
#        The maximum number of Functional Module per time step.
# @param nTimeSeriesFunctionalModules
#        The number of time series functional modules being executed.
# @param fltWayPointTimes
#        An array of WayPoint times.  The size of this array is equal to the number of
#        time steps.  These times must be increasing.  
# @param intSingleTimeFunctionalModuleTags
#        A two dimensional array, sized by the number of time steps and the number of
#        maximum Functional Modules, that contains tags associated to the time.  The 
#        Functional Module tags are used to access Functional Modules that occur at
#        those times.
# @param intTimeSeriesFunctionalModuleTags
#        This is an array of tags that represent the time series events that will be
#        performed.
# @result
#        An integer that is 0 if everything has occurred as expected.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_exec_create_mission.restype = A2_IK
ANOPP2.a2py_exec_create_mission.argtypes =                          \
  [POINTER(A2_IK), POINTER(A2_CK), A2_IK, A2_IK, A2_IK, A2_IK, A2_IK, POINTER \
    (A2_RK), POINTER(A2_IK), POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# This function performs all operations that have been previously set up in the mission.
# This includes executing all Functional Modules in the order the user has dicteted.
# -----------------------------------------------------------------------------------------
# @param intMissionTag
#        This tag is associated to the mission that is to be executed.
# @result
#        An integer that is 0 if everything has occurred as expected.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_exec_execute_mission.restype = A2_IK
ANOPP2.a2py_exec_execute_mission.argtypes = [A2_IK]



# -----------------------------------------------------------------------------------------
#  This function corrects the acoustic data available in the Observer for differences
#  in ambient conditions (ambient density and speed of sound).
# -----------------------------------------------------------------------------------------
#  @param dmy_intObserverTag
#         This is the tag that is associated to the Observer Data Structure within the
#         Observer API.
#  @param dmy_intResultTag
#         This tag communicates what result is desired to be corrected.
#  @param dmy_intAtmosphereTag
#         This is the tag that is associated to the Atmosphere Data Structure within the
#         Atmosphere API.
#  @param dmy_intFlightPathTag
#         This is the tag that is associated to the Flight Path Data Structure within the
#         Flight Path API.
#  @param dmy_fltAmbientDensityIn
#         This is the ambient density that the acoustic data is currently based on.
#  @param dmy_fltSpeedOfSoundIn
#         This is the speed of sound that the acoustic data is currently based on.
#  @param dmy_enumAcousticMetric
#         This is the type of the acoustic metric contained in the Observer.
#  @result
#         An integer that is 0 if everything has occurred as expected.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_exec_ambient_correction.restype = A2_IK
ANOPP2.a2py_exec_ambient_correction.argtypes = \
  [A2_IK, A2_IK, A2_IK, A2_IK, A2_RK, A2_RK, A2_EK]



# -----------------------------------------------------------------------------------------
#  The Acoustic Data Interpolation routine calculates acoustic data (such as a spectra) 
#  at collections of nodes in 3-Dimensional space using a provided database of acoustic 
#  data. The interpolation occurs in space as well as over a set of parameters. In the
#  context of most situations that may have use for this routine, the acoustic data is a 
#  function of a series of Flight Conditions. This routine will use a database of 
#  acoustic data set in a node configuration (henceforth referred to as the Known 
#  Geometry) and a series of parameters (henceforth referred to as the Known Parameters) 
#  and interpolate the acoustic data at a desired node configuration (henceforth referred 
#  to as the Desired Geometry) and desired parameter set (henceforth referred to as the 
#  Desired Parameters). The routine calculates the values of the spectra by interpolation 
#  on a database of known noise levels that is structured in the same manner as the 
#  desired data.  Therefore, the database holds Known Parameter Sets and the Known 
#  Geometry, and the routine calculates the desired spectra, comprised of the desired 
#  frequencies and desired amplitudes, for the Desired Parameter Sets and Desired 
#  Geometry.  
# 
#  The user of this routine is responsible for providing the Known Database and the 
#  Desired variables as the input arguments to this routine.  The Known Parameters array 
#  is sized to be the number of known parameters in each set by the number of known 
#  Parameter sets. Likewise the Desired Parameters array is sized to be the number of 
#  Desired Parameters in each set by the number of Desired Parameters. For the 
#  interpolation to occur, the number of Known Parameters and Desired Parameters must
#  match, and hereafter will be referred to simply as the number of Parameters.  The 
#  number of Known and Desired Parameter sets, however, do not need to match, nor do the 
#  values of the Parameters in the sets.
# 
#  The Known and Desired Geometries follow the same pattern.  The first dimension for 
#  both is sized to be the number of spatial dimensions for the nodes (three), and the
#  second dimension, which may differ between the Known and Desired Geometries, is sized
#  to be the number to geometric nodes.  The major difference between the Parameters and
#  Geometry, from a usage perspective, is that both the Known and Desired Parameter
#  arrays must be provided whereas only the Known Geometry array must be provided.  The
#  Desired Geometry array is retrieved from the Observer Data Structure (whose tag must
#  be provided as an input to the routine).
# 
#  Unlike the Parameters and Geometry, the Known Amplitudes for the spectra are not 
#  passed to the routine as an array.  This is because several types of spectra, like 
#  Power Spectral Density, may have tens of thousands of frequencies. When compounded by 
#  several hundred nodes and multiple Parameter Sets, the array needed to hold the data 
#  could require gigabytes of memory.  Instead the routine requires a pointer to a 
#  function that will return all of the amplitudes for a requested frequency.  The 
#  function must be written by the routine's user, and it must follow the format of the 
#  procedure interface a2py_exec_macro_interface_interpolate as shown below (for more 
#  information, see the @param section).  Essentially this pointer function allows the 
#  routine to call back to the user's code and access the data, which would otherwise be 
#  out of the routine's authority to retrieve (scope).  The Known Amplitudes array 
#  returned by the pointer function must be sized to the number of Known Parameter Sets by 
#  the number of Known Geometric Nodes.
# 
#  In addition to the Known and Desired Database, the user must provide several tags,
#  enumerators, and auxiliary arrays.  As mentioned above, a tag for an Observer Data
#  Structure must be specified, as well as a tag for the Result on the Observer, which is 
#  where the output of the interpolation will be stored.  An array of times from
#  the flight path, called Source Times, must be supplied whose size matches the number 
#  of Desired Parameter Sets.  This is needed so that the routine knows what times should 
#  be associated with Desired Amplitudes it inserts into the Observer Data Structure 
#  Result for each of the Desired Parameter Sets.  Enumerators must be given for the 
#  metric type and the noise level type, and a boolean array of size two must be provided 
#  that specifies if Doppler shift and convective amplification are present (true) or 
#  absent (false) respectively.  An array of frequencies must be given that tells the 
#  routine what frequencies are present in the database, and these are also the 
#  frequencies that will be inserted into the observer, since no interpolation of 
#  frequency will be performed. Finally two auxiliary arrays that hold metadata about the 
#  Parameters and Geometry must  be supplied.  These specify whether the data is 
#  structured or unstructured, whether Cartesian or spherical interpolation should be 
#  performed, and hold values for either the power factor or the extents of Known 
#  Geometry array.  A complete list of the cases for these arrays as well as a full 
#  description of all of the inputs mentioned above can be found in the @param section 
#  for the arguments.
#   
#  When the routine is called, error checking is performed and variables are initialized.
#  If those operations are successful, the routine loops through the frequencies,
#  performing the interpolation for each. The interpolation occurs in two 
#  phases, first for the parameters, and then for the geometry.  If an proportional band 
#  spectrum is specified, the amplitudes are assumed to be input in Decibels and are 
#  converted to  pressure squared before any interpolation is performed.  The Parameter 
#  interpolation is for Scattered data and uses the inverse distance weighted method that 
#  is adjusted via a power factor.  Once the Desired Amplitudes for the Desired 
#  Parameters Sets are found, the amplitudes are transposed so that the Geometry 
#  interpolation can be performed. This is necessary because the calls to the Math API 
#  that do the calculations require the independent variables to be the first dimension 
#  of the passed array.  Originally the first dimension of the Known Amplitudes array is 
#  parallel to the Known Parameter Sets and the first dimension of the Desired Amplitudes 
#  array that is returned is parallel to the Desired Parameter Sets.  Once the Desired 
#  Amplitudes array is transposed, the first dimension is parallel to the Geometric 
#  Nodes. 
# 
#  The Geometry interpolation can be for Scattered or Structured data, and can be done 
#  using Cartesian or spherical coordinates.  For Scattered data the inverse distance 
#  weighting method is used.  If the interpolation is Cartesian, it is just performed on 
#  the three spatial dimension of the Known and Desired Geometries.  If it is spherical, 
#  the geometries are first converted from Cartesian coordinates to spherical 
#  coordinates, then interpolation is performed on the polar and azimuth angles (Theta 
#  and Phi) only.  When the interpolation is finished the amplitudes are adjusted for any 
#  difference in radius between the Known and Desired Geometry nodes.  For Structured 
#  data, linear grid interpolation is used, but the methodology for Cartesian or 
#  spherical treatment is the same as for Scattered data.
# 
#  With the interpolation complete, the routine checks if the spectra was originally an 
#  proportional band spectrum, and if so converts it from a narrow band (it was converted 
#  to narrow band before the interpolation) back to an proportional band spectrum. The 
#  routine inserts the final amplitudes for the given frequency into the Result on the 
#  Observer Data Structure and proceeds to the next frequency.
# 
#  Although the routine is quite flexible, it does have some limitations. If the 
#  Geometric Nodes are structured, they must be axis-aligned.  There is no calculation in 
#  place to handle non-axis-aligned structured data, however the scattered data 
#  interpolation could be used in this case.  Currently there is only one method 
#  available for scattered data interpolation, inverse distance weighting, however in the 
#  future other methods will be added (e.g. barycentric weighting and radial basis 
#  function).  In addition, only narrow band, proportional band spectra, and power 
#  spectral densities can be handled at present.  In the future the routine will be 
#  expanded to handle other metrics as well as tonal content.
# 
#  The most common errors that will be encountered are as follows:
#  - The number of Parameters which will be interpolated on must match between the Known 
#    and Desired sets.  These values are equal to the extent of the first dimension of 
#    the Known Parameters and Desired Parameters arrays.
#  - The number of Source Times from the Flight Path and the number of Desired Parameter
#    sets must match.  The latter is equal to the extent of the second dimension of the
#    Desired Parameters array.
#  - The range of each Parameter must not be zero.  This is equal to the range of each
#    extent across the Parameter sets in the Known Parameters array.
#  - The Parameters auxiliary values array must be size 2, with a valid enumerator for
#    structuredness and a valid power factor.
#  - The Geometry auxiliary values array must be between a size of 3 and 5, inclusive,  
#    with a valid enumerator for structuredness, a valid enumerator for the coordinate
#    system, valid sizes for the dimensions if structured (e.g. # of Xs, Ys, and Zs), and
#    a valid power factor if unstructured.
#  - The metric enumerator must be for narrow band, proportional band spectrum, or power 
#    spectral density. 
# -----------------------------------------------------------------------------------------
#  @param intObserverTag
#         This is the tag associated with the Observer Data Structure within the Observer
#         API.
#  @param intResultTag
#         This is the result within the Observer Data Structure in the Observer API.
#  @param fltSourceTimes
#         This is the array of Source Times from the Flight Path where the Flight
#         Conditions (Desired Parameter Sets) will be found for the interpolation.
#  @param enumMetric
#         This is the enumeration for the metric that is being interpolated. The options 
#         currently available are a2_aa_nbs, a2_aa_psd, and a2_aa_pbs (see the 
#         Acoustic Analysis API enumerations for a list of all metrics).
#  @param enumNoiseLevelType
#         This is the enumeration for the noise level type (Absolute or Change in Level).
#         See Acoustic Analysis API for enumerations).
#  @param blnIncludesFlightEffects
#         This is an array of 2 logicals where the first indicates if the data does 
#         (TRUE) or does not (FALSE) include Doppler Effect and the second indicates if 
#         it does (TRUE) or does not (FALSE) include Convective Amplification.
#  @param fltKnownParameters
#         This is the array of the known parameter sets.  The first dimension is the
#         number of parameters to be considered in each set, and the second dimension is 
#         the number of parameter sets.  For instance if the parameters are Mach number, 
#         throttle setting, and angle of attack, and there are ten known combinations of
#         these parameters, then the array would be three by ten.
#  @param fltDesiredParameters
#         This is the array of the desired parameter sets, that is, the points that
#         will be solved for by the interpolation routine.  The first dimension is the
#         number of parameters to be considered, and the second dimension is the number 
#         of Flight Conditions.  For instance if the parameters are Mach number, throttle 
#         setting, and angle of attack, and there are twenty Flight Conditions (desired 
#         combinations of these parameters), then the array would be three by twenty.
#         Note that the number of parameters in this array must match the number of
#         parameters in the Known Parameters array, but the number of sets may vary
#         between the two.
#  @param fltKnownGeometry
#         This is the array of geometric nodes where the acoustic spectra are known for 
#         all of the known parameter sets.  The Known Geometry must be input as Cartesian
#         coordinates, so the size of the array is three by the number of nodes.  Note 
#         that the same geometry is used for all parameter sets.
#  @param fltFrequencies
#         This is the array of frequencies of acoustic spectra. There is no interpolation
#         on frequency, so the known and desired data must have the same frequencies.
#  @param procGetAcousticData
#         This is a pointer to the function that gets the uninterpolated data (for all 
#         geometric nodes) from the data source for a specific frequency. This pointer,
#         along with the function it points to, must be provided by the user of this 
#         routine. The function must follow the interface defined below for
#         a2py_exec_interface_interpolate, where the frequency of interest is input,
#         and the array of amplitudes is output.  The amplitudes array must be of size
#         equal to the number of Known Parameter sets by the number of Known Geometric
#         Nodes, and the ordering of the amplitudes must match those arrays.
#  @param fltParameterAuxiliaryArray
#         This is the auxiliary data that is needed for interpolating on the parameters.
#         The first index indicates whether the data is structured or unstructured, and 
#         is specified by the enumerators:
#         a2_geo_structured  or  a2_geo_unstructured  
#         (currently only unstructured is supported for parameters)
#         The second index is the Power Factor for the Inverse Distance weighting
#         performed by the interpolation.   The recommended value is 2.0, however this
#         may be adjusted based on the shape of the acoustic data being provided.
#  @param fltGeometryAuxiliaryArray
#         This is the auxiliary data that is needed for interpolating on the geometry, 
#         and the size of the array will vary between three and five, based on the 
#         combination of the first two indices.
# 
#         The first index indicates whether the data is structured or unstructured, and 
#         is specified by the enumerators:
#         a2_geo_structured  or  a2_geo_unstructured
# 
#         The second index indicates whether the known geometry nodes are denoted in 
#         Cartesian or spherical coordinates, and are specified by the enumerators:
#         a2_geo_Cartesian  or  a2_geo_spherical
# 
#         If structured data is being used, then the following indices provide the 
#         number of points along the geometric axes.  If spherical coordinates are being 
#         used, then two values must be provided, one for the number of Theta's and the 
#         other for the number of Phi's.  If Cartesian coordinates are used, then three 
#         values must be provided for the number of Xs, Ys, and Zs respectively.
# 
#         If instead the data is unstructured, then only one value is provided - the 
#         power factor for the inverse distance weighting.  A value of 2.0 is
#         recommended, but may be adjusted for the particular shape of the acoustic data.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_exec_interpolate.restype = A2_IK
ANOPP2.a2py_exec_interpolate.argtypes =                                     \
  [A2_IK, A2_IK, A2_IK, POINTER(A2_RK), A2_EK, A2_EK, POINTER(A2_LK), A2_IK, A2_IK,   \
   POINTER(A2_RK), A2_IK, A2_IK, POINTER(A2_RK), A2_IK, A2_IK, POINTER(A2_RK), A2_IK, \
   POINTER(A2_RK), A2_IK, A2_IK, POINTER(A2_RK), A2_IK, POINTER(A2_RK), A2_LK]
#-----------------------------------------------------------------------------------------
# This file is the interface file for the fortran subroutines that are available from
# ANOPP2's Self Noise Internal Functional Module (ANOPP2..  These routines are derived
# from the functions available in RP1218's documentation.
#-----------------------------------------------------------------------------------------
# @file ANOPP2.api.py
# @author The ANOPP2 Development Team
# @version 1.0.0
#-----------------------------------------------------------------------------------------



#========================================================================================= \
# First part of this section of the contains interfaces into the available functions.
#========================================================================================= \



#------------------------------------------------------------------------------------------
#> This routine calculates the boundary layer thickness as a function of angle of attack,
#> chord, velocity, whether it has been tripped, and the atmospheric conditions using
#> empirical formula reported in RP1218.
#------------------------------------------------------------------------------------------
#> @param fltChordLength
#>        This is the chord length of the 0012 airfoil.
#> @param fltFreestreamVelocity
#>        This is the freestream flow velocity.
#> @param fltAngleOfAttack
#>        This is the angle of attack of the airfoil.
#> @param intTripSetting
#>        This is the trip setting.  If intTripSetting is equal to 0, it is untripped.
#>        If it is equal to 1, it is tripped.  If it is equal to 2, { it is tripped
#>        and the zero lift angle of attack displacement thickness is modifed by a 
#>        factor of 0.6 (not recommended).
#> @param fltSpeedOfSound
#>        The speed of sound of the fluid.
#> @param fltKinematicViscosity
#>        The kinematic viscosity of the fluid.
#> @param fltPressureSideBoundaryLayerThickness
#>        Returned by this function, this is the pressure side boundary layer thickness.
#> @param fltPressureSideDisplacementThickness
#>        Returned by this function, this is the pressure side discplacement thickness.
#> @param fltSuctionSideDisplacementThickness
#>        Returned by this function, this is the suction side discplacement thickness.
#> @result
#>        An integer representing success of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_asnifm_calc_boundary_layer_properties.restype  = A2_IK
ANOPP2.a2py_asnifm_calc_boundary_layer_properties.argtypes = [
  A2_RK,          # fltChordLength
  A2_RK,          # fltFreestreamVelocity
  A2_RK,          # fltAngleOfAttack
  A2_RK,          # fltTripSetting
  A2_RK,          # fltSpeedOfSound
  A2_RK,          # fltKinematicViscosity
  POINTER(A2_RK), # fltSuctionSideBoundaryLayerThickness
  POINTER(A2_RK), # fltPressureSideBoundaryLayerThickness
  POINTER(A2_RK), # fltSuctionSideDisplacementThickness
  POINTER(A2_RK), # fltPressureSideDisplacementThickness
  POINTER(A2_RK), # fltSuctionSideMomentumThickness
  POINTER(A2_RK)  # fltPressureSideMomentumThickness
]


#------------------------------------------------------------------------------------------
# This file is the interface file for the fortran subroutines that are available from
# ANOPP2's FRAME Internal Functional Module (ANOPP2..
#------------------------------------------------------------------------------------------
# @file ANOPP2.api.f90
# @author The ANOPP2 Development Team
# @version 1.0.0
#------------------------------------------------------------------------------------------



#==========================================================================================
# First part of this section of the contains interfaces into the available functions.
#==========================================================================================




#------------------------------------------------------------------------------------------
# This routine takes in a FRAME database file name, reads it, and stores its contents.
#------------------------------------------------------------------------------------------
# @param nCharacters
#        This is the number of characters in the FRAME database NetCDF file name.
# @param strDatabaseName
#        This is the name of the FRAME database NetCDF file.
# @result
#        An integer representing success of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_afifm_read_frame_database.restype  = A2_IK
ANOPP2.a2py_afifm_read_frame_database.argtypes = [A2_IK, POINTER(A2_CK)]



#------------------------------------------------------------------------------------------
# This routine clears the FRAME data from its memory
#------------------------------------------------------------------------------------------
# @result
#        An integer representing success of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_afifm_clear_frame_data.restype  = A2_IK
ANOPP2.a2py_afifm_clear_frame_data.argtypes = []



#------------------------------------------------------------------------------------------
# This function gets the number of and an array of group variables in the database.
#------------------------------------------------------------------------------------------
# @param nVariables
#        This is the number of variables in the database.
# @param nCharacters
#        This is the maximum number of characters in the Variables array.
# @param strVariables
#        This is the array of variables in the database.
# @result
#        An integer communicating success of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_afifm_get_variables.restype  = A2_IK
ANOPP2.a2py_afifm_get_variables.argtypes = \
  [POINTER(A2_IK), POINTER(A2_IK), POINTER(POINTER(A2_CK))]



#------------------------------------------------------------------------------------------
# This function enables the definition of the following:
#  1. The variables in the database that define the observer-related data such as,
#     phi, theta, and radius.
#  2. The variables in the database that are mapped to the variables in the Flight
#     Path config file.
#  3. The variables in the database that contain acoustic data that has to be 
#     interpolated along the flight path.
# Syntax for each of the strings is as follows:
#  Observer: <ANOPP2 variable name> = <Database variable name>
#  Flight Path: <ANOPP2 variable name> = <Database variable name>
#  Acoustic: <Database variable name>; <units>
#------------------------------------------------------------------------------------------
# @param dmy_strObserver
#        This is an array of observer variables representing a map between the ANOPP2
#        variables and the database variables.
# @param dmy_strFlightPath
#        This is an array of flight path variables representing a map between the ANOPP2
#        variables and the database variables.
# @param dmy_strAcoustic
#        This is an array of acoustic variables the database variables to be determined
#        along the flight path followed by the units in which the acoustic data is
#        available in the database.
# @result
#        An integer communicating success of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_afifm_define_variables.restype  = A2_IK
ANOPP2.a2py_afifm_define_variables.argtypes = \
  [A2_IK, POINTER(A2_CK), A2_IK, POINTER(A2_CK), POINTER(A2_CK), A2_IK, POINTER(A2_CK)]



#------------------------------------------------------------------------------------------
# This function exports the contents of the ANOPP2.database to a Tecplot-friendly file.
#------------------------------------------------------------------------------------------
# @param this
#        The ANOPP2.class that contains the data culled from the FRAME database.
# @param dmy_iGroup
#        This is the set of data that will be exported.
# @result
#        An integer communicating success of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_afifm_export_oaspl.restype  = A2_IK
ANOPP2.a2py_afifm_export_oaspl.argtypes = [POINTER(A2_CK), A2_IK]



#------------------------------------------------------------------------------------------
# The function queries the FRAME database and extracts the data related to Observer 
# provided therein.
#------------------------------------------------------------------------------------------
# @param nTheta
#        This is the number of theta values for which data is available in the database.
# @param nPhi
#        This is the number of phi values for which data is available in the database.
# @param nRadius
#        This is the number of radius values for which data is available in the database
# @param fltTheta
#        This is the array of theta values for which data is available in the database.
# @param fltPhi
#        This is the array of phi values for which data is available in the database.
# @param fltRadius
#        This is the array of radius values for which data is available in the database.
# @result
#         An integer indicating success(0) or failure(-1) of the operation
#------------------------------------------------------------------------------------------
ANOPP2.a2py_afifm_query_for_obs.restype  = A2_IK
ANOPP2.a2py_afifm_query_for_obs.argtypes =                        \
  [POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), POINTER(POINTER(A2_RK)), \
   POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK))]



#------------------------------------------------------------------------------------------
# The function queries the FRAME database and extracts the data related to the fixed
# parameters (headers) therein.
#------------------------------------------------------------------------------------------
# @param nfixedParameters
#        This is the number of fixed parameters provided in the FRAME database.
# @param strFixedParameters
#        This is an array of fixed parameters provided in the FRAME database
# @param strParameterTypes
#        This is an array of fixed parameter types provided in the FRAME database
# @param strParameterValues
#        This is an array of the fixed parameter values provided in the FRAME database
#        returned as a string for each parameter.
# @result
#         An integer indicating success(0) or failure(-1) of the operation
#------------------------------------------------------------------------------------------
ANOPP2.a2py_afifm_query_for_fixed_parameters.restype  = A2_IK
ANOPP2.a2py_afifm_query_for_fixed_parameters.argtypes =    \
  [POINTER(A2_IK), POINTER(POINTER(A2_CK)), POINTER(POINTER(A2_CK)), \
   POINTER(POINTER(A2_CK))]



#------------------------------------------------------------------------------------------
# The function queries the FRAME database and extracts the data related to the dependent
# variables therein.
#------------------------------------------------------------------------------------------
# @param nDependentParameters
#        This is the number of dependent parameters provided in the FRAME database.
# @param nDependentData
#        This is an array of number of dependent data for each dependent parameter.
# @param strDependentParameters
#        This is an array of dependent parameters provided in the FRAME database
# @param strDependentParameterTypes
#        This is an array of dependent parameter types provided in the FRAME database
# @result
#         An integer indicating success(0) or failure(-1) of the operation
#------------------------------------------------------------------------------------------
ANOPP2.a2py_afifm_query_for_dependent_parameters.restype  = A2_IK
ANOPP2.a2py_afifm_query_for_dependent_parameters.argtypes = \
  [POINTER(A2_IK), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_CK)),  \
   POINTER(POINTER(A2_CK))]



#------------------------------------------------------------------------------------------
# The function queries the FRAME database and extracts the data related to the condition
# variables therein.
#------------------------------------------------------------------------------------------
# @param nConditionVariables
#        This is the number of condition variables provided in the FRAME database.
# @param strConditionVariables
#        This is an array of condition variables provided in the FRAME database
# @param strConditionVariableTypes
#        This is an array of condition variable types provided in the FRAME database
# @param nConditions
#        This is an array of number of conditions for each condition variable.
# @param fltConditions
#        This is an array of the condition variable values provided in the FRAME database
#        returned as a real for each parameter.
# @param intConditions
#        This is an array of the condition variable values provided in the FRAME database
#        returned as an integer for each parameter.
# @param strConditions
#        This is an array of the condition variable values provided in the FRAME database
#        returned as a string for each parameter.
# @param nUniqueConditions
#        This is an array of number of unique conditions for each condition variable.
# @param fltUniqueConditions
#        This is an array of unique condition variable values provided in the FRAME 
#        database returned as a real for each parameter.
# @param intUniqueConditions
#        This is an array of unique condition variable values provided in the FRAME 
#        database returned as an integer for each parameter.
# @param strUniqueConditions
#        This is an array of unique condition variable values provided in the FRAME 
#        database returned as a string for each parameter.
# @result
#         An integer indicating success(0) or failure(-1) of the operation
#------------------------------------------------------------------------------------------
ANOPP2.a2py_afifm_query_for_condition_variables.restype  = A2_IK
ANOPP2.a2py_afifm_query_for_condition_variables.argtypes =          \
  [POINTER(A2_IK), POINTER(POINTER(A2_CK)), POINTER(POINTER(A2_CK)),          \
   POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), \
   POINTER(POINTER(A2_CK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_RK)), \
   POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_CK))]
#!/usr/bin/env python
# 
# -----------------------------------------------------------------------------------------
#  This is the Flight Path API interface file.  It contains definitions for all the
#  functions available in the Flight Path API.  This includes initializations, creation,
#  insertion, exporting, etc.  See API manual for more information.
# ------------------------------------------------------------------------------------------
#  @file ANOPP2.api.f90
#  @author The ANOPP2 Development Team
#  @version 1.0.0
# -----------------------------------------------------------------------------------------
# 



# 
# =========================================================================================
#  First part of this section of the contains interfaces into the available ANOPP2 API 
#  functions.
# =========================================================================================
# 





# -----------------------------------------------------------------------------------------
#  This function initializes the ANOPP2 API by setting internal variables and function
#  parameters that must exist before any other call to the API can be made.
# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
#  This routine executes the unit tests in the Acoustic Analysis module.  The unit 
#  tests execute all the tests implemented in the Acoustic Analysis API.
# -----------------------------------------------------------------------------------------
#  @result
#         An integer that is the total number of failed asserts encountered during
#         unit testing.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_fp_unit_test.restype = int



# -----------------------------------------------------------------------------------------
#  This routine creates an flight path data structure in the ANOPP2 API.  A tag is 
#  returned which is used by the calling program to access that data structure.  The
#  input into this routine is the name of a settings file.  The settings file must 
#  contain one of the known types of flight paths.  See Documentation for more 
#  information on the format of the settings file.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is an integer that is returned by this function.  It is used to access
#         the data structure that is created.
#  @param strSettingsFile
#         This is the name of the input file that contains the settings for the new
#         flight path. See Documentation for more information.
#  @param intAtmosphereTag
#         This is the tag associated to the atmosphere that the aircraft is flying
#         through.
#  @result
#         An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_fp_create.restype = int
ANOPP2.a2py_fp_create.argtypes = [POINTER(A2_IK), POINTER(A2_CK), A2_IK]



# -----------------------------------------------------------------------------------------
#  This function takes in a tag representing an flight path and returns true if it exists
#  in the API and false if it does not.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is the tag associated to the flight path that is being searched for.
#  @result
#        A bool that returns true if the observer exists and false if it does not.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_fp_exists.restype = bool
ANOPP2.a2py_fp_exists.argtypes = [A2_IK]



# -----------------------------------------------------------------------------------------
#  This routine saves the flight path data structure in a preopened file by exporting
#  all internal data.  The file format is specific to the data structure being written
#  out.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is an integer representation of the flight path to be saved
#  @param strRestartFile
#         This is the name of the file being created.
#  @result
#         An integer representing success of this function
# -----------------------------------------------------------------------------------------
# 
ANOPP2.a2py_fp_save.restype = int
ANOPP2.a2py_fp_save.argtypes = [A2_IK, POINTER(A2_CK)]



# -----------------------------------------------------------------------------------------
#  This routine creates a flight path data structure in the ANOPP2 API.  A tag is
#  returned which is used by the calling program to access that data structure.  The
#  input into this routine is the name of a settings file.  The settings file must
#  contain one of the known types of flight paths.  See Documentation for more information
#  on the format of the settings file.
# -----------------------------------------------------------------------------------------
#  @param dmy_intTag
#         This is the tag that will be returned to the user after the object has
#         been created.
#  @param dmy_enumFlightPath
#         This is the enumeration of the flight path desired in the Catalog.
#  @param dmy_intAtmosphereTag
#         This is the tag of the atmosphere to be used to create the flight path.
#  @param dmy_strRestartFile
#         This is the file name of the user supplied restart.  If the enumeration
#         provided does not exist, it will be loaded from this restart file.
#  @result
#         An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_fp_load.restype = int
ANOPP2.a2py_fp_load.argtypes = [POINTER(A2_IK), A2_IK, A2_IK, POINTER(A2_CK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the source times for the flight path associated with the tag.
# -----------------------------------------------------------------------------------------
#  @param dmy_intTag
#         This is an integer that is used to access the flight path data structure.
#  @param nTimes
#         The number of source times that are being returned.  This is equal to the size
#         of the dmy_fltSourceTimes Array
#  @param dmy_fltSourceTimes
#         This is the array of source times
#  @result
#         An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_fp_get_source_times.restype = int
ANOPP2.a2py_fp_get_source_times.argtypes = \
   [A2_IK, POINTER(A2_IK), POINTER(POINTER(A2_RK))]



# -----------------------------------------------------------------------------------------
#  This routine returns the Source Times for the flight path associated with the tag.
# -----------------------------------------------------------------------------------------
#  @param dmy_intTag
#         This is an integer that is used to access the flight path data structure.
#  @param nFlightTimes
#         The number of flight times that are being returned.  This is equal to the size
#         of the dmy_fltFlightTimes Array
#  @param dmy_FlightTimes
#         This is the array of flight times
#  @result
#         An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_fp_get_flight_times.restype = int
ANOPP2.a2py_fp_get_flight_times.argtypes = \
   [A2_IK, POINTER(A2_IK), POINTER(POINTER(A2_RK))]


# -----------------------------------------------------------------------------------------
#  This routine returns the flight data for the flight path associated with the tag.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is an integer that is used to access the flight path data structure.
#  @param intIndex
#         This is the index where the values should be retrieved
#  @param fltFlightTime
#         This is the flight time at the given index
#  @param fltCgLocation
#         This is the location vector for the given index.  This is a one dimensional
#         array of size 3.
#  @param fltBodyToWindAngles
#         This is Body to Wind Angles vector for the given indexThis is a one dimensional
#         array of size 3.
#  @param fltEarthToBodyAngles
#         This is the Earth to Body Angles vector the given indexThis is a one dimensional
#         array of size 3.
#  @result
#         An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_fp_get_flight_data_at_index.restype = int
ANOPP2.a2py_fp_get_flight_data_at_index.argtypes = \
   [A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the Flight Data for the flight path associated with the tag at
#  the time specified in the input argument.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is an integer that is used to access the flight path data structure.
#  @param fltFlightTime
#         This is the time where the Flight Data is desired.
#  @param fltCgLocation
#         This is the location vector for the given time.  This is a one dimensional 
#         array of size 3.
#  @param fltBodyToWindAngles
#         This is Body to Wind Angles vector for the given time. This is a one 
#         dimensional array of size 3.
#  @param fltEarthToBodyAngles
#         This is the Earth to Body Angles vector the given time. This is a one 
#        dimensional array of size 3.
#  @result
#         An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_fp_get_flight_data_at_time.restype = int
ANOPP2.a2py_fp_get_flight_data_at_time.argtypes = \
   [A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the flight cindition for the flight path associated with the tag.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is an integer that is used to access the flight path data structure.
#  @param intIndex
#         This is the index where the values should be retrieved
#  @param fltTime
#         This is the time at the given index
#  @param dmy_fltPosition
#         This is a position at the flight condition.  It is a one dimension array of
#         size three, with the X, Y, and Z coordinates
#  @param intLandingGear
#         This is the Landing gear setting at the given index and is represented by a
#         logical value where false (0) corresponds to the landing gear up and true (1)
#         corresponds to the landing gear down
#  @param fltFlap
#         This is the flap setting at the given index
#  @param fltMach
#         This is the Mach Number at the given index
#  @param fltAngleOfAttack
#         This is the Angle of Attack at the given index
#  @result
#         An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_fp_get_flight_condition_at_index.restype = int
ANOPP2.a2py_fp_get_flight_condition_at_index.argtypes =                       \
   [A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_IK), \
    POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the flight condition for the flight path associated with the tag
#  for the time specified as the input.  Note: This is not the Flight Condition object, 
#  but data consistant with the flight condition.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is an integer that is used to access the flight path from the manager.
#  @param fltFlightTime
#         This is the time where the Flight Condition is desired.
#  @param fltPosition
#         This is a position at the flight condition.  It is a one dimension array of
#         size three, with the X, Y, and Z coordinates
#  @param fltThrottle
#         This is the throttle sitting at the given time
#  @param blnLandingGear
#         This is the Landing gear setting at the given time and is represented by a
#         logical value where false (0) corresponds to the landing gear up and true (1)
#         corresponds to the landing gear down
#  @param fltFlap
#         This is the flap setting at the given time
#  @param fltMach
#         This is the Mach Number at the given time
#  @param fltAngleOfAttack
#         This is the Angle of Attack at the given time
#  @result
#         An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_fp_get_flight_condition_at_time.restype = int
ANOPP2.a2py_fp_get_flight_condition_at_time.argtypes =                 \
   [A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_IK), \
    POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the Mach for the flight path associated with the tag at the
#  specified time.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is an integer that is used to access the flight path from the manager.
#  @param fltFlightTime
#         This is the time where the Mach number is desired
#  @param fltMachNumber
#         This is the Mach number at the desired time
#  @result
#         An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_fp_get_mach_number_at_time.restype = int
ANOPP2.a2py_fp_get_mach_number_at_time.argtypes = \
   [A2_IK, POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the velocity for the flight path associated with the tag at the
#  specified time.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is an integer that is used to access the flight path from the manager.
#  @param fltFlightTime
#         This is the time where the velocity is desired
#  @param fltVelocity
#         This is the velocity at the desired time
#  @result
#         An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_fp_get_velocity_at_time.restype = int
ANOPP2.a2py_fp_get_velocity_at_time.argtypes = \
   [A2_IK, POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
#  This routine writes out the information stored in the flight path to an external file.
#  The output includes information about the aircraft at way points time as well as those
#  defined at the flight time.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is the tag value of the flight path being written out.
#  @param nCharsFlightData
#         The number of characters in the Flight Data file name
#  @param nCharsFlightCondition
#         The number of characters in the Flight Condition file name
#  @param strFlightDataFileName
#         This is the name of the file that will contain the Flight Data output.
#  @param strFlightConditionFileName
#         This is the name of the file that will contain the Flight Condition output.
#  @result
#         An integer representing success of this operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_fp_export.restype = int
ANOPP2.a2py_fp_export.argtypes = [A2_IK, POINTER(A2_CK), POINTER(A2_CK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the tag associated with the Kinematics data structure created by 
#  the flight path associated with the tag.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is an integer that is used to access the flight path data structure.
#  @param intKinematicsTag
#         This is the tag associated with the Kinematics created by this flight path.
#  @result
#         An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_fp_get_kinematics_tag.restype = int
ANOPP2.a2py_fp_get_kinematics_tag.argtypes = [A2_IK, POINTER(A2_IK)]



#------------------------------------------------------------------------------------------
# This routine reorients the vectors such that they are all in the same frame of 
# reference.  The frame of reference could be the ground frame or the local frame 
# depending on the enumerator provided.
#------------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag that relates to the ANOPP2.data structure.
# @param fltCurrentFor
#        This is the coordinates in the current frame of reference.
# @param fltNewFor
#        This is the transformed coordinates in the new frame of reference.
# @param enumTargetFor
#        This is an enumerator that represents the Frame of Reference to which the 
#        coordinates have to be transformed.
# @result
#        An integer representation of success.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_fp_reorient.restype = int
ANOPP2.a2py_fp_reorient.argtypes = \
   [A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), A2_IK]
#!/usr/bin/env python
# -----------------------------------------------------------------------------------------
#  These enumerators are to communicate to the API which built in flight path is to be 
#  loaded from the internal catalog.  This is optional, the user may wish to specify a
#  flight path config file and execute a module to generate a flight path. 
# -----------------------------------------------------------------------------------------

#  This enumerator is for an undefined flight path
a2_fp_undefined = 1

#  This enumerator is for a trivial flight path
a2_fp_trivial = 2

#  This enumerator is for a straight and level flight path using ANOPP.
a2_fp_straight_and_level = 3

#  This enumerator is for a basic takeoff.
a2_fp_takeoff = 4
    
#This enumerator is for a basic takeoff using FLOPS.
a2_fp_flops_takeoff = 5
    
#This enumerator is for a basic landing using FLOPS.
a2_fp_flops_landing = 6

# This enumerator is for a kinematics based straight and level flight path.
a2_fp_kine_steady_flyover = 7



#!/usr/bin/env python
# -----------------------------------------------------------------------------------------
# This is the ANOPP2.API interface file.  It contains definitions for all the
# functions available in the observer API.  This includes initializations, creation,
# insertion, exporting, etc.  See API manual for more information.
# -----------------------------------------------------------------------------------------
# @file ANOPP2.api.f90
# @author The ANOPP2 Development Team
# @version 1.0.0
# -----------------------------------------------------------------------------------------



# =========================================================================================
# First part of this section of the contains interfaces into the available ANOPP2.API
# functions.
# =========================================================================================





# -----------------------------------------------------------------------------------------
# This subroutine initializes the observer API and should be included at the very
# start of your program (before any other subroutines are called).  The debug flag
# turns on/off debugging for the entire system.
# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
# This routine executes the unit tests in the ANOPP2.Data Structure.  The unit
# tests execute all the tests implemented in the ANOPP2.API.
# -----------------------------------------------------------------------------------------
# @result
#        An integer that is the total number of failed asserts during unit testing.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_unit_test.restype = A2_IK



# -----------------------------------------------------------------------------------------
# This routine creates an ANOPP2.Data Structure in the ANOPP2.API.  A tag is
# returned which is used by the calling program to access that Data Structure.  The
# input into this routine is the name of a configuration file (also known as Settings
# file).  The configuration file must contain one of the known types of observers. By
# providing the values of several parameters in these configuration files, the Observer
# API generates one or more nodes that define the geometry of the ANOPP2.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is an integer that is returned by this function.  It is used to access
#        the data structure that is created.
# @param strConfigurationFile
#        This is the name of the input file that contains the settings for the new
#        observer. See Documentation for more information.
# @result
#        An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_create.restype = A2_IK
ANOPP2.a2py_obs_create.argtypes = [POINTER(A2_IK), POINTER(A2_CK)]



# -----------------------------------------------------------------------------------------
# This routine will destroy an observer in the ANOPP2.API. The entire data structure,
# including any noise results, calculated metrics, geometric configurations, and
# motion will be deleted.  The tag will be unassociated to any information within the
# data structure.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is the tag associated with the ANOPP2.API being destroyed.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_delete.restype = A2_IK
ANOPP2.a2py_obs_delete.argtypes = [POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# This routine saves the ANOPP2.API in a preopened file by exporting
# all internal data.  The file format is specific to the data structure being written
# out.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is an integer representation of the observer to be saved
# @param strRestartFile
#        This is the name of the file being created.
# @result
#        An integer representing success of this function
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_save.restype = A2_IK
ANOPP2.a2py_obs_save.argtypes = [A2_IK, POINTER(A2_CK)]



#------------------------------------------------------------------------------------------
#> This routine saves the AODS Result in a preopened file by exporting all internal result 
#> data.  The file format is specific to the data structure being written out.
#------------------------------------------------------------------------------------------
#> @param intANOPP2.ag
#>        This is the tag associated with the observer.
#> @param intResultTags
#>        This is a pointer to an array of integers representing tags associated with 
#>        observer results to be saved.
#> @param strRestartFile
#>        This is the name of the file being created.
#> @result
#>        An integer representing success of this function
#------------------------------------------------------------------------------------------
ANOPP2.a2py_obs_save_results.restype = A2_IK
ANOPP2.a2py_obs_save_results.argtypes = [A2_IK, A2_IK, POINTER(A2_IK), POINTER(A2_CK)]



# -----------------------------------------------------------------------------------------
# This routine creates an ANOPP2.API in the ANOPP2 API.  A tag is
# returned which is used by the calling program to access that data structure.  The
# input into this routine is the name of a settings file.  The settings file must
# contain one of the known types of observers.  See Documentation for more information
# on the format of the settings file.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is the tag that will be returned to the user after the object has
#        been created.
# @param enumObserver
#        This is the enumeration of the observer desired in the Catalog.
# @param strRestartFile
#        This is the file name of the user supplied restart file.  If the enumeration
#        provided does not exist, the data structure  will be created form this restart
#        file.
# @param nResults
#        This is the size of the result tags array returned by this function.
# @param intResultTags
#        If the observer is loaded from a restart file, this pointer array will point
#        to a number of integers that are the result tags.  This is left as null if the
#        observer is loaded from the catalog.
# @result
#        An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_load.restype = A2_IK
ANOPP2.a2py_obs_load.argtypes = \
   [POINTER(A2_IK), A2_EK, POINTER(A2_CK), POINTER(A2_IK), POINTER(POINTER(A2_IK))]



#------------------------------------------------------------------------------------------
#> This routine creates an ANOPP2.Result in the AODS API.  A tag is returned which is 
#> used by the calling program to access that data structure.  The input into this routine 
#> is the name of a restart file.  The restart file must contain one of the known types of 
#> observers.  See Documentation for more information on the format of the restart file.
#------------------------------------------------------------------------------------------
#> @param intTag
#>        This is the tag that should have been created ahead of time.
#> @param strRestartFile
#>        This is the file name of the user supplied restart file.
#> @param intResultTags
#>        If the observer results are loaded from a restart file, this pointer array will 
#>        point to a number of integers that are the result tags.
#> @result
#>        An integer that is returned 0 when everything occurred correctly.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_obs_load_results.restype = A2_IK
ANOPP2.a2py_obs_load_results.argtypes = \
   [A2_IK, POINTER(A2_CK), POINTER(A2_IK), POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This routine sends the noise results at a given node to a destination processor.
# This routine will use the MPI library (if available) to send the data.  See MPI
# library detail for more information on Message Passing Interface (MPI).
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        The ANOPP2 tag associated with the observer that contains the data.
# @param intNode
#        The observer node of the data being sent.
# @param intResultTag
#        The tag of the result being sent.
# @param intDestination
#        The rank of the desination processor.
# @param intMessageTag
#        The MPI tag of the message.
# @result
#        An integer reprenting success of this operation
# -----------------------------------------------------------------------------------------
# ANOPP2.a2py_obs_send.restype = A2_IK
# ANOPP2.a2py_obs_send.argtypes = [A2_IK, A2_IK, A2_IK, A2_IK, A2_IK]



# -----------------------------------------------------------------------------------------
# This routine receives the noise results at a given node from a source processor.
# This routine will use the MPI library (if available) to receive the data.  See MPI
# library detail for more information on Message Passing Interface (MPI).
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        The ANOPP2 tag associated with the observer that will contain the data.
# @param intNode
#        The observer node of the data being received
# @param intResultTag
#        The tag of the result being received.
# @param intSource
#        The rank of the source processor.
# @param intMessageTag
#        The MPI tag of the message.
# @result
#        An integer reprenting success of this operation
# -----------------------------------------------------------------------------------------
# ANOPP2.a2py_obs_receive.restype = A2_IK
# ANOPP2.a2py_obs_receive.argtypes = [A2_IK, A2_IK, A2_IK, A2_IK, A2_IK]



# -----------------------------------------------------------------------------------------
# This routine broadcasts the noise results on the observer inthe master process to the
# the observers of all the slave processes. This routine will use the MPI library
# (if available) to broadcast the data.  See MPI library detail for more information on
# Message Passing Interface (MPI).
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        The ANOPP2 tag associated with the observer that contains the data.
# @param nResults
#        The number of results in the results array.
# @param intResultTags
#        An array of tags of the results being broadcast.
# @result
#        An integer reprenting success of this operation
# -----------------------------------------------------------------------------------------
# ANOPP2.a2py_obs_broadcast.restype = A2_IK
# ANOPP2.a2py_obs_broadcast.argtypes = [A2_IK, A2_IK, POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# This function takes in a tag representing an observer and returns true if it exists
# in the API and false if it does not.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is the tag associated to the observer that is being searched for.
# @result
#        A bool that is returned true if the observer exists and false if it does not.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_exists.restype = A2_LK
ANOPP2.a2py_obs_exists.argtypes = [A2_IK]



# -----------------------------------------------------------------------------------------
# This function takes in a tag representing an observer and observer result and returns
# true if they exists in the API and false if it does not.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated to the observer that is being searched for.
# @param intResultTag
#        This is the tag associated with the observer result.
# @result
#        An integer that is returned 0 if the observer exists.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_result_exists.restype = A2_IK
ANOPP2.a2py_obs_result_exists.argtypes = [A2_IK, A2_IK]



# -----------------------------------------------------------------------------------------
# This function returns a string of information about an observer structure in the
# ANOPP2.API.  The information string will contain information such as geometric
# parameters (number of points in certain directions), results and what metric are
# available in each, etc.
# -----------------------------------------------------------------------------------------
# @param intTag
#        The tag value associated to the data structure of interest.
# @param strInformation
#        A string of information that is returned by this function.
# @param nResults
#        The number of results for which the the user would like to retrieve info.
# @param intResultTags
#        This is an input array of Result Tags associated with each of the results that
#        the user would like to retrieve the information on.
# @param nMetrics
#        This is the maximum number of results (second dimension of available metrics).
# @param enumAvailableMetrics
#        This is a two-dimensional array of Available Metrics, the first dimension is
#        the results and the second is the maximum number of metrics available.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_info.restype = A2_IK
ANOPP2.a2py_obs_info.argtypes = \
   [A2_IK, POINTER(A2_CK), POINTER     \
     (A2_IK), POINTER(A2_IK), POINTER(A2_IK), POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This function returns the number of nodes in an observer geometry.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is the tag associated to the observer that is being accessed
# @param nNodes
#        This is the number of nodes in the geometry of the observer.
# @result
#        An integer that is returned 0 if the observer exists.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_number_of_nodes.restype = A2_IK
ANOPP2.a2py_obs_number_of_nodes.argtypes = [A2_IK, POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# This function returns the number of results that are in the observer API.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is the tag of the observer in the API.
# @param nResults
#        This is the number of results, returned by this function.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_number_of_results.restype = A2_IK
ANOPP2.a2py_obs_number_of_results.argtypes = [A2_IK, POINTER(A2_IK)]




# -----------------------------------------------------------------------------------------
# This function returns the number of segments for a given result in an observer
# structure.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated to the observer that is being accessed
# @param intResultTag
#        This ist he tag associated with the result being accessed.
# @param iNode
#        This is the node number of interest
# @param nSegments
#        This is the number of segments in the geometry of the observer.
# @result
#        An integer that is returned 0 if the observer exists.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_number_of_segments.restype = A2_IK
ANOPP2.a2py_obs_number_of_segments.argtypes = [A2_IK, A2_IK, A2_IK, POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# This routine initializes new results in the ANOPP2.API.  New results
# must be initialized in the ANOPP2.API before noise can be added to it.
# This routine takes in a tag representation of the observer and a number of results
# that will be added into the ANOPP2.API.  An array is allocated to the
# size of the number of results that will be added.  This array of result tags is then
# used to insert noise later in the system.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is an integer representation of the observer to be modified with a
#        new result (or several).
# @param nResults
#        This is the number of results that are going to be added.
# @param enumCoordinateSystems
#        These are the coordinate systems on which the result is based.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @param nChars
#        This is the length of the strings in the array of result names.
# @param strResultNames
#        These are the names of the results that are being added to the system.  They
#        are stored in the ANOPP2.API and used when Exporting on the
#        results.
# @param intResultTags
#        These are the tags of the results after they are initialized in the observer
#        data structure
# @result
#        An integer representing success of the creation of the new result
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_new_results.restype = A2_IK
ANOPP2.a2py_obs_new_results.argtypes = \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, POINTER(A2_CK), POINTER(POINTER(A2_IK))]




# -----------------------------------------------------------------------------------------
# This function inserts a node into the ANOPP2.API.  The arguments include
# the position of the new node in the local frame of reference.  The new node will move
# as defined by the kinematics list, similar to the already existing nodes in the data
# structure.  The noise associated with the new node will not exist, no addition
# acoustic data is created.  This routine may not work with all possible configurations
# of the observer's geometry.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag that is associated with the ANOPP2.API being
#        modified.
# @param fltPosition
#        This is an array of size 3 of single precision reals that is the position of the
#        new node.
# @result
#        This is an integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_new_node.restype = A2_IK
ANOPP2.a2py_obs_new_node.argtypes = [A2_IK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine deletes a result in the ANOPP2.API.  The result is removed
# from memory and the data structure completely.  There is an optional argument that is
# a node number.  If the node number is 0, results for all nodes will be deleted.  If
# the node number is not 0 (and less than the total number of observer nodes) then only
# the result at that node number is destroyed.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is an integer representation of the observer to be modified by deleting
#        a result (or several).
# @param nResults
#        This is the number of results that are there in the ANOPP2.Data Structure for
#        the node.
# @param intResultTags
#        These are the tags of the results after they are deleted from the observer data
#        structure
# @param intNodeNumber
#        This is the number of the node whose results are going to be deleted.  If this
#        is given as zero, all nodes will have their results deleted.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_delete_results.restype = A2_IK
ANOPP2.a2py_obs_delete_results.argtypes = [A2_IK, A2_IK, POINTER(A2_IK), A2_IK]


# -----------------------------------------------------------------------------------------
# This function deletes a metric from a given result in the ANOPP2.API.
# A tag is provided for the ANOPP2.API and the result.  An enumerator
# for the metric to be deleted is also provided.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is an integer representation of the observer to be modified by deleting
#        a metric.
# @param intResultTags
#        These are the tags of the results after they are modified the observer data
#        structure
# @param enumMetric
#        This is the enumerator for the metric that is to be deleted.  The list of
#        enumerators that are available are in this file.
# @result
#        An integer represeting success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_delete_metric.restype = A2_IK
ANOPP2.a2py_obs_delete_metric.argtypes = [A2_IK, A2_IK, POINTER(A2_IK), A2_EK]



# -----------------------------------------------------------------------------------------
# This routine returns the position of the ith node in the observer geometry.  The
# function returns an array of size 3 that is the position of the observer in the global
# frame.
# -----------------------------------------------------------------------------------------
# @param intTag
#        The integer tag of the observer contained within the ANOPP2.API
# @param fltTime
#        This is the time that the positions are wanted.
# @param enumFrame
#        This is an enumeration for the frame of reference of the position, it can be
#        either local or global
# @param iNode
#        This is the ith node index.
# @param Coordinates
#        This is the enumerator for the Coordinate system in which the position has to
#        be returned in: a2_geo_cartesian for Cartesian and a2_geo_spherical for
#        spherical polar coordinate systems.
# @param fltPosition
#        A two dimensional array that returns the 3 dimensional node locations
# @result
#        An integer represeting success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_position.restype = A2_IK
ANOPP2.a2py_obs_get_position.argtypes = \
  [A2_IK, A2_RK, A2_EK, A2_IK, A2_EK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine returns the positions of the observer locations at a time.  It returns
# an integer that is the number of positions and an array of the positions.  The
# returned array has dimensions 3 by the number of observer nodes.
# -----------------------------------------------------------------------------------------
# @param intTag
#        The integer tag of the observer contained within the ANOPP2.API
# @param fltTime
#        This is the time that the positions are wanted.
# @param nPositions
#        This is the number of positions that is returned by this function.
# @param enumFrame
#        This is an enumeration for the frame of reference of the position, it can be
#        either local or global
# @param enumCoordinates
#        This is the enumerator for the Coordinate system in which the position has to
#        be returned in: a2_geo_cartesian for Cartesian and a2_geo_spherical for
#        spherical polar coordinate systems.
# @param nPositions
#        The number of node positions on the observer.
# @param fltPositions
#        A two dimensional array that returns the 3 dimensional node locations
# @result
#        An integer represeting success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_positions.restype = A2_IK
ANOPP2.a2py_obs_get_positions.argtypes = \
   [A2_IK, A2_RK, A2_EK, A2_EK, POINTER(A2_EK), POINTER(POINTER(A2_RK))]



# -----------------------------------------------------------------------------------------
# This routine returns the theta and phi indices corresponding to a node index in an
# ANOPP2.that is either a sphere or a spherical arc.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param iNode
#        This is the node index for which the indices of theta and phi are sought.
# @param iIndex
#        This is an array of indices up to three dimensions in a spherical geometry 
#        returned by this routine.
# @result
#        An integer represeting success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_convert_node_number_to_index.restype = A2_IK
ANOPP2.a2py_obs_convert_node_number_to_index.argtypes = [A2_IK, A2_IK, POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# This routine returns the node index corresponding to theta and phi indices in an
# ANOPP2.that is either a sphere or a spherical arc.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param iIndex
#        This is an array of indices up to three dimensions in a spherical geometry 
#        returned by this routine.
# @param iNode
#        This is the node index for which the indices of theta and phi are sought.
# @result
#        An integer represeting success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_convert_node_index_to_number.restype = A2_IK
ANOPP2.a2py_obs_convert_node_index_to_number.argtypes = \
  [A2_IK, POINTER(A2_IK), POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# This routine returns the theta and phi arrays, and radius in a spherical surface or
# a spherical arc.
# -----------------------------------------------------------------------------------------
# @param intTag
#        The integer tag of the observer contained within the ANOPP2.API.
# @param fltTheta
#        This is an array of polar angles in the ANOPP2.
# @param fltPhi
#        This is an array of azimuthal angles in the ANOPP2.
# @param fltRadius
#        This is the radius of the spherical ANOPP2.
# @result
#        An integer represeting success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_spherical_geometric_info.restype = A2_IK
ANOPP2.a2py_obs_get_spherical_geometric_info.argtypes =          \
  [A2_IK, POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(A2_IK), \
  POINTER(POINTER(A2_RK)), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of acoustic pressure time history into the observer
# data structure. A tag is provided that is the observer that is receiving the noise.
# A list of prediction tags are provided for the results being inserted.  Reception
# times are also provided for each observer position.  A list of time histories for
# the acoustic pressure are given as well.  The SPLs are kept in a 3 dimensional array
# of size number of results by number of observer by number of reception times.  A
# success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param nTimes
#        This is the number of Reception Times in the Pressure-Time History array.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTimes
#        These are the segment times of the observers.  This time is recorded by the
#        observer data segment
# @param fltReceptionTimes
#        These are the reception times of the pressure time history that is being
#        inserted into the ANOPP2.API.
# @param fltAcousticPressures
#        These are the acoustic pressures.  This is a 3 dimensional array where the
#        first dimension is the number of unique predictions, second dimension is
#        number of observer points, and third is the number of reception times in the
#        pressure time history.
# @param enumTimeHistoryGroup
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is for a segment or for the entire time range.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_apth.restype = A2_IK
ANOPP2.a2py_obs_insert_apth.argtypes =                                         \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, POINTER(A2_RK), A2_IK, POINTER(A2_RK), \
    POINTER(A2_RK), A2_EK, POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of acoustic pressure time history for a node into the
# ANOPP2.API. A tag is provided that is the observer that is receiving the
# noise. A list of prediction tags are provided for the results being inserted.
# Reception times are also provided for each observer position.  A list of time
# histories for the acoustic pressures are given as well. The pressures are kept
# in a 2 dimensional array of size number of results by number of reception times. A
# success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nTimes
#        This is the number of Reception Times in the Pressure-Time History array.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTime
#        These is the segment times of the observers.  This time is recorded by the
#        ANOPP2.API.
# @param fltReceptionTimes
#        These are the reception times of the pressure time history that is being
#        inserted into the ANOPP2.API.
# @param fltAcousticPressures
#        These are the acoustic pressures.  This is a 2 dimensional array where the
#        first dimension is the number of unique predictions and the second dimension
#        is the number of reception times in the pressure time history.
# @param intNodeNumber
#        This is the node number for which the pressure time history has to
#        inserted.
# @param enumTimeHistoryGroup
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is for a segment or for the entire time range.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_apth_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_apth_by_node.argtypes =                                        \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_RK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_IK, \
    A2_EK, POINTER(A2_EK), POINTER(2*A2_LK)]


# -----------------------------------------------------------------------------------------
# This routine inserts a list of pressure sensitivities into the observer data
# structure.  A tag is provided that is the observer that is receiving the noise.  A
# list of prediction tags are provided for the results being inserted.  Segment
# times are also provided for each observer position.  The sensitivities are kept in a 
# 3-dimensional array of size number of non-zero sensitivity values by number of observers
# by number of results.  A success value is returned communicating the success of this 
# operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSegmentTimes
#        These are the segment times of the observers.  This time is recorded by the
#        observer data segment.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param enumTimeHistoryGroup
#        This is an enumerator that specifies whether the pressure sensitivity time 
#        history is for a segment or for the entire time range.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_apth_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_insert_apth_sensitivity.argtypes =                                     \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, POINTER(A2_RK), A2_IK, A2_IK, POINTER (A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), A2_EK,               \
    POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of pressure sensitivities for a node into the observer
# data structure.  A tag is provided that is the observer that is receiving the noise.
# A list of prediction tags are provided for the results being inserted.  The segment
# time is also provided for the observer position.  The sensitivities are kept in a 
# 2-dimensional array of size number of non-zero sensitivity values by number of results.
# A success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSegmentTime
#        This is the segment time of the observer.  This time is recorded by the
#        observer data segment.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param intNodeNumber
#        This is the node number for which the pressure time history has to
#        inserted.
# @param enumTimeHistoryGroup
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is for a segment or for the entire time range.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_apth_sensitivity_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_apth_sensitivity_by_node.argtypes =                      \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, A2_RK, A2_IK, A2_IK, POINTER(A2_IK),    \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), A2_IK, A2_EK, \
    POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of acoustic velocity time history into the observer
# data structure. A tag is provided that is the observer that is receiving the noise.
# A list of prediction tags are provided for the results being inserted.  Reception
# times are also provided for each observer position.  A list of time histories for
# the acoustic velocity are given as well.  The values are kept in a 4 dimensional array
# of size number of results by number of observer by number of reception times by 3.  A
# success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param nTimes
#        This is the number of Reception Times in the Pressure-Time History array.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTimes
#        These are the segment times of the observers.  This time is recorded by the
#        observer data segment
# @param fltReceptionTimes
#        These are the reception times of the velocity time history that is being
#        inserted into the ANOPP2.API.
# @param fltAcousticVelocity
#        These are the acoustic velocities.  This is a 4 dimensional array where the
#        first dimension is the number of unique predictions, second dimension is
#        number of observer points, and third is the number of reception times in the
#        velocity time history, and the fourth is 3.
# @param enumTimeHistoryGroup
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is for a segment or for the entire time range.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_avth.restype = A2_IK
ANOPP2.a2py_obs_insert_avth.argtypes =                                         \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, POINTER(A2_RK), A2_IK, POINTER(A2_RK), \
    POINTER(A2_RK), A2_EK, POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of acoustic velocity time history for a node into the
# ANOPP2.API. A tag is provided that is the observer that is receiving the
# noise. A list of prediction tags are provided for the results being inserted.
# Reception times are also provided for each observer position.  A list of time
# histories for the acoustic velocity are given as well. The velocities are kept
# in a 3 dimensional array of size number of results by number of reception times. A
# success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nTimes
#        This is the number of Reception Times in the Velocity-Time History array.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTime
#        These is the segment times of the observers.  This time is recorded by the
#        ANOPP2.API.
# @param fltReceptionTimes
#        These are the reception times of the velocity time history that is being
#        inserted into the ANOPP2.API.
# @param fltAcousticVelocity
#        These are the acoustic velocities.  This is a 3 dimensional array where the
#        first dimension is the number of unique predictions and the second dimension
#        is the number of reception times in the velocity time history, and the third
#        dimension is 3 for a vector quantity.
# @param intNodeNumber
#        This is the node number for which the acoustic velocity time history has to
#        inserted.
# @param enumTimeHistoryGroup
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is for a segment or for the entire time range.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_avth_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_avth_by_node.argtypes =                                        \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_RK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_IK, \
    A2_EK, POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of AVTH sensitivities into the observer data structure.  A
# tag is provided that is the observer that is receiving the noise.  A list of prediction
# tags are provided for the results being inserted.  Segment times are also provided for
# each observer position.  The sensitivities are kept in a 3-dimensional array of size
# number of non-zero sensitivity values by number of observers by number of results.  A
# success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSegmentTimes
#        These are the segment times of the observers.  This time is recorded by the
#        observer data segment.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param enumTimeHistoryGroup
#        This is an enumerator that specifies whether the AVTH sensitivity is for a
#        segment or for the entire time range.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_avth_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_insert_avth_sensitivity.argtypes =                                     \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, POINTER(A2_RK), A2_IK, A2_IK, POINTER (A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), A2_EK,               \
    POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of AVTH sensitivities for a node into the observer
# data structure.  A tag is provided that is the observer that is receiving the noise.
# A list of prediction tags are provided for the results being inserted.  The segment
# time is  also provided for the observer position.  The sensitivities are kept in a 
# 2-dimensional array of size number of non-zero sensitivity values by number of results.
# A success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSegmentTime
#        This is the segment time of the observer.  This time is recorded by the
#        observer data segment.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param intNodeNumber
#        This is the node number for which the AVTH sensitivity has to inserted.
# @param enumTimeHistoryGroup
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is for a segment or for the entire time range.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_avth_sensitivity_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_avth_sensitivity_by_node.argtypes =                      \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, A2_RK, A2_IK, A2_IK, POINTER(A2_IK),    \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), A2_IK, A2_EK, \
    POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of pressure gradient time history into the observer
# data structure. A tag is provided that is the observer that is receiving the noise.
# A list of prediction tags are provided for the results being inserted.  Reception
# times are also provided for each observer position.  A list of time histories for
# the pressure gradients are given as well.  The values are kept in a 4 dimensional array
# of size number of results by number of observer by number of reception times by 3.  A
# success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param nTimes
#        This is the number of Reception Times in the Gradient-Time History array.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTimes
#        These are the segment times of the observers.  This time is recorded by the
#        observer data segment
# @param fltReceptionTimes
#        These are the reception times of the pressure time history that is being
#        inserted into the ANOPP2.API.
# @param fltPressureGradient
#        These are the pressure gradient.  This is a 4 dimensional array where the
#        first dimension is the number of unique predictions, second dimension is
#        number of observer points, and third is the number of reception times in the
#        pressure gradient time history, and the last is 3 for a vector quantity.
# @param enumTimeHistoryGroup
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is for a segment or for the entire time range.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pgth.restype = A2_IK
ANOPP2.a2py_obs_insert_pgth.argtypes =                        \
   [A2_IK, A2_IK, POINTER(A2_IK), A2_IK, POINTER(A2_RK), A2_IK, \
    POINTER(A2_RK), POINTER(A2_RK), A2_EK, POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of pressure gradient time history for a node into the
# ANOPP2.API. A tag is provided that is the observer that is receiving the
# noise. A list of prediction tags are provided for the results being inserted.
# Reception times are also provided for each observer position.  A list of time
# histories for the pressure gradients are given as well. The pressures are kept
# in a 3 dimensional array of size number of results by number of reception times. A
# success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nTimes
#        This is the number of Reception Times in the Gradient-Time History array.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTime
#        These is the segment times of the observers.  This time is recorded by the
#        ANOPP2.API.
# @param fltReceptionTimes
#        These are the reception times of the pressure time history that is being
#        inserted into the ANOPP2.API.
# @param fltPressureGradient
#        These are the pressure gradient.  This is a 3 dimensional array where the
#        first dimension is the number of unique predictions and the second dimension
#        is the number of reception times in the pressure time history, and the last
#        is 3 for vector quantity.
# @param intNodeNumber
#        This is the node number for which the pressure time history has to inserted.
# @param enumTimeHistoryGroup
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is for a segment or for the entire time range.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pgth_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_pgth_by_node.argtypes =                                       \
   [A2_IK, A2_IK, POINTER(A2_IK), A2_RK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_IK, \
    A2_EK, POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of PGTH sensitivities into the observer data structure.  A
# tag is provided that is the observer that is receiving the noise.  A list of prediction
# tags are provided for the results being inserted.  Segment times are also provided for
# each observer position.  The sensitivities are kept in a 3-dimensional array of size
# number of non-zero sensitivity values by number of observers by number of results.  A
# success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSegmentTimes
#        These are the segment times of the observers.  This time is recorded by the
#        observer data segment.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param enumTimeHistoryGroup
#        This is an enumerator that specifies whether the PGTH sensitivity is for a
#        segment or for the entire time range.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pgth_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_insert_pgth_sensitivity.argtypes =                                     \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, POINTER(A2_RK), A2_IK, A2_IK, POINTER (A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), A2_EK,               \
    POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of PGTH sensitivities for a node into the observer
# data structure.  A tag is provided that is the observer that is receiving the noise.
# A list of prediction tags are provided for the results being inserted.  The segment
# time is  also provided for the observer position.  The sensitivities are kept in a 
# 2-dimensional array of size number of non-zero sensitivity values by number of results.
# A success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSegmentTime
#        This is the segment time of the observer.  This time is recorded by the
#        observer data segment.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param intNodeNumber
#        This is the node number for which the PGTH sensitivity has to inserted.
# @param enumTimeHistoryGroup
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is for a segment or for the entire time range.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pgth_sensitivity_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_pgth_sensitivity_by_node.argtypes =                      \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, A2_RK, A2_IK, A2_IK, POINTER(A2_IK),    \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), A2_IK, A2_EK, \
    POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of tone spectra into the ANOPP2.API.
# A tag is provided that is the observer that is receiving the noise. A list of
# prediction tags are provided for the results being inserted.  Segment times are
# also provided for each observer position.  A list of frequencies for the tonal
# spectra are given as well.  The SPLs are kept in a 3 dimensional array of size
# number of results by number of observer nodes by number of frequencies.  A success
# value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param nFreqs
#        This is the number of Frequencies in tone spectra.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTimes
#        These are the segment times of the observer nodes.  These times are recorded
#        by the observer data segment.
# @param fltFrequencies
#        These are the frequencies of the pure tone spectrum that is being inserted into 
#        the observer data structure.
# @param fltTones
#        These are the pure tone levels.  This is a 3 dimensional array where the first
#        dimension is the number of unique predictions, second dimension is number of
#        observer nodes, and third is the number of tones in the spectra.
# @param fltPhase
#        These are the phase values of the pure tones.  The size and shape of this array
#        is the same as the tones array.
# @param enumNoiseLevelTypes
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is absolute levels or change in levels.  This is of size equal to the number
#        of results.
# @param blnIncludesFlightEffects
#        This flag communicates whether the input being provided to this routine
#        includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pts.restype = A2_IK
ANOPP2.a2py_obs_insert_pts.argtypes =                                          \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, POINTER(A2_RK), A2_IK, POINTER(A2_RK), \
    POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of tone spectra into the ANOPP2.API for a
# node. A tag is provided that is the observer that is receiving the noise. A list of
# prediction tags are provided for the results being inserted.  Reception times are
# also provided for the observer position.  A list of frequencies for the tonal
# spectra are given as well.  The SPLs are kept in a 2 dimensional array of size
# number of results by number of frequencies.  A success value is returned communicating
# the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nFreqs
#        This is the number of Frequencies in tone spectra.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTime
#        Thiis is the segment time at which the tones have to be inserted.  This time
#        is recorded by the ANOPP2.API.
# @param fltFrequencies
#        These are the frequencies of the pure tone spectrum that is being inserted into 
#        the observer data structure.
# @param fltTones
#        These are the pure tone levels.  This is a 2 dimensional array where the
#        first dimension is the number of unique predictions and the second is the
#        number of tones in the spectra.
# @param fltPhase
#        These are the phase values of the pure tones.  The size and shape of this array
#        is the same as the tones array.
# @param intNodeNumber
#        This is the node number for which the pure tone spectrum are to be inserted
# @param enumNoiseLevelTypes
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is absolute levels or change in levels.  This is of size equal to the number
#        of results.
# @param blnIncludesFlightEffects
#        This flag communicates whether the input being provided to this routine
#        includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pts_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_pts_by_node.argtypes =                                  \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_RK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), \
    POINTER(A2_RK), A2_IK, POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of PTS sensitivities into the observer data structure.  A
# tag is provided that is the observer that is receiving the noise.  A list of prediction
# tags are provided for the results being inserted.  Segment times are also provided for
# each observer position.  The sensitivities are kept in a 3-dimensional array of size
# number of non-zero sensitivity values by number of observer by number of results.  A
# success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSegmentTimes
#        These are the segment times of the observers.  This time is recorded by the
#        observer data segment.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pts_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_insert_pts_sensitivity.argtypes =                                      \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, POINTER(A2_RK), A2_IK, A2_IK, POINTER (A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_EK),      \
    POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of PTS sensitivities for a node into the observer data
# structure.  A tag is provided that is the observer that is receiving the noise.  A list
# of prediction tags are provided for the results being inserted.  The segment time is
# also provided for the observer position.  The sensitivities are kept in a 2-dimensional
# array of size number of non-zero sensitivity values by number of results.  A success
# value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSegmentTime
#        This is the segment time of the observer.  This time is recorded by the
#        observer data segment.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param intNodeNumber
#        This is the node number for which the pressure time history has to inserted.
# @param enumNoiseLevelTypes
#        This is an enumerator that specifies whether the pressure sensitivity time 
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pts_sensitivity_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_pts_sensitivity_by_node.argtypes =                    \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, A2_RK, A2_IK, A2_IK, POINTER(A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), A2_IK,     \
    POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of narrow band spectra into the ANOPP2.API.
# A tag is provided that is the observer that is receiving the noise. A list of
# prediction tags are provided for the results being inserted.  Segment times are
# also provided for each observer position.  A list of frequencies for the narrowband
# spectra are given as well.  The SPLs are kept in a 3 dimensional array of size
# number of results by number of observer by number of frequencies.  A success value
# is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param nFreqs
#        This is the number of Frequencies in Narrow Band spectra.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTimes
#        These are the segment times of the observers.  These times are recorded by the
#        observer data segment
# @param fltFrequencies
#        These are the frequencies of the SPL that is being inserted into the observer
#        data structure.
# @param fltNarrowband
#        These are the narrow band spectra.  This is a 3 dimensional array where
#        the first dimension is the number of unique predictions, second dimension is
#        number of observer points, and third is the number of bins in the spectra.
# @param fltPhase
#        These are the phase values of the narrow band.  The size and shape of this
#        array is the same as the tones array.
# @param enumNoiseLevelTypes
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is absolute levels or change in levels.  This is of size equal to the number
#        of results.
# @param blnIncludesFlightEffects
#        This flag communicates whether the input being provided to this routine
#        includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_nbs.restype = A2_IK
ANOPP2.a2py_obs_insert_nbs.argtypes =                         \
   [A2_IK, A2_IK, POINTER(A2_IK), A2_IK, POINTER(A2_RK), A2_IK, \
    POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of narrow band spectra into the ANOPP2.API
# for a node. A tag is provided that is the observer that is receiving the noise. A list
# of prediction tags are provided for the results being inserted.  Reception times are
# also provided for the observer position.  A list of frequencies for the
# spectra are given as well.  The SPLs are kept in a 2 dimensional array of size
# number of results by number of frequencies.  A success value is returned communicating
# the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nFreqs
#        This is the number of Frequencies in Narrow Band spectra.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTime
#        These is the segment times of the observers.  This time is recorded by the
#        ANOPP2.API.
# @param fltFrequencies
#        These are the frequencies of the SPL that is being inserted into the observer
#        data structure.
# @param fltNarrowband
#        These are the narrow band spectra.  This is a 2 dimensional array where the
#        first dimension is the number of unique predictions and the second is the
#        number of bins in the spectra.
# @param fltPhase
#        These are the phase values of the narrow band.  The size and shape of this
#        array is the same as the tones array.
# @param intNodeNumber
#        This is the node number for which the narrow band spectrum is to be inserted
# @param enumNoiseLevelTypes
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is absolute levels or change in levels.  This is of size equal to the number
#        of results.
# @param blnIncludesFlightEffects
#        This flag communicates whether the input being provided to this routine
#        includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_nbs_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_nbs_by_node.argtypes =                 \
   [A2_IK, A2_IK, POINTER(A2_IK), A2_RK, A2_IK, POINTER(A2_RK), \
    POINTER(A2_RK), POINTER(A2_RK), A2_IK, POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of NBS sensitivities into the observer data structure.  A
# tag is provided that is the observer that is receiving the noise.  A list of prediction
# tags are provided for the results being inserted.  Segment times are also provided for
# each observer position.  The sensitivities are kept in a 3-dimensional array of size
# number of non-zero sensitivity values by number of observer by number of results.  A
# success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSegmentTimes
#        These are the segment times of the observers.  This time is recorded by the
#        observer data segment.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_nbs_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_insert_nbs_sensitivity.argtypes =                                      \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, POINTER(A2_RK), A2_IK, A2_IK, POINTER (A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_EK),      \
    POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of NBS sensitivities for a node into the observer data
# structure.  A tag is provided that is the observer that is receiving the noise.  A list
# of prediction tags are provided for the results being inserted.  The segment time is
# also provided for the observer position.  The sensitivities are kept in a 2-dimensional
# array of size number of non-zero sensitivity values by number of results.  A success
# value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSegmentTime
#        This is the segment time of the observer.  This time is recorded by the
#        observer data segment.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param intNodeNumber
#        This is the node number for which the pressure time history has to inserted.
# @param enumNoiseLevelTypes
#        This is an enumerator that specifies whether the pressure sensitivity time 
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_nbs_sensitivity_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_nbs_sensitivity_by_node.argtypes =                    \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, A2_RK, A2_IK, A2_IK, POINTER(A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), A2_IK,     \
    POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of power spectral densities into the observer data
# structure.  A tag is provided that is the observer that is receiving the noise. A list
# of prediction tags are provided for the results being inserted.  Segment times are
# also provided for each observer position.  A list of frequencies for the
# spectra are given as well.  The SPLs are kept in a 3 dimensional array of size
# number of results by number of observer by number of frequencies.  A success value
# is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param nFreqs
#        This is the number of Frequencies in Power Spectral Densities.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTimes
#        These are the segment times of the observers.  These times are recorded by the
#        observer data segment
# @param fltFrequencies
#        These are the frequencies of the SPL that is being inserted into the observer
#        data structure.
# @param fltPowerSpectralDensity
#        These are the power spectral densities.  This is a 3 dimensional array where
#        the first dimension is the number of unique predictions, second dimension is
#        number of observer points, and third is the number of bins in the spectra.
# @param fltPhase
#        These are the phase values of the power spectrum.  The size and shape of this
#        array is the same as the tones array.
# @param enumNoiseLevelTypes
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is absolute levels or change in levels.  This is of size equal to the number
#        of results.
# @param blnIncludesFlightEffects
#        This flag communicates whether the input being provided to this routine
#        includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_psd.restype = A2_IK
ANOPP2.a2py_obs_insert_psd.argtypes =                                          \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, POINTER(A2_RK), A2_IK, POINTER(A2_RK), \
    POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of power spectral density into the ANOPP2.API
# for a node. A tag is provided that is the observer that is receiving the noise. A list
# of prediction tags are provided for the results being inserted.  Reception times are
# also provided for the observer position.  A list of frequencies for the
# spectra are given as well.  The SPLs are kept in a 2 dimensional array of size
# number of results by number of frequencies.  A success value is returned communicating
# the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nFreqs
#        This is the number of Frequencies in Power Spectral Densities.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTime
#        These is the segment times of the observers.  This time is recorded by the
#        ANOPP2.API.
# @param fltFrequencies
#        These are the frequencies of the SPL that is being inserted into the observer
#        data structure.
# @param fltPowerSpectralDensity
#        These are the power spectral density.  This is a 2 dimensional array where the
#        first dimension is the number of unique predictions and the second is the
#        number of bins in the spectra.
# @param fltPhase
#        These are the phase values of the power spectrum.  The size and shape of this
#        array is the same as the tones array.
# @param intNodeNumber
#        This is the node number for which the power spectral density is to be inserted
# @param enumNoiseLevelTypes
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is absolute levels or change in levels.  This is of size equal to the number
#        of results.
# @param blnIncludesFlightEffects
#        This flag communicates whether the input being provided to this routine
#        includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_psd_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_psd_by_node.argtypes =                                  \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_RK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), \
    POINTER(A2_RK), A2_IK, POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of PSD sensitivities into the observer data structure.  A
# tag is provided that is the observer that is receiving the noise.  A list of prediction
# tags are provided for the results being inserted.  Segment times are also provided for
# each observer position.  The sensitivities are kept in a 3-dimensional array of size
# number of non-zero sensitivity values by number of observer by number of results.  A
# success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSegmentTimes
#        These are the segment times of the observers.  This time is recorded by the
#        observer data segment.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_psd_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_insert_psd_sensitivity.argtypes =                                      \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, POINTER(A2_RK), A2_IK, A2_IK, POINTER (A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_EK),      \
    POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of PSD sensitivities for a node into the observer data
# structure.  A tag is provided that is the observer that is receiving the noise.  A list
# of prediction tags are provided for the results being inserted.  The segment time is
# also provided for the observer position.  The sensitivities are kept in a 2-dimensional
# array of size number of non-zero sensitivity values by number of results.  A success
# value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSegmentTime
#        This is the segment time of the observer.  This time is recorded by the
#        observer data segment.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param intNodeNumber
#        This is the node number for which the pressure time history has to inserted.
# @param enumNoiseLevelTypes
#        This is an enumerator that specifies whether the pressure sensitivity time 
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_psd_sensitivity_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_psd_sensitivity_by_node.argtypes =                    \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, A2_RK, A2_IK, A2_IK, POINTER(A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), A2_IK,     \
    POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
#  This routine inserts a list of proportional band spectra into the ANOPP2.API.
#  A tag is provided that is the observer that is receiving the noise.  A list of
#  prediction tags are provided for the results being inserted.  Segment times are
#  also provided for each observer position.  A list of frequencies for the proportional
#  band spectra are given as well.  The SPLs are kept in a 3 dimensional array of size
#  number of results by number of observer by number of frequencies.  A success value
#  is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param nFreqs
#        This is the number of Frequencies in the proportional band spectrum.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTimes
#        These are the segment times of the observers.  These times are recorded by the
#        observer data segment
# @param fltFrequencies
#        These are the frequencies of the SPL that is being inserted into the observer
#        data structure.
# @param fltSoundPressureLevels
#        These are the sound pressure levels.  This is a 3 dimensional array where the
#        first dimension is the number of unique predictions, second dimension is
#        number of observer points, and third is the number of bands in the SPL.
# @param fltProportionalNumber
#        This is the proportional number.  For example: 3 for 1/3 Octave.
# @param enumNoiseLevelTypes
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is absolute levels or change in levels.  This is of size equal to the number
#        of results.
# @param blnIncludesFlightEffects
#        This flag communicates whether the input being provided to this routine
#        includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pbs.restype = A2_IK
ANOPP2.a2py_obs_insert_pbs.argtypes =                                          \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, POINTER(A2_RK), A2_IK, POINTER(A2_RK), \
    POINTER(A2_RK), A2_RK, POINTER(A2_EK), POINTER(2*A2_LK)]



#------------------------------------------------------------------------------------------
#> This routine inserts a list of proportional band spectrogram into the ANOPP2.API.
#> A tag is provided that is the observer that is receiving the noise.  A list of 
#> prediction tags are provided for the results being inserted.  Segment times are
#> also provided for each observer position.  A list of frequencies and time for the 
#> proportional band spectrogram are given as well.  The SPLs are kept in a 4 dimensional
#> array of size number of results by number of observer by number of frequencies and time.
#>  A success value is returned communicating the success of this operation.
#------------------------------------------------------------------------------------------
#> @param intANOPP2.ag
#>        This is the tag associated with the observer that is getting the result.
#> @param intResultTags
#>        These are the hash numbers that are associated to the prediction results stored
#>        in the ANOPP2.API.
#> @param fltSegmentTimes
#>        These are the segment times of the observers.  These times are recorded by the
#>        observer data segment 
#> @param fltFrequencies
#>        These are the frequencies of the SPL that is being inserted into the observer
#>        data structure.
#> @param fltReceptionTimes
#>        This is an array of reception times of the spectrogram.
#> @param fltSoundPressureLevels
#>        These are the sound pressure levels.  This is a 3 dimensional array where the
#>        first dimension is the number of unique predictions, second dimension is
#>        number of observer points, and third is the number of bands in the SPL.
#> @param fltProportionalNumber
#>        This is the proportional number.  For example: 3 for 1/3 Octave.
#> @param enumNoiseLevelTypes
#>        This is an enumeration setting that tells the ANOPP2.API if this
#>        is absolute levels or change in levels.  This is of size equal to the number
#>        of results.
#> @param blnIncludesFlightEffects
#>        This flag communicates whether the input being provided to this routine 
#>        includes the Doppler frequency shift and convective amplification.
#> @result
#>        An integer representing success of the insertion.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pbsg.restype = A2_IK
ANOPP2.a2py_obs_insert_pbsg.argtypes =                                         \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, POINTER(A2_RK), A2_IK, POINTER(A2_RK), \
    A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_RK, POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of proportional band spectra into the ANOPP2.API
# for a node. A tag is provided that is the observer that is receiving the noise. A list
# of prediction tags are provided for the results being inserted.  Reception times are
# also provided for the observer position.  A list of frequencies for the proportional
# band spectra are given as well.  The SPLs are kept in a 2 dimensional array of size
# number of results by number of frequencies.  A success value is returned communicating
# the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nFreqs
#        This is the number of Frequencies in the proportional band spectrum.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTime
#        These is the segment times of the observers.  This time is recorded by the
#        ANOPP2.API.
# @param fltFrequencies
#        These are the frequencies of the SPL that is being inserted into the observer
#        data structure.
# @param fltSoundPressureLevels
#        These are the sound pressure levels.  This is a 2 dimensional array where the
#        first dimension is the number of unique predictions and the second is the
#        number of bands in the SPL.
# @param intNodeNumber
#        This is the node number for which the proportional band spectrum is to be
#        inserted.
# @param fltProportionalNumber
#        This is the proportional number.  For example: 3 for 1/3 Octave.
# @param enumNoiseLevelTypes
#        This is an enumeration setting that tells the ANOPP2.API if this
#        is absolute levels or change in levels.  This is of size equal to the number
#        of results.
# @param blnIncludesFlightEffects
#        This flag communicates whether the input being provided to this routine
#        includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pbs_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_pbs_by_node.argtypes =                                         \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_RK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_IK, \
    A2_RK, POINTER(A2_EK), POINTER(2*A2_LK)]



#------------------------------------------------------------------------------------------
#> This routine inserts a list of proportional band spectrogram for a node into the 
#> ANOPP2.API. A tag is provided that is the observer that is receiving the noise.  A 
#> list of prediction tags are provided for the results being inserted.  Segment times are
#> also provided for each observer position.  A list of frequencies and time for the 
#> proportional band spectrogram are given as well.  The SPLs are kept in a 4 dimensional
#> array of size number of results by number of observer by number of frequencies and time.
#>  A success value is returned communicating the success of this operation.
#------------------------------------------------------------------------------------------
#> @param intANOPP2.ag
#>        This is the tag associated with the observer that is getting the result.
#> @param intResultTags
#>        These are the hash numbers that are associated to the prediction results stored
#>        in the ANOPP2.API.
#> @param fltSegmentTimes
#>        These are the segment times of the observers.  These times are recorded by the
#>        observer data segment 
#> @param fltFrequencies
#>        These are the frequencies of the SPL that is being inserted into the observer
#>        data structure.
#> @param fltReceptionTimes
#>        This is an array of reception times of the spectrogram.
#> @param fltSoundPressureLevels
#>        These are the sound pressure levels.  This is a 3 dimensional array where the
#>        first dimension is the number of unique predictions, second dimension is
#>        number of observer points, and third is the number of bands in the SPL.
#> @param fltProportionalNumber
#>        This is the proportional number.  For example: 3 for 1/3 Octave.
#> @param enumNoiseLevelTypes
#>        This is an enumeration setting that tells the ANOPP2.API if this
#>        is absolute levels or change in levels.  This is of size equal to the number
#>        of results.
#> @param blnIncludesFlightEffects
#>        This flag communicates whether the input being provided to this routine 
#>        includes the Doppler frequency shift and convective amplification.
#> @result
#>        An integer representing success of the insertion.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pbsg_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_pbsg_by_node.argtypes =                                        \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_RK, A2_IK, POINTER(A2_RK), A2_IK, POINTER(A2_RK), \
   POINTER(A2_RK), A2_IK, A2_RK, POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of PBS sensitivities into the observer data structure.  A
# tag is provided that is the observer that is receiving the noise.  A list of prediction
# tags are provided for the results being inserted.  Segment times are also provided for
# each observer position.  The sensitivities are kept in a 3-dimensional array of size
# number of non-zero sensitivity values by number of observer by number of results.  A
# success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSegmentTimes
#        These are the segment times of the observers.  This time is recorded by the
#        observer data segment.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param enumNoiseLevelTypes
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pbs_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_insert_pbs_sensitivity.argtypes =                                      \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, POINTER(A2_RK), A2_IK, A2_IK, POINTER (A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_EK),      \
    POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
#  This routine inserts a list of PBS sensitivities for a node into the observer data
#  structure.  A tag is provided that is the observer that is receiving the noise.  A list
#  of prediction tags are provided for the results being inserted.  The segment time is
#  also provided for the observer position.  The sensitivities are kept in a 2-dimensional
#  array of size number of non-zero sensitivity values by number of results.  A success
#  value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSegmentTime
#        This is the segment time of the observer.  This time is recorded by the
#        observer data segment.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param intNodeNumber
#        This is the node number for which the pressure time history has to
#        inserted.
# @param enumNoiseLevelTypes
#        This is an enumerator that specifies whether the pressure sensitivity time 
#        These are the characteristics of the results that define whether the noise
#        data is absolute or change in levels.
# @param blnIncludesFlightEffects
#        This array of flags communicates whether the input being provided to this
#        routine includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pbs_sensitivity_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_pbs_sensitivity_by_node.argtypes =                    \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, A2_RK, A2_IK, A2_IK, POINTER(A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), A2_IK,     \
    POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a OASPL and OASPLA value into the ANOPP2.API.
# A tag is provided that is the observer that is receiving the noise.  A list of
# prediction tags are provided for the results being inserted. A list of reception times
# are also provided for each observer position. The OASPL and OASPLa are kept in a 2
# dimensional array of size number of results by number of observer.  A success value is
# returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTimes
#        These are the segment times of the observers.  This time is recorded by the
#        observer data segment
# @param fltOaspl
#        These are the overall sound pressure levels.  This is a 2 dimensional array
#        where the first dimension is the number of unique predictions and the second
#        dimension is number of observer points.
# @param fltOaspla
#        These are the a-weighted overall sound pressure levels. This is a 2 dimensional
#        array where the first dimension is the number of unique predictions and the
#        second dimension is number of observer points.
# @param blnIncludesFlightEffects
#        This flag communicates whether the input being provided to this routine
#        includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_oaspl.restype = A2_IK
ANOPP2.a2py_obs_insert_oaspl.argtypes =                                \
   [A2_IK, A2_IK, POINTER(A2_IK), A2_IK, POINTER(A2_RK), POINTER(A2_RK), \
    POINTER(A2_RK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a OASPL and OASPLa that pertains to a single observer node into
# ANOPP2.API. A tag is provided that is the observer that is receiving the
# noise.  A list of prediction tags are provided for the results being inserted.
# Reception time is also provided for the observer position.  The OASPL and OASPLa are
# kept in a 1 dimensional array each, the size is the number of results.  A success
# value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTime
#        This is the segment times of the observers.  This time is recorded by the
#        observer data segment
# @param fltFrequencies
#        These are the frequencies of the SPL that is being inserted into the observer
#        data structure.
# @param fltOaspl
#        These are the overall sound pressure levels.  This is a 1 dimensional array
#        where the size is the number of unique predictions.
# @param fltOaspla
#        These are the a-weighted overall sound pressure levels. This is a 1 dimensional
#        array where the size is the number of unique predictions.
# @param intNodeNumber
#        This is the node number for which the acoustic pressure time history
#        has to be inserted.
# @param blnIncludesFlightEffects
#        This flag communicates whether the input being provided to this routine
#        includes the Doppler frequency shift and convective amplification.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_oaspl_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_oaspl_by_node.argtypes =                        \
   [A2_IK, A2_IK, POINTER(A2_IK), A2_RK, POINTER(A2_RK), POINTER(A2_RK), \
    A2_IK, POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a PNL and PNLT value into the ANOPP2.API.
# A tag is provided that is the observer that is receiving the noise.  A list of
# prediction tags are provided for the results being inserted. A list of reception times
# are also provided for each observer position. The PNLs and PNLTs are kept in a 2
# dimensional array of size number of results by number of observer.  A success value is
# returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTimes
#        These are the segment times of the observers.  This time is recorded by the
#        observer data segment
# @param fltPNLs
#        These are the perceived noise levels.  This is a 2 dimensional array where the
#        first dimension is the number of unique predictions and the second dimension
#        is number of observer points.
# @param fltPNLTs
#        These are the tone-corrected perceived noise levels.  This is a 2 dimensional
#        array where the first dimension is the number of unique predictions and the
#        second dimension is number of observer points.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pnl.restype = A2_IK
ANOPP2.a2py_obs_insert_pnl.argtypes = \
   [A2_IK, A2_IK, POINTER             \
     (A2_IK), A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine inserts a PNL and PNLT that pertains to a single observer node into the
# ANOPP2.API. A tag is provided that is the observer that is receiving the
# noise.  A list of prediction tags are provided for the results being inserted.
# Reception time is also provided for the observer position.  The PNLs and PNLTs are
# kept in a 1 dimensional array each, the size is the number of results.  A success
# value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltSegmentTime
#        This is the segment times of the observers.  This time is recorded by the
#        observer data segment
# @param fltFrequencies
#        These are the frequencies of the SPL that is being inserted into the observer
#        data structure.
# @param fltPNLs
#        These are the perceived noise levels.  This is a 1 dimensional array where the
#        size is the number of unique predictions.
# @param fltPNLTs
#        These are the tone-corrected perceived noise levels.  This is a 1 dimensional
#        array where the size is the number of unique predictions.
# @param intNodeNumber
#        This is the node number for which the acoustic pressure time history
#        has to be inserted.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_pnl_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_pnl_by_node.argtypes = \
   [A2_IK, A2_IK, POINTER(A2_IK), A2_RK, POINTER(A2_RK), POINTER(A2_RK), A2_IK]



# -----------------------------------------------------------------------------------------
# This routine inserts an EPNL value into the ANOPP2.API. A tag is provided that is
# the observer that is receiving the noise.  A list of prediction tags are provided for
# the results being inserted. The EPNLs are kept in a 2 dimensional array of size number
# of results by number of observer.  A success value is returned communicating the
# success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltEPNLs
#        These are the effective perceived noise levels.  This is a 2 dimensional array
#        where the first dimension is the number of unique predictions and the second
#        dimension is number of observer points.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_epnl.restype = A2_IK
ANOPP2.a2py_obs_insert_epnl.argtypes = \
   [A2_IK, A2_IK, POINTER(A2_IK), A2_IK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine inserts an EPNL that pertains to a single observer node into the
# ANOPP2.API. A tag is provided that is the observer that is receiving the
# noise.  A list of prediction tags are provided for the results being inserted.
# The EPNLs are kept in a 1 dimensional array each, the size is the number of results.
# A success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltFrequencies
#        These are the frequencies of the SPL that is being inserted into the observer
#        data structure.
# @param fltEPNLs
#        These are the effective perceived noise levels.  This is a 1 dimensional array
#        where the size is the number of unique predictions.
# @param intNodeNumber
#        This is the node number for which the acoustic pressure time history
#        has to be inserted.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_epnl_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_epnl_by_node.argtypes = \
   [A2_IK, A2_IK, POINTER(A2_IK), POINTER(A2_RK), A2_IK]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of EPNL sensitivities into the observer data structure.  A
# tag is provided that is the observer that is receiving the noise.  A list of prediction
# tags are provided for the results being inserted.  The sensitivities are kept in a
# 3-dimensional array of size number of non-zero sensitivity values by number of nodes by
# number of results.  A success value is returned communicating the success of this
# operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_epnl_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_insert_epnl_sensitivity.argtypes =                     \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, A2_IK, A2_IK, POINTER (A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
#  This routine inserts a list of EPNL sensitivities for a node into the observer data
#  structure.  A tag is provided that is the observer that is receiving the noise.  A list
#  of prediction tags are provided for the results being inserted.  The sensitivities are
#  kept in a 2-dimensional array of size number of non-zero sensitivity values by number
#  of results.  A success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param intNodeNumber
#        This is the node number for which the pressure time history has to
#        inserted.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_epnl_sensitivity_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_epnl_sensitivity_by_node.argtypes =            \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, A2_IK, A2_IK, POINTER(A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), A2_IK]



# -----------------------------------------------------------------------------------------
# This routine inserts an SEL value into the ANOPP2.API. A tag is provided that is
# the observer that is receiving the noise.  A list of prediction tags are provided for
# the results being inserted. The SELs are kept in a 2 dimensional array of size number
# of results by number of observer.  A success value is returned communicating the
# success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param fltSELs
#        These are the sound exposure levels.  This is a 2 dimensional array
#        where the first dimension is the number of unique predictions and the second
#        dimension is number of observer points.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_sel.restype = A2_IK
ANOPP2.a2py_obs_insert_sel.argtypes = \
   [A2_IK, A2_IK, POINTER(A2_IK), A2_IK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine inserts an SEL that pertains to a single observer node into the
# ANOPP2.API. A tag is provided that is the observer that is receiving the
# noise.  A list of prediction tags are provided for the results being inserted.
# The SELs are kept in a 1 dimensional array each, the size is the number of results.
# A success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param fltFrequencies
#        These are the frequencies of the SPL that is being inserted into the observer
#        data structure.
# @param fltSELs
#        These are the sound exposure levels.  This is a 1 dimensional array
#        where the size is the number of unique predictions.
# @param intNodeNumber
#        This is the node number for which the acoustic pressure time history
#        has to be inserted.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_sel_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_sel_by_node.argtypes = \
  [A2_IK, A2_IK, POINTER(A2_IK), POINTER(A2_RK), A2_IK]



# -----------------------------------------------------------------------------------------
# This routine inserts a list of SEL sensitivities into the observer data structure.  A
# tag is provided that is the observer that is receiving the noise.  A list of prediction
# tags are provided for the results being inserted.  The sensitivities are kept in a
# 3-dimensional array of size number of non-zero sensitivity values by number of nodes by
# number of results.  A success value is returned communicating the success of this
# operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_sel_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_insert_sel_sensitivity.argtypes =                      \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, A2_IK, A2_IK, POINTER (A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
#  This routine inserts a list of SEL sensitivities for a node into the observer data
#  structure.  A tag is provided that is the observer that is receiving the noise.  A list
#  of prediction tags are provided for the results being inserted.  The sensitivities are
#  kept in a 2-dimensional array of size number of non-zero sensitivity values by number
#  of results.  A success value is returned communicating the success of this operation.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the observer that is getting the result.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the hash numbers that are associated to the prediction results stored
#        in the ANOPP2.API.
# @param nNodes
#        This is the number of nodes in the ANOPP2.Data Structure.
# @param nRows
#        This is the number of rows.
# @param nColumns
#        This is the number of columns.
# @param nNonZeroValues
#        Array for holding the number of non-zero sensitivity values.
# @param fltNonZeroValues
#        These are the nonzero values passed.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param intSensitivityDimensions
#        An array of integers that are the dimensions of the sensitivity matrix.
# @param intNodeNumber
#        This is the node number for which the pressure time history has to
#        inserted.
# @result
#        An integer representing success of the insertion.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_insert_sel_sensitivity_by_node.restype = A2_IK
ANOPP2.a2py_obs_insert_sel_sensitivity_by_node.argtypes =             \
   [A2_IK, A2_IK, POINTER (A2_IK), A2_IK, A2_IK, A2_IK, POINTER(A2_IK), \
    POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), A2_IK]



# -----------------------------------------------------------------------------------------
# This routine returns a pointer to a 2-dimensional array of integers.  The integer
# array first dimension is sized by the number of cells in the geometry.  The second
# dimension is sized by the maximum number of nodes in a given cell (also returned by
# this function).  Each row of this array contains first the number of nodes in the
# cell, for example 3 for a tetrahedral, etc.  After the first, is the node number of a
# given cell.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#       This is the observer tag provided to this routine.
# @param Cells
#       This is the number of cells in the observer geometry.
# @param intMaximumNodesPerCell
#       This is the maximum number of nodes in any given cell.
# @param intConnectivity
#       This is a pointer to a 2-dimensional array that will contain the connectivity.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_connectivity.restype = A2_IK
ANOPP2.a2py_obs_get_connectivity.argtypes = \
  [A2_IK, POINTER(A2_IK), POINTER(A2_IK), POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This routine returns the name of a given result associated with a tag.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer in the API.
# @param intResultTag
#        This is the tag of the result at the observer in the API.
# @param nChars
#        The length of the result name string.
# @param strResultName
#        This is the name of the result.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_result_name.restype = A2_IK
ANOPP2.a2py_obs_get_result_name.argtypes = [A2_IK, A2_IK, A2_IK, POINTER(A2_CK)]



# -----------------------------------------------------------------------------------------
# This routine returns the tag of a given result associated with an index.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer in the API.
# @param intResultIndex
#        This is the index of the result at the observer in the API.
# @param intResultTag
#        This is the tag of the result.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_result_tag.restype = A2_IK
ANOPP2.a2py_obs_get_result_tag.argtypes = [A2_IK, A2_IK, POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# > This routine returns the includes Flight Effects logical array of a given result
# > associated with the input tag.
# -----------------------------------------------------------------------------------------
# > @param intANOPP2.ag
# >        This is the tag of the observer in the API.
# > @param intResultTag
# >        This is the tag of the result at the observer in the API.
# > @param blnIncludesFlightEffects
# >        This is the size 2 boolean array that indicates whether the result includes
# >        Flight Effects, with the first index being Doppler shift, and the second
# >        index being convective amplification.
# > @result
# >        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_includes_flight_effects.restype = A2_IK
ANOPP2.a2py_obs_get_includes_flight_effects.argtypes = [A2_IK, A2_IK, POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# > This routine returns the noise level type enumerator of a given result associated
# > with the input tag.
# -----------------------------------------------------------------------------------------
# > @param intANOPP2.ag
# >        This is the tag of the observer in the API.
# > @param intResultTag
# >        This is the tag of the result at the observer in the API.
# > @param enumNoiseLevelType
# >        This is the enumerator of the noise level type.
# > @result
# >        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_noise_level_type.restype = A2_IK
ANOPP2.a2py_obs_get_noise_level_type.argtypes = [A2_IK, A2_IK, POINTER(A2_EK)]



# -----------------------------------------------------------------------------------------
# > This routine gets the Coordinate System in the specified ANOPP2.and Result.
# ---------------------------------------------------------------------------------------
# > @param intANOPP2.ag
# >        This is the tag of the observer in the API.
# > @param intResultTag
# >        This is the tag of the result at the observer in the API.
# > @param enumCoordinateSystem
# >        This is the enumerator of the noise level type.
# > @result
# >        An integer representing success of this operation.
# ---------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_coordinate_system.restype = A2_IK
ANOPP2.a2py_obs_get_coordinate_system.argtypes = [A2_IK, A2_IK, POINTER(A2_EK)]



#----------------------------------------------------------------------------------------
#> This function gets the Proportional Number attached to the ANOPP2.Result.
#----------------------------------------------------------------------------------------
#> @param this
#>        The observer data object.
#> @param dmy_intTag
#>        This is the tag of the result that the data should be retrieved from.
#> @param dmy_fltProportionalNumber
#>        This is the proportional number that will be retrieved.
#> @result
#>        An integer determining success of this operation.
#----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_proportional_number.restype = A2_IK
ANOPP2.a2py_obs_get_proportional_number.argtypes = [A2_IK, A2_IK, POINTER(A2_RK)]



# ---------------------------------------------------------------------------------------
# This function returns an array of segment times for a given node and result.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer being accessed.
# @param intResultTag
#        This ist he tag o the result being accessed.
# @param iNode
#        This is the node number being accessed.
# @param nSegmentTimes
#        This is the number of segment times that are in the fltSegmentTime array.
# @param fltSegmentTime
#        This is a pointer to an array of segment times.
# @result
#        An integer representing success of this operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_segment_time.restype = A2_IK
ANOPP2.a2py_obs_get_segment_time.argtypes = \
   [A2_IK, A2_IK, A2_IK, POINTER(A2_IK), POINTER(POINTER(A2_RK))]



# -----------------------------------------------------------------------------------------
# This routine returns the acoustic pressure time history at a particular index in time
# for an observer result.  Pointers are returned that are associated to the time
# and pressure values.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param iTime
#        This is the index in time desired by the user.  If the complete time history
#        is desired, this is set to 0.
# @param nReceptionTimes
#        This is the size of the time and pressure arrays that are returned.
# @param fltTime
#        This is a pointer to the time of the acoustic pressure time history.
# @param fltAcousticPressure
#        This is a pointer to pressure values in the ANOPP2.API
# @param enumNoiseLevelType
#        This is the noise level type of the acoustic pressure time history.  This
#        enumeration is set to one of the available options for noise level type.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @param blnIncludesFlightEffects
#        These flags communicate the flight effects of the spectrum.  The first is
#        for including Doppler frequency shift and the second is for convective
#        amplification.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_apth.restype = A2_IK
ANOPP2.a2py_obs_get_apth.argtypes =                                     \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(POINTER(A2_RK)), \
    POINTER(POINTER(A2_RK)), POINTER(A2_EK), POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine returns the pressure sensitivity time history at a particular index in
# time for an observer result.  Pointers are returned that are associated to the time
# and pressure values.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param iTime
#        This is the index in time desired by the user.  If the complete time history
#        is desired, this is set to 0.
# @param nReceptionTimes
#        This is the size of the time and pressure arrays that are returned.
# @param fltTime
#        This is a pointer to the time of the acoustic pressure time history.
# @param fltApthSensitivity
#        This is a pointer to pressure sensitivity values in the ANOPP2.API
# @param nSensitivityDimensions
#        This is the number of values in the sensitivity dimensions array.
# @param intSensitivityDimensions
#        This is a pointer to an array of integers that are the dimensions of the
#        sensitivity matrix.
# @param enumNoiseLevelType
#        This is the noise level type of the acoustic pressure time history.  This
#        enumeration is set to one of the available options for noise level type.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based on.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @param blnIncludesFlightEffects
#        These flags communicate the flight effects of the spectrum.  The first is
#        for including Doppler frequency shift and the second is for convective
#        amplification.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_apth_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_get_apth_sensitivity.argtypes =                              \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(A2_IK),               \
    POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK)), \
    POINTER(A2_IK), POINTER(POINTER(A2_IK)), POINTER(A2_EK), POINTER(A2_EK),   \
    POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine returns the acoustic velocity time history at a particular index in time
# for an observer result.  Pointers are returned that are associated to the time
# and acoustic velocity values.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param iTime
#        This is the index in time desired by the user.  If the complete time history
#        is desired, this is set to 0.
# @param nReceptionTimes
#        This is the size of the time and velocity arrays that are returned.
# @param fltTime
#        This is a pointer to the time of the acoustic velocity time history.
# @param fltAcousticVelocity
#        This is a pointer to acoustic velocity values in the observer API.
#        The first dimension is returned as nReceptionTimes and the second is 3 for a
#        vector quantity.
# @param enumNoiseLevelType
#        This is the noise level type of the acoustic pressure time history.  This
#        enumeration is set to one of the available options for noise level type.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @param blnIncludesFlightEffects
#        These flags communicate the flight effects of the spectrum.  The first is
#        for including Doppler frequency shift and the second is for convective
#        amplification.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_avth.restype = A2_IK
ANOPP2.a2py_obs_get_avth.argtypes =                                     \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(POINTER(A2_RK)), \
    POINTER(POINTER(A2_RK)), POINTER(A2_EK), POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the AVTH sensitivity time history at a particular index in
#  time for an observer result.  Pointers are returned that are associated to the non-
#  zero sensitivity values and their row and column indices.
# -----------------------------------------------------------------------------------------
#  @param intANOPP2.ag
#         This is the tag of the observer that is being accessed.
#  @param intResultTag
#         This is the tag for the result desired by the user.
#  @param iNode
#         This is the node index of the metric being returned.
#  @param iTime
#         This is the index in time desired by the user.  If the complete time history
#         is desired, this is set to 0.
#  @param nReceptionTimes
#         This is the number of reception times (the number of rows in the COO matrix).
#  @param nNonZeroValues
#         This is the number of non-zero values.
#  @param fltNonZeroValues
#         These are the nonzero values of the AVTH sensitivity matrix.
#  @param intRowIndices
#         These are the row indices of the non-zero elements of the sensitivity matrix.
#  @param intColumnIndices
#         These are the column indices of the non-zero elements.
#  @param nSensitivityDimensions
#         This is the number of values in the sensitivity dimensions array.
#  @param intSensitivityDimensions
#         This is a pointer to an array of integers that are the dimensions of the
#         sensitivity matrix.
#  @param enumNoiseLevelType
#         This is the noise level type of the AVTH sensitivity.  This enumeration is set
#         to one of the available options for noise level type.
#  @param enumCoordinateSystem
#         This is the coordinate system on which the result is based on.
#         Choices of this enumerator are:
#           a2_obs_aircraft_body: Aircraft body coordinate system
#           a2_obs_wind         : Wind coordinate system
#           a2_obs_horizon_fixed: Horizon fixed coordinate system
#  @param blnIncludesFlightEffects
#         These flags communicate the flight effects of the spectrum.  The first is
#         for including Doppler frequency shift and the second is for convective
#         amplification.
#  @result
#         An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_avth_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_get_avth_sensitivity.argtypes =                              \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(A2_IK),               \
    POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK)), \
    POINTER(A2_IK), POINTER(POINTER(A2_IK)), POINTER(A2_EK), POINTER(A2_EK),   \
    POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the pressure gradient time history at a particular index in time
#  for an observer result.  Pointers are returned that are associated to the time
#  and pressure gradient values.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param iTime
#        This is the index in time desired by the user.  If the complete time history
#        is desired, this is set to 0.
# @param nReceptionTimes
#        This is the size of the time and velocity arrays that are returned.
# @param fltTime
#        This is a pointer to the time of the pressure gradient time history.
# @param fltPressureGradient
#        This is a pointer to pressure gradient values in the ANOPP2.API
#        The first dimension is returned as nReceptionTimes and the second is 3 for a
#        vector quantity.
# @param enumNoiseLevelType
#        This is the noise level type of the acoustic pressure time history.  This
#        enumeration is set to one of the available options for noise level type.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @param blnIncludesFlightEffects
#        These flags communicate the flight effects of the spectrum.  The first is
#        for including Doppler frequency shift and the second is for convective
#        amplification.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_pgth.restype = A2_IK
ANOPP2.a2py_obs_get_pgth.argtypes =                                     \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(POINTER(A2_RK)), \
    POINTER(POINTER(A2_RK)), POINTER(A2_EK), POINTER(A2_EK), POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the PGTH sensitivity time history at a particular index in
#  time for an observer result.  Pointers are returned that are associated to the non-
#  zero sensitivity values and their row and column indices.
# -----------------------------------------------------------------------------------------
#  @param intANOPP2.ag
#         This is the tag of the observer that is being accessed.
#  @param intResultTag
#         This is the tag for the result desired by the user.
#  @param iNode
#         This is the node index of the metric being returned.
#  @param iTime
#         This is the index in time desired by the user.  If the complete time history
#         is desired, this is set to 0.
#  @param nReceptionTimes
#         This is the number of reception times (the number of rows in the COO matrix).
#  @param nNonZeroValues
#         This is the number of non-zero values.
#  @param fltNonZeroValues
#         These are the nonzero values of the PGTH sensitivity matrix.
#  @param intRowIndices
#         These are the row indices of the non-zero elements of the sensitivity matrix.
#  @param intColumnIndices
#         These are the column indices of the non-zero elements.
#  @param nSensitivityDimensions
#         This is the number of values in the sensitivity dimensions array.
#  @param intSensitivityDimensions
#         This is a pointer to an array of integers that are the dimensions of the
#         sensitivity matrix.
#  @param enumNoiseLevelType
#         This is the noise level type of the PGTH sensitivity.  This enumeration is set
#         to one of the available options for noise level type.
#  @param enumCoordinateSystem
#         This is the coordinate system on which the result is based on.
#         Choices of this enumerator are:
#           a2_obs_aircraft_body: Aircraft body coordinate system
#           a2_obs_wind         : Wind coordinate system
#           a2_obs_horizon_fixed: Horizon fixed coordinate system
#  @param blnIncludesFlightEffects
#         These flags communicate the flight effects of the spectrum.  The first is
#         for including Doppler frequency shift and the second is for convective
#         amplification.
#  @result
#         An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_pgth_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_get_pgth_sensitivity.argtypes =                              \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(A2_IK),               \
    POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK)), \
    POINTER(A2_IK), POINTER(POINTER(A2_IK)), POINTER(A2_EK), POINTER(A2_EK),   \
    POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine returns the tonal content spectrum at a particular index in time
# for an observer result.  Pointers are returned that are associated to the frequency
# and spectrum values.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param iTime
#        This is the index in time desired by the user.
# @param nFrequencies
#        This is the size of the frequencies and spectrum arrays.
# @param fltFrequencies
#        This is a pointer to the frequencies of the pure tones in the spectrum.
# @param fltAmplitude
#        This is a pointer to pure tone amplitudes in the observer data structure
# @param fltPhase
#        This is a pointer to pure tone phase in the observer data structure
# @param blnIncludesFlightEffects
#        These flags communicate the flight effects of the spectrum.  The first is
#        for including Doppler frequency shift and the second is for convective
#        amplification.
# @param enumNoiseLevelType
#        This is the noise level type of the pure tone spectrum.  This
#        enumeration is set to one of the available options for noise level type.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_pts.restype = A2_IK
ANOPP2.a2py_obs_get_pts.argtypes =                                                    \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(POINTER(A2_RK)),               \
    POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), POINTER(2*A2_LK), POINTER(A2_EK), \
    POINTER(A2_EK)]



# -----------------------------------------------------------------------------------------
# This routine returns the pure tone spectrum (NBS) sensitivity at a particular index in
# time for an observer result.  Pointers are returned that are associated to the non-zero
# sensitivity values and their row and column indices.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param iTime
#        This is the index in time desired by the user.
# @param nRows
#        This is the number of rows in the COO matrix.
# @param nNonZeroValues
#        This is the number of non-zero values.
# @param fltNonZeroValues
#        These are the nonzero values of the pressure sensitivity matrix.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity matrix.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param nSensitivityDimensions
#        This is the number of values in the sensitivity dimensions array.
# @param intSensitivityDimensions
#        This is an array of dimensions of the sensitivity matrix.  The first 
#        dimension are additive, the second multiplicative.
# @param enumNoiseLevelType
#        This is the noise level type.  This enumeration is set to one of the available
#        options for noise level type.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based on.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @param blnIncludesFlightEffects
#        These flags communicate the flight effects of the spectrum.  The first is
#        for including Doppler frequency shift and the second is for convective
#        amplification.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_pts_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_get_pts_sensitivity.argtypes =                               \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(A2_IK),               \
    POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK)), \
    POINTER(A2_IK), POINTER(POINTER(A2_IK)), POINTER(A2_EK), POINTER(A2_EK),   \
    POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine returns the narrow band spectrum at a particular index in time
# for an observer result.  Pointers are returned that are associated to the frequency
# and spectrum values.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param iTime
#        This is the index in time desired by the user.
# @param nFrequencies
#        This is the size of the frequencies and spectrum arrays.
# @param fltFrequencies
#        This is a pointer to the bin frequencies of the narrow band spectrum
#        level.
# @param fltAmplitude
#        This is a pointer to narrow band amplitudes in the observer data structure
# @param fltPhase
#        This is a pointer to narrow band phase in the observer data structure
# @param blnIncludesFlightEffects
#        These flags communicate the flight effects of the spectrum.  The first is
#        for including Doppler frequency shift and the second is for convective
#        amplification.
# @param enumNoiseLevelType
#        This is the noise level type of the narrow band spectrum.  This
#        enumeration is set to one of the available options for noise level type.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_nbs.restype = A2_IK
ANOPP2.a2py_obs_get_nbs.argtypes =                                                   \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(POINTER(A2_RK)),              \
    POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)),POINTER(2*A2_LK), POINTER(A2_EK), \
    POINTER(A2_EK)]



# -----------------------------------------------------------------------------------------
# This routine returns the narrowband spectrum (NBS) sensitivity at a particular
# index in time for an observer result.  Pointers are returned that are associated to the
# non-zero sensitivity values and their row and column indices.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param iTime
#        This is the index in time desired by the user.
# @param nRows
#        This is the number of rows in the COO matrix.
# @param nNonZeroValues
#        This is the number of non-zero values.
# @param fltNonZeroValues
#        These are the nonzero values of the pressure sensitivity matrix.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity matrix.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param nSensitivityDimensions
#        This is the number of values in the sensitivity dimensions array.
# @param intSensitivityDimensions
#        This is an array of dimensions of the sensitivity matrix.  The first 
#        dimension are additive, the second multiplicative.
# @param enumNoiseLevelType
#        This is the noise level type.  This enumeration is set to one of the available
#        options for noise level type.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based on.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @param blnIncludesFlightEffects
#        These flags communicate the flight effects of the spectrum.  The first is
#        for including Doppler frequency shift and the second is for convective
#        amplification.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_nbs_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_get_nbs_sensitivity.argtypes =                               \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(A2_IK),               \
    POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK)), \
    POINTER(A2_IK), POINTER(POINTER(A2_IK)), POINTER(A2_EK), POINTER(A2_EK),   \
    POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the power spectral density at a particular index in time
#  for an observer result.  Pointers are returned that are associated to the frequency
#  and spectrum values.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param iTime
#        This is the index in time desired by the user.
# @param nFrequencies
#        This is the size of the frequencies and spectrum arrays.
# @param fltFrequencies
#        This is a pointer to the bin frequencies of the power spectral density
#        level.
# @param fltAmplitude
#        This is a pointer to PSD amplitudes in the observer data structure
# @param fltPhase
#        This is a pointer to PSD phase in the observer data structure
# @param blnIncludesFlightEffects
#        These flags communicate the flight effects of the spectrum.  The first is
#        for including Doppler frequency shift and the second is for convective
#        amplification.
# @param enumNoiseLevelType
#        This is the noise level type of the power spectral density.  This
#        enumeration is set to one of the available options for noise level type.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_psd.restype = A2_IK
ANOPP2.a2py_obs_get_psd.argtypes =                                                    \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(POINTER(A2_RK)),               \
    POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), POINTER(2*A2_LK), POINTER(A2_EK), \
    POINTER(A2_EK)]


# -----------------------------------------------------------------------------------------
# This routine returns the power spectral density (PSD) sensitivity at a particular
# index in time for an observer result.  Pointers are returned that are associated to the
# non-zero sensitivity values and their row and column indices.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param iTime
#        This is the index in time desired by the user.
# @param nRows
#        This is the number of rows in the COO matrix.
# @param nNonZeroValues
#        This is the number of non-zero values.
# @param fltNonZeroValues
#        These are the nonzero values of the pressure sensitivity matrix.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity matrix.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param nSensitivityDimensions
#        This is the number of values in the sensitivity dimensions array.
# @param intSensitivityDimensions
#        This is an array of dimensions of the sensitivity matrix.  The first 
#        dimension are additive, the second multiplicative.
# @param enumNoiseLevelType
#        This is the noise level type.  This enumeration is set to one of the available
#        options for noise level type.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based on.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @param blnIncludesFlightEffects
#        These flags communicate the flight effects of the spectrum.  The first is
#        for including Doppler frequency shift and the second is for convective
#        amplification.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_psd_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_get_psd_sensitivity.argtypes =                               \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(A2_IK),               \
    POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK)), \
    POINTER(A2_IK), POINTER(POINTER(A2_IK)), POINTER(A2_EK), POINTER(A2_EK),   \
    POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the proportional band spectrum at a particular index in time
#  for an observer result.  Pointers are returned that are associated to the frequency
#  and sound pressure levels.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param iTime
#        This is the index in time desired by the user.
# @param nFrequencies
#        This is the size of the frequencies and spectrum arrays.
# @param fltFrequencies
#        This is a pointer to the center band frequencies of the proportional band 
#        spectrum.
# @param fltSoundPressureLevels
#        This is a pointer to sound pressure levels in the ANOPP2.API
# @param fltProportionalNumber
#        This is the proportional number of the proportional band spectrum.
# @param blnIncludesFlightEffects
#        These flags communicate the flight effects of the spectrum.  The first is
#        for including Doppler frequency shift and the second is for convective
#        amplification.
# @param enumNoiseLevelType
#        This is the noise level type of the proportional band spectrum.  This
#        enumeration is set to one of the available options for noise level type.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_pbs.restype = A2_IK
ANOPP2.a2py_obs_get_pbs.argtypes =                                           \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(POINTER(A2_RK)),      \
    POINTER(POINTER(A2_RK)), POINTER(A2_RK), POINTER(2*A2_LK), POINTER(A2_EK), \
    POINTER(A2_EK)]



#------------------------------------------------------------------------------------------
#> This routine returns the proportional band spectrogram at a particular index in time
#> for an observer result.  Pointers are returned that are associated to the frequency
#> and sound pressure levels. 
#------------------------------------------------------------------------------------------
#> @param dmy_intANOPP2.ag
#>        This is the tag of the observer that is being accessed.
#> @param dmy_intResultTag
#>        This is the tag for the result desired by the user.
#> @param dmy_iNode
#>        This is the node index of the metric being returned.
#> @param dmy_iTime
#>        This is the index in time desired by the user.
#> @param dmy_nFrequencies
#>        This is the size of the frequencies and spectrum arrays.
#> @param dmy_fltFrequencies
#>        This is a pointer to the center band frequencies of the proportional band 
#>        spectrum.
#> @param dmy_nReceptionTimes
#>        These are the number of reception times.
#> @param dmy_fltReceptionTimes
#>        This is an array of the reception times.
#> @param dmy_fltSoundPressureLevels
#>        This is a pointer to sound pressure levels in the ANOPP2.API
#> @param dmy_fltProportionalNumber
#>        This is the Proportional number of the proportional band spectrum.
#> @param dmy_blnIncludesFlightEffects
#>        These flags communicate the flight effects of the spectrum.  The first is
#>        for including Doppler frequency shift and the second is for convective
#>        amplification.
#> @param dmy_enumNoiseLevelType
#>        This is the noise level type of the proportional band spectrum.  This
#>        enumeration is set to one of the available options for noise level type.
#> @param dmy_enumCoordinateSystem
#>        This is the coordinate system on which the result is based.
#>        Choices of this enumerator are:
#>          a2_obs_aircraft_body: Aircraft body coordinate system
#>          a2_obs_wind         : Wind coordinate system
#>          a2_obs_horizon_fixed: Horizon fixed coordinate system
#> @result
#>        An integer representing success of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_pbsg.restype = A2_IK
ANOPP2.a2py_obs_get_pbsg.argtypes =                                                 \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(POINTER(A2_RK)),             \
    POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), POINTER(A2_RK), \
    POINTER(2*A2_LK), POINTER(A2_EK),  POINTER(A2_EK)]



# -----------------------------------------------------------------------------------------
# This routine returns the proportional band spectrum (PBS) sensitivity at a particular
# index in time for an observer result.  Pointers are returned that are associated to the
# non-zero sensitivity values and their row and column indices.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param iTime
#        This is the index in time desired by the user.
# @param nRows
#        This is the number of rows in the COO matrix.
# @param nNonZeroValues
#        This is the number of non-zero values.
# @param fltNonZeroValues
#        These are the nonzero values of the pressure sensitivity matrix.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity matrix.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param nSensitivityDimensions
#        This is the number of values in the sensitivity dimensions array.
# @param intSensitivityDimensions
#        This is an array of dimensions of the sensitivity matrix.  The first 
#        dimension are additive, the second multiplicative.
# @param enumNoiseLevelType
#        This is the noise level type.  This enumeration is set to one of the available
#        options for noise level type.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based on.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @param blnIncludesFlightEffects
#        These flags communicate the flight effects of the spectrum.  The first is
#        for including Doppler frequency shift and the second is for convective
#        amplification.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_pbs_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_get_pbs_sensitivity.argtypes =                               \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(A2_IK),               \
    POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK)), \
    POINTER(A2_IK), POINTER(POINTER(A2_IK)), POINTER(A2_EK), POINTER(A2_EK),   \
    POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# This routine returns the overall sound pressure level at a particular index in time
# for an observer result.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param iTime
#        This is the index in time desired by the user.
# @param fltOaspl
#        This is the value of the overall sound pressure level.
# @param fltOaspla
#        This is the value of the a-weighted overall sound pressure level.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_oaspl.restype = A2_IK
ANOPP2.a2py_obs_get_oaspl.argtypes =                                            \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), \
    POINTER(A2_EK)]



# -----------------------------------------------------------------------------------------
# This routine returns the perceived noise level at a particular index in time
# for an observer result.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param iTime
#        This is the index in time desired by the user.
# @param fltPnl
#        This is the value of the perceived noise level.
# @param fltPnlt
#        This is the value of the tone-corrected perceived noise level.
# @param fltBandFrequency
#        This is the 1/3 Octave SPL center band frequency that accrues the tone penalty
# @param fltToneCorrection
#        This is the value of the tone correction penalty
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_pnl.restype = A2_IK
ANOPP2.a2py_obs_get_pnl.argtypes =                                              \
   [A2_IK, A2_IK, A2_IK, A2_IK, POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), \
   POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), POINTER(A2_EK)]



# -----------------------------------------------------------------------------------------
# This routine returns the sound exposure level at a particular node index.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param fltSel
#        This is the value of the sound exposure level.
# @param fltD
#        This is the duration factor determined when calculating SEL.
# @param fltTimeRange
#        This is the minimum and maximum time of the SEL integration.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_sel.restype = A2_IK
ANOPP2.a2py_obs_get_sel.argtypes =                                       \
   [A2_IK, A2_IK, A2_IK, POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), \
   POINTER(POINTER(A2_RK)), POINTER(A2_EK)]



# -----------------------------------------------------------------------------------------
# This routine returns the SEL sensitivity at a particular node for an observer result.
# Pointers are returned that are associated to the non-zero sensitivity values and their
# row and column indices.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param nRows
#        This is the number of rows in the COO matrix.
# @param nNonZeroValues
#        This is the number of non-zero values.
# @param fltNonZeroValues
#        These are the nonzero values of the pressure sensitivity matrix.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity matrix.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param nSensitivityDimensions
#        This is the number of values in the sensitivity dimensions array.
# @param intSensitivityDimensions
#        This is an array of dimensions of the sensitivity matrix.  The first 
#        dimension are additive, the second multiplicative.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based on.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_sel_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_get_sel_sensitivity.argtypes =                                   \
   [A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(A2_IK), POINTER(POINTER(A2_RK)), \
    POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK)), POINTER(A2_IK),              \
    POINTER(POINTER(A2_IK)), POINTER(A2_EK)]



# -----------------------------------------------------------------------------------------
# This routine returns the effective perceived noise level at a particular node index.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param fltEpnl
#        This is the value of the effective perceived noise level.
# @param fltD
#        This is the duration factor determined when calculating EPNL.
# @param fltTimeRange
#        This is the minimum and maximum time of the EPNL integration.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_epnl.restype = A2_IK
ANOPP2.a2py_obs_get_epnl.argtypes =                                      \
   [A2_IK, A2_IK, A2_IK, POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), \
   POINTER(POINTER(A2_RK)), POINTER(A2_EK)]



# -----------------------------------------------------------------------------------------
# This routine returns the EPNL sensitivity at a particular node for an observer result.
# Pointers are returned that are associated to the non-zero sensitivity values and their
# row and column indices.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag of the observer that is being accessed.
# @param intResultTag
#        This is the tag for the result desired by the user.
# @param iNode
#        This is the node index of the metric being returned.
# @param nRows
#        This is the number of rows in the COO matrix.
# @param nNonZeroValues
#        This is the number of non-zero values.
# @param fltNonZeroValues
#        These are the nonzero values of the pressure sensitivity matrix.
# @param intRowIndices
#        These are the row indices of the non-zero elements of the sensitivity matrix.
# @param intColumnIndices
#        These are the column indices of the non-zero elements.
# @param nSensitivityDimensions
#        This is the number of values in the sensitivity dimensions array.
# @param intSensitivityDimensions
#        This is an array of dimensions of the sensitivity matrix.  The first 
#        dimension are additive, the second multiplicative.
# @param enumCoordinateSystem
#        This is the coordinate system on which the result is based on.
#        Choices of this enumerator are:
#          a2_obs_aircraft_body: Aircraft body coordinate system
#          a2_obs_wind         : Wind coordinate system
#          a2_obs_horizon_fixed: Horizon fixed coordinate system
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_epnl_sensitivity.restype = A2_IK
ANOPP2.a2py_obs_get_epnl_sensitivity.argtypes =                                  \
   [A2_IK, A2_IK, A2_IK, POINTER (A2_IK), POINTER(A2_IK), POINTER(POINTER(A2_RK)), \
    POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK)), POINTER(A2_IK),              \
    POINTER(POINTER(A2_IK)), POINTER(A2_EK)]



# -----------------------------------------------------------------------------------------
# This function combines two observer predictions together.  The inputs into this function
# are a tag for the observer to be modified, a list of tags to be combined together,
# and the name of the result.  The output of this funciton a tag of the result in the
# ANOPP2.API.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated to the observer that is being modified.
# @param nInputs
#        This is the number of Predictions whose results have to be combined.
# @param intInputTags
#        This is a list of tags that are associated to predictions that will be
#        combined.
# @param enumMetric
#        This is the metric in the input that will be combined and made available in the
#        the result observer associated by the result tag.
# @param strResultName
#        This is the name of the result of this function
# @param intResultTag
#        This is a tag returned by the ANOPP2.API that is used to associate
#        to the combination of the inputs.
# @result
#        An integer representing success of the combine operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_combine_results.restype = A2_IK
ANOPP2.a2py_obs_combine_results.argtypes = \
   [A2_IK, A2_IK, POINTER(A2_IK), A2_EK, POINTER(A2_CK), POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# This routine copies a result in the ANOPP2.API associated with an observer tag.  The
# calling program must specify an observer tag and a result tag to which is associated
# the results that are going to be copied.  This routine retuns a new result tag that
# can be used to access the copied results.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the ANOPP2.Data Structure within the
#        ANOPP2.API.  It contains the result to be copied and will contain the new
#        result once it is copied from the existing result.
# @param intExistingResultTag
#        This is a tag associated with the result within the ANOPP2.Data Structure.
# @param enumMetric
#        If this input is specified as non-zero, it will copy ONLY this metric in the
#        observer data structure.  All other noise metrics from the existing result will
#        not exist in the new result.  If all metrics are to be copied over, set this to
#        zero.  These metrics are defined in the Acoustic Analysis API.
# @param strResultName
#        This is a string representing the name of the result created by copying an
#        existing result.
# @param intNewResultTag
#        This is a tag that will be set by this routine once a result has been created
#        from a copy of the old result.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_copy_result.restype = A2_IK
ANOPP2.a2py_obs_copy_result.argtypes = \
   [A2_IK, A2_IK, A2_EK, POINTER(A2_CK), POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# This routine calculates a noise metric in an ANOPP2.API.  The metric
# that is calculated is communicated by the enumeration of the metric the user would
# like to calculate.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is a tag that is used to associated to the observer being modified.
# @param nResults
#        This is the number of results in the ANOPP2.Data Structure.
# @param intResultTags
#        These are the tags associated to the results being modified.  This is an array
#        of result tags, one for each result.
# @param enumMetric
#        This is the enumerator of the metric that the user would like to calculate.
# @param enumTimeHistoryGroup
#        This is a setting of whether the acoustic pressure will be segmented or
#        analyzed as a whole.  Options are either segment or complete.
# @result
#        This is an integer that is used to communicate success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_calc_metric.restype = A2_IK
ANOPP2.a2py_obs_calc_metric.argtypes = [A2_IK, A2_IK, POINTER(A2_IK), A2_EK, A2_EK]



# -----------------------------------------------------------------------------------------
# This function creates results within the ANOPP2.Data Structure for every waypoint
# and node.  The results are dummy results, either 0 for no noise or some low noise in
# decibels.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the observer tag associated with the observer data structure.
# @param intResulTag
#        This is a tag associated with the result that is being initialized.
# @param enumMetric
#        This is the enumeration for the metric that is being created. (see 
#        Acoustic Analysis API enumerations).
# @param enumNoiseLevelType
#        This is the enumeration for the noise level type (Absolute or Change in Level).
#        See Acoustic Analysis API for enumerations).
# @param nSegmentTimes
#        This is the number of segment times that are in the fltSegmentTime array.
# @param fltSegmentTime
#        This is an array of segment times associated with the datasets.
# @param nIndependents
#        Number of independents in the fltIndependent array.
# @param fltIndependent
#        This is the independent array either frequency or time.
# @param blnIncludesFlightEffects
#        These are 2 logicals that determine Doppler and Convective Amplification.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_create_dummy_result.restype = A2_IK
ANOPP2.a2py_obs_create_dummy_result.argtypes =                             \
  [A2_IK, A2_IK, A2_EK, A2_EK, A2_IK, POINTER(A2_RK), A2_IK, POINTER(A2_RK), \
  POINTER(2*A2_LK)]

  
  
# -----------------------------------------------------------------------------------------
# This function changes a single value in the noise at every segment and node within
# the ANOPP2.Data Structure result.  So, if say, the ANOPP2.Data Structure is a
# hemisphere defined at 10 waypoints with 20 nodes and each node/waypoint has a spectra,
# the ith value will be changed to the given values.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#       This is the tag associated with the observer being modified.
# @param intResultTag
#       This is the result tag associated with the observer result being modified.
# @param enumMetric
#       This is the metric that is being modified (see Acoustic Analysis API).
# @param iIndependent
#       This is the index of the independent value (time or frequency if applicable).
# @param nNodes
#       This is the number of nodes in the ANOPP2.
# @param nSegments
#       This is the number of segments in the ANOPP2.
# @param fltValues
#       This is a 2 dimension array of values sized number of nodes by number of 
#       segments.  The ith value of the noise will be changed to these values.
# @result
#       An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_set_noise_value.restype = A2_IK
ANOPP2.a2py_obs_set_noise_value.argtypes = \
  [A2_IK, A2_IK, A2_EK, A2_IK, A2_IK, A2_IK, POINTER(A2_RK)]
  



# -----------------------------------------------------------------------------------------
# This function filters noise metrics within an observer result.  The filter function
# is a simple top hat filter where anything lower than a certain frequency is removed
# and anything above a higher frequency is removed.  These frequencies are provided
# in the filter frequencies array.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag that is associated to the ANOPP2.Data Structure within the
#        ANOPP2.API.
# @param intResultTag
#        This is the tag associated to the result that is to be filterd.
# @param fltFilterFrequencies
#        This is an array of dimension 2 that includes the high pass and low pass
#        filter frequencies.
# @result
#        This is an integer that is used to communicate success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_filter_result.restype = A2_IK
ANOPP2.a2py_obs_filter_result.argtypes = [A2_IK, A2_IK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# > This routine sets the includes Flight Effects logical array of a given result
# > associated with the input tag.
# -----------------------------------------------------------------------------------------
# > @param intANOPP2.ag
# >        This is the tag of the observer in the API.
# > @param intResultTag
# >        This is the tag of the result at the observer in the API.
# > @param blnIncludesFlightEffects
# >        This is the size 2 boolean array that indicates whether the result includes
# >        Flight Effects, with the first index being Doppler shift, and the second
# >        index being convective amplification.
# > @result
# >        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_set_includes_flight_effects.restype = A2_IK
ANOPP2.a2py_obs_set_includes_flight_effects.argtypes = [A2_IK, A2_IK, POINTER(2*A2_LK)]



# -----------------------------------------------------------------------------------------
# > This routine sets the noise level type enumerator of a given result associated
# > with the input tag.
# -----------------------------------------------------------------------------------------
# > @param intANOPP2.ag
# >        This is the tag of the observer in the API.
# > @param intResultTag
# >        This is the tag of the result at the observer in the API.
# > @param enumNoiseLevelType
# >        This is the enumerator of the noise level type.
# > @result
# >        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_set_noise_level_type.restype = A2_IK
ANOPP2.a2py_obs_set_noise_level_type.argtypes = [A2_IK, A2_IK, POINTER(A2_EK)]



# -----------------------------------------------------------------------------------------
# > This routine sets the Coordinate System in the specified ANOPP2.and Result.
# ---------------------------------------------------------------------------------------
# > @param intANOPP2.ag
# >        This is the tag of the observer in the API.
# > @param intResultTag
# >        This is the tag of the result at the observer in the API.
# > @param enumCoordinateSystem
# >        This is the coordinate system on which the result is based on.
# >        Choices of this enumerator are:
# >          a2_obs_aircraft_body: Aircraft body coordinate system
# >          a2_obs_wind         : Wind coordinate system
# >          a2_obs_horizon_fixed: Horizon fixed coordinate system
# > @result
# >        An integer representing success of this operation.
# ---------------------------------------------------------------------------------------
ANOPP2.a2py_obs_set_coordinate_system.restype = A2_IK
ANOPP2.a2py_obs_set_coordinate_system.argtypes = [A2_IK, A2_IK, POINTER(A2_EK)]



# ---------------------------------------------------------------------------------------
# This function access an observer and tells it to export the noise.  The function
# takes in an observer tag and a metric tag and commands the observer to write out
# a given result to a file.  The file can be of a specified format and for a given
# program.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        The tag that connets the working code to the ANOPP2 API.
# @param intResultTag
#        This tag communicates what result is desired in the output file.
# @param strFileName
#        This is the file file name that the Export will go in.
# @param enumMetric
#        A tag that communicates to the observer what metric to calculate and Export on.
#        These are defined in the Acoustic Analysis API.
# @param enumFrameOfReference
#        This is an enumeration for the frame of reference of the output geometry.
#        This can be in either local or global frame of reference.
# @param enumFormat
#        This is the format of the file: options may include formatted or binary.
# @param enumProgram
#        This is the program that will eventually read the file.  This may include
#        Tecplot or NetCDF.
# @result
#        An integer that is 0 if everything has occurred as expected.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_export.restype = A2_IK
ANOPP2.a2py_obs_export.argtypes = \
   [A2_IK, A2_IK, POINTER(A2_CK), A2_EK, A2_EK, A2_EK, A2_EK]



# -----------------------------------------------------------------------------------------
# This routine takes in an observer tag associated with an observer data structure within
# the ANOPP2.API and exports all the results within associated to a provided metric.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#       This is the tag associated with the observer data structure within the Observer
#       API.
# @param strFilePrefix
#       This is the beginning of the file names that this will create.
# @param strFileSuffix
#       This is the end of the file names that this will create.
# @param enumMetric
#       This is the enumeration of the acoustic metric desired.
# @param enumFrame
#       This is the frame of reference for the geometry (i.e around aircraft
#       or as seen from ground).
# @param enumFormat
#       This is the enumeration for the formatting of the output file (formatted or
#       binary)
# @param enumProgram
#       This is the enumeration for the target program to read the file (i.e. tecplot)
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_obs_export_results.restype = A2_IK
ANOPP2.a2py_obs_export_results.argtypes = \
   [A2_IK, POINTER(A2_CK), POINTER(A2_CK), A2_EK, A2_EK, A2_EK, A2_EK]



#------------------------------------------------------------------------------------------
# This routine takes in an observer tag associated with an observer data structure within
# the ANOPP2.API and returns the tag associated with the kinematics of this observer.
#------------------------------------------------------------------------------------------
# @param dmy_intANOPP2.ag
#        This is the tag associated with the observer data structure within the Observer
#        API.
# @param dmy_intKinematicsTag
#        The tag associated with the kinematics of the observer.
# @param intSuccess
#        The success (0) or failure (non-zero) of this routine.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_kinematics_tag.restype = A2_IK
ANOPP2.a2py_obs_get_kinematics_tag.argtypes = [A2_IK, POINTER(A2_IK)]



#------------------------------------------------------------------------------------------
# This routine takes in an observer tag associated with an observer data structure within
# the ANOPP2.API and a tag associated with a kinematics and appends this kinematics to
# the kinematics in the observer.
#------------------------------------------------------------------------------------------
# @param dmy_intANOPP2.ag
#        This is the tag associated with the observer data structure within the Observer
#        API.
# @param dmy_intKinematicsTag
#        The tag associated with the kinematics of the observer.
# @param intSuccess
#        The success (0) or failure (non-zero) of this routine.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_obs_append_kinematics.restype = A2_IK
ANOPP2.a2py_obs_append_kinematics.argtypes = [A2_IK, A2_IK]



#------------------------------------------------------------------------------------------
# This routine takes in an observer tag associated with an observer data structure within
# the ANOPP2.API and a tag associated with a kinematics and prepends this kinematics to
# the kinematics in the observer.
#------------------------------------------------------------------------------------------
# @param dmy_intANOPP2.ag
#        This is the tag associated with the observer data structure within the Observer
#        API.
# @param dmy_intKinematicsTag
#        The tag associated with the kinematics of the observer.
# @param intSuccess
#        The success (0) or failure (non-zero) of this routine.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_obs_prepend_kinematics.restype = A2_IK
ANOPP2.a2py_obs_prepend_kinematics.argtypes = [A2_IK, A2_IK]



#------------------------------------------------------------------------------------------
# This routine creates a copy of an ANOPP2.including geometry, kinematics, and 
# results.
#------------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag associated with the ANOPP2.Data Structure within the 
#        ANOPP2.API.  It contains the result to be copied and will contain the new
#        result once it is copied from the existing result.
# @param intNewANOPP2.ag
#        This is a tag that will be set by this routine once a result has been created
#        from a copy of the old result.
# @result
#        An integer representing success of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_obs_copy.restype = A2_IK
ANOPP2.a2py_obs_copy.argtypes = [A2_IK, POINTER(A2_IK)]



#------------------------------------------------------------------------------------------
# Assigns an observer_range_scheduler to the observer data pointed to by the ANOPP2.Tag.
# The scheduler is a callback function accepts calls from ANOPP2 and returns the beginning
# and end of the range or a subrange of observer nodes in the function's arguments.  The
# logic for returning different ranges with successive calls is at the user's discretion
# allowing the user to implement schemes such as partitioning of nodes by MPI rank and
# asynchronous parallelization.
#------------------------------------------------------------------------------------------
# @param dmy_intTag
#        The tag of the observer item being assigned a observer scheduler.
# @param observer_range_scheduler
#        A callback method providing observer range scheduling.
# @param shedule_index
#        An object for communicating data and instructions between the user code calling
#        this method and the observer_range_scheduler callback method.
# @result
#        An integer that is returned 0 when everything occurred correctly.
#------------------------------------------------------------------------------------------
ANOPP2.obs_scheduler_prototype = CFUNCTYPE(
  None,
  POINTER(c_void_p), POINTER(A2_IK), POINTER(A2_IK)
  )

ANOPP2.a2py_obs_assign_scheduler.restype = A2_IK
ANOPP2.a2py_obs_assign_scheduler.argtypes = \
  [POINTER(A2_IK), ANOPP2.obs_scheduler_prototype, POINTER(c_void_p)]



#------------------------------------------------------------------------------------------
# Gets the range for the observer object from the observer's scheduler.
#------------------------------------------------------------------------------------------
# @param dmy_intTag
#        The tag of the observer item being queried.
# @param range_begin
#        The beginning of the range.
# @param range_end
#        The end of the range.
# @result
#        An integer that is returned 0 when everything occurred correctly.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_obs_get_range.restype = A2_IK
ANOPP2.a2py_obs_get_range.argtypes = [POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK)]



#!/usr/bin/env python
# =========================================================================================
#  Next part of this interface file contains hardcoded enumerators used by the user's
#  program.
# =========================================================================================



# -----------------------------------------------------------------------------------------
#  This enum is a list of potential observers that can be loaded from the catalog.  The
#  first is actually a 'does not exist' enumeration which tells the system to use a
#  restart file.  The other's do not require a restart file.
# -----------------------------------------------------------------------------------------

#  This enumerator is for an undefined observer (not in the catalog)
a2_obs_undefined = 1

#  This enumeration if for an observer that is at 0.0,0.0,0.0 (the origin).
a2_obs_origin = 2

#  Use this enumerator to receive an observer that is a polar arc.
a2_obs_polar_arc = 3

#  This enumeration if for a surface of observer microphone locations in the shape of a
#  hemisphere. 
a2_obs_hemisphere = 4

#  This enumeration if for a surface of observer microphone locations in the shape of a
#  sphere. 
a2_obs_full_sphere = 5

#  This is an empty point cloud. It contains no nodes and the a2f_obs_new_node function can
#  be used to insert nodes.
a2_obs_empty_point_cloud = 6

#  This enumeration if for an observer that is at 1000.0, 1000.0, 1.0 (the sideline).
a2_obs_sideline = 7



# -----------------------------------------------------------------------------------------
#  This enumeration is for the pressure time history. The pressure time history can be
#  cast on a segment or on the entire time range.
# -----------------------------------------------------------------------------------------

#  This is the enumeration for casting the pressure time history on a segment
a2_obs_segment = 1

#  This is the enumeration for casting the pressure time history on the entire time range
a2_obs_complete = 2



# -----------------------------------------------------------------------------------------
#  This enumerator group are the different types of observer within the ANOPP2.API.
#  This includes point, line, surface, point cloud, and sphere.
# -----------------------------------------------------------------------------------------

#  This is the enumerator for an observer point
a2_obs_point = 1

#  This is the enumerator for an observer line
a2_obs_line = 2

#  This is the enumerator for an observer surface
a2_obs_surface = 3

#  This is the enumerator for an observer volume
a2_obs_volume = 4

#  This is the enumerator for an observer sphere
a2_obs_sphere = 5

#  This is the enumerator for an observer point cloud
a2_obs_point_cloud = 6

#  This is the enumerator for an observer spherical arc
a2_obs_spherical_arc = 7

  
  
#------------------------------------------------------------------------------------------
# This enumerator group are the different coordinate systems that could be attached to
# an observer result.  This includes, the aircraft body coordinate system, the wind
# coordinate system, and the horizon fixed coordinate system.
#------------------------------------------------------------------------------------------

# This is the enumerator for the aircraft body coordinate system.
a2_obs_aircraft_body = 1

# This is the enumerator for the wind coordinate system.
a2_obs_wind = 2

# This is the enumerator for the horizon fixed coordinate system.
a2_obs_horizon_fixed = 3

# This is the enumerator for the pitch fixed coordinate system.
a2_obs_pitch_fixed = 4




#------------------------------------------------------------------------------------------
# VOLATILE, NOT IMPLEMENTED
#------------------------------------------------------------------------------------------

#
a2_obs_augment_sensitivity_matrix = 1

#
a2_obs_add_sensitivity_matrix = 2



#!/usr/bin/env python
# -----------------------------------------------------------------------------------------
#  This file is the interface file for the Fortran subroutines in the Atmosphere
#  Application Programming Interface (API).  This file should be copied to your local
#  directory and an "include 'ANOPP2.api.f90'" must be present in your
#  program.  See AcousticAnalysisAPIDemonstrator.cplusplus.cpp for an example including
#  using all present subroutines.
# -----------------------------------------------------------------------------------------
#   @file ANOPP2.api.h
#   @author The ANOPP2 Development Team
#   @version 1.0.0
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
#  This subroutine initializes the ANOPP2.API and should be included at the very
#  start of your program (before any other subroutines are called).
# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
#  This routine executes the unit tests in the ANOPP2.API.  The unit 
#  tests execute all the tests implemented in the ANOPP2.API.
# -----------------------------------------------------------------------------------------
# @result
#         The total number of failed asserts encountered during unit testing.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_atm_unit_test.restype = A2_IK



# -----------------------------------------------------------------------------------------
#  This routine creates an atmosphere data structure in the ANOPP2 API.  A tag is 
#  returned which is used by the calling program to access that data structure.  The
#  input into this routine is the name of a settings file.  The settings file must 
#  contain one of the known types of atmopsheres.  See Documentation for more information
#  on the format of the settings file.
# -----------------------------------------------------------------------------------------
#  @param dmy_intTag
#         This is an integer that is returned by this function.  It is used to access
#         the data structure that is created.
#  @param dmy_strConfigurationFile
#         This is the name of the input file that contains the configuration for the new
#         atmosphere. See Documentation for more information.
#  @result
#         An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_atm_create.restype = A2_IK
ANOPP2.a2py_atm_create.argtypes = [POINTER(A2_IK), POINTER(A2_CK)]



# -----------------------------------------------------------------------------------------
#  This function takes in a tag representing an atmosphere and returns true if it exists
#  in the API and false if it does not.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is the tag associated to the atmosphere that is being searched for.
#  @result
#          A bool that returns true if the observer exists and false if it does not.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_atm_exists.restype = A2_LK
ANOPP2.a2py_atm_exists.argtypes = [A2_IK]



# -----------------------------------------------------------------------------------------
#  This routine saves the atmosphere data structure in a preopened file by exporting
#  all internal data.  The file format is specific to the data structure being written
#  out.
# -----------------------------------------------------------------------------------------
#  @param dmy_intTag
#         This is an integer representation of the atmosphere to be saved
#  @param dmy_strRestartFile
#         This is the name of the file being created.
#  @result
#         An integer representing success of this function
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_atm_save.restype = A2_IK
ANOPP2.a2py_atm_save.argtypes = [A2_IK, POINTER(A2_CK)]



# -----------------------------------------------------------------------------------------
# This routine creates an atmosphere data structure in the ANOPP2 API.  A tag is
# returned which is used by the calling program to access that data structure.  The
# input into this routine is the name of a settings file.  The settings file must
# contain one of the known types of atmosphere.  See Documentation for more information
# on the format of the settings file.
# -----------------------------------------------------------------------------------------
#  @param dmy_intTag
#         This is the tag that will be returned to the user after the object has
#         been created.
#  @param dmy_enumAtmosphere
#         This is the enumeration of the atmosphere desired in the Catalog.
#  @param dmy_strRestartFile
#         This is the file name of the user supplied restart.  If the enumeration
#         provided does not exist, it will be loaded from this restart file.
#  @result
#         An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_atm_load.restype = A2_IK
ANOPP2.a2py_atm_load.argtypes = [POINTER(A2_IK), A2_EK, POINTER(A2_CK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the atmospheric properties at a given location and time. The
#  atmosphere tag is passed to this routine along with the location and time, and the
#  properties are returned.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is the tag that is associated with the atmosphere in the library.
#  @param fltPosition
#         This is the XYZ location where the properties are desired.
#  @param fltTime
#         This is the time where the properties are desired.
#  @param fltSpeedOfSound
#         This is the speed of sound at the position and time.
#  @param fltAmbientTemperature
#         This is the ambient temperature at the position and time.
#  @param fltAmbientPressure
#         This is the ambient pressure at the position and time.
#  @param fltRelativeHumidity
#         This is the percent relative humidity at the position and time.
#  @param fltAmbientDensity
#         This is the ambient density at the position and time.
#  @result
#         An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_atm_get_properties.restype = A2_IK
ANOPP2.a2py_atm_get_properties.argtypes =                                     \
   [A2_IK, POINTER(A2_RK), A2_RK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), \
    POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the speed of sound at a given location and time. The
#  atmosphere tag is passed to this routine along with the location and time, and the
#  speed of sound is returned.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is the tag that is associated with the atmosphere in the library.
#  @param fltPosition
#         This is the XYZ location where the properties are desired.
#  @param fltTime
#         This is the time where the properties are desired.
#  @param fltSpeedOfSound
#         This is the speed of sound at the position and time.
#  @result
#         An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_atm_get_speed_of_sound.restype = A2_IK
ANOPP2.a2py_atm_get_speed_of_sound.argtypes = \
   [A2_IK, POINTER(A2_RK), A2_RK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the ambient temperature at a given location and time. The
#  atmosphere tag is passed to this routine along with the location and time, and the
#  ambient temperature is returned.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is the tag that is associated with the atmosphere in the library.
#  @param fltPosition
#         This is the XYZ location where the properties are desired.
#  @param fltTime
#         This is the time where the properties are desired.
#  @param fltAmbientTemperature
#         This is the ambient temperature at the position and time.
#  @result
#         An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_atm_get_ambient_temperature.restype = A2_IK
ANOPP2.a2py_atm_get_ambient_temperature.argtypes = \
   [A2_IK, POINTER(A2_RK), A2_RK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the ambient pressure at a given location and time. The
#  atmosphere tag is passed to this routine along with the location and time, and the
#  ambient pressure is returned.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is the tag that is associated with the atmosphere in the library.
#  @param fltPosition
#         This is the XYZ location where the properties are desired.
#  @param fltTime
#         This is the time where the properties are desired.
#  @param fltAmbientPressure
#         This is the ambient pressure at the position and time.
#  @result
#         An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_atm_get_ambient_pressure.restype = A2_IK
ANOPP2.a2py_atm_get_ambient_pressure.argtypes = \
   [A2_IK, POINTER(A2_RK), A2_RK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the ambient density at a given location and time. The
#  atmosphere tag is passed to this routine along with the location and time, and the
#  ambient density is returned.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is the tag that is associated with the atmosphere in the library.
#  @param fltPosition
#         This is the XYZ location where the properties are desired.
#  @param fltTime
#         This is the time where the properties are desired.
#  @param fltAmbientDensity
#         This is the ambient density at the position and time.
#  @result
#         An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_atm_get_ambient_density.restype = A2_IK
ANOPP2.a2py_atm_get_ambient_density.argtypes = \
   [A2_IK, POINTER(A2_RK), A2_RK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the relative humidity at a given location and time. The
#  atmosphere tag is passed to this routine along with the location and time, and the
#  relative humidity is returned.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is the tag that is associated with the atmosphere in the library.
#  @param fltPosition
#         This is the XYZ location where the properties are desired.
#  @param fltTime
#         This is the time where the properties are desired.
#  @param fltRelativeHumidity
#         This is the relative humidity at the position and time.
#  @result
#         An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_atm_get_relative_humidity.restype = A2_IK
ANOPP2.a2py_atm_get_relative_humidity.argtypes = \
   [A2_IK, POINTER(A2_RK), A2_RK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the kinematic viscosity at a given location and time.  The
#  atmosphere tag is passed to this routine along with the location and time, and the
#  kinematic viscosity is returned.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is the tag that is associated with the atmosphere in the library.
#  @param fltPosition
#         This is the XYZ location where the properties are desired.
#  @param fltTime
#         This is the time where the properties are desired.
#  @param fltKinematicViscosity
#         This is the kinematic viscosity at the position and time.
#  @result
#         An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_atm_get_kinematic_viscosity.restype = A2_IK
ANOPP2.a2py_atm_get_kinematic_viscosity.argtypes = \
   [A2_IK, POINTER(A2_RK), A2_RK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
#  This routine returns the dynamic viscosity at a given location and time.  The
#  atmosphere tag is passed to this routine along with the location and time, and the
#  dynamic viscosity is returned.
# -----------------------------------------------------------------------------------------
#  @param intTag
#         This is the tag that is associated with the atmosphere in the library.
#  @param fltPosition
#         This is the XYZ location where the properties are desired.
#  @param fltTime
#         This is the time where the properties are desired.
#  @param fltDynamicViscosity
#         This is the dynamic viscosity at the position and time.
#  @result
#         An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_atm_get_dynamic_viscosity.restype = A2_IK
ANOPP2.a2py_atm_get_dynamic_viscosity.argtypes = \
   [A2_IK, POINTER(A2_RK), A2_RK, POINTER(A2_RK)]



#------------------------------------------------------------------------------------------
# This routine returns the average absorption coefficient and speed of sound for a given
# atmosphere.
#------------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        The tag of the atmosphere object being accessed
# @param fltFrequencies 
#        The frequencies where the absorption coefficient is wanted in Hertz.
# @param enumAbsorptionMethod 
#        Enumerator of the absorption method to be used to determine the noise absorbed.
# @param fltEmissionTime 
#        The emission time of the observer.
# @param fltSourceLocation
#        The source position in the atmosphere at emission time.
# @param fltObserverLocation
#        The observer at reception time.
# @param enumAcousticMetric
#        This is the enumerator for the acoustic metric to which the frequencies array
#        provided are part of.
# @param fltProportionalNumber
#        This is the proportional number of the spectrum the given frequencies array
#        is part of.
# @param fltAbsorptionCoefficient
#        The average absorption coefficients in db/meter
# @param fltSpeedOfSound
#        The average speed of sound.
# @result
#        An integer representing success of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_atm_ave_absorption_and_sos.restype = A2_IK
ANOPP2.a2py_atm_ave_absorption_and_sos.argtypes =                                    \
   [A2_IK, A2_IK, POINTER(A2_RK), A2_EK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), \
    A2_EK, POINTER(A2_RK), POINTER(POINTER(A2_RK)), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
#  This function accesses the ANOPP2.API and tells it to export the atmospheric
#  properties.  The function takes in an atmosphere tag and commands the ANOPP2.API 
#  to write out atmospheric data to a file.  The file can be of a specified format.
# -----------------------------------------------------------------------------------------
#  @param intANOPP2.ag
#         The tag that connects the application code to the ANOPP2 API.
#  @param strFileName
#         This is the name of the file to which the Export routine writes.
#  @param enumFormat
#         This is the format of the file: options may include formatted or binary.
#  @param enumProgram
#         This is the program that will eventually read the file.  This may include
#         Tecplot or NetCDF.
#  @result
#         An integer that is 0 if everything has occurred as expected.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_atm_export.restype = A2_IK
ANOPP2.a2py_atm_export.argtypes = [A2_IK, POINTER(A2_CK), A2_EK, A2_EK]



#!/usr/bin/env python
# =========================================================================================
#  Next part of this section of the interface file contains hardcoded enumerators used by
#  the user's program to communicate parameters and settings to the API.
# =========================================================================================



# -----------------------------------------------------------------------------------------
#  These enumerators are to communicate to the API, certain default objects that can be
#  loaded via the catalog.  This is optional.  A user may also specify an input
#  configuration file if they wish with more detail on the object to be constructed.
#  See User's Manual for more information.
# -----------------------------------------------------------------------------------------

# This enumerator is for an undefined atmosphere (not in the catalog)
a2_atm_undefined = 1

# conditions.
a2_atm_sea_level = 2

# the catalog.
a2_atm_standard_day = 3



# -----------------------------------------------------------------------------------------
#  These are the different types of atmospheres available in the ANOPP2.API.  These
#  include a uniform atmosphere and an altitude profile atmosphere.
# -----------------------------------------------------------------------------------------

# This enumerator is for the uniform atmosphere.
a2_atm_uniform = 1

# This enumerator is for the altitude profile.
a2_atm_altitude_profile = 2



#------------------------------------------------------------------------------------------
# These are the different methods available to determine atmospheric absorption in the
# ANOPP2.API.
#------------------------------------------------------------------------------------------

# This enumerator is for SAE ARP 866A method.
a2_atm_sae_arp_866a = 1

# This enumerator is to use ANSI S1.26-1978 method.
a2_atm_ansi_s1_26_1978 = 2

# This enumerator is to use ANSI S1.26-2014 method.
a2_atm_ansi_s1_26_2014 = 3

# This enumerator is to use 1978 ICAO Reference procedure.
a2_atm_1978_icao = 4


#!/usr/bin/env python
# -----------------------------------------------------------------------------------------
# This file is the ANOPP2.API interface file. It contains definitions for all the
# routines available in the ANOPP2.API.
# Please see the API manual provided with ANOPP2.
# -----------------------------------------------------------------------------------------
# @file ANOPP2.api.f90
# @author The ANOPP2 Development Team
# @version 1.0.0
# -----------------------------------------------------------------------------------------



# =========================================================================================
# First part of this section contains interfaces into the available ANOPP2.API
# routines.
# =========================================================================================



# -----------------------------------------------------------------------------------------
# This subroutine initializes the ANOPP2.API by setting internal variables and
# function parameters that must exist before any other call to the API can be made.
# This routine should be the first call in all programs.
# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
# This routine executes the unit tests defined for the ANOPP2.api.
# -----------------------------------------------------------------------------------------
# @result
#        An integer that contains the number of failed asserts during unit testing.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_ps_unit_test.restype = A2_IK




# -----------------------------------------------------------------------------------------
# This routine creates a propulsion data structure in the ANOPP2.API.  A tag is
# returned which is used by the calling program to access that data structure.  The
# input into this routine is the name of a settings file.  The settings file must
# contain one of the known types of propulsion systems.  See Documentation for more
# information on the format of the settings file.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is the tag that will be returned to the user after the object has
#        been created.
# @param strConfigurationFile
#        This is a file that contains the inputs required for the object to be
#        created. This is typically a namelist file.
# @result
#        An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_ps_create.restype = A2_IK
ANOPP2.a2py_ps_create.argtypes = [POINTER(A2_IK), POINTER(A2_CK)]



# ---------------------------------------------------------------------------------------
#  This function takes in a tag representing a propulsion and returns true if it exists
#  in the API and false if it does not.
# ---------------------------------------------------------------------------------------
#  @param intTag
#         This is the tag associated to the propulsion that is being searched for.
#  @result
#         A bool that is returned true if the propulsion exists and false if it does not.
# ---------------------------------------------------------------------------------------
ANOPP2.a2py_ps_exists.restype = A2_LK
ANOPP2.a2py_ps_exists.argtypes = [A2_IK]



# -----------------------------------------------------------------------------------------
# This function returns the characterstics of the propulsions system
# -----------------------------------------------------------------------------------------
# @param intTag
#        The tag that identifies ANOPP2.System.
# @param fltReferenceArea
#        The (fan) reference area used to non-dimensinoalize the data
# @param fltScaleFactor
#        The engine scale factor (area base) used to estimate effect of engine sizing
#        on noise. This input typically is determined via an airplane sizing analysis
#        (e.g., Flops). It is used to make crude corrections to area-based parmeterters
#        (e.g. , massflow, thrust, areas, etc. ).
# @result
#        An integer that is 0 if everything has occurred as expected.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_ps_get_characteristics.restype = A2_IK
ANOPP2.a2py_ps_get_characteristics.argtypes = \
  [A2_IK, POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This function returns the requested engine performance properties at a particular
# state
# -----------------------------------------------------------------------------------------
# @param intTag
#        The tag that identifies ANOPP2.System.
# @param fltAltitude
#        The altitude at which the data represents
# @param fltMachNumber
#        The Mach number at which the data represents
# @param fltThrottle
#        The throttle in percent at which the data represents
# @param nParameters
#        This is the size of the enumerator and float parameters array.
# @param enumParameters
#        An array holding the requested performance data to be returned. Please see the
# @param fltParameters
#        An array of the requested engine performance properties at the identified
#        state.
# @result
#        An integer that is 0 if everything has occurred as expected.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_ps_get_state.restype = A2_IK
ANOPP2.a2py_ps_get_state.argtypes = \
  [A2_IK, A2_RK, A2_RK, A2_RK, A2_IK, POINTER(A2_EK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This function returns the requested geometry properties.
# -----------------------------------------------------------------------------------------
# @param intTag
#        The tag that identifies ANOPP2.System.
# @param nParameters
#        This is the size of the enumerator and float parameters array.
# @param enumParameters
#        An array holding the requested geometry data to be returned. Please see the
# @param fltParameters
#        An array of the requested geometry data.
# @result
#        An integer that is 0 if everything has occurred as expected.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_ps_get_geometry.restype = A2_IK
ANOPP2.a2py_ps_get_geometry.argtypes = \
  [A2_IK, A2_IK, POINTER(A2_EK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This function sets the geometric properties of the propulsion system to the given
# values.  A list of enumerators are specified with a list of values.  The Propulsion
# System API takes those values and assigns them to the ANOPP2.System associated
# with the provided tag.
# -----------------------------------------------------------------------------------------
# @param intTag
#        The tag that identifies ANOPP2.System.
# @param nParameters
#        This is the size of the enumerator and float parameters array.
# @param enumParameters
#        An array holding the requested geometry data to be returned. Please see the
# @param fltParameters
#        An array of the requested geometry data.
# @result
#        An integer that is 0 if everything has occurred as expected.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_ps_set_geometry.restype = A2_IK
ANOPP2.a2py_ps_set_geometry.argtypes = \
  [A2_IK, A2_IK, POINTER(A2_EK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This function exports all the properties in a propulsion system to an output file.
# -----------------------------------------------------------------------------------------
# @param intTag
#        The tag that identifies ANOPP2.System.
# @param strStateFile
#        This is the export file name to contain the state information.
# @param strGeometryFile
#        This is the name of the file that will contain the geometric information.
# @param enumFormat
#        This is the format of the file: options may include formatted or binary.
# @param enumProgram
#        This is the program that will eventually read the file.  This may include
#        Tecplot or NetCDF.
# @result
#        An integer that is 0 if everything has occurred as expected.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_ps_export.restype = A2_IK
ANOPP2.a2py_ps_export.argtypes = [A2_IK, POINTER(A2_CK), POINTER(A2_CK), A2_EK, A2_EK]



#------------------------------------------------------------------------------------------
# This routine takes in a propulsion system tag associated with a propulsion system data 
# structure within the ANOPP2.System API and returns the tag associated with the 
# kinematics of this propulsion system.
#------------------------------------------------------------------------------------------
# @param intANOPP2.ystemTag
#        This is the tag associated with the ANOPP2.System data structure within the 
#        ANOPP2.System API.
# @param intKinematicsTag
#        The tag associated with the kinematics of the propulsion system.
# @param intSuccess
#        The success (0) or failure (non-zero) of this routine.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_ps_get_kinematics_tag.restype = A2_IK
ANOPP2.a2py_ps_get_kinematics_tag.argtypes = [A2_IK, POINTER(A2_IK)]



#------------------------------------------------------------------------------------------
# This routine takes in a propulsion system tag associated with a propulsion system data 
# structure within the ANOPP2.System API and a tag associated with a kinematics and 
# appends this kinematics to the kinematics in the propulsion system.
#------------------------------------------------------------------------------------------
# @param intANOPP2.ystemTag
#        This is the tag associated with the propulsion system data structure within the 
#        ANOPP2.System API.
# @param intKinematicsTag
#        The tag associated with the kinematics of the propulsion system.
# @param intSuccess
#        The success (0) or failure (non-zero) of this routine.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_ps_append_kinematics.restype = A2_IK
ANOPP2.a2py_ps_append_kinematics.argtypes = [A2_IK, A2_IK]



#------------------------------------------------------------------------------------------
# This routine takes in a propulsion system tag associated with a propulsion system data 
# structure within the ANOPP2.System API and a tag associated with a kinematics and 
# prepends this kinematics to the kinematics in the propulsion system.
#------------------------------------------------------------------------------------------
# @param intANOPP2.ystemTag
#        This is the tag associated with the propulsion system data structure within the 
#        ANOPP2.System API.
# @param intKinematicsTag
#        The tag associated with the kinematics of the propulsion system.
# @param intSuccess
#        The success (0) or failure (non-zero) of this routine.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_ps_prepend_kinematics.restype = A2_IK
ANOPP2.a2py_ps_prepend_kinematics.argtypes = [A2_IK, A2_IK]



#------------------------------------------------------------------------------------------
# This routine takes in a tag for a propulsion system and creates a deep copy of the 
# propulsion system.
#------------------------------------------------------------------------------------------
# @param dmy_intTagOfSource
#        This is a tag for a propulsion system list that is going to be copied.
# @param dmy_intTagOfCopy
#        This is a tag that will be created for the copy.
# @result
#        An integer representing success of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_ps_copy.restype = A2_IK
ANOPP2.a2py_ps_copy.argtypes = [A2_IK, POINTER(A2_IK)]



#!/usr/bin/env python
# =========================================================================================
#   Next part of this section of the interface file contains hardcoded enumerators used by
#   the user's program to communicate parameters and settings to the API.
# =========================================================================================
# =========================================================================================



# -----------------------------------------------------------------------------------------
#  These enumerations determine what type of geometric quantities are stored in the
#  ANOPP2.API.  The enumerations are listed out here and grouped by component.  The
#  components include Fan, Turbine, Core, and Jet.
# -----------------------------------------------------------------------------------------

# This is the fan face reference area
a2_ps_fan_fra = 1

# This is the fan hub diameter.
a2_ps_fan_hd = 2

# This is the fan outer diameter.
a2_ps_fan_od = 3

# This is the fan diameter ratio (Hub/Outer).
a2_ps_fan_dr = 4

# This is the fan flow area.
a2_ps_fan_fa = 5

# This is the fan design tangential tip Mach number.
a2_ps_fan_dttmn = 6

# This is the fan design helical tip Mach number.
a2_ps_fan_dhtmn = 7

# This is the fan face design Mach number.
a2_ps_fan_fdmn = 8

# This is the number of fan rotor blades.
a2_ps_fan_nb = 9

# This is the number of fan exit guide vanes.
a2_ps_fan_nv = 10

# This is the fan rotor-stator spacing in meters.
a2_ps_fan_rss = 11

# This is the ratio of the fan rotor-stator spacing to the fan
a2_ps_fan_rssr = 12

# This is the average fan blade axial length.
a2_ps_fan_abal = 13

# This is the average fan blade chord length
a2_ps_fan_abcl = 14

# This is the tip fan blade axial length.
a2_ps_fan_tbal = 15

# This is the tip fan blade chord length
a2_ps_fan_tbcl = 16

# This is the fan blade projected axial chord length
a2_ps_fan_bpacl = 17

# This is the fan blade aspect ratio
a2_ps_fan_bar = 18

# This is the design rotor rate.
a2_ps_fan_drr = 19

# This is the axial length of the inlet.
a2_ps_fan_ial = 20

# This is the axial length of the inlet treated area.
a2_ps_fan_ital = 21

# This is the axial length of the aft treated area.
a2_ps_fan_atal = 22

# This is the average inlet radius of treated region.
a2_ps_fan_aitr = 23

# This is the average duct height of aft treated region.
a2_ps_fan_atadh = 24

# This is the primary nozzle plug diameter at the throat.
a2_ps_prim_ptd = 25

# This is the primary nozzle outer diameter at throat.
a2_ps_prim_otd = 26

# This is the plug diameter at the exit.
a2_ps_prim_ped = 27

# This is the primary nozzle outer diameter at exit.
a2_ps_prim_oet = 28

# This is the distance from the core nozzle exit to the tip.
a2_ps_prim_ett = 29

# This is the ratio of the wetted perimeter with chevrons to without.
a2_ps_prim_wpr = 30

# This is the radius of curvative of the plug tip.
a2_ps_prim_ptrc = 31

# This is the height of the jet (if rectangular).
a2_ps_prim_h = 32

# This is the width of the jet (if rectangular).
a2_ps_prim_w = 33

# This is the fan nozzle inner diameter at the throat.
a2_ps_sec_itd = 34

# This is the fan nozzle outer diameter at the throat.
a2_ps_sec_otd = 35

# This is the fan nozzle inner diameter at the exit.
a2_ps_sec_ied = 36

# This is the fan nozzle outer diameter at the exit.
a2_ps_sec_oed = 37

# This is the distance from the secondary exit to the primary exit.
a2_ps_sec_dtp = 38

# This is the ratio of the wetted perimeter with chevrons to without.
a2_ps_sec_wpr = 39

# This is the combustor inlet inner radius
a2_ps_comb_iir = 40

# This is the combustor inlet outer radius
a2_ps_comb_ior = 41

# This is the combustor inlet area
a2_ps_comb_ia = 42

# This is the combustor exit area.
a2_ps_comb_ea = 43

# This is the speed of sound based on the tailpipe mean static temperature.
a2_ps_comb_tsos = 44

# This is the diameter of the tailpipe.
a2_ps_comb_td = 45

# This is the LPT stage count.
a2_ps_turb_sc = 46

# This is the number of turbine rotor blades.
a2_ps_turb_nb = 47

# This is the turbine rotor spacing.
a2_ps_turb_rs = 48

# This is the turbine rotor diameter
a2_ps_turb_rd = 49

# This is the turbine final stage tip diameter.
a2_ps_turb_fstp = 50

# This is the turbine final stage hub diameter
a2_ps_turb_fshd = 51

# This is the turbine exit area.
a2_ps_turb_ea = 52

# This is a general property.

# 53 This is the engine scale factor based on area.
a2_ps_esf = 53

# 54 This is the nozzle type
# There are 7 nozzle types available:
#> These nozzle types are defined identical to those in the TSS code.
  #> = 0 for no suppression (NOsupp)
  #> = 1 for 1-stream flow (SMPLsupp)
  #> = 2 for 2-stream separate flow (TWOsupp)
  #> = 3 for 3-stream separate flow (THRDsupp)
  #> = 4 for 2-stream inverted velocity profile (IVPsupp)
  #> = 5 for 3-stream on IVP (IVPsupp + IVPshieldsupp)
  #> = 6 for 3-stream separate flow with offset duct (OSTsupp)
a2_ps_nozt = 54

# Tertiary stream properties.
# These are the tertiary jet geometric properties.  They include diameters of throat
# and exit, etc.

# 55 This is the tertiary nozzle inner diameter at the throat.
a2_ps_ter_itd = 55

# 56 This is the tertiary nozzle outer diameter at the throat.
a2_ps_ter_otd = 56

# 57 This is the tertiary nozzle inner diameter at the exit.
a2_ps_ter_ied = 57

# 58 This is the tertiary nozzle outer diameter at the exit.
a2_ps_ter_oed = 58

# 59 This is the ratio of the wetted perimeter with chevrons to without.
a2_ps_ter_wpr = 59

# 60 This is the tertiary nozzle throat area.
# This enumerator is deprecated and replaced with a2_ps_noz_3_ta
a2_ps_ter_ta = 60

# 61 This is the primary nozzle plug diameter at the throat.
# This enumerator is temporarily assigned as a2_ps_noz_geo_1_itd. It will become
# a2_ps_noz_1_itd after ANOPP2.API refactoring
a2_ps_noz_geo_1_itd = 61

# 62 This is the primary nozzle outer diameter at throat.
# This enumerator is temporarily assigned as a2_ps_noz_geo_1_otd. It will become
# a2_ps_noz_1_otd after ANOPP2.API refactoring
a2_ps_noz_geo_1_otd = 62

# 63 This is the plug diameter at the exit.
# This enumerator is temporarily assigned as a2_ps_noz_geo_1_ied. It will become
# a2_ps_noz_1_ied after ANOPP2.API refactoring
a2_ps_noz_geo_1_ied = 63

# 64 This is the primary nozzle outer diameter at exit.
# This enumerator is temporarily assigned as a2_ps_noz_geo_1_oed. It will become
# a2_ps_noz_1_oed after ANOPP2.API refactoring
a2_ps_noz_geo_1_oed = 64

# 65 This is the distance from the core nozzle exit to the tip.
a2_ps_noz_1_l = 65

# 66 This is the ratio of the wetted perimeter with chevrons to without.
# This enumerator is temporarily assigned as a2_ps_noz_geo_1_wpr. It will become
# a2_ps_noz_1_wpr after ANOPP2.API refactoring
a2_ps_noz_geo_1_wpr = 66

# 67 This is the radius of curvature of the plug tip.
a2_ps_noz_1_trc = 67

# 68 This is the height of the jet (if rectangular).
a2_ps_noz_1_h = 68

# 69 This is the width of the jet (if rectangular).
a2_ps_noz_1_w = 69

# Fan Bypass (Secondary Jet) Properties
# These are the secondary jet geometric properties.  They include diameters of throat
# and exit, etc.

# 70 This is the fan nozzle inner diameter at the throat.
# This enumerator is temporarily assigned as a2_ps_noz_geo_2_itd. It will become
# a2_ps_noz_2_itd after ANOPP2.API refactoring
a2_ps_noz_geo_2_itd = 70

# 71 This is the fan nozzle outer diameter at the throat.
# This enumerator is temporarily assigned as a2_ps_noz_geo_2_otd. It will become
# a2_ps_noz_2_otd after ANOPP2.API refactoring
a2_ps_noz_geo_2_otd = 71

# 72 This is the fan nozzle inner diameter at the exit.
# This enumerator is temporarily assigned as a2_ps_noz_geo_2_ied. It will become
# a2_ps_noz_2_ied after ANOPP2.API refactoring
a2_ps_noz_geo_2_ied = 72

# 73 This is the fan nozzle outer diameter at the exit.
# This enumerator is temporarily assigned as a2_ps_noz_geo_2_oed. It will become
# a2_ps_noz_2_oed after ANOPP2.API refactoring
a2_ps_noz_geo_2_oed = 73

# 74 This is the distance from the secondary exit to the primary exit.
a2_ps_noz_2_l = 74
 
# 75 This is the ratio of the wetted perimeter with chevrons to without.
# This enumerator is temporarily assigned as a2_ps_noz_geo_2_wpr. It will become
# a2_ps_noz_2_wpr after ANOPP2.API refactoring
a2_ps_noz_geo_2_wpr = 75

# 76 This is the tertiary nozzle inner diameter at the throat.
# This enumerator is temporarily assigned as a2_ps_noz_geo_3_itd. It will become
# a2_ps_noz_3_itd after ANOPP2.API refactoring
a2_ps_noz_geo_3_itd = 76

# 77 This is the tertiary nozzle outer diameter at the throat.
# This enumerator is temporarily assigned as a2_ps_noz_geo_3_otd. It will become
# a2_ps_noz_3_otd after ANOPP2.API refactoring
a2_ps_noz_geo_3_otd = 77

# 78 This is the tertiary nozzle inner diameter at the exit.
# This enumerator is temporarily assigned as a2_ps_noz_geo_3_ied. It will become
# a2_ps_noz_3_ied after ANOPP2.API refactoring
a2_ps_noz_geo_3_ied = 78

# 79 This is the tertiary nozzle outer diameter at the exit.
# This enumerator is temporarily assigned as a2_ps_noz_geo_3_oed. It will become
# a2_ps_noz_3_oed after ANOPP2.API refactoring
a2_ps_noz_geo_3_oed = 79
 
# 80 This is the ratio of the wetted perimeter with chevrons to without.
# This enumerator is temporarily assigned as a2_ps_noz_geo_3_wpr. It will become
# a2_ps_noz_3_wpr after ANOPP2.API refactoring
a2_ps_noz_geo_3_wpr = 80
 
# 81 This is the tertiary nozzle throat area.
a2_ps_noz_3_ta = 81

# 82 This is the nozzle type
# There are 7 nozzle types available:
#> These nozzle types are defined identical to those in the TSS code.
  #> = 0 for no suppression (NOsupp)
  #> = 1 for 1-stream flow (SMPLsupp)
  #> = 2 for 2-stream separate flow (TWOsupp)
  #> = 3 for 3-stream separate flow (THRDsupp)
  #> = 4 for 2-stream inverted velocity profile (IVPsupp)
  #> = 5 for 3-stream on IVP (IVPsupp + IVPshieldsupp)
  #> = 6 for 3-stream separate flow with offset duct (OSTsupp)
a2_ps_noz_type = 82

# ---------------------------------------------------------------------------------------
#  These enumerators list out the supported state properties that may be available
#  within the ANOPP2.API.  Not all state properties are supported by all propulsion
#  systems.  For instance, a ANOPP2.System Data Structure created with ANOPP Engine
#  State Table will not have as many state variables as a Data Structure created with
#  an NPSS data set.  See manual for complete list.
# ---------------------------------------------------------------------------------------

# This is the gross thrust of the engine.
a2_ps_gt = 1

# This is the ram drag of the engine.
a2_ps_rd = 2

# This is the net thrust of the engine.
a2_ps_nt = 3

# This is the max net thrust of the engine.
a2_ps_mnt = 4

# This is the power code of the engine.
a2_ps_pc = 5

# This is the fuel mass flow rate
a2_ps_fmfr = 6

# This is the fan bypass ratio.
a2_ps_br = 7

# The ambient pressure.
a2_ps_amb_p = 8

# The ambient temperature.
a2_ps_amb_t = 9

# The ambient specific heat ratio
a2_ps_amb_shr = 10

# The ambient density.
a2_ps_amb_d = 11

# The ambient speed of sound.
a2_ps_amb_sos = 12

# Fan mass flow rate.
a2_ps_fan_mfr = 13

# This is the shaft speed of the fan
a2_ps_fan_ss = 14

# This is the maximum shaft speed of the fan.
a2_ps_fan_mss = 15

# This is the shaft speed fraction.
a2_ps_fan_ssf = 16

# This is the fan tip speed.
a2_ps_fan_ts = 17

# This is the fan inlet total temperature.
a2_ps_fan_itt = 18

# This is the fan exit total temperature.
a2_ps_fan_ett = 19

# This is the fan pressure ratio.
a2_ps_fan_pr = 20

# This is the fan adiabatic efficiency.
a2_ps_fan_ae = 21

# Primary nozzle total temperature
a2_ps_prim_tt = 22

# Primary nozzle total pressure
a2_ps_prim_tp = 23

# Primary nozzle pressure ratio
a2_ps_prim_pr = 24

# Primary nozzle exit ideal velocity
a2_ps_prim_iv = 25

# Primary nozzle exit area
a2_ps_prim_ea = 26

# Primary nozzle mass flow rate
a2_ps_prim_mfr = 27

# Primary nozzle specific heat ratio
a2_ps_prim_shr = 28

# Primary nozzle exit static density
a2_ps_prim_sd = 29

# Primary nozzle exit Mach number
a2_ps_prim_m = 30

# Secondary nozzle total temperature
a2_ps_sec_tt = 31

# Secondary nozzle total pressure
a2_ps_sec_tp = 32

# Secondary nozzle pressure ratio
a2_ps_sec_pr = 33

# Secondary nozzle ideal velocity
a2_ps_sec_iv = 34

# Secondary nozzle exit area
a2_ps_sec_ea = 35

# Secondary nozzle mass flow rate
a2_ps_sec_mft = 36

# Secondary nozzle specific heat ratio
a2_ps_sec_shr = 37

# Secondary nozzle exit static density
a2_ps_sec_sd = 38

# Secondary nozzle exit Mach number
a2_ps_sec_m = 39

# 40 This is the fuel flow rate.
a2_ps_comb_ffr = 40

# Combustor mass flow rate 
a2_ps_comb_mfr = 41

# Combustor inlet total pressure
a2_ps_comb_itp = 42

# Combustor inlet total temperature
a2_ps_comb_itt = 43

# Combustor exit total pressure
a2_ps_comb_etp = 44

# Combustor exit total temperature
a2_ps_comb_ett = 45

# Low pressure turbine exit total temperature
a2_ps_turb_ett = 46

# Temperature drop through turbines
a2_ps_turb_td = 47

# 48 This is the aircraft altitude
a2_ps_alt = 48

# 49 This is the aircraft Mach Number.
a2_ps_mn = 49

# 50 This is the throttle setting.
a2_ps_ts = 50

# The following are parameters related to experimental jet configurations. Though some
# of them may appear to be geometric properties, they do vary with altitude, Mach
# number, and throttle settings and therefore are state parameters.

# 51 This is the area of nozzle 1 (primary nozzle)
a2_ps_a1 = 51

# 52 This is the area of nozzle 2 (secondary nozzle)
a2_ps_a2 = 52

# 53 This is the area of nozzle 3 (tertiary nozzle)
a2_ps_a3 = 53

# 54 This is the nozzle pressure ratio of nozzle 1 (primary nozzle)
a2_ps_pr1 = 54

# 55 This is the nozzle pressure ratio of nozzle 2 (secondary nozzle)
a2_ps_pr2 = 55

# 56 This is the nozzle pressure ratio of nozzle 3 (tertiary nozzle)
a2_ps_pr3 = 56

# 57 This is the equivalent diameter (ft) (for use in JSI).
a2_ps_eqdj = 57

# 58 This is the equivalent velocity (ft/s) (for use in JSI).
a2_ps_eqvj = 58

# 59 This is the equivalent total temperature (R) (for use in JSI).
a2_ps_eqtt = 59

# 60 This is the tertiary stream total temperature.
a2_ps_ter_tt = 60

# 61 This is the tertiary stream ideally expanded velocity.
a2_ps_ter_iv = 61

# 62 This is the exhaust #1 inner diameter at the throat.
a2_ps_itd1 = 62

# 63 This is the exhaust #1 outer diameter at the throat.
a2_ps_otd1 = 63

# 64 This is the exhaust #1 inner diameter at the exit.
a2_ps_ied1 = 64

# 65 This is the exhaust #1 outer diameter at the exit.
a2_ps_oed1 = 65

# This is the exhaust #1 outer diameter at the exit.
# 66 This is the wetted perimeter ratio for exhaust #1.
a2_ps_wpr1 = 66

# 67 This is the exhaust #2 inner diameter at the throat.
a2_ps_itd2 = 67

# 68 This is the exhaust #2 outer diameter at the throat.
a2_ps_otd2 = 68

# 69 This is the exhaust #2 inner diameter at the exit.
a2_ps_ied2 = 69

# 70 This is the exhaust #2 outer diameter at the exit.
a2_ps_oed2 = 70

# 71 This is the wetted perimeter ratio for exhaust #2.
a2_ps_wpr2 = 71

# 72 This is the exhaust #3 inner diameter at the throat.
a2_ps_itd3 = 72

# 73 This is the exhaust #3 outer diameter at the throat.
a2_ps_otd3 = 73

# 74 This is the exhaust #3 inner diameter at the exit.
a2_ps_ied3 = 74

# 75 This is the exhaust #3 outer diameter at the exit.
a2_ps_oed3 = 75

# 76 This is the wetted perimeter ratio for exhaust #3.
a2_ps_wpr3 = 76

# 77 Primary nozzle total temperature
a2_ps_noz_1_tt = 77

# 78 Primary nozzle total pressure
a2_ps_noz_1_tp = 78

# 79 Primary nozzle pressure ratio.
a2_ps_noz_1_pr = 79

# 80 Primary nozzle exit ideal velocity
a2_ps_noz_1_vi = 80

# 81 Primary nozzle exit area
a2_ps_noz_1_a = 81

# 82 Primary nozzle mass flow rate
a2_ps_noz_1_mfr = 82

# 83 Primary nozzle specific heat ratio
a2_ps_noz_1_shr = 83

# 84 Primary nozzle exit static density
a2_ps_noz_1_sd = 84

# 85 Primary nozzle exit Mach number
a2_ps_noz_1_m = 85

# Fan Bypass (Secondary Jet) Properties
# These properties are for the fan (bypass fan).  They include flow variables and
# bypass fan performance.

# 86 Secondary nozzle total temperature
a2_ps_noz_2_tt = 86

# 87 Secondary nozzle total pressure
a2_ps_noz_2_tp = 87

# 88 Secondary nozzle pressure ratio
a2_ps_noz_2_pr = 88

# 89 Secondary nozzle ideal velocity
a2_ps_noz_2_vi = 89

# 90 Secondary nozzle exit area
a2_ps_noz_2_a = 90

# 91 Secondary nozzle mass flow rate
a2_ps_noz_2_mfr = 91

# 92 Secondary nozzle specific heat ratio
a2_ps_noz_2_shr = 92

# 93 Secondary nozzle exit static density
a2_ps_noz_2_sd = 93

# 94 Secondary nozzle exit Mach number
a2_ps_noz_2_m = 94

# 95 Equivalent diameter of the exhaust.
a2_ps_noz_deq = 95

# 96 Equivalent velocity of the exhaust.
a2_ps_noz_vieq = 96

# 97 Equivalent total temperature of the exhaust.
a2_ps_noz_tteq = 97

# 98 Tertiary stream total temperature
a2_ps_noz_3_tt = 98

# 99 Tertiary stream ideally expanded velocity
a2_ps_noz_3_vi = 99

# 100 Primary Nozzle inner throat diameter.
a2_ps_noz_1_itd = 100

# 101 Primary Nozzle outer throat diameter.
a2_ps_noz_1_otd = 101

# 102 Primary Nozzle inner exit diameter.
a2_ps_noz_1_ied = 102

# 103 Primary Nozzle outer exit diameter.
a2_ps_noz_1_oed = 103

# 104 Primary Nozzle wetted perimeter ratio.
a2_ps_noz_1_wpr = 104

# 105 Secondary Nozzle inner throat diameter.
a2_ps_noz_2_itd = 105

# 106 Secondary Nozzle outer throat diameter.
a2_ps_noz_2_otd = 106

# 107 Secondary Nozzle inner exit diameter.
a2_ps_noz_2_ied = 107

# 108 Secondary Nozzle outer exit diameter.
a2_ps_noz_2_oed = 108

# 109 Secondary Nozzle wetted perimeter ratio.
a2_ps_noz_2_wpr = 109

# 110 Tertiary Nozzle inner throat diameter.
a2_ps_noz_3_itd = 110

# 111 Tertiary Nozzle outer throat diameter.
a2_ps_noz_3_otd = 111

# 112 Tertiary Nozzle inner exit diameter.
a2_ps_noz_3_ied = 112

# 113 Tertiary Nozzle outer exit diameter.
a2_ps_noz_3_oed = 113

# 114 Tertiary Nozzle wetted perimeter ratio.
a2_ps_noz_3_wpr = 114

# 115 Tertiary Nozzle exit area.
a2_ps_noz_3_a = 115

# 116 Tertiary Nozzle pressure ratio.
a2_ps_noz_3_pr = 116
#!/usr/bin/env python
# -----------------------------------------------------------------------------------------
# This file is the interface file for the fortran subroutines in the ANOPP2
# Application Programming Interface (API).  This file should be copied to your local
# directory and an "#include 'ANOPP2.api.h'" must be present in your
# program.  See anyone of the demonstrators provided with this API in the Demos
# directory.  For explanation of how to call and the theory behind each function, 
# please see the API manual provided with ANOPP2.
# -----------------------------------------------------------------------------------------
# @file ANOPP2.api.h
# @author The ANOPP2 Development Team
# @version 1.0.0
# -----------------------------------------------------------------------------------------



# =========================================================================================
# First part of this section of the contains interfaces into the available ANOPP2 API 
# functions.
# =========================================================================================



# -----------------------------------------------------------------------------------------
# This function initializes the ANOPP2.API by setting internal variables and function
# parameters that must exist before any other call to the API can be made.
# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
# This routine executes the unit tests in the ANOPP2.Data Structure.  The unit 
# tests execute all the tests implemented in the ANOPP2.API.
# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_geo_unit_test.restype = A2_IK



# -----------------------------------------------------------------------------------------
# This routine creates an acoustic data surface in the ANOPP2.API.  A tag is returned
# which is used by the calling program to access that data surface.  The input into
# this routine is the name of a settings file.  The configuration file must contain 
# one of the known types of acoustic data surfaces.  See Documentation for more 
# information on the format of the settings file.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is an integer that is returned by this function.  It is used to access
#        the data structure that is created.
# @param strConfigurationFile
#        This is the name of the input file that contains the settings for the new
#        acoustic data surface. See Documentation for more information.
# @result
#        An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_geo_create.restype = A2_IK 
ANOPP2.a2py_geo_create.argtypes = [POINTER(A2_IK), POINTER(A2_CK)]



# -----------------------------------------------------------------------------------------
# This routine will destroy a geometry in the ANOPP2.API. The entire data structure,
# including any calculated metrics, geometric configurations, and motion will be deleted.
# The tag will be unassociated to any information within the data structure.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is the tag associated with the ANOPP2.API being destroyed.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_geo_delete.restype = A2_IK
ANOPP2.a2py_geo_delete.argtypes = [POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# This subroutine adds an offset to the acoustic data surface.  There are 3 offsets that
# need to be accounted for.  The first is the flow data on the surface offset.  This
# is typically used when blades are periodic, and the only difference between one blade
# and another (in terms of flow data) is a time offset.  The second offset is similar
# to the first and is the geometric offset.  This is only applicable if the geometry
# is deforming and is typically the same as the flow data offset.  The last offset is
# an angle offset.  This accounts for the position of the blade with respect to the
# first blade.  This function takes in these 3 offset values and a tag of the original
# data surface.  The function returns a new tag value that can be used to refer to the
# offset acoustic data surface.
# -----------------------------------------------------------------------------------------
# @param intNewTag
#        This is the tag value of the data surface that is being created inside the
#        API.  This will be the same as the data surface associated to the existing tag
#        with new offset paraeters.
# @param intExistingTag
#        This is the tag value of the acoustic data surface being copied.
# @param fltDataTimeOffset
#        This is the time value of the offset of the flow data applied to the surface.
# @param fltANOPP2.imeOffset
#        This is the time value of the offset of the deformable geometry.
# @param fltAngleOffset
#        This is the offset of the angle position of the new geometry.  The existing
#        acoustic data surface must have a periodic rotation frame of reference change
#        in order to correctly account for the position of the new data surface.
# @result
#        An integer communicating success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_geo_copy_ads.restype = A2_IK 
ANOPP2.a2py_geo_copy_ads.argtypes = [POINTER(A2_IK), A2_IK, A2_RK , A2_RK , A2_RK ]



# -----------------------------------------------------------------------------------------
# This subroutine calculates integrated metrics from the geometry data structure.  This
# includes such things as thrust and torque from surfaces representing a blade surface.
# Currently, the function takes in an enumerator for the desired metric.  Examples are
# a2_torque for torque or a2_force for integrated forces such as thrust. See
# documentation for more information on the available metrics.
# -----------------------------------------------------------------------------------------
# @param intTag
#        This is the tag representing the geometry data structure.
# @param fltTime
#        This is the time that the metric will be calculated.
# @param enumMetric
#        This enumerator is the desired metric for example a2_torque or
#        a2_force.
# @param nDimensions
#        This is the number of dimensions in the metric to be calculated. For example, 
#        if force is the desired metric, this should be 3 corresponding to its 3 
#        components, Fx, Fy, and Fz.
# @param fltMetric
#        This is an allocatable array that is allocated by this function and set to the
#        result.  For example, if force is specified, this is an array of size 3 and
#        the elements of the fltMetric array are set to Fx, Fy, and Fz.  
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_geo_calc_metric.restype = A2_IK 
ANOPP2.a2py_geo_calc_metric.argtypes = \
   [A2_IK, A2_RK , A2_EK, A2_IK, POINTER(A2_RK )]



# ---------------------------------------------------------------------------------------
# This function takes in a tag representing a geometry and returns true if it exists
# in the API and false if it does not.
# ---------------------------------------------------------------------------------------
# @param intTag
#        This is the tag associated to the geometry that is being searched for.
# @result
#        A bool that is returned true if the geometry exists and false if it does not.
# ---------------------------------------------------------------------------------------
ANOPP2.a2py_geo_exists.restype = A2_LK
ANOPP2.a2py_geo_exists.argtypes = [A2_IK]



# -----------------------------------------------------------------------------------------
# This routine accesses the ANOPP2.API and tells it to export a ANOPP2.Data
# Structure.  The routine accepts a geometry tag and an output file name and causes 
# the ANOPP2.API to write out the data structure to the file in the specified format
# for a given target application.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        The tag that connects the application code to the ANOPP2 API.
# @param strFileName
#        This is the name of the file that the Export routine writes.
# @param enumFormat
#        This is the format of the file: options may include formatted or binary.
# @param enumProgram
#        This is the program that will eventually read the file.  This may include
#        Tecplot or NetCDF.
# @result
#        An integer that is 0 if everything has occurred as expected.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_geo_export.restype = A2_IK 
ANOPP2.a2py_geo_export.argtypes = [A2_IK, POINTER(A2_CK), A2_EK, A2_EK]



# -----------------------------------------------------------------------------------------
# This routine modifies a geometry by a percentage or delta (specified by dmy_fltDelta)
# at a specified datum, time, and grid position or data condition.  Only a grid position
# or data condition can be specified, not both.  For example, for a surface like that
# pictured below, the ith face as indicated has been chosen for modification.
# 
# Surface :               __________________________________
#                         |__|__|__|__|__|__|__|__|__|__|__|
#                         |__|__|__|__|__|__|__|__|__|__|__|
#                         |__|__|__|__|__|__|__|__|__|__|__|
#                         |__|__|__|__|__|__|__|__|__|__|__|
#                         |__|__|__|__|__|__|__|__|__|__|_<|---ith face
#                         |__|__|__|__|__|__|__|__|__|__|_ |   time index is 1
#                         |__|__|__|__|__|__|__|__|__|__|__|   delta is 0.5
#                         |__|__|__|__|__|__|__|__|__|__|__|   factor setting is present
#                         |__|__|__|__|__|__|__|__|__|__|__|
# 
# For this surface, the geometry grid is constant, so the time index is meaningless.
# The delta set at 0.5 and will be used as a percentage.
#
# The incoming delta may be a percent or a value.  The dmy_enumFactorSetting specifies
# whether the delta is a percentage (a2_geo_percentage_of_bounding_box) or a value 
# (a2_geo_delta).  If specified as a percentage, upon output, the delta is modified.
# If specifying a grid coordinate, delta is equal to the percentage of the bounding box
# around the specified position.  For example, if the incoming delta is 0.1 and the
# determined bounding box area is 4 meters, the resulting delta is 0.001 x 4.  Upon
# output, if specifying data variable, the delta is a value equal to the percentage of
# minimum and maximum of that data variable.  The resulting delta is used for subsequent
# sensitivity calculations in which the derived sensitivity is divided by the delta.
# 
# Note:
#   1. The time index will be set to 0 for a constant geometry grid regardless of the
#      input time index.  If a non-constant geometry grid is chosen, the time index
#      must be within the time index range.
#   2. The grid variable, if specified, must be 1 (x), 2 (y), or 3 (z).
#   3. The data variable, if specified, must be within the valid range.
#   4. Both grid variable and data variable values can't be 0.
#   5. Both grid variable and data varaible values can't be non-zero.
# -----------------------------------------------------------------------------------------
# @param this
#        The geometry being accessed.
# @param dmy_intDatumVariable
#        The datum at which the geometry is to be modified.  This could be a node number
#        or a number face.
# @param dmy_intTimeIndex
#        The index into the time array at which the geometry is to be modified.  If the
#        grid geometry is constant, the time variable has no meaning.
# @param dmy_intGridVariable
#        If not 0, this is the x (1), y (2), or z (3) coordinate position at which the
#        geometry is to be modified.  The variable can not be less than 0 or greater
#        than 3.
# @param dmy_intDataVariable
#        If not 0, the data condition at which the geometry is to be modified.  Examples
#        of valid data variables are 1 for impermeable and 5 for permeable.
# @param dmy_enumFactorSetting
#        Enumerator specifying whether incoming delta is a percentage of the bounding
#        box or a delta.
# @param dmy_fltDelta
#        On input the percentage or delta for modification.  On output, how the delta
#        was perturbed (will be used in sensitivity calculations).
# @result
#        An integer representing the success of this function.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_geo_modify.restype = A2_IK 
ANOPP2.a2py_geo_modify.argtypes = \
   [A2_IK, A2_IK, A2_IK, A2_IK, A2_IK, A2_EK, POINTER (A2_RK)]



#------------------------------------------------------------------------------------------
# Assigns a geometry_range_scheduler to the geometry data pointed to by the ANOPP2.Tag.
# The scheduler is a callback function accepts calls from ANOPP2 and returns the beginning
# and end of the range or a subrange of geometry nodes in the function's arguments.  The
# logic for returning different ranges with successive calls is at the user's discretion
# allowing the user to implement schemes such as partitioning of nodes by MPI rank and
# asynchronous parallelization.
#------------------------------------------------------------------------------------------
# @param dmy_intTag
#        The tag of the geometry item being assigned a geometry scheduler.
# @param geometry_range_scheduler
#        A callback method providing geometry range scheduling.
# @param shedule_index
#        An object for communicating data and instructions between the user code calling
#        this method and the geometry_range_scheduler callback method.
# @result
#        An integer that is returned 0 when everything occurred correctly.
#------------------------------------------------------------------------------------------
ANOPP2.geo_scheduler_prototype = CFUNCTYPE(
  None,
  POINTER(c_void_p), POINTER(A2_IK), POINTER(A2_IK)
  )

ANOPP2.a2py_geo_assign_scheduler.restype = A2_IK
ANOPP2.a2py_geo_assign_scheduler.argtypes = \
  [POINTER(A2_IK), ANOPP2.geo_scheduler_prototype, POINTER(c_void_p)]



#-----------------------------------------------------------------------------------------
# Gets the size of the geometry object in terms of number of nodes.
#-----------------------------------------------------------------------------------------
# @param dmy_intTag
#        The tag of the geometry item being queried.
# @param number_of_nodes
#        The size of the geometry object.
# @result
#        An integer that is returned 0 when everything occurred correctly.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_geo_get_number_of_nodes.restype = A2_IK
ANOPP2.a2py_geo_get_number_of_nodes.argtypes = [POINTER(A2_IK), POINTER(A2_IK)]



#------------------------------------------------------------------------------------------
# Gets the range for the geometry object from the geometry's scheduler.
#------------------------------------------------------------------------------------------
# @param dmy_intTag
#        The tag of the geometry item being queried.
# @param range_begin
#        The beginning of the range.
# @param range_end
#        The end of the range.
# @result
#        An integer that is returned 0 when everything occurred correctly.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_geo_get_range.restype = A2_IK
ANOPP2.a2py_geo_get_range.argtypes = [POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK)]



#!/usr/bin/env python
# =========================================================================================
#  Next part of this section of the interface file contains hardcoded enumerators used by
#  the user's program to communicate parameters and settings to the API.
# =========================================================================================



# ---------------------------------------------------------------------------------------
#  These enumerators are to communicate to the API, certain default objects that can be
#  loaded via the catalog.  This is optional.  A user may also specify an input
#  configuration file if they wish with more detail on the object to be constructed.
#  See User's Manual for more information.
# ---------------------------------------------------------------------------------------
#  


#  The first enumerator is for the force of a surface. This is the integral of pressure time \
#   the normal vector for an impenetrable surface or p*nj + rho*ui*                          \
#    (un - vn) for an impenetrable surface.
a2_force = 1

#  This is the torque of a surface. It is the cross product of r times the force.
a2_torque = 2



#----------------------------------------------------------------------------------------
# These enumerations are the coordinate types of the geometry. Options are spherical 
# or cartesian.
#----------------------------------------------------------------------------------------
# An enumeration for spherical coordinates
a2_geo_spherical = 1

# An enumeration for cartesian coordinates
a2_geo_cartesian = 2



#------------------------------------------------------------------------------------------
# This enumerator list defines the possible factor settings for delta when modifying
# a geometry.  Options include delta as a percentage of the bounding box or simply a
# delta value.
#------------------------------------------------------------------------------------------
# The first enumerator is a percentage of the bounding box.
a2_geo_percentage_of_bounding_box = 1

# The second is for a delta value.
a2_geo_delta = 2



#!/usr/bin/env python
# -----------------------------------------------------------------------------------------
# This is an Application Program Interface (API) that works directly with the user's  
# code such that the user does not interact with the source code.  This API
# contains the routines that the user will need to create  frame changes and
# calculate the position, velocity, acceleration, jerk, or snap.  This interface calls
# the routines that calculate these values.
# -----------------------------------------------------------------------------------------
# @file ANOPP2.api.h
# @author ANOPP2 Development Team
# @version 1.0.0
# -----------------------------------------------------------------------------------------



# =========================================================================================
# First part of this section contains interfaces into the available routines included in
# this API.
# =========================================================================================



# -----------------------------------------------------------------------------------------
# This subroutine initializes the ANOPP2.API and all modules it depends
# on.
# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
# This routine executes the unit tests in the ANOPP2.API.  The unit 
# tests execute all the tests implemented in the ANOPP2.API.
# -----------------------------------------------------------------------------------------
# @result
#        The number of failed asserts found while performing unit tests.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_unit_test.restype = A2_IK



# ---------------------------------------------------------------------------------------
#  This function takes in a tag representing a kinematics and returns true if it exists in 
# the API and false if it does not.
# ---------------------------------------------------------------------------------------
#  @param intTag
#         This is the tag associated with the ANOPP2.that is being searched for.
#  @result
#         A bool that is returned true if the frame of reference  exists and false if it
#         does not.
# ---------------------------------------------------------------------------------------
ANOPP2.a2py_kine_exists.restype = A2_LK
ANOPP2.a2py_kine_exists.argtypes = [A2_IK]



# ---------------------------------------------------------------------------------------
#  This routine creates a ANOPP2.Data Structure in the ANOPP2.API.  A tag is
#  returned which is used by the calling program to access that Data Structure.  The
#  input into this routine is the name of a configuration file (also known as Settings
#  file).  The configuration file must contain one of the known types of observers. By
#  providing the values of several parameters in these configuration files, the Observer
#  API generates one or more nodes that define the geometry of the Observer.
# ---------------------------------------------------------------------------------------
#  @param intTag
#         This is an integer that is returned by this function.  It is used to access
#         the data structure that is created.
#  @param strConfigurationFile
#         This is the name of the input file that contains the settings for the new
#         kinematics. See Documentation for more information.
#  @result
#         An integer that is returned 0 when everything occurred correctly.
# ---------------------------------------------------------------------------------------
ANOPP2.a2py_kine_create.restype = A2_IK
ANOPP2.a2py_kine_create.argtypes = [POINTER(A2_IK), POINTER(A2_CK)]



# ---------------------------------------------------------------------------------------
# This routine loads a kinematics list from the ANOPP2.catalog.  The catalog defines
# some standard kinematics rotations and translations that the user can use to shortcut
# creating a kinematics list that is easily specified.  See documentation for details
# on available kineatics list.  The user can also use the restart functionality to 
# save a kinematics list and load it at a later time.
# ---------------------------------------------------------------------------------------
# @param dmy_intTag
#        This is the tag of the kinematics list being created by the catalog or
#        restart file.
# @param dmy_enumCatalog
#        This is the enumerator associated with the kinematics that will be read in from
#        the catalog.  The user should be 0 here if the kinematics will be created from
#        a restart file.  ANOPP2.enumerators are defined in the enumerator section of
#        the kinematics API but including such things as a2_kine_trivial, 
#        a2_kine_rotate_about_y_axis, etc.
# @param dmy_strRestartFile
#        This is a restart file that was saved at a previous time via the 
#        [prefix]_kine_save routine.
# @result
#        An integer representing success of this operation.
# ---------------------------------------------------------------------------------------
ANOPP2.a2py_kine_load.restype = A2_IK
ANOPP2.a2py_kine_load.argtypes = [POINTER(A2_IK), A2_EK, POINTER(A2_CK)]



# ---------------------------------------------------------------------------------------
#  This routine takes in a tag for a kinematics and creates a deep copy of the
#  kinematics.
# ---------------------------------------------------------------------------------------
#  @param intTagOfSource
#         This is a tag for a kinematics list that is going to be copied.
#  @param intTagOfCopy
#         This is a tag that will be created for the copy.
#  @result
#         An integer representing success of this operation.
# ---------------------------------------------------------------------------------------
ANOPP2.a2py_kine_copy.restype = A2_IK
ANOPP2.a2py_kine_copy.argtypes = [A2_IK, POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# This routine does a combine for intTagForA and intTagForB.
# -----------------------------------------------------------------------------------------
# @param intTagForA
#        This is a tag for a kinematics list that is going to be combined.
# @param intTagForB
#        This is a tag for a kinematics list that is going to be combined.
# @param intTagForC
#        This is a tag that will be created for the combine.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_combine.restype = A2_IK
ANOPP2.a2py_kine_combine.argtypes = [A2_IK, A2_IK, POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# This routine appends a ANOPP2.data structure to an existing ANOPP2.data structure
# by reading them from a file in a similar way that the Create routine operates
# -----------------------------------------------------------------------------------------
# @param strSettingsFile
#        The name of the file that contains the necessary data-values describing how
#        the object moves (rotation, translation, and their time derivatives).
#        This is usually a .config file.  This file should only
#        contain one object's motion. Multiple objects will not be read in. A
#        single file will need to be read in for each of the different configurations.
#        This parameter is a C-C++ string and will need to be converted to a
#        Fortran string before it is used by the modules.
# @param intTag
#        This is an integer that is returned by this routine.  It be used to access
#        the Frame of Reference list that was appended.
# @result
#        An integer representation of success.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_append.restype = A2_IK
ANOPP2.a2py_kine_append.argtypes = [POINTER(A2_CK), POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# This routine takes an existing kinematics frame of reference list and appends a
# constant frame change (translation or rotation, but not both) onto the list.  If an
# existing transformation exists, it will be deleted since it would no longer be valid.
# -----------------------------------------------------------------------------------------
# @param intTag
#        The tag of an existing kinematics structure that will have a constant frame
#        change appended to it.
# @param fltTranslation
#        If the constant frame change is a translation, this is the translation value.
# @param fltRotationAxis
#        If the constant frame change is a rotation, this is the axis of rotation.
# @param fltRotationAngle
#        If the constant frame change is a rotation, this is the angle of rotation.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_append_constant_frame_change.restype = A2_IK
ANOPP2.a2py_kine_append_constant_frame_change.argtypes = \
     [A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_RK]



# -----------------------------------------------------------------------------------------
# This routine takes an existing kinematics frame of reference list and appends a
# polynomial frame change (translation or rotation, but not both) onto the list.  If an
# existing transformation exists, it will be deleted since it would no longer be valid.
# The polynomial frame change takes in a list of coefficients (angle or translation)
# and applies it to the following: x = SUM_1^N (x_i * t^(i-1))
# -----------------------------------------------------------------------------------------
# @param intTag
#        The tag of an existing kinematics structure that will have a polynomial frame
#        change appended to it.
# @param dmy_intNumberTranslationCoefficients
#        If the polynomial frame change is a translation, this is the number of
#        translation coefficients.
# @param dmy_fltTranslationCoefficients
#        If the polynomial frame change is a translation, this is the translation 
#        coefficients.  This is a two-dimensional array, dimensioned 3 (for a vector)
#        by the number of coefficients. 
# @param dmy_intNumberRotationCoefficients
#        If the polynomial frame change is a rotation, this is the number of rotation
#        coefficients.
# @param dmy_fltRotationAxis
#        If the polynomial frame change is a rotation, this is the axis of rotation.
# @param dmy_fltRotationCoefficients
#        If the polynomial frame change is a rotation, this is the angle of rotation
#        coefficients.
# @param blnPeriodicRotationFlag
#        This flag sets the frame change being appended as the periodic rotation flag
#        which will define the Psi in the periodic angle rotation.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_append_polynomial_frame_change.restype = A2_IK
ANOPP2.a2py_kine_append_polynomial_frame_change.argtypes = \
     [A2_IK, A2_IK, POINTER(A2_RK), A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_LK]



# -----------------------------------------------------------------------------------------
# This routine takes an existing kinematics frame of reference list and appends a periodic
# frame change (rotation only) onto the list.  If an  existing transformation exists, it
# will be deleted since it would no longer be valid. The periodic frame change takes in a
# list of coefficients (A0, An, and Bn) and applies it to the following: 
# x = A0 + SUM_1^N [An * COS (Psi) + Bn * SIN (Psi)] where Psi is defined from a frame
# change that has blnPeriodicRotationFlag set at true.
# -----------------------------------------------------------------------------------------
# @param dmy_intTag
#        The tag of an existing kinematics structure that will have a periodic frame
#        change appended to it.
# @param dmy_fltRotationAxis
#        If the constant frame change is a rotation, this is the axis of rotation.
# @param dmy_fltA0
#        This is the A0 coefficient.
# @param dmy_intNumberPeriodicCoefficients
#        This is the number of periodic coefficients.
# @param dmy_fltAn
#        This is a list of An coefficients.
# @param dmy_fltBn
#        This is a list of Bn coeficients.
# @param dmy_fltOmega
#        These are the omega coefficients.
# @param dmy_fltPeriodicOffset
#        This is the angle offset.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_append_periodic_frame_change.restype = A2_IK
ANOPP2.a2py_kine_append_periodic_frame_change.argtypes =               \
     [A2_IK, POINTER(A2_RK), A2_RK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), \
      POINTER(A2_RK), A2_RK]



# -----------------------------------------------------------------------------------------
# This routine takes an existing kinematics frame of reference list and appends an
# aperiodic frame change (rotation only) onto the list.  The periodic frame change
# takes in a list of either translation or rotation angle as a series of time values.
# -----------------------------------------------------------------------------------------
# @param dmy_intTag
#        The tag of an existing kinematics structure that will have an aperiodic frame
#        change appended to it.
# @param dmy_nTimes
#        This is the size of the time array.
# @param dmy_fltTime
#        This is the time array that defines where the translation or angles are defined.
# @param dmy_nTranslation
#        This is the size of the translation array.
# @param dmy_fltTranslation
#        This is a two-dimensional array sized as the number of time steps by 3 (for
#        vector quantitiy).  If this is a translation frame change, then this array is
#        filled.  If this frame change is an angle frame change, this array must be sized
#        to zero.
# @param dmy_fltRotationAxis
#        If the aperiodic frame change is a rotation, this is the axis of rotation.
# @param dmy_nRotationAngle
#        This is the size of the rotation angle array.
# @param dmy_fltRotationAngle
#        This is a time history of rotation angles.  This array contains angles at every
#        time step defined in the time array.  This array must be sized the same as the
#        the time array or sized to zero if this is a translation frame change.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_append_aperiodic_frame_change.restype = A2_IK
ANOPP2.a2py_kine_append_aperiodic_frame_change.argtypes =              \
     [A2_IK, A2_IK, POINTER(A2_RK), A2_IK, POINTER(A2_RK), POINTER(A2_RK), \
      A2_IK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine destroys a ANOPP2.data structure.  This is because
# the user no longer needs the ANOPP2.data.
# -----------------------------------------------------------------------------------------
# @param tag
#        An integer that is associated with the ANOPP2.that will be destroyed.  After
#        this destruction, the integer will have no meaning in the registry.
# @result
#        An integer representation of success.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_destroy.restype = A2_IK
ANOPP2.a2py_kine_destroy.argtypes = [POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# This routine calculates a transformation object by reducing a FoR at a single time.
# The transformation object can be of type position, velocity, acceleration, jerk,
# or snap depending on the argument provided by the user.
# -----------------------------------------------------------------------------------------
# @param intANOPP2.ag
#        This is the tag that is associated with the ANOPP2.Data Structure.
# @param fltTime
#        This is the time setting when the FoR linked list will be reduced to a 
#        single transformation.  The FoR linked list represents all motion throughout 
#        all time, a transformation is instantaneous.  This time value is the 
#        instantaneous value when the FoR linked list will be reduced to a single 
#        transformation. 
# @param enumTransType
#        This refers to the type of transformation that is to be performed 
#        (i.e.. position, velocity, acceleration, jerk, or snap). \n
#        position     => a2_kine_position \n
#        velocity     => a2_kine_velocity \n
#        acceleration => a2_kine_acceleration \n
#        jerk         => a2_kine_jerk \n
#        snap         => a2_kine_snap
# @result
#        An integer representation of success.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_calculate_transformation.restype = A2_IK
ANOPP2.a2py_kine_calculate_transformation.argtypes = [A2_IK, A2_RK, A2_IK]



# -----------------------------------------------------------------------------------------
#  This function reorients the vectors such that they are all in the same frame of
#  reference.  The frame of reference could be the ground frame or the local frame 
#  depending on the enumerator provided.
# -----------------------------------------------------------------------------------------
#  @param dmy_intANOPP2.ag
#         This is tha tag of the ANOPP2.data structure containing the data.
#  @param dmy_fltCurrentFor
#         This is the coordinates in the current frame of reference.
#  @param dmy_fltNewFor
#         This is the transformed coordinates in the new frame of reference.
#  @param dmy_enumTargetFor
#         This is an enumerator that represents the Frame of Reference to which the 
#         coordinates have to be transformed.
#  @intSuccess
#         An integer representing success of this operation
#  @note Copying over each of the 3 elements in a vector proved to be faster than
#        using the array copy.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_reorient.restype = A2_IK
ANOPP2.a2py_kine_reorient.argtypes = [A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_IK]



# -----------------------------------------------------------------------------------------
# This routine calculates the position of each of the frames of reference in the 
# global or ground frame.  This routine returns the position of a point in the 
# global frame. 
# -----------------------------------------------------------------------------------------
# @param dmy_intANOPP2.ag
#        This is tha tag of the ANOPP2.data structure containing the data.
# @param xLocal
#        This is the local coordinates for the current frame of reference.
# @param xGlobal
#        This is the local position transformed into the global frame of 
#        reference, also referred to as the ground frame of reference.
# @result
#        An integer representation of success.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_position.restype = A2_IK
ANOPP2.a2py_kine_position.argtypes = [A2_IK, POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine derives the necessary values from the position vector to calculate  
# the velocity vector in the global frame (ground frame).  This routine returns 
# the velocity of a point in the global frame. In this routine the motion of the 
# point in the local frame is stationary.
# -----------------------------------------------------------------------------------------
# @param dmy_intANOPP2.ag
#        This is tha tag of the ANOPP2.data structure containing the data.
# @param xLocal
#        This is the local coordinates for the current frame of reference.
# @param vGlobal
#        This is the velocity vector in the global frame or the ground frame.
# @result
#        An integer representation of success.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_velocity.restype = A2_IK
ANOPP2.a2py_kine_velocity.argtypes = [A2_IK, POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine derives the necessary values from the position vector to calculate  
# the acceleration vector in the global frame (ground frame).  This routine returns 
# the acceleration of a point in the global frame.  This routine assumes the source
# does not move in the local frame of reference.
# -----------------------------------------------------------------------------------------
# @param dmy_intANOPP2.ag
#        This is tha tag of the ANOPP2.data structure containing the data.
# @param xLocal
#        This is the local coordinates for the current frame of reference.
# @param aGlobal
#        This is the acceleration vector in the global frame or the ground frame.
# @result
#        An integer representation of success.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_acceleration.restype = A2_IK
ANOPP2.a2py_kine_acceleration.argtypes = [A2_IK, POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine derives the necessary values from the position vector to calculate  
# the jerk vector in the global frame (ground frame).  This routine returns the 
# jerk of a source point in the global frame of reference.  
# -----------------------------------------------------------------------------------------
# @param dmy_intANOPP2.ag
#        This is tha tag of the ANOPP2.data structure containing the data.
# @param xLocal
#        This is the local coordinates for the current frame of reference.
# @param jGlobal
#        This is the jerk vector in the global frame or the ground frame.
# @result
#        An integer representation of success.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_jerk.restype = A2_IK
ANOPP2.a2py_kine_jerk.argtypes = [A2_IK, POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine derives the necessary values from the position vector to calculate  
# the snap vector in the global frame (ground frame).  This routine returns the 
# snap of a source point in the global frame of reference.  
# -----------------------------------------------------------------------------------------
# @param dmy_intANOPP2.ag
#        This is tha tag of the ANOPP2.data structure containing the data.
# @param xLocal
#        This is the local coordinates for the current frame of reference.
# @param jGlobal
#        This is the snap vector in the global frame or the ground frame.
# @result
#        An integer representation of success.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_kine_snap.restype = A2_IK
ANOPP2.a2py_kine_snap.argtypes = [A2_IK, POINTER(A2_RK), POINTER(A2_RK)]



#!/usr/bin/env python
# =========================================================================================
# Next part of this interface file contains hardcoded enumerators used by the user's
# program.
# =========================================================================================



# -----------------------------------------------------------------------------------------
#  This enumeration is for the geometry of the output file.  The geometry can be in
#  a global frame of reference or a local frame of reference.
# -----------------------------------------------------------------------------------------


#  This is the enumeration for a global frame of reference 
a2_global = 1

#  This is the enumeration for a local frame of reference
a2_local = 2
  
  
  
#------------------------------------------------------------------------------------------
# These enumerators define what transformation is to take place, position, velocity,
# acceleration, jerk, or snap.
#------------------------------------------------------------------------------------------
# This enumeration is to specify that a position transformation is to take place.
a2_kine_position = 1

# This enumeration is to specify that a velocity transformation is to take place.
a2_kine_velocity = 2

# This enumeration is to specify that an acceleration transformation is to take place.
a2_kine_acceleration = 3 

# This enumeration is to specify that a jerk transformation is to take place.
a2_kine_jerk = 4  

# This enumeration is to specify that a snap transformation is to take place.
a2_kine_snap = 5 



#------------------------------------------------------------------------------------------
# These are the enumerators associated with the kinematics catalog.
#------------------------------------------------------------------------------------------
# The first enumerator is for a trivial kinematics list, meaning no translation and
# no rotation.
a2_kine_trivial = 1



# -----------------------------------------------------------------------------------------
# This file is the interface file for the Fortran subroutines in the acoustic analysis
# Application Programming Interface (API).  This file should be copied to your local
# directory and an "include 'ANOPP2.odule.api.f90'" must be present in your
# program.  See ANOPP2.PIDemonstrator.cplusplus.cpp for an example including
# using all present subroutines.
# -----------------------------------------------------------------------------------------
# @file ANOPP2.api.py
# @author The ANOPP2 Development Team
# @version 1.0.0
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
# This subroutine initializes the acoustic analysis API and should be included at
# the very start of your program (before any other acoustic analysis subroutines
# are called).
# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
# This routine executes the unit tests in the Acoustic Analysis module.  The unit
# tests execute all the tests implemented in the Acoustic Analysis API.
# -----------------------------------------------------------------------------------------
# @result
#        An integer of the number of failed asserts that occurred during testing.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_unit_test.restype = A2_IK



# -----------------------------------------------------------------------------------------
# This routine converts an input from one unit to another.  The supported units are
# defined in the acoustic units enumeration list and include pressure, pressure squared,
# and decibels.
# -----------------------------------------------------------------------------------------
# @param enumInput
#        This is the enumerator for the units of the input.
# @param fltInput
#        This is the value of the input in the units defined by enumInput.  This
#        will be converted to a different unit defined by enumOutput.
# @param enumOutput
#        This is the enumeration of the desired units.  fltInput will be converted
#        to these units.
# @param fltOutput
#        This is the result of this routine: fltInput which is in units enumInput
#        converted to the units defined by enumOutput.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_convert.restype = A2_IK
ANOPP2.a2py_aa_convert.argtypes = [A2_EK, A2_RK, A2_EK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine returns a segment of a long pressure time history.  This routine takes
# in the long acoustic pressure time history (including the time and pressure values)
# and desired segment size, step size, and the segment number wanted.  This routine
# returns the size of the new segment and the time and function of the segment.
# This routine returns a logical success flag that is returned as false if the
# segmentation has failed.  This can occur if the segment size or step size are less
# than zero, or the segment number requested is less than 1.  \n
# If the segment size or segment step size do not fall on exact values of the discrete
# time samples in the long time and long func arrays, they are 'fudged' slightly to
# fall on exact samples.
# -----------------------------------------------------------------------------------------
# @param intNl
#        The size of the long acoustic pressure time history.  This is the size of the
#        longTime and longFunc arrays.
# @param fltLongTime
#        The long acoustic pressure time history.  This is used to create the segment.
# @param fltLongFunction
#        The long acoustic pressure time history.  This is the acoustic pressure at
#        the time samples in the long time array.
# @param fltTs
#        The size of the segment requested.  This number might be 'fudged' slightly to
#        fall on a sample in the long time and long func arrays.  This is returned
#        modified.
# @param fltS
#        The size of the incremental step between segments.  This can be less than or
#        greater than the segment size.  This number may be 'fudged' slightly if the
#        size is not exactly divisible by the time step size in the long time array.
# @param intN
#        The segment number in the long time history wanted.  This number must be
#        greater than 1 and less than the maximum number of segments possible.
# @param intNs
#        This is the size of the resultant segmented time and function arrays.  This
#        is returned by this subroutine.
# @param fltSegmentTime
#        The time array of the segment data.  This array is allocated by this routine
#        to size M (param m).  The last time step minimum the first time step should be
#        equal to the 'fudged' segment size.
# @param fltSegmentFunction
#        The function array of the segmented data.  This is the function at the time
#        steps in the segment time array.
# @result
#        An integer representation of success.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_segment.restype = A2_IK
ANOPP2.a2py_aa_segment.argtypes =                                  \
     [A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), \
      A2_IK, POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK))]



# -----------------------------------------------------------------------------------------
# This routine windows an acoustic pressure time history.  The window function is
# chosen via an enumerator passed to the routine as the first argument.  The size of the
# time array and function must be the same.  A logical success parameter is returned
# by the function.  This is true if the routine succeeded, and false if it failed.
# -----------------------------------------------------------------------------------------
# @param enumWindow
#        The enumerated value of the window function to be applied.
# @param intN
#        The size of the function and time arrays.
# @param fltTime
#        The time where the acoustic pressure is sampled.  This must be an evenly
#        spaced array.
# @param fltFunction
#        The array of acoustic pressure to be windowed.  These values in is the
#        function at the time in the time array.  This is overwritten by the
#        windowed function.
# @result
#        An integer representation of success.  0 indicates no errors.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_window.restype = A2_IK
ANOPP2.a2py_aa_window.argtypes = [A2_EK, A2_IK, POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine applies a high and low pass filter to a Pressure Time History
# -----------------------------------------------------------------------------------------
# @param intN
#        This is the size of the Time and Pressure arrays.
# @param fltTime
#        This is the array of times for the Time History.
# @param fltPressure
#        This is the array of pressures for the Time History
# @param fltHighPassFrequency
#        This is the frequency for the low pass filter, which will set all lower
#        frequencies to zero.
# @param fltLowPassFrequency
#        This is the frequency for the high pass filter, which will set all higher
#        frequencies to zero.
# @result
#        A logical success flag.  If the routine succeeded, this is returned as
#        true.  If it failed, this is returned as false.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_filter.restype = A2_IK
ANOPP2.a2py_aa_filter.argtypes = \
     [A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_RK, A2_RK]



# -----------------------------------------------------------------------------------------
# This routine calculates the narrowband spectrum for three types of input. The
# input types may be a pressure time history, a proportional band spectrum, or
# a power spectral density.  The power spectral density may have units of dB/Hz or
# Pascals squared per Hz.  If the input is a pressure time history, the independent
# variable will be time and the dependent variable will be acoustic pressure.  If the
# input is a proportional band spectrum or a power spectral density with units of
# dB/Hz, the independent variable will be frequency and the dependent variable will be
# decibels.  If the input is a power spectral density with units of Pascals squared per
# Hz, the independent variable will be frequency and the dependent variable will be
# acoustic pressure.  The parameter enumInputType  identifies the input type.  The
# integer intM is the size of the result arrays of frequency and narrowband spectrum.
# If the input is pressure time history, intM is returned as INT(intN/2) because of
# the Nyquist criteria.  If the input is a proportional band spectrum or a power spectral 
# density, intM is input for the size of the resultant narrowband spectrum.
# -----------------------------------------------------------------------------------------
# @param enumMetric
#        This input parameter identifies the input type of the independent and dependent
#        variables.  This must be one of the supported metrics in the enumeration for
#        metrics.  Supported metrics include: a2_aa_apth, a2_aa_psd, and a2_aa_pbs.
# @param enumUnits
#        These are the units of the input.  Supported units include a2_aa_ap, a2_aa_msp,
#        and a2_aa_db.  Not all combinations of metric and units are supported.  An
#        a2_aa_apth must be accompanied by a2_aa_ap.  An a2_aa_psd can be in either
#        a2_aa_msp or a2_aa_db.  An a2_aa_pbs must be accompanied by a2_aa_db.
# @param intN
#        Dimension of the independent and dependent arrays
# @param fltIndepedent
#        The independent variable will be time when the input type is a pressure time
#        series.  The independent variable will be frequencies when dependent variable
#        is a proportional band spectrum or a power spectral density. The independent
#        variable must be evenly spaced times when the input type is a pressure time
#        history or a power spectral density. The independent variable is not required to
#        be evenly spaced when the input type is a proportional band spectrum.
# @param fltDependent
#        The dependent variable is an array of pressures squared or decibels depending
#        on the value of the parameter enumInputType.
# @param intM
#        The dimension of the resultant frequency and narrowband spectrum.  This number
#        is returned by this routine if the input is an acoustic pressure time series.
#        This number is input if the input is a proportional band spectrum or power
#        spectral density.
# @param fltFrequency
#        The frequencies of the result narrowband spectrum.  This array is filled and
#        returned by this routine.
# @param fltNbsMsp
#        The narrowband spectrum.  This array is filled, and returned by this routine.
# @param fltPhase
#        The phase of the narrowband spectrum.
# @result
#        An integer success flag.  If the routine succeeded, this is returned as
#        0.  If it failed, this is returned as a non-zero number
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_nbs.restype = A2_IK
ANOPP2.a2py_aa_nbs.argtypes =                                    \
     [A2_EK, A2_EK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_IK), \
      POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK))]



# -----------------------------------------------------------------------------------------
# This routine calculates the power spectral density for three types of input. The
# input types may be a pressure time history, a proportional band spectrum, or
# a narrowband spectrum.  The narrowband spectrum may have units of dB/Hz or Pascals
# squared per Hz.  If the input is a pressure time history, the independent variable
# will be time and the dependent variable will be acoustic pressure.  If the input is a
# proportional band spectrum or a power spectral density with units of dB/Hz, the
# independent variable will be frequency and the dependent variable will be decibels.
# If the input is  a narrowband spectrum with units of Pascals squared per Hz, the
# independent variable will be frequency and the dependent variable will be acoustic
# pressure.  The parameter Input  identifies the input type.  The integer intM is the
# size of the result arrays of frequency and narrowband spectrum.  If the input is
# pressure time history, M is returned as INT(N/2) because of the Nyquist criteria.
# If the input is a proportional band spectrum or a narrowband spectrum, M  is input for 
# the size of the resultant narrowband spectrum.\n
# This is the Fortran binding of this routine.
# -----------------------------------------------------------------------------------------
# @param enumMetric
#        This input parameter identifies the input type of the independent and dependent
#        variables.  This must be one of the supported metrics in the enumeration for
#        metrics.  Supported metrics include: a2_aa_apth, a2_aa_nbs, and a2_aa_pbs.
# @param enumUnits
#        These are the units of the input.  Supported units include a2_aa_ap, a2_aa_msp,
#        and a2_aa_db.  Not all combinations of metric and units are supported.  An
#        a2_aa_apth must be accompanied by a2_aa_ap.  An a2_aa_nbs can be in either
#        a2_aa_msp or a2_aa_db.  An a2_aa_pbs must be accompanied by a2_aa_db.
# @param intN
#        The size of the time and pressure arrays provided to this routine.
# @param fltIndepedent
#        The independent variable will be time when the input type is a pressure time
#        series.  The independent variable will be frequencies when the dependent
#        variable is a proportional band spectrum or a narrowband spectrum. The
#        independent  variable must be evenly spaced times when the input type is a
#        pressure time history or a power spectral density. The independent variable is
#        not required to be evenly spaced when the input type is a proportional band 
#        spectrum.
# @param fltDependent
#        The acoustic pressure at the time values in the time array or the SPL values
#        of the proportional band spectrum.
# @param intM
#        The size of the resultant frequency and PSD arrays.  This number is returned
#        by this routine if the input is an acoustic pressure time series.  This number
#        is input if the input was a proportional band spectrum.
# @param fltFrequency
#        The frequencies of the result power spectral density.  This array is filled
#        and allocated by this routine.
# @param fltNbsMsp
#        The power spectral density of the pressure time history.  This array is
#        allocated, filled, and returned by this routine.
# @param fltPhase
#        The phase of the power spectral density
# @result
#        An integer success flag.  If the routine succeeded, this is returned as
#        0.  If it failed, this is returned as a non-zero number
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_psd.restype = A2_IK
ANOPP2.a2py_aa_psd.argtypes =                                    \
     [A2_EK, A2_EK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_IK), \
      POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK))]



# -----------------------------------------------------------------------------------------
# This routine alters a function as to a frequency weighting parameter.  Possible
# weighting functions are: A, B, C, and none.  This function takes in a frequency
# and noise array and an enumerator for the weighting function.  At each frequency
# a weight is calculated.  This weight is multiplied times the function which is
# overwritten and returned.  This routine also returns a success flag which can
# be false if the weight function is invalid.
# -----------------------------------------------------------------------------------------
# @param enumWeight
#        An integer representing the weighting that is applied.  Refer to the manual
#        for specifics on the enumeration options of frequency weighting.
# @param enumMetric
#        This is an enumeration for the acoustic metric that is provided as input.  The
#        options include a2_aa_nb for Narrowband Spectrum, a2_aa_psd for Power Spectral
#        Density, and a2_aa_pbs for a proportional band spectrum.  If the input is a
#        proportional band spectrum then each band is separated into subbands which are
#        weighted separately then summed.
# @param enumUnits
#        An enumeration identifying the units of the input function.  This is specified
#        via the units enumerations.
# @param intM
#        The size of the function that is to be weighted.  This integer is the size
#        of the following 2 arguments.
# @param fltFrequencies
#        The frequencies of the spectrum that is to be weighted.  This is used to
#        calculate the weighting function at that frequency that is applied to the
#        function
# @param fltFunction
#        The function that is to be weighted.  The weighting parameter is a function
#        of frequency.  For each frequency, that weighting function is multiplied
#        time the function.  The result is placed into this array.
# @result
#        An integer representing the success of this program.  This will be returned
#        as non zero if this routine fails.  Reason for failing may be an unknown
#        weight parameter.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_weight.restype = A2_IK
ANOPP2.a2py_aa_weight.argtypes = \
     [A2_EK, A2_EK, A2_EK, A2_IK, POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This function calculates the proportional band spectrum given a power spectral 
# density,narrowband spectrum, or tone spectrum.  The psd can be input with units of 
# dB/Hz or Pascals squared/Hz. Similarly, the narrowband spectrum can be input with 
# units of dB/Bin of Pascals squared/Bin.  The tone spectrum consists of pure tones at 
# the provide frequencies, input with units of Pascals squared. This function calculates 
# the center frequencies based on the proportional number requested and the band type. 
# The result is returned in two arrays that are allocated by this routine.  The arrays 
# are filled with the center band frequencies and proportional band levels that were 
# able to be calculated by this routine.  The number and spacing of the proportional 
# band center frequencies are determined by the proportional number (3 for 1/3, etc.).
# -----------------------------------------------------------------------------------------
# @param enumMetric
#        This input parameter identifies the input type of the independent and dependent
#        variables.  This must be one of the supported metrics in the enumeration for
#        metrics.  Supported metrics include: a2_aa_psd, a2_aa_nb, a2_aa_pts.
# @param enumUnits
#        These are the units of the input.  Supported units include a2_aa_msp and
#        a2_aa_db.
# @param intM
#        The dimension of the spectrum and frequency arrays.
# @param fltInputFrequencies
#        The frequencies of the input spectrum.  The frequency array must be
#        an evenly spaced array for power spectral density or narrowband.  This array 
#        will be used to calculate the lower and upper limits of the lower and upper 
#        most proportional band.  For PSD and NB, if a band does not contain enough 
#        information (frequency content) to fill a band, it is not returned.
# @param fltInputSpectrum
#        The input spectrum used to calculate the proportional band spectrum.
#        The values are at the frequencies given in the inputFrequencies array.
#        For PSD and NB, an integral of this array over the bounds of the proportional 
#        band is used to calculate the value at the proportional band center frequency.
#        For tones, the energy at each tone from this array is added to whichever band
#        it falls within.
# @param fltProportionalNumber
#        The proportional number, ex. 3 for 1/3rd, etc.  This number must be greater than
#        zero. (See Notes: #1, VALUE attribute)
# @param enumBandType
#        This enumeration will set the proportional band center frequency approximation
#        algorithm. This means the code will use an approximate algorithm instead of
#        the exact algorithm.  a2_aa_exact for exact, a2_aa_approximate for approximate,
#        and, a2_aa_preferred for preferred.
# @param intNb
#        The size of the frequency and noise arrays of the result proportional band
#        spectrum (next two arguments).
# @param fltCenterBandFrequencies
#        The frequency array of the proportional band spectrum.  This is the center band
#        frequencies of each band in the proportional band spectrum.  Each band 
#        represents a range of frequencies.  For PSD and NB, these center frequencies 
#        are only set if enough information is present to integrate over the entire 
#        band. For Tones, any energy in a band will cause the band to be present.
#        Since the Acoustic Analysis requires a minimum of two bands, if the Tonal 
#        energy only constitutes one band, a second band with a level corresponding to
#        the noise floor will be created.
# @param fltPbs
#        The integrated psd over the band width for each center frequency, or the tonal
#        energy that falls within the band. This array is the same size as the 
#        proportional band frequencies array (m)..
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_pbs.restype = A2_IK
ANOPP2.a2py_aa_pbs.argtypes =                                  \
     [A2_EK, A2_EK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_RK, A2_EK, \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK))]

  
  
#------------------------------------------------------------------------------------------
# This function splits a given third-octave band sound pressure levels into subbands or
# combines a given subbanded sound pressure levels into third-octave band sound pressure
# levels.
#------------------------------------------------------------------------------------------
# @param dmy_fltProportionalNumber  
#        The proportional number of the input data (i.e. for 1/3 octaveband this would 
#        be 3).
# @param dmy_enumBandType  
#        The band type to be used.  
#          a2_aa_exact is exact, 
#          a2_aa_preferred is preferred, and 
#          a2_aa_approximate is approximate.
#        Note: Approximate may only be used for splitting an octave into a 1/3 octave or
#        combining a 1/3 octave into an octave.  If approximate is indicated with any
#        other conditions, the exact method will be used instead.
# @param dmy_enumAction
#        An enumerator that determines if the 1/N Octave bands should be divided or 
#        combined. 
#          a2_aa_subband_divide:  Bands will be divided into dmy_intFactor sub-bands.
#          a2_aa_subband_combine: Every dmy_intFactor bands will be combined into a 
#                                 single band.
# @param dmy_intFactor  
#        The number n for band factoring (i.e. 3 would split each band into 3
#        sub-bands if dmy_enumAction is a2_aa_subband_divide or combine 3 bands into 1 
#        if dmy_enumAction is a2_aa_subband_combine.
# @param dmy_intM
#        This is the dimension of the input spectrum and the input frequencies arrays.
# @param dmy_fltInputFrequencies  
#        The frequency array of the input spectra.
# @param dmy_fltInputPbs  
#        The input spectra. These must be in dB/Band or MSP/Band.
# @param dmy_enumUnits
#        This is the enumerator for the units of the input spectrum.  Only a2_aa_db or
#        a2_aa_msp will be accepted.  Others will result in an error.
# @param dmy_intN
#        This is the dimension of the output spectrum and the output frequencies arrays.
# @param dmy_fltOutputFrequencies  
#        The frequency array for the output spectra.
# @param dmy_fltOutputPbs  
#        The input spectra. These will be in dB/Band.
# @result
#        An integer representing success of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_aa_subband_pbs.restype = A2_IK
ANOPP2.a2py_aa_subband_pbs.argtypes =                                          \
     [POINTER(A2_RK), A2_EK, A2_EK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_EK, \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK))]



# -----------------------------------------------------------------------------------------
# This routine calculates the overall sound pressure level from a power spectral
# density spectrum or a proportional band spectrum (1/3rd, 1/8th octave etc).  The inputs 
# to this function are N, the size of the input arrays, a noise spectrum which can be
# a power spectral density or a proportional band spectrum, a frequency array and units 
# of the noise.  If the units are set to 0, the noise argument is the power
# spectral density.  If the units are provided at 1, the noise argument is
# a proportional band spectrum.  The result is the overall sound pressure
# level, overall the frequencies.  An integer is returned that is the success of this
# routine. \n
# -----------------------------------------------------------------------------------------
# @param enumMetric
#        This input parameter identifies the input type of the independent and dependent
#        variables.  This must be one of the supported metrics in the enumeration for
#        metrics.  Supported metrics include: a2_aa_psd and a2_aa_nb.
# @param enumUnits
#        These are the units of the input.  Supported units include a2_aa_msp and
#        a2_aa_db.
# @param intM
#        The size of the noise and frequencies array.
# @param fltFrequencies
#        If this array is specified, this is the frequencies of the power spectral
#        density provided in the noise argument.  It has to be the same size as the
#        noise array.
# @param fltNnoise
#        Either the power spectral density or proportional band spectrum.  If the units 
#        are 0 this is the power spectral density and the overall level is calculated by
#        integrating over frequency.  If the units are 1 than this is the proportional 
#        band spectrum.  Since that is already integrated, the overall
#        level is the sum of the spectrum.
# @param fltL
#        The overall sound pressure level of the spectrum.  This is returned
#        by this routine.
# @result
#        A integer success flag that is zero if this routine has succeeded and
#        non-zero if it has not.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_oaspl.restype = A2_IK
ANOPP2.a2py_aa_oaspl.argtypes = \
     [A2_EK, A2_EK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine calculates the perceived noise level and tone-corrected perceived
# noise level from a given 1/3 octave sound pressure level array.  The SPL array
# must exist over the standard 24 1/3 octave bins.  These are from 50Hz to 10kHz.
# -----------------------------------------------------------------------------------------
# @param fltSpl
#        The 24 1/3 octave sound pressure level array.  These the SPL values at
#        the 1/3 octave center band frequencies from 50 to 10,000 Hz.
# @param blnIgnoreTones800HzAndBelow
#        Logical flag to ignore tones of 800 Hz and below
# @param fltPnl
#        The perceived noise level calculated from the 1/3 octave SPL.
# @param fltPnlt
#        The tone-corrected perceived noise level calculated from the 1/3 octave
#        sound pressure level.
# @param fltBandFrequency
#        This is the 1/3 octave Band SPL frequency that causes the highest tone
#        correction penalty.  This is returned as 0 Hz if no tone-correction.
# @param fltToneCorrection
#        This is the value of the tone correction at the above frequency.
# @result
#        A success integer that is returned 0 if this routine has succeeded.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_pnl_pnlt.restype = A2_IK
ANOPP2.a2py_aa_pnl_pnlt.argtypes =                                  \
     [POINTER(A2_RK), c_bool, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), \
      POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine calculates the effective perceived noise level from a series of
# tone-corrected perceived noise levels.  Inputs into this routine are the size of
# the time and PNLT arrays, a time array, and a PNLT array.  The result is a real
# value for the effective perceived noise level.  This routine returns a success
# parameter that is true if the routine has succeeded and false if it has failed.
# -----------------------------------------------------------------------------------------
# @param intN
#        The size of the time and tone-corrected perceived noise arrays.
# @param fltTime
#        An array of time where the PNLT values are computed.  This does not have to
#        evenly spaced in time.
# @param fltPnlt
#        Tone-corrected perceived noise level at the times in the time array.
# @param fltEpnl
#        Output Effective Perceived Noise Level.
# @param fltDuration
#        This is the duration factor D, calculated during the EPNL calculation.
# @param fltTimeRange
#        This is the minimum and maximum time of the EPNL calculation.
# @result
#        An integer that is returned 0 if this routine has succeeded.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_epnl.restype = A2_IK
ANOPP2.a2py_aa_epnl.argtypes =                                     \
     [A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), \
      POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine calculates the sound exposure level given a time array and an array
# of overall sound pressure levels.  This calculates the sound exposure level and
# returns a logical for success.
# -----------------------------------------------------------------------------------------
# @param intN
#        The size of the time and overall sound pressure level arrays
# @param fltTime
#        An array of time variables.  This are the time locations of the overall sound
#        pressure levels.  This does not have to be evenly spaced.
# @param fltL
#        An array of overall sound pressure levels at the time in the time array.
# @param fltSEL
#        Output Sound Exposure Level.
# @param fltDuration
#        This is the duration factor D, calculated during the SEL calculation.
# @param fltTimeRange
#        This is the minimum and maximum time of the SEL calculation.
# @result
#        An integer that is returned 0 if this routine has succeeded.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_sel.restype = A2_IK
ANOPP2.a2py_aa_sel.argtypes =                                      \
     [A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), \
      POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This function calculates the single event noise exposure level given a time array, an
# array of overall sound pressure levels, and a time range.  The routine calculates the
# single event noise exposure level and returns an integer representing the success or
# failure of the operation.
# -----------------------------------------------------------------------------------------
# @param intN
#        The size of the time and overall sound pressure level arrays.
# @param fltTime
#        An array of time variables.  This are the time locations of the overall sound
#        pressure levels.  This does not have to be evenly spaced.
# @param fltL
#        An array of overall sound pressure levels at the time in the time array.
# @param fltTimeRange
#        This is the minimum and maximum time of the SENEL calculation.
# @param fltSenel
#        Output Single Event Noise Exposure Level.
# @result
#        An integer that is returned 0 if this routine has succeeded.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_senel.restype = A2_IK
ANOPP2.a2py_aa_senel.argtypes = \
     [A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This function calculates the contour area using the zeroth-order method or first-order
# interpolation method for three-dimensional surfaces that have been defined with an
# unstructured grid and connectivity array.
# -----------------------------------------------------------------------------------------
# @param intPositions
#        Number of positions defining surface.
# @param fltPositions
#        Single dimension array representing the two dimensional array of positions.
# @param intCells
#        Number of cells defining surface.
# @param intConnectivity
#        Single dimension array representing a list of connectivity for the positions on
#        the surface.
# @param fltL
#        Single dimension array representing the two dimensional array of noise levels
#        (typically SEL or EPNL).
# @param fltContour
#        Contour level within which the area is to be calculated.
# @param enumMethod
#        Contour area method enumeration.
#        dmy_enumMethod = a2_aa_zeroth (Zeroth order method)
#        dmy_enumMethod = a2_aa_first (First order method)
# @param dmy_strTecplotFile
#        Character string for name of Tecplot file to be created.  This is not used for
#        zeroth order method.  If a blank string is passed, no Tecplot file is created. 
# @param fltArea
#        The area of all rectangles determined by specified method with noise levels
#        greater than or equal the noise level specified by dmy_fltContour.
# @result
#        An integer result that returns 0 if the area calculation is successfully.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_exposure_area_unstructured.restype = A2_IK
ANOPP2.a2py_aa_exposure_area_unstructured.argtypes =                    \
     [A2_IK, POINTER(A2_RK), A2_IK, POINTER(A2_IK), POINTER(A2_RK), A2_RK, A2_EK, \
      POINTER(A2_CK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This function calculates the contour area using the zeroth-order method or first-order
# interpolation method for three-dimensional surfaces that have been defined with a
# structured grid.
# -----------------------------------------------------------------------------------------
# @param intPositions
#        Number of positions defining surface.
# @param fltPositions
#        Single dimension array representing the two dimensional array of positions.
# @param fltL
#        Single dimension array representing the two dimensional array of noise levels
#        (typically SEL or EPNL).
# @param dmy_nGridNodes
#        Array dimensioned 2 to hold the number of points in each direction of the surface.
# @param fltContour
#        Contour level within which the area is to be calculated.
# @param enumMethod
#        Contour area method enumeration.
#        dmy_enumMethod = a2_aa_zeroth (Zeroth order method)
#        dmy_enumMethod = a2_aa_first (First order method)
# @param dmy_strTecplotFile
#        Character string for name of Tecplot file to be created.  This is not used for
#        zeroth order method.  If a blank string is passed, no Tecplot file is created. 
# @param fltArea
#        The area of all rectangles determined by specified method with noise levels
#        greater than or equal the noise level specified by dmy_fltContour.
# @result
#        An integer result that returns 0 if the area calculation is successfully.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_exposure_area_structured.restype = A2_IK
ANOPP2.a2py_aa_exposure_area_structured.argtypes =               \
     [A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_IK), A2_RK, A2_EK, \
      POINTER(A2_CK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine calculates the contour area using the zeroth order interpolation
# method or the first order interpolation method.
# -----------------------------------------------------------------------------------------
# @param intN
#        Integer number of X nodes.
# @param intM
#        Integer number of Y nodes.
# @param fltX
#        Single dimension array representing the two dimensional array of X coordinates.
# @param fltY
#        Single dimension array representing the two dimensional array of Y coordinates.
# @param fltL
#        Single dimension array representing the two dimensional array of noise levels
#        (typically SEL or EPNL).
# @param fltContour
#        Contour level within which the area is to be calculated.
# @param enumMethod
#        Contour area method enumeration.
# @param dmy_strTecplotFile
#        Character string for name of Tecplot file to be created.  This is not used for
#        zeroth order method.  If a blank string is passed, no Tecplot file is created. 
# @param fltArea
#        The area in square kilometers of all rectangles with center noise levels
#        greater than or equal to the noise level specified by Contour.
# @result
#        An integer result that returns 0 if the area calculation is successfully.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_exposure_area.restype = A2_IK
ANOPP2.a2py_aa_exposure_area.argtypes =                          \
     [A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), A2_RK, \
      A2_EK, POINTER(A2_CK), POINTER(A2_RK)]



#------------------------------------------------------------------------------------------
# This routine combines two acoustic spectra to produce a new spectrum.  The supported 
# metrics are acoustic pressure time histories, acoustic velocity time histories, tonal
# spectra, narrow band spectra, power spectral densities, and proportional band spectra.
# There are three supported combine methods, union, intersection, and merge.  In 
# addition, the rountine can combine absolute 
#------------------------------------------------------------------------------------------
# @param enumMetric
#        This is the enumerator for the metric type: APTH, AVTH, PGTH, NBS, PSD, 
#        Tones, or PBS
# @param enumUnits
#        This is the enumerator for the units.
# @param enumMethod
#        This is the method for combining the spectra: union, intersection, or merge.
# @param enumLevelsA
#        This is the enumerator for the level type of A, either absolute or delta.
# @param nPairsA
#        The number of pairs in the A spectrum, i.e. the number of independents
# @param nVectorA
#        The number of vectors in the A spectrum, i.e. the second dimension of the 
#        dependent variable array.
# @param fltIndependentsA
#        This is the array of independent values for A
# @param fltDependentsA
#        This is the array of dependent values for A.  It is two dimensional, where the 
#        first dimension corresponds to the number of independent values, and the second
#        dimension is either 3 (for acoustic velocities), 2 (for spectra with phase), or
#        1 (for everything else).  Note: the second dimension must match that of
#        spectra B for and acoustic velocity.  If only one spectra has phase, phase data
#        will be ignored.
# @param enumLevelsB
#        This is the enumerator for the level type of B, either absolute or delta.
# @param nPairsB
#        The number of pairs in the A spectrum, i.e. the number of independents
# @param nVectorB
#        The number of vectors in the A spectrum, i.e. the second dimension of the 
#        dependent variable array.
# @param fltIndependentsB
#        This is the array of independent values for B
# @param fltDependentsB
#        This is the array of dependent values for B.  It is two dimensional, where the 
#        first dimension corresponds to the number of independent values, and the second
#        dimension is either 3 (for acoustic velocities), 2 (for spectra with phase), or
#        1 (for everything else).  Note: the second dimension must match that of
#        spectra A for and acoustic velocity.
# @param enumLevelsC
#        This is the enumerator for the level type of the output, either absolute or 
#        delta.
# @param nPairsC
#        The number of pairs in the output spectrum, i.e. the number of independents
# @param nVectorC
#        The number of vectors in the output spectrum, i.e. the second dimension of the 
#        dependent variable array.
# @param fltIndependentsC
#        This is the output array of independent values
# @param fltDependentsC
#        This is the output array of dependent values.  It is two dimensional, where the 
#        first dimension corresponds to the number of independent values, and the second
#        dimension is either 3 (for acoustic velocities), 2 (for spectra with phase), or
#        1 (for everything else).  Note: the second dimension will always match that of 
#        the smaller of the inputs (that is, if only one spectra has phase, phase data
#        will be ignored).
# @result
#        An integer representing success of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_aa_combine.restype = A2_IK
ANOPP2.a2py_aa_combine.argtypes =                                     \
     [A2_EK, A2_EK, A2_EK, A2_EK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), \
      A2_EK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_EK),      \
      POINTER(A2_IK), POINTER(A2_IK), POINTER(POINTER(A2_RK)),                  \
      POINTER(POINTER(A2_RK))]



# -----------------------------------------------------------------------------------------
# This routine converts an input from one unit to another and determines the sensitivity
# of the conversion.  The supported units are defined in the acoustic units enumeration
# list and include pressure, pressure squared, and decibels.
# -----------------------------------------------------------------------------------------
# @param enumInput 
#        This is the enumerator for the units of the input.
# @param fltInput
#        This is the value of the input in the units defined by dmy_enumInput.  This
#        will be converted to a different unit defined by dmy_enumOutput.
# @param enumOutput
#        This is the enumeration of the desired units.  dmy_fltInput will be converted
#        to these units.
# @param fltOutput
#        This is the result of this function: dmy_fltInput which is in units
#        dmy_enumInput converted to the units defined by dmy_enumOutput.
# @param fltSensitivity
#        This is the sensitivity the convert calculation.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_convert_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_convert_sensitivity.argtypes =  \
     [A2_EK, A2_RK, A2_EK, POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine returns a segment of a long pressure time history and the sensitivity
# of the segmenting. This routine takes in the long acoustic pressure time history
# (including the time and pressure values) and desired segment size, step size, and
# the segment number wanted.  This routine returns the size of the new segment and
# the time and function of the segment.  An integer success parameter is returned by the
# function.  This is set to 0 if the function succeeded.  If function failed, a non-zero
# integer is returned (check error log).  This can occur if the segment size or step size
# are less than zero, or the segment number requested is less than 1.  If the segment
# size or segment step size do not fall on exact values of the discrete time samples in
# the long time and long func arrays, they are 'fudged' slightly to fall on exact samples.
# -----------------------------------------------------------------------------------------
# @param intNl
#        The size of the long acoustic pressure time history.  This is the size of the
#        longTime and longFunc arrays.
# @param fltLongTime
#        The long acoustic pressure time history.  This is used to create the segment.
# @param fltLongFunction
#        The long acoustic pressure time history.  This is the acoustic pressure at
#        the time samples in the long time array.
# @param fltTs
#        The size of the segment requested.  This number might be 'fudged' slightly to
#        fall on a sample in the long time and long func arrays.  This is returned
#        modified.
# @param fltS
#        The size of the incremental step between segments.  This can be less than or
#        greater than the segment size.  This number may be 'fudged' slightly if the
#        size is not exactly divisible by the time step size in the long time array.
# @param intN
#        The segment number in the long time history wanted.  This number must be
#        greater than 1 and less than the maximum number of segments possible.
# @param intNs
#        This is the size of the resultant segmented time and function arrays.  This
#        is returned by this subroutine.
# @param fltSegmentTime
#        The time array of the segment data.  This array is allocated by this routine
#        to size M (param m).  The last time step minimum the first time step should be
#        equal to the 'fudged' segment size.
# @param fltSegmentFunction
#        The function array of the segmented data.  This is the function at the time
#        steps in the segment time array.
# @param nNonZero
#        The number of non-zero sensitivity array values.
# @param fltNonZeroValues
#        Matrix of sensitivity values of the segmentation (only non-zero values
#        of the what is really a larger two-dimensional array are stored in a
#        Coordinate matrix).
# @param intRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# @result
#        An integer representation of success.  Return of 0 indicates no errors.
#        A non-zero return indicates an error (see error log).
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_segment_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_segment_sensitivity.argtypes =                        \
     [A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK),   \
      A2_IK, POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),        \
      POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This routine windows an acoustic pressure time history and determines the window
# sensitivity.  The window function is chosen via an enumerator passed to the routine
# as the first argument.  The size of the time array and function must be the same.  An
# integer success parameter is returned by the function.  This is set to 0 if the function
# succeeded.  If function failed, a non-zero integer is returned (check error log).
# -----------------------------------------------------------------------------------------
# @param enumWindow
#        The enumerated value of the window function to be applied.
# @param intN
#        The size of the function and time arrays.
# @param fltTime
#        The array of times where the acoustic pressure is sampled.  This must be an evenly
#        spaced array.
# @param fltFunction
#        The array of acoustic pressures to be windowed.  The values input are the 
#        function values at times in the time array.  This is overwritten by the 
#        windowed function.
# @param nNonZero
#        The number of non-zero sensitivity array values.
# @param fltNonZeroValues
#        Matrix of sensitivity values of the windowing with respect to the time history
#        (only non-zero values of the larger sensitivity array are stored in a
#        coordinate matrix).
# @param intRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# @result
#        An integer representation of success.  Return of 0 indicates no errors.
#        A non-zero return indicates an error (see error log).
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_window_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_window_sensitivity.argtypes =              \
     [A2_EK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_IK), \
      POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
#  This routine applies a high and low pass filter to a Pressure Time History including
#  the sensitivity of the filtered acoustic pressure time history with respect to the
#  unfiltered acoustic pressure time history.
# -----------------------------------------------------------------------------------------
# @param intN
#        This is the size of the Time and Pressure arrays.
# @param fltTime
#        This is the array of times for the Time History.
# @param fltPressure
#        This is the array of pressures for the Time History
# @param fltHighPassFrequency
#        This is the frequency for the low pass filter, which will set all lower
#        frequencies to zero.
# @param fltLowPassFrequency
#        This is the frequency for the high pass filter, which will set all higher
#        frequencies to zero.
# @param nNonZero
#        The number of non-zero sensitivity array values.
# @param fltNonZeroValues
#        Matrix of sensitivity values of the filtering with respect to the time history
#        (only non-zero values of the larger sensitivity array are stored in a
#        coordinate matrix).
# @param intRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# @result
#        A logical success flag.  If the routine succeeded, this is returned as
#        true.  If it failed, this is returned as false.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_filter_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_filter_sensitivity.argtypes =                     \
     [A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_RK, A2_RK, POINTER(A2_IK), \
      POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This routine calculates the sensitivity of the narrowband spectrum for two types of
# input.  The input types may be a pressure time history or a power spectral density.  The
# power spectral density may have units of dB/Hz or Pascals squared per Hz.  If the input
# is a pressure time history, the independent variable will be time and the dependent
# variable will be acoustic pressure.  If the is a power spectral density with units of
# dB/Hz, the independent variable will be frequency and the dependent variable will be
# decibels.  If the input is a power spectral density with units of Pascals squared per
# Hz, the independent variable will be frequency and the dependent variable will be
# acoustic pressure.  The parameter enumInputType  identifies the input type.  The
# integer intM is the size of the result arrays of frequency and narrowband spectrum.
# If the input is pressure time history, intM is returned as INT(intN/2) because of
# the Nyquist criteria.  If the input is a power spectral density, intM is input for
# the size of the resultant narrowband spectrum.
# -----------------------------------------------------------------------------------------
# @param enumMetric
#        This input parameter identifies the input type of the independent and dependent
#        variables.  This must be one of the supported metrics in the enumeration for
#        metrics.  Supported metrics include: a2_aa_apth and a2_aa_psd.
# @param enumUnits
#        These are the units of the input.  Supported units include a2_aa_ap, a2_aa_msp,
#        and a2_aa_db.  Not all combinations of metric and units are supported.  An
#        a2_aa_apth must be accompanied by a2_aa_ap.  An a2_aa_psd can be in either
#        a2_aa_msp or a2_aa_db.
# @param intN
#        Dimension of the independent and dependent arrays.
# @param fltIndepedent
#        The independent variable will be time when the input type is a pressure time
#        series.  The independent variable will be frequencies when dependent variable
#        is an octave sound pressure level or a power spectral density. The independent
#        variable must be evenly spaced times when the input type is a pressure time
#        history or a power spectral density.
# @param fltDependent
#        The dependent variable is an array of pressures squared or decibels depending
#        on the value of the parameter enumInputType.
# @param intM
#        The dimension of the resultant frequency and narrowband spectrum.  This number
#        is returned by this routine if the input is an acoustic pressure time series.
#        This number is input if the input is a power spectral density.
# @param fltFrequency
#        The frequencies of the result narrowband spectrum.  This array is filled and
#        returned by this routine.
# @param fltNbsMsp
#        The narrowband spectrum.  This array is filled, and returned by this routine.
# @param fltPhase
#        The phase of the narrowband spectrum.
# ================= Coo Matrix - Frequency Sensitivity WRT Independents ===================
# @param dmy_nDeltaFrequencyDeltaIndependentNonZero
#        The number of non-zero sensitivity array values for delta in frequency wrt to
#        delta in independent.
# @param dmy_fltDeltaFrequencyDeltaIndependentNonZeroValues
#        Matrix of sensitivity values for the delta in frequency wrt delta in independent.
# @param dmy_intDeltaFrequencyDeltaIndependentRowIndices
#        Matrix of row indices for non-zero sensitivity values for delta in frequency
#        wrt to delta in independent.
# @param dmy_intDeltaFrequencyDeltaIndependentColumnIndices
#        Matrix of column indices for non-zero sensitivity values for delta in frequency
#        wrt to delta in independent.
# ================= Coo Matrix - Frequency Sensitivity WRT Dependents   ===================
# @param dmy_nDeltaFrequencyDeltaDependentNonZero
#        The number of non-zero sensitivity array values for delta in frequency wrt to
#        delta in dependent.
# @param dmy_fltDeltaFrequencyDeltaDependentNonZeroValues
#        Matrix of sensitivity values for the delta in frequency wrt delta in dependent.
# @param dmy_intDeltaFrequencyDeltaDependentRowIndices
#        Matrix of row indices for non-zero sensitivity values for delta in frequency
#        wrt to delta in dependent.
# @param dmy_intDeltaFrequencyDeltaDependentColumnIndices
#        Matrix of column indices for non-zero sensitivity values for delta in frequency
#        wrt to delta in dependent.
# ================= Coo Matrix - NBS Msp Sensitivity WRT Independents   ===================
# @param dmy_nDeltaNbsMspDeltaIndependentNonZero
#        The number of non-zero sensitivity array values for delta in NBS MSP wrt to
#        delta in independent.
# @param dmy_fltDeltaNbsMspDeltaIndependentNonZeroValues
#        Matrix of sensitivity values for the delta in NBS MSP wrt delta in independent.
# @param dmy_intDeltaNbsMspDeltaIndependentRowIndices
#        Matrix of row indices for non-zero sensitivity values for delta in NBS MSP
#        wrt to delta in independent.
# @param dmy_intDeltaNbsMspDeltaIndependentColumnIndices
#        Matrix of column indices for non-zero sensitivity values for delta in NBS MSP
#        wrt to delta in independent.
# ================= Coo Matrix - NBS Msp Sensitivity WRT Dependents     ===================
# @param dmy_nDeltaNbsMspDeltaDependentNonZero
#        The number of non-zero sensitivity array values for delta in NBS MSP wrt to
#        delta in dependent.
# @param dmy_fltDeltaNbsMspDeltaDependentNonZeroValues
#        Matrix of sensitivity values for the delta in NBS MSP wrt delta in dependent.
# @param dmy_intDeltaNbsMspDeltaDependentRowIndices
#        Matrix of row indices for non-zero sensitivity values for delta in NBS MSP
#        wrt to delta in dependent.
# @param dmy_intDeltaNbsMspDeltaDependentColumnIndices
#        Matrix of column indices for non-zero sensitivity values for delta in NBS MSP
#        wrt to delta in dependent.
# ================= Coo Matrix - Phase Sensitivity WRT Independents     ===================
# @param dmy_nDeltaPhaseDeltaIndependentNonZero
#        The number of non-zero sensitivity array values for delta in phase wrt to
#        delta in independent.
# @param dmy_fltDeltaPhaseDeltaIndependentNonZeroValues
#        Matrix of sensitivity values for the delta in phase wrt delta in independent.
# @param dmy_intDeltaPhaseDeltaIndependentRowIndices
#        Matrix of row indices for non-zero sensitivity values for delta in phase
#        wrt to delta in independent.
# @param dmy_intDeltaPhaseDeltaIndependentColumnIndices
#        Matrix of column indices for non-zero sensitivity values for delta in phase
#        wrt to delta in independent.
# ================= Coo Matrix - Phase Sensitivity WRT Dependents       ===================
# @param dmy_nDeltaPhaseDeltaDependentNonZero
#        The number of non-zero sensitivity array values for delta in phase wrt to
#        delta in dependent.
# @param dmy_fltDeltaPhaseDeltaDependentNonZeroValues
#        Matrix of sensitivity values for the delta in phase wrt delta in dependent.
# @param dmy_intDeltaPhaseDeltaDependentRowIndices
#        Matrix of row indices for non-zero sensitivity values for delta in phase
#        wrt to delta in dependent.
# @param dmy_intDeltaPhaseDeltaDependentColumnIndices
#        Matrix of column indices for non-zero sensitivity values for delta in phase
#        wrt to delta in dependent.
# @result
#        An integer success flag.  If the routine succeeded, this is returned as
#        0.  If it failed, this is returned as a non-zero number.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_nbs_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_nbs_sensitivity.argtypes =                              \
     [A2_EK, A2_EK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_IK),       \
      POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),          \
      POINTER(POINTER(A2_IK)),                                                   \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),          \
      POINTER(POINTER(A2_IK)),                                                   \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),          \
      POINTER(POINTER(A2_IK)),                                                   \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),          \
      POINTER(POINTER(A2_IK)),                                                   \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),          \
      POINTER(POINTER(A2_IK)),                                                   \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),          \
      POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This routine calculates the sensitivity of the power spectral density for two types
# of input.  The input types may be a pressure time history or a narrowband spectrum.
# The narrowband spectrum may have units of dB/Hz or Pascals squared per Hz.  If the
# input is a pressure time history, the independent variable will be time and the
# dependent variable will be acoustic pressure.  If the input is a power spectral
# density with units of dB/Hz, the independent variable will be frequency and the
# dependent variable will be decibels.  If the input is a narrowband spectrum with
# units of Pascals squared per Hz, the independent variable will be frequency and the
# dependent variable will be acoustic pressure.  The parameter enumMetric  identifies
# the input type.  The integer intM is the size of the result arrays of frequency and
# narrowband spectrum.  If the input is pressure time history, M is returned as INT(N/2)
# because of the Nyquist criteria.  If the input is a narrowband spectrum, M  is input
# for the size of the resultant narrowband spectrum.
# This is the Fortran binding of this routine.
# -----------------------------------------------------------------------------------------
# @param enumMetric
#        This input parameter identifies the input type of the independent and dependent
#        variables.  This must be one of the supported metrics in the enumeration for
#        metrics.  Supported metrics include: a2_aa_apth and a2_aa_nbs.
# @param enumUnits
#        These are the units of the input.  Supported units include a2_aa_ap, a2_aa_msp,
#        and a2_aa_db.  Not all combinations of metric and units are supported.  An
#        a2_aa_apth must be accompanied by a2_aa_ap.  An a2_aa_nbs can be in either
#        a2_aa_msp or a2_aa_db.
# @param intN
#        The size of the time and pressure arrays provided to this routine.
# @param fltIndepedent
#        The independent variable will be time when the input type is a pressure time
#        series.  The independent variable will be frequencies when the dependent
#        variable is an octave sound pressure level or a narrowband spectrum. The
#        independent  variable must be evenly spaced times when the input type is a
#        pressure time history or a power spectral density.
# @param fltDependent
#        The acoustic pressure at the time values in the time array or the SPL values
#        of the octave sound pressure level.
# @param intM
#        The size of the resultant frequency and PSD arrays.  This number is returned
#        by this routine if the input is an acoustic pressure time series.
# @param fltFrequency
#        The frequencies of the result power spectral density.  This array is filled
#        and allocated by this routine.
# @param fltNbsMsp
#        The power spectral density of the pressure time history.  This array is
#        allocated, filled, and returned by this routine.
# @param fltPhase
#        The phase of the power spectral density
# ================ Coo Matrix - Frequency Sensitivity WRT Independents ====================
# @param dmy_nDeltaFrequencyDeltaIndependentNonZero
#        The number of non-zero sensitivity array values for delta in frequency wrt to
#        delta in independent.
# @param fltDeltaFrequencyDeltaIndependentNonZeroValues
#        Matrix of sensitivity values for the delta in frequency wrt delta in independent.
# @param intDeltaFrequencyDeltaIndependentRowIndices
#        Matrix of row indices for non-zero sensitivity values for delta in frequency
#        wrt to delta in independent.
# @param intDeltaFrequencyDeltaIndependentColumnIndices
#        Matrix of column indices for non-zero sensitivity values for delta in frequency
#        wrt to delta in independent.
# ================ Coo Matrix - Frequency Sensitivity WRT Dependents   ====================
# @param dmy_nDeltaFrequencyDeltaDependentNonZero
#        The number of non-zero sensitivity array values for delta in frequency wrt to
#        delta in dependent.
# @param fltDeltaFrequencyDeltaDependentNonZeroValues
#        Matrix of sensitivity values for the delta in frequency wrt delta in dependent.
# @param intDeltaFrequencyDeltaDependentRowIndices
#        Matrix of row indices for non-zero sensitivity values for delta in frequency
#        wrt to delta in dependent.
# @param intDeltaFrequencyDeltaDependentColumnIndices
#        Matrix of column indices for non-zero sensitivity values for delta in frequency
#        wrt to delta in dependent.
# ================= Coo Matrix - PSD Msp Sensitivity WRT Independents =====================
# @param dmy_nDeltaPsdMspDeltaIndependentNonZero
#        The number of non-zero sensitivity array values for delta in PSD MSP wrt to
#        delta in independent.
# @param fltDeltaPsdMspDeltaIndependentNonZeroValues
#        Matrix of sensitivity values for the delta in PSD MSP wrt delta in independent.
# @param intDeltaPsdMspDeltaIndependentRowIndices
#        Matrix of row indices for non-zero sensitivity values for delta in PSD MSP
#        wrt to delta in independent.
# @param intDeltaPsdMspDeltaIndependentColumnIndices
#        Matrix of column indices for non-zero sensitivity values for delta in PSD MSP
#        wrt to delta in independent.
# ================= Coo Matrix - PSD Msp Sensitivity WRT Dependents   =====================
# @param dmy_nDeltaPsdMspDeltaDependentNonZero
#        The number of non-zero sensitivity array values for delta in PSD MSP wrt to
#        delta in dependent.
# @param fltDeltaPsdMspDeltaDependentNonZeroValues
#        Matrix of sensitivity values for the delta in PSD MSP wrt delta in dependent.
# @param intDeltaPsdMspDeltaDependentRowIndices
#        Matrix of row indices for non-zero sensitivity values for delta in PSD MSP
#        wrt to delta in dependent.
# @param intDeltaPsdMspDeltaDependentColumnIndices
#        Matrix of column indices for non-zero sensitivity values for delta in PSD MSP
#        wrt to delta in dependent.
# ================= Coo Matrix - Phase Sensitivity WRT Independents =======================
# @param dmy_nDeltaPhaseDeltaIndependentNonZero
#        The number of non-zero sensitivity array values for delta in phase wrt to
#        delta in independent.
# @param fltDeltaPhaseDeltaIndependentNonZeroValues
#        Matrix of sensitivity values for the delta in phase wrt delta in independent.
# @param intDeltaPhaseDeltaIndependentRowIndices
#        Matrix of row indices for non-zero sensitivity values for delta in phase
#        wrt to delta in independent.
# @param intDeltaPhaseDeltaIndependentColumnIndices
#        Matrix of column indices for non-zero sensitivity values for delta in phase
#        wrt to delta in independent.
# ================= Coo Matrix - Phase Sensitivity WRT Dependents   =======================
# @param dmy_nDeltaPhaseDeltaDependentNonZero
#        The number of non-zero sensitivity array values for delta in phase wrt to
#        delta in dependent.
# @param fltDeltaPhaseDeltaDependentNonZeroValues
#        Matrix of sensitivity values for the delta in phase wrt delta in dependent.
# @param intDeltaPhaseDeltaDependentRowIndices
#        Matrix of row indices for non-zero sensitivity values for delta in phase
#        wrt to delta in dependent.
# @param intDeltaPhaseDeltaDependentColumnIndices
#        Matrix of column indices for non-zero sensitivity values for delta in phase
#        wrt to delta in dependent.
# @result
#        An integer success flag.  If the routine succeeded, this is returned as
#        0.  If it failed, this is returned as a non-zero number
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_psd_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_psd_sensitivity.argtypes =                              \
     [A2_EK, A2_EK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_IK),       \
      POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),          \
      POINTER(POINTER(A2_IK)),                                                   \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),          \
      POINTER(POINTER(A2_IK)),                                                   \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),          \
      POINTER(POINTER(A2_IK)),                                                   \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),          \
      POINTER(POINTER(A2_IK)),                                                   \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),          \
      POINTER(POINTER(A2_IK)),                                                   \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),          \
      POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This routine alters a function as to a frequency weighting parameter and provides the
# sensitivity of the weighted spectrum with respect to the input spectrum.  Possible
# weighting functions are: A, B, C, and none.  This function takes in a frequency
# and noise array and an enumerator for the weighting function.  At each frequency
# a weight is calculated.  This weight is multiplied times the function which is
# overwritten and returned.  This routine also returns an integer success flag which
# will be a non-zero value if the weight function is invalid.
# -----------------------------------------------------------------------------------------
# @param enumWeight
#        An integer representing the weighting that is applied.  Refer to the manual
#        for specifics on the enumeration options of frequency weighting.
# @param enumMetric
#        This is an enumeration for the acoustic metric that is provided as input.  The
#        options include a2_aa_nb for Narrowband Spectrum and a2_aa_psd for Power
#        Spectral Density.
# @param enumUnits
#        An enumeration identifying the units of the input function.  This is specified
#        via the units enumerations.
# @param intM
#        The size of the function that is to be weighted.  This integer is the size
#        of the following 2 arguments.
# @param fltFrequencies
#        The frequencies of the spectrum that is to be weighted.  This is used to
#        calculate the weighting function at that frequency that is applied to the
#        function.
# @param fltFunction
#        The function that is to be weighted.  The weighting parameter is a function
#        of frequency.  For each frequency, that weighting function is multiplied
#        time the function.  The result is placed into this array.
# @param nNonZero
#        The number of non-zero sensitivity array values.
# @param fltNonZeroValues
#        Matrix of sensitivity values.  The large sensitivity matrix is a square matrix
#        whose side size is the number of frequencies.  The first dimension cooresponds
#        with the frequencies of the input (i.e. goes along with the function provided by
#        the user).  The second dimension (also frequency) is the sensitivity of thei
#        answer with respect to each function input.  However, the 2-dimensional
#        sensitivity array has values only on the diagonal.  We will use a Coordinate
#        matrix to store only non-zero values of the larger sensitivity array.
# @param intRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# @result
#        An integer representing the success of this program.  This will be returned
#        as non zero if this routine fails.  Reason for failing may be an unknown
#        weight parameter.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_weight_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_weight_sensitivity.argtypes =                            \
     [A2_EK, A2_EK, A2_EK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_IK), \
      POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This routine calculates the proportional band spectrum given a power spectral density or 
# a narrowband spectrum including the sensitivity of the proportional band spectrum's SPL 
# levels with respect to the input.  The psd can be input with units of dB/Hz of Pascals 
# squared/Hz. Similarly, the narrowband spectrum can be input with units of dB/Bin of 
# Pascals squared/Bin.  This function calculates the center frequencies based on the 
# proportional number requested and the band type.  The result is returned in two arrays 
# that are allocated by this routine.  The arrays are filled with the center band 
# frequencies and proportional band levels that were able to be calculated by this routine.  
# The number and spacing of the proportional band center frequencies are determined by the 
# proportional number (3 for 1/3, etc.).
# -----------------------------------------------------------------------------------------
# @param enumMetric
#        This input parameter identifies the input type of the independent and dependent
#        variables.  This must be one of the supported metrics in the enumeration for
#        metrics.  Supported metrics include: a2_aa_nbs and a2_aa_psd.
# @param enumUnits
#        These are the units of the input.  Supported units include a2_aa_msp and
#        a2_aa_db.
# @param intM
#        The dimension of the spectrum and frequency arrays.
# @param fltFrequencies
#        The frequencies of the input spectrum.  The frequency array must be
#        an evenly spaced array.  This array will be used to calculate the lower
#        and upper limits of the lower and upper most proportional band.  If a band does
#        not contain enough information (frequency content) to fill a band, it is
#        not returned.
# @param fltSpectrum
#        The input spectrum used to calculate the proportional band spectrum.
#        The values are at the frequencies given in the inputFrequencies array.
#        An integral of this array over the bounds of the proportional band is used to
#        calculate the value at the proportional band center frequency.
# @param fltProportionalNumber
#        The proportional number, ex. 3 for 1/3rd, etc.  This number must be greater than
#        zero. (See Notes: #1, VALUE attribute)
# @param enumBandType
#        This enumeration will set the proportional band center frequency approximation
#        algorithm. This means the code will use an approximate algorithm instead of
#        the exact algorithm.  a2_aa_exact for exact, a2_aa_approximate for approximate,
#        and, a2_aa_preferred for preferred.
# @param intNb
#        The size of the frequency and noise arrays of the result proportional band
#        spectrum (next two arguments).
# @param fltCenterBandFrequencies
#        The frequency arrays of the proportional band spectrum.  This is the center band
#        frequencies of each band in the proportional band spectrum.  Each band represents 
#        a range of frequencies, these center frequencies are only set if enough 
#        information is present to integrate over the entire band.
# @param fltPbs
#        The integrated psd over the band width for each center frequency. This array
#        is the same size as the proportional band spectrum frequencies array (m).
# @param nNonZero
#        The number of non-zero sensitivity array values.
# @param fltNonZeroValues
#        Matrix of sensitivity values of the proportional band spectrum calculation (only 
#        non-zero values of the what is really a larger two-dimensional array are stored 
#        in a one-dimensional array).
# @param intRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# @result
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_pbs_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_pbs_sensitivity.argtypes =                      \
     [A2_EK, A2_EK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_RK, A2_EK, \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)),  \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),  \
      POINTER(POINTER(A2_IK))]



#------------------------------------------------------------------------------------------
# This function splits a given third-octave band sound pressure levels into subbands or
# combines a given subbanded sound pressure levels into third-octave band sound pressure
# levels.  The sensitivity of the subbanding or combining is also determined.
#------------------------------------------------------------------------------------------
# @param dmy_fltProportionalNumber  
#        The proportional number of the input data (i.e. for 1/3 octaveband this would 
#        be 3).
# @param dmy_enumBandType  
#        The band type to be used.  
#          a2_aa_exact is exact, 
#          a2_aa_preferred is preferred, and 
#          a2_aa_approximate is approximate.
#        Note: Approximate may only be used for splitting an octave into a 1/3 octave or
#        combining a 1/3 octave into an octave.  If approximate is indicated with any
#        other conditions, the exact method will be used instead.
# @param enumAction
#        An enumerator that determines if the 1/N Octave bands should be divided or 
#        combined. 
#          a2_aa_subband_divide:  Bands will be divided into dmy_intFactor sub-bands.
#          a2_aa_subband_combine: Every dmy_intFactor bands will be combined into a 
#                                 single band.
# @param dmy_intFactor  
#        The number n for band factoring (i.e. 3 would split each band into 3
#        sub-bands if dmy_enumAction is a2_aa_subband_divide or combine 3 bands into 1 
#        if dmy_enumAction is a2_aa_subband_combine).
# @param dmy_intM
#        This is the dimension of the input spectrum and the input frequencies arrays.
# @param dmy_fltInputFrequencies  
#        The frequency array of the input spectra.
# @param dmy_fltInputSpectrum  
#        The input spectra. These must be in dB/Band or MSP/Band.
# @param dmy_enumUnits
#        This is the enumerator for the units of the input spectrum.  Only a2_aa_db or
#        a2_aa_msp will be accepted.  Others will result in an error.
# @param dmy_intN
#        This is the dimension of the output spectrum and the output frequencies arrays.
# @param dmy_fltOutputFrequencies  
#        The frequency array for the output spectra.
# @param dmy_fltOutputPbs  
#        The input spectra. These will be in dB/Band.
# ================= Coo Matrix - Frequency Sensitivity WRT Frequencies ====================
# @param dmy_nDeltaFrequenciesDeltaFrequenciesNonZero
#        The number of non-zero sensitivity array values for frequencies with respect
#        to a change in frequency.
# @param fltNonZeroValuesDeltaFrequenciesDeltaFrequencies
#        Matrix of non-zero sensitivity values of frequences with respect to a change
#        in frequency.
# @param intRowIndicesDeltaFrequenciesDeltaFrequencies
#        Matrix of row indices for non-zero sensitivity values.
# @param intColumnIndicesDeltaFrequenciesDeltaFrequencies
#        Matrix of column indices for non-zero sensitivity values.
# ================= Coo Matrix - Frequency Sensitivity WRT Spls        ====================
# @param dmy_nDeltaFrequenciesDeltaSplsNonZero
#        The number of non-zero sensitivity array values for frequencies with respect
#        to a change in SPL.
# @param fltNonZeroValuesDeltaFrequenciesDeltaSpls
#        Matrix of non-zero sensitivity values of frequences with respect to a change
#        in SPL.
# @param intRowIndicesDeltaFrequenciesDeltaSpls
#        Matrix of row indices for non-zero sensitivity values.
# @param intColumnIndicesDeltaFrequenciesDeltaSpls
#        Matrix of column indices for non-zero sensitivity values.
# ================= Coo Matrix - SPL Sensitivity WRT Frequencies       ====================
# @param dmy_nDeltaSplsDeltaFrequenciesNonZero
#        The number of non-zero sensitivity array values for SPLs with respect
#        to a change in frequency.
# @param fltNonZeroValuesDeltaSplsDeltaFrequencies
#        Matrix of non-zero sensitivity values of SPLs with respect to a change
#        in frequency.
# @param intRowIndicesDeltaSplsDeltaFrequencies
#        Matrix of row indices for non-zero sensitivity values.
# @param intColumnIndicesDeltaSplsDeltaFrequencies
#        Matrix of column indices for non-zero sensitivity values.
# ================= Coo Matrix - SPL Sensitivity WRT Spls              ====================
# @param dmy_nDeltaSplsDeltaSplsNonZero
#        The number of non-zero sensitivity array values for SPLs with respect
#        to a change in Spl.
# @param fltNonZeroValuesDeltaSplsDeltaSpls
#        Matrix of non-zero sensitivity values of SPLs with respect to a change
#        in Spl.
# @param intRowIndicesDeltaSplsDeltaSpls
#        Matrix of row indices for non-zero sensitivity values.
# @param intColumnIndicesDeltaSplsDeltaSpls
#        Matrix of column indices for non-zero sensitivity values.
# @result
#        An integer representing success of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_aa_subband_pbs_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_subband_pbs_sensitivity.argtypes =                              \
     [POINTER(A2_RK), A2_EK, A2_EK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_EK, \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_RK)), POINTER(A2_IK),  \
      POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK)),         \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),                  \
      POINTER(POINTER(A2_IK)), POINTER(A2_IK), POINTER(POINTER(A2_RK)),                  \
      POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK)), POINTER(A2_IK),                  \
      POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This function calculates the sensitivity overall sound pressure level due to changes in
# power spectral density, a narrowband spectrum, or a proportional band spectrum (1/3rd, 
# 1/8th octave etc).  The inputs to this function are the size of the input arrays defined 
# by the parameter intM, a noise spectrum, the units of the noise spectrum, and a 
# frequency array.  The result is the sensitivty of the overall sound pressure level as 
# well as the value of overall sound pressure level.  An integer is returned that is the 
# success of this function.
# -----------------------------------------------------------------------------------------
# @param enumMetric
#        This input parameter identifies the input type of the independent and dependent
#        variables.  This must be one of the supported metrics in the enumeration for
#        metrics.  Supported metrics include: a2_aa_psd and a2_aa_nb.
# @param enumUnits
#        These are the units of the input.  Supported units include a2_aa_msp and
#        a2_aa_db.
# @param intM
#        The size of the noise and frequencies array.
# @param fltFrequencies
#        If this array is specified, this is the frequencies of the power spectral
#        density provided in the noise argument.  It has to be the same size as the
#        noise array.
# @param fltNnoise
#        Either the power spectral density or proportional band spectrum.  If the units are 
#        0 this is the power spectral density and the overall level is calculated by
#        integrating over frequency.  If the units are 1 than this is a proportional band
#        spectrum.  Since that is already integrated, the overall
#        level is the sum of the spectrum.
# @param fltL
#        The overall sound pressure level of the spectrum.  This is returned
#        by this routine.
# @param nNonZero
#        The number of non-zero sensitivity array values.
# @param fltNonZeroValues
#        Matrix of sensitivity values of the OASPL with respect to the input.  In other
#        words: dL/dS_i where S_i is the value at each frequency of the input (only non-
#        zero values of the larger sensitivity array are stored in a coordinate matrix).
# @param intRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# @result
#        A integer success flag that is zero if this routine has succeeded and
#        non-zero if it has not.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_oaspl_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_oaspl_sensitivity.argtypes =                      \
     [A2_EK, A2_EK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),    \
      POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This routine calculates the sensitivity of the perceived noise level and tone-
# corrected perceived noise level from a given 1/3 octave sound pressure level array.
# The SPL array must exist over the standard 24 1/3 octave bins.  These are from 50Hz
# to 10kHz.
# -----------------------------------------------------------------------------------------
# @param fltSpl
#        The 24 1/3 octave sound pressure level array.  These the SPL values at
#        the 1/3 octave center band frequencies from 50 to 10,000 Hz.
# @param blnIgnoreTones800HzAndBelow
#        Logical flag to ignore tones of 800 Hz and below
# @param fltPnl
#        The perceived noise level calculated from the 1/3 octave SPL.
# @param fltPnlt
#        The tone-corrected perceived noise level calculated from the 1/3 octave
#        sound pressure level.
# @param fltBandFrequency
#        This is the 1/3 octave Band SPL frequency that causes the highest tone
#        correction penalty.  This is returned as 0 Hz if no tone-correction.
# @param fltToneCorrection
#        This is the value of the tone correction at the above frequency.
# =========================== Coo Matrix - Pnl Sensitivity ================================
# @param nPnlNonZero
#        The number of non-zero sensitivity array values for PNL.
# @param fltDeltaPnlDeltaSpl
#        Matrix of sensitivity values for PNL (only non-zero values of the larger array
#        are stored in a coordinate matrix).
# @param intPnlRowIndices
#        Matrix of row indices for non-zero sensitivity values for PNL.
# @param intPnlColumnIndices
#        Matrix of column indices for non-zero sensitivity values for PNL.
# =========================== Coo Matrix - Pnlt Sensitivity ===============================
# @param nPnltNonZero
#        The number of non-zero sensitivity array values for PNLT.
# @param fltDeltaPnltDeltaSpl
#        Matrix of sensitivity values for PNLT (only non-zero values of the larger array
#        are stored in a coordinate matrix).
# @param intPnltRowIndices
#        Matrix of row indices for non-zero sensitivity values for PNLT.
# @param intPnltColumnIndices
#        Matrix of column indices for non-zero sensitivity values for PNLT.
# =========================== Coo Matrix - Tone correction Sensitivity ====================
# @param nToneCorrectionNonZero
#        The number of non-zero sensitivity array values for tone correction.
# @param fltDeltaToneCorrectionDeltaSpl
#        Matrix of sensitivity values for tone correction (only non-zero values of the
#        larger array are stored in a coordinate matrix).
# @param intToneCorrectionRowIndices
#        Matrix of row indices for non-zero sensitivity values for tone correction.
# @param intToneCorrectionColumnIndices
#        Matrix of column indices for non-zero sensitivity values for tone correction.
# @result
#        A success integer that is returned 0 if this routine has succeeded.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_pnl_pnlt_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_pnl_pnlt_sensitivity.argtypes =                                \
     [POINTER(A2_RK), c_bool, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK),           \
      POINTER(A2_RK), POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), \
      POINTER(POINTER(A2_IK)), POINTER(A2_IK), POINTER(POINTER(A2_RK)),                 \
      POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK)), POINTER(A2_IK),                 \
      POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This function calculates the sensitivity of the sound exposure level given a time array
# and an array of overall sound pressure levels.  This calculates the sound exposure level
# and returns an integer representing the success failure of the operation.
# -----------------------------------------------------------------------------------------
# @param intN
#        The size of the time and overall sound pressure level arrays.
# @param fltTime
#        An array of time variables.  This are the time locations of the overall sound
#        pressure levels.  This does not have to be evenly spaced.
# @param fltL
#        An array of overall sound pressure levels at the time in the time array.
# @param fltSEL
#        Output Sound Exposure Level.
# @param fltDuration
#        This is the duration factor D, calculated during the SEL calculation.
# @param fltTimeRange
#        This is the minimum and maximum time of the SEL calculation.
# =========================== Coo Matrix - Sel Sensitivity ================================
# @param nSelNonZero
#        The number of non-zero sensitivity array values.
# @param fltDeltaSelDeltaOaspl
#        Matrix of sensitivity values of SEL with respect to OASPL.
# @param intSelRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intSelColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# =========================== Coo Matrix - Duration Sensitivity ===========================
# @param nDurationNonZero
#        The number of non-zero sensitivity array values.
# @param fltDeltaDurationDeltaOaspl
#        Matrix of sensitivity values for the sensitivity of the duration factor.
# @param intDurationRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intDurationColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# =========================== Coo Matrix - Time range Sensitivity =========================
# @param nTimeRangeNonZero
#        The number of non-zero sensitivity array values.
# @param fltDeltaTimeRangeDeltaOaspl
#        Matrix of sensitivity values for the sensitivity of the time range with respect
#        to the input.
# @param intTimeRangeRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intTimeRangeColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# @result
#        An integer that is returned 0 if this routine has succeeded.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_sel_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_sel_sensitivity.argtypes =                                     \
     [A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK),            \
      POINTER(A2_RK), POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), \
      POINTER(POINTER(A2_IK)), POINTER(A2_IK), POINTER(POINTER(A2_RK)),                 \
      POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK)), POINTER(A2_IK),                 \
      POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This function calculates the sensitivity of the single event noise exposure level given
# a time array, an array of overall sound pressure levels, and a time range.  This
# calculates the single event noise exposure level and returns an integer representing the
# success or failure of the operation.
# -----------------------------------------------------------------------------------------
# @param intN
#        The size of the time and overall sound pressure level arrays.
# @param fltTime
#        An array of time variables.  This are the time locations of the overall sound
#        pressure levels.  This does not have to be evenly spaced.
# @param fltL
#        An array of overall sound pressure levels at the time in the time array.
# @param fltTimeRange
#        This is the minimum and maximum time of the SENEL calculation.
# @param fltSenel
#        Output Single Event Noise Exposure Level.
# @param nNonZero
#        The number of non-zero sensitivity array values.
# @param fltNonZeroValues
#        Matrix of SENEL sensitivity values with respect to the overall levels that were
#        provided as input (only non-zero values of the larger sensitivity array are
#        stored in a coordinate matrix).
# @param intRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# @result
#        An integer that is returned 0 if this routine has succeeded.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_senel_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_senel_sensitivity.argtypes =                        \
     [A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), \
      POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),      \
      POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This function calculates the sensitivity of the effective perceived noise level given 
# a time array and an array of tone corrected perceived noise levels.  This calculates
# the EPNL and returns an integer for success.  This routine also calculates the
# sensitivity of the EPNL with respect to the inputs.
# -----------------------------------------------------------------------------------------
# @param intN
#        The size of the time and tone-corrected perceived noise arrays.
# @param fltTime
#        An array of time where the PNLT values are computed.  This does not have to
#        evenly spaced in time.
# @param fltPnlt
#        Tone-corrected perceived noise level at the times in the time array.
# @param fltEpnl
#        Output Effective Perceived Noise Levels.
# @param fltDuration
#        This is the duration factor D, calculated during the EPNL calculation.
# @param fltTimeRange
#        This is the minimum and maximum time of the EPNL calculation.
# =========================== Coo Matrix - Epnl Sensitivity ===============================
# @param nEpnlNonZero
#        The number of non-zero sensitivity array values.
# @param fltDeltaEpnlDeltaPnlt
#        Matrix of sensitivity values of the EPNL with respect to the perceived noise
#        levels that were provided as input.
# @param intEpnlRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intEpnlColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# =========================== Coo Matrix - Duration Sensitivity ===========================
# @param nDurationNonZero
#        The number of non-zero sensitivity array values.
# @param fltDeltaDurationDeltaPnlt
#        Matrix of sensitivity values for the sensitivity of the duration faction.
# @param intDurationRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intDurationColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# =========================== Coo Matrix - Time range Sensitivity =========================
# @param nTimeRangeNonZero
#        The number of non-zero sensitivity array values.
# @param fltDeltaTimeRangeDeltaPnlt
#        Matrix of sensitivity values for the sensitivity of the time range with respect
#        to the input.
# @param intTimeRangeRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intTimeRangeColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# @result
#        An integer that is returned 0 if this routine has succeeded.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_epnl_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_epnl_sensitivity.argtypes =                                    \
     [A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK),            \
      POINTER(A2_RK), POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), \
      POINTER(POINTER(A2_IK)), POINTER(A2_IK), POINTER(POINTER(A2_RK)),                 \
      POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK)), POINTER(A2_IK),                 \
      POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This function calculates the contour area using first-order interpolation method for
# three-dimensional surfaces that have been defined with an unstructured grid and
# connectivity array.
# -----------------------------------------------------------------------------------------
# @param intPositions
#        Number of positions defining surface.
# @param fltPositions
#        Single dimension array representing the two dimensional array of positions.
# @param intCells
#        Number of cells defining surface.
# @param intConnectivity
#        Single dimension array representing a list of connectivity for the positions on
#        the surface.
# @param fltL
#        Single dimension array representing the two dimensional array of noise levels
#        (typically SEL or EPNL).
# @param fltContour
#        Contour level within which the area is to be calculated.
# @param enumMethod
#        Contour area method enumeration.
#        dmy_enumMethod = a2_aa_zeroth (Zeroth order method)
#        dmy_enumMethod = a2_aa_first (First order method)
# @param dmy_strTecplotFile
#        Character string for name of Tecplot file to be created.  This is not used for
#        zeroth order method.  If a blank string is passed, no Tecplot file is created. 
# @param fltArea
#        The area of all rectangles determined by specified method with noise levels
#        greater than or equal the noise level specified by dmy_fltContour.
# @param dmy_nNonZero
#        The number of non-zero sensitivity array values.
# @param dmy_fltNonZeroValues
#        Matrix of sensitivity values of the exposure area (only non-zero values
#        of the larger two-dimensional array are stored in a Coordinate matrix).
# @param dmy_intRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param dmy_intColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# @result
#        An integer result that returns 0 if the area calculation is successfully.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_exposure_area_unstructured_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_exposure_area_unstructured_sensitivity.argtypes =        \
     [A2_IK, POINTER(A2_RK), A2_IK, POINTER(A2_IK), POINTER(A2_RK), A2_RK, A2_EK, \
      POINTER(A2_CK), POINTER(A2_RK), POINTER(A2_IK), POINTER(POINTER(A2_RK)),    \
      POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This function calculates the contour area using first-order interpolation method for
# three-dimensional surfaces that have been defined with a structured grid.
# -----------------------------------------------------------------------------------------
# @param intPositions
#        Number of positions defining surface.
# @param fltPositions
#        Single dimension array representing the two dimensional array of positions.
# @param fltL
#        Single dimension array representing the two dimensional array of noise levels
#        (typically SEL or EPNL).
# @param dmy_nGridNodes
#        Array dimensioned 2 to hold the number of points in each direction of the surface.
# @param fltContour
#        Contour level within which the area is to be calculated.
# @param enumMethod
#        Contour area method enumeration.
#        dmy_enumMethod = a2_aa_zeroth (Zeroth order method)
#        dmy_enumMethod = a2_aa_first (First order method)
# @param dmy_strTecplotFile
#        Character string for name of Tecplot file to be created.  This is not used for
#        zeroth order method.  If a blank string is passed, no Tecplot file is created. 
# @param fltArea
#        The area of all rectangles determined by specified method with noise levels
#        greater than or equal the noise level specified by dmy_fltContour.
# @param dmy_nNonZero
#        The number of non-zero sensitivity array values.
# @param dmy_fltNonZeroValues
#        Matrix of sensitivity values of the exposure area (only non-zero values
#        of the larger two-dimensional array are stored in a Coordinate matrix).
# @param dmy_intRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param dmy_intColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# @result
#        An integer result that returns 0 if the area calculation is successfully.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_exposure_area_structured_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_exposure_area_structured_sensitivity.argtypes =       \
     [A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_IK), A2_RK, A2_EK,     \
      POINTER(A2_CK), POINTER(A2_RK), POINTER(A2_IK), POINTER(POINTER(A2_RK)), \
      POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This function calculates the sensitivity of exposure area when the first-order method
# is used.
# -----------------------------------------------------------------------------------------
# @param intN
#        Integer number of X nodes.
# @param intM
#        Integer number of Y nodes.
# @param fltX
#        Single dimension array representing the two dimensional array of X coordinates.
# @param fltY
#        Single dimension array representing the two dimensional array of Y coordinates.
# @param fltL
#        Single dimension array representing the two dimensional array of noise levels
#        (typically SEL or EPNL).
# @param fltContour
#        Contour level within which the area is to be calculated.
# @param enumMethod
#        Contour area method enumeration.
# @param fltArea
#        The area in square kilometers of all rectangles with center noise levels
#        greater than or equal to the noise level specified by Contour.
# @param dmy_nNonZero
#        The number of non-zero sensitivity array values.
# @param dmy_fltNonZeroValues
#        Matrix of sensitivity values of the exposure area (only non-zero values
#        of the larger two-dimensional array are stored in a Coordinate matrix).
# @param dmy_intRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param dmy_intColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# @result 
#        An integer representing success of this operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_aa_exposure_area_sensitivity.restype = A2_IK
ANOPP2.a2py_aa_exposure_area_sensitivity.argtypes =              \
     [A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), A2_RK, \
      A2_EK, POINTER(A2_RK), POINTER(A2_IK), POINTER(POINTER(A2_RK)),      \
      POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK))]



# =========================================================================================
#  The next part of this interface file contains enumerators for the Acoustic Analysis API.
# =========================================================================================



# -----------------------------------------------------------------------------------------
#  This enumeration grouping is to convey what type of noise metric.  Each enumerator is
#  equivalient to a type of noise metric such as Acoustic Pressure Time History or
#  Power Spectral Density.
# -----------------------------------------------------------------------------------------

# The initial post process level is just for acoustic pressure time history
a2_aa_apth = 1

# This enumerator is for the acoustic velocity time history.
a2_aa_avth = 2

# This enumerator is for pressure gradient time history.
a2_aa_pgth = 3

# This enumerator is for pure tones spectrum
a2_aa_pts = 4

# This enumerator is for narrow band spectrum
a2_aa_nbs = 5

# The next post processing level is for the power spectral density spectrum
a2_aa_psd = 6

# The next is for proportional band sound pressure levels
a2_aa_pbs = 7

# The next is for overall sound pressure levels
a2_aa_oaspl = 8

# The next is for perceived noise level
a2_aa_pnl = 9

# The is for the sound exposure noise level
a2_aa_sel = 10

# An enumerator for the effective perceived noise level
a2_aa_epnl = 11

# An enumerator for the surface area under a limit of dB.
a2_aa_ea = 12

# The next is for proportional band spectrograms
a2_aa_pbsg = 13



# -----------------------------------------------------------------------------------------
#  This enumeration grouping is to determine what characteristic of the noise is included
#  by the metric.  Options include absolute levels (such as a noise measurement) or
#  change in levels (such as a suppression).
# -----------------------------------------------------------------------------------------

# These are the default, which is an unknown level.
a2_aa_unknown_levels = 1

# This is the enumeration of absolute levels
a2_aa_absolute_levels = 2

# This is the enumeration of change in levels
a2_aa_change_in_levels = 3



# -----------------------------------------------------------------------------------------
#  These enumerations are to convery what type of units the noise metrics are in.  These
#  are passed to the Acoustic Analysis API routines with the metric so the Acoustic
#  Analysis computations are performed correctly.
# -----------------------------------------------------------------------------------------

# This enumerator is for pressure in Pascals.
a2_aa_pa = 1

# This enumerator is for mean squared pressure (MSP).
a2_aa_msp = 2

# This enumerator is for decibels.
a2_aa_db = 3



# -----------------------------------------------------------------------------------------
#  These enumerators are for the different time-domain windows that are available in the
#  acoustic analysis API.  As of now there are 5 different window types available:
#  No window, Blackman, flat top, Hanning, and Hamming windows.
# -----------------------------------------------------------------------------------------

# This enumerator is for no window.
a2_aa_no_window = 1

# This enumerator is for the Blackman window.
a2_aa_blackman_window = 2

# This enumerator is for the flat top window.
a2_aa_flat_top_window = 3

# This enumerator is for the Hanning window.
a2_aa_hanning_window = 4

# This enumerator is for the Hamming window.
a2_aa_hamming_window = 5



# -----------------------------------------------------------------------------------------
#  These enumerators are for the frequency weighting of narrowband spectra, power
#  spectral densities, and 1/3rd-Octave sound pressure levels.  There are 4 different
#  weighting functions: no-weighting, a-weighting, b-weighting, and c-weighting.  Refer
#  to the  Acoustic Analysis API Manual for more information.
# -----------------------------------------------------------------------------------------

# This enumerator is for no-weight
a2_aa_no_weight = 1

# This enumerator is for a-weight
a2_aa_a_weight = 2

# This enumerator is for b-weight
a2_aa_b_weight = 3

# This enumerator is for c-weight
a2_aa_c_weight = 4



# -----------------------------------------------------------------------------------------
#  These enumerators are for the different types of calculating the proportional band 
#  center, lower limit, and upper limit frequencies.  Currently, there are 3 methods: 
#  exact, preferred, and approximate.  Please see documentation on the different methods.
# -----------------------------------------------------------------------------------------

# This enumerator is for the exact method.
a2_aa_exact = 1

# This is for the preferred method.
a2_aa_preferred = 2

# This is for the approximate method.
a2_aa_approximate = 3



# -----------------------------------------------------------------------------------------
#  This enumerator group defines the different methods for determining a contour area.
#  Currently 2 options exist: Zeroth order interpolation and First order interpolation.
# -----------------------------------------------------------------------------------------

# This enumerator is for the zeroth order interpolation.
a2_aa_zeroth = 1

# This enumerator is for the first order interpolation.
a2_aa_first = 2



#---------------------------------------------------------------------------------------
# These enumerators are for the different combine methods offered by the acoustic
# analysis.  There are currently 3 different options: combine by union, merge, or
# intersection.  These are specified to the acoustic analysis by way of these three
# enumerators.
#---------------------------------------------------------------------------------------

# This is the enumerator for combining two methods by union.
a2_aa_combine_by_union = 1

# This enumerator is for combining by intersection.
a2_aa_combine_by_intersection = 2

# This last enumerator is for combining by merging.
a2_aa_combine_by_merge = 3


  
#----------------------------------------------------------------------------------------
# These enumerators are for the two actions possible while subbanding: dividing or
# combining.
#----------------------------------------------------------------------------------------

# This is the enumerator for dividing a band into subbands.
a2_aa_subband_divide = 1

# This is the enumerator for combining several subbands into 1.
a2_aa_subband_combine = 2




# =========================================================================================
#  These are the parameters defined in the Acoustic Analysis API.
# =========================================================================================



# -----------------------------------------------------------------------------------------
#  This array contains the standard 24 1/3rd-Octave Sound Pressure Level center band
#  frequencies ranging from 50 to 10,000 Hz.
# -----------------------------------------------------------------------------------------
a2_aa_std_center_frequencies =                                                         \
     (A2_RK * 24)(*[50.0, 63.0, 80.0, 100.0, 125.0, 160.0, 200.0, 250.0, 315.0, 400.0, \
        500.0, 630.0, 800.0, 1000.0, 1250.0, 1600.0, 2000.0, 2500.0, 3150.0, 4000.0,   \
        5000.0, 6300.0, 8000.0, 10000.0])

# -----------------------------------------------------------------------------------------
#  Reference pressure in Pascals, used to calculate sound pressure levels
# -----------------------------------------------------------------------------------------
a2_aa_reference_pressure = A2_RK (20.0e-6)

# -----------------------------------------------------------------------------------------
#  This is the noise floor in decibels.  Since 0 pressure is negative inf in decibels
#  this number represents the smallest value of decibel the code will allow. This is the
#  decibel value of the smallest number that can be represented by single precision.
# -----------------------------------------------------------------------------------------
a2_aa_noise_floor_db = A2_RK (-285.3183943)

# -----------------------------------------------------------------------------------------
#  This is the equivalent number for the noise floor in pressure. This is the smallest
#  possible number that can be represented in single precision (2^-126).
# -----------------------------------------------------------------------------------------
a2_aa_noise_floor_msp = A2_RK (1.1754944e-38)



#!/usr/bin/env python
# -----------------------------------------------------------------------------------------
# This is the ANOPP2.API interface file.  It contains definitions for all the
# functions available in the ANOPP2.API.  This includes initialization, calculations, etc.
# See the API manual for more information.
# -----------------------------------------------------------------------------------------
# @file ANOPP2.api.py
# @author The ANOPP2 Development Team
# @version 1.0.0
# -----------------------------------------------------------------------------------------



# =========================================================================================
# First part of this section contains interfaces into the available routines included in
# this API.
# =========================================================================================



# -----------------------------------------------------------------------------------------
# This routine initializes the ANOPP2 API by setting internal variables and function
# parameters that must exist before any other call to the API can be made.  The debug
# flag will turn on or off debugging of the API (only written out if the debug verison
# of the API is used).
# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
# This routine executes the unit tests in the ANOPP2.module.  The unit
# tests execute all the tests implemented in the ANOPP2.API.
# -----------------------------------------------------------------------------------------
# @result
#        The number of failed asserts found during unit testing.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_unit_test.restype = A2_IK



# -----------------------------------------------------------------------------------------
# This routine performs an N-dimensional interpolation to calculate desired dependent
# variables for desired independent variables from arrays of known dependent and
# independent variable pairs of Structured data that is axis aligned.  The dependent
# variables may be a single value or an array of values.
# -----------------------------------------------------------------------------------------
# @param  fltKnownIndependents
#         The N-dimensional coordinates where the values are known.  This array is sized
#         equal to the number of independent variables by the number of known
#         independent varialbe sets. The data must be structured, and because there
#         must be a minimum of 2 values along each axis, the minimum number of known
#         data sets is 2 ^ Ndimensions. For example, if the independent variables are
#         Mach number, throttle setting, and angle of attack, and there are two values
#         for each, then there are eight total combinations of these variables, and
#         the array would be three by eight: {[kM1,kT1,kA1], [kM2,kT1,kA1],
#         [kM1,kT2,kA1], [kM2,kT2,kA1],[kM1,kT1,kA2],[kM2,kT1,kA2],[kM1,kT2,kA2],
#         [kM2,kT2,kA2]}.  The ordering of the independent varialbe sets must be
#          consistent with the order indicated by the auxiliary variables, where the first
#          index repeats the fastest in memory, then the second, then the third.
# @param  fltKnownDependents
#         The known dependent variables.  This array is sized equal to the number of
#         known independent variable sets by the number of dependent values (which may
#         be any integer greater than zero).  For the example above, eight by N:
#         {[kV1,kV2,...,kVN]kMTA1, [kV1,kV2,...,kVN]kMTA2, [kV1,kV2,...,kVN]kMTA3,
#          [kV1,kV2,...,kVN]kMTA4, [kV1,kV2,...,kVN]kMTA5, [kV1,kV2,...,kVN]kMTA6,
#          [kV1,kV2,...,kVN]kMTA7, [kV1,kV2,...,kVN]kMTA8}
# @param  fltDesiredIndependents
#         The N-dimensional coordinates where the values are desired. This array is
#         sized equal to the number of independent variables by the number of desired
#         independent varialbe sets. There may be any number of desired independent
#         variable sets, and then need not be strucured. For the example above, if
#         are five desired combinations of Mach number, throttle setting, and angle of
#         attack, then the array would be three by five: {[dM1,dT1,dA1], [dM2,dT2,dA2],
#         [dM3,dT3,dA3], [dM4,dT4,dA4], [dM5,dT5,dA5]}
#         Note, the first dimension must match that of fltKnowIndependents.
# @param  fltDesiredDependents
#         The desired dependent variables.  This array is sized equal to the number of
#         desired independent variable sets by the number of known dependent values
#         (which must match that of fltKnownDependents). For the example above,
#         five by N: {[dV1,dV2,...,dVN]dMTA1, [dV1,dV2,...,dVN]dMTA2,
#         [dV1,dV2,...,dVN]dMTA3, [dV1,dV2,...,dVN]dMTA4, [dV1,dV2,...,dVN]dMTA5}
#         This array is the result of the interpolation.
# @param  enumInterpolationMethod
#         An enumeration for the interpolation method that should be used.
#           For Axis-Aligned Data: a2_math_interp_axis_aligned
#           For Non Axis-Aligned Data: a2_math_interp_axis_not_aligned
# @param  fltAuxiliaryVariables
#         Auxiliary variables for the interpolation method.
#           For Axis-Aligned 2D : [nX, nY]
#           For Axis-Aligned 3D : [nX, nY, nZ]
#         The order of these variables must be consistent with the order of the known
#         independents.  Since nX is first, this means that the x dimension of the known
#         independents cycles most quickly. nY is second, so it cycles with every pass
#         of nX.  nZ is last, so it cycles after a complete pass of nX and nY. If the 
#         dimension are nX =5, nY=4, and nZ = 2, the order would be:
#         111, 211, 311, 411, 511, 121, 221, 321, 421, 521, 131, 231, 331, 431, 531,
#         141, 241, 341, 441, 541, 151, 251, 351, 451, 551, 112, 212, 312, 412, 512, 
#         122, 222, 322, 422, 522, 132, 232, 332, 432, 532, 142, 242, 342, 442, 542, 
#         152, 252, 352, 452, 552,
# @result
#         An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_interpolate_structured.restype = A2_IK
ANOPP2.a2py_math_interpolate_structured.argtypes =                      \
  [A2_IK, A2_IK, A2_IK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), \
    POINTER(A2_RK), POINTER(A2_RK), A2_EK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine performs an N-dimensional interpolation to calculate desired dependent
# variables for desired independent variables from arrays of known dependent and
# independent variable pairs of Scattered Data.  The dependent variables may be a single
# value or an array of values.
# -----------------------------------------------------------------------------------------
# @param  fltKnownIndependents
#         The N-dimensional coordinates where the values are known.  This array is sized
#         equal to the number of independent variables by the number of known
#         independent varialbe sets. For example, if the independent variables are Mach
#         number, throttle setting, and angle of attack, and there are ten known
#         combinations of these variables, then the array would be three by ten.
# @param  fltKnownDependents
#         The known dependent variables.  This array is sized equal to the number of
#         known independent variable sets by the number of dependent values (which may
#         be any integer greater than zero).  For the example above, ten by N.
# @param  fltDesiredIndependents
#         The N-dimensional coordinates where the values are desired. This array is
#         sized equal to the number of independent variables by the number of desired
#         independent varialbe sets. For example, if the independent variables are Mach
#         number, throttle setting, and angle of attack, and there are twenty desired
#         combinations of these variables, then the array would be three by twenty.  The
#         first dimension must match that of fltKnowIndependents, but the second
#         dimension may be any postive integer.
# @param  fltDesiredDependents
#         The desired dependent variables.  This array is sized equal to the number of
#         desired independent variable sets by the number of known dependent values
#         (which must match that of fltKnownDependents). For the example above,
#         twenty by N.  This array is the result of the interpolation.
# @param  enumInterpolationMethod
#         An enumeration for the interpolation method that should be used.
#         For Inverse Distance Weighting: a2_math_interp_inverse_distance
#         For Radial Basis Function: a2_math_interp_radial_basis (currently disabled)
# @param  fltAuxiliaryVariables
#         Auxiliary varialbes for the interpolation method. In this case it should
#         be one value: the power factor for the Inverse Distance Weighting.
# @result
#        An integer that is returned 0 when everything occurred correctly.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_interpolate_scattered.restype = A2_IK
ANOPP2.a2py_math_interpolate_scattered.argtypes =                       \
  [A2_IK, A2_IK, A2_IK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), \
    POINTER(A2_RK), POINTER(A2_RK), A2_EK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This function takes a known function of Independant/Dependent pairs and uses a cubic 
# spline fit to interpolate desired dependents for a set of provided desired independents.
# -----------------------------------------------------------------------------------------
# @param enumBoundaryCondition
#        The boundary condition for the interpolation, which must ber either:
#        a2_math_zero_slope     - for zero slope at the end points
#        a2_math_zero_curvature - for zero curvature at the end points.
# @param fltKnownIndependents
#        The input array of know independent values.
# @param fltKnownDependents
#        The input array of know dependent values, which must be the same size as the
#        known independents array.
# @param fltDesiredIndependents
#        The array of independent values where the dependent values are desired.
#        Note: This array must be larger than the size of the known values.
# @param fltDesiredDependents
#        The array that will contain the resulting desired dependent values. This must be 
#        the same size as the known independents array.
# @result 
#        An integer indicating the success status of the operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_interpolate_cubic_spline.restype = A2_IK
ANOPP2.a2py_math_interpolate_cubic_spline.argtypes = \
  [A2_EK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This function takes a known function of Independant/Dependent pairs and uses a cubic 
# spline fit to interpolate desired dependents for a set of provided desired independents.
# The sensitivity of the desired dependents to the inputs is determined.
# -----------------------------------------------------------------------------------------
# @param enumBoundaryCondition
#        The boundary condition for the interpolation, which must ber either:
#        a2_math_zero_slope     - for zero slope at the end points
#        a2_math_zero_curvature - for zero curvature at the end points.
# @param dmy_nKnownPairs
#        The number of known pairs of independent/dependent values.
# @param dmy_nDesiredPairs
#        The number of desired pairs of independent/dependent values.
# @param fltKnownIndependents
#        The input array of know independent values.
# @param fltKnownDependents
#        The input array of know dependent values, which must be the same size as the
#        known independents array.
# @param fltDesiredIndependents
#        The array of independent values where the dependent values are desired.
#        Note: This array must be larger than the size of the known values.
# @param fltDesiredDependents
#        The array that will contain the resulting desired dependent values. This must be 
#        the same size as the known independents array.
# @param dmy_hdlDeltaDesiredDepDeltaKnownIndependents
#        The handle for the CooMatrix for desired dependents sensitivity with respect to
#        known independents.
# @param dmy_hdlDeltaDesiredDepDeltaKnownDependents
#        The handle for the CooMatrix for desired dependents sensitivity with respect to
#        known dependents.
# @param dmy_hdlDeltaDesiredDepDeltaDesiredIndependents
#        The handle for the CooMatrix for desired dependents sensitivity with respect to
#        desired independents.
# @result 
#        An integer indicating the success status of the operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_interpolate_cubic_spline_sensitivity.restype = A2_IK
ANOPP2.a2py_math_interpolate_cubic_spline_sensitivity.argtypes =                          \
  [A2_EK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_RK), \
   POINTER(c_void_p), POINTER(c_void_p), POINTER(c_void_p)]



# -----------------------------------------------------------------------------------------
#  This routine takes input arrays of known independent and dependent values and
#  calculates the slope at each input point.
# -----------------------------------------------------------------------------------------
# @param dmy_nKnownPairs
#        The number of known pairs of independent/dependent values.
# @param dmy_fltXi
#        The input array of know independent values.
# @param dmy_fltFi
#        The input array of know dependent values.
# @param dmy_enumBoundaryCondition
#        The boundary condition at the endpoints.  
#        a2_math_zero_slope     - for zero slope at the end points.
#        a2_math_zero_curvature - for zero curvature at the end points.
# @param dmy_fltFPrimeXi
#        The output array of slopes at each Xi value.
# @result 
#        An integer indicating the success status of the operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_differentiate_cubic_spline.restype = A2_IK
ANOPP2.a2py_math_differentiate_cubic_spline.argtypes = \
  [A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_EK, POINTER(A2_RK)]



# -----------------------------------------------------------------------------------------
# This routine takes input arrays of known independent and dependent values and
# calculates the slope at each input point.  Handles to the sensitivity of fltFPrimeXi
# with respect to the inputs are returned.
# -----------------------------------------------------------------------------------------
# @param dmy_nKnownPairs
#        The number of known pairs of independent/dependent values.
# @param dmy_fltXi
#        The input array of know independent values.
# @param dmy_fltFi
#        The input array of know dependent values.
# @param dmy_enumBoundaryCondition
#        The boundary condition at the endpoints.  
#        a2_math_zero_slope     - for zero slope at the end points.
#        a2_math_zero_curvature - for zero curvature at the end points.
# @param dmy_fltFPrimeXi
#        The output array of slopes at each Xi value.
# @param dmy_hdlDeltaFPrimeXiDeltaKnownIndependents
#        The handle for the CooMatrix for FPrimeXi sensitivity with respect to
#        known independents.
# @param dmy_hdlDeltaFPrimeXiDeltaKnownDependents
#        The handle for the CooMatrix for FPrimeXisensitivity with respect to
#        known dependents.
# @result 
#        An integer indicating the success status of the operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_differentiate_cubic_spline_sensitivity.restype = A2_IK
ANOPP2.a2py_math_differentiate_cubic_spline_sensitivity.argtypes =                    \
  [A2_IK, POINTER(A2_RK), POINTER(A2_RK), A2_EK, POINTER(A2_RK), POINTER(c_void_p), \
   POINTER(c_void_p)]



# -----------------------------------------------------------------------------------------
# This function solves a uder-defined equation defined by an equation string by using an
# array of known variables, another array of corresponding values, and returns the
# result of the equation as a real value.  This function performs the same equation on
# an array of values and returns an array of results. This assumes that the values are
# all real.
# -----------------------------------------------------------------------------------------
# @param strEquation
#        This is a string representing the equation to be solved.
# @param nVariables
#        This is the number of variables in the strVariables array.
# @param nDataSets
#        This is the number of sets of data on which the equation has to be solved.
# @param strVariables
#        This is an 1D array of strings representing variables (e.q., MachNumber) for
#        which values are available.  The variables in the Equation string should be
#        among those provided in this array.
# @param fltVariables
#        This is a 2D array of real values of the variables identified in the
#        strVariables array. The first dimension is the number of variables
#        provided in the strVariables array and the second dimension is the number
#        of times this equation has to be solved repetetitvely.
# @param fltResult
#        This is an 1D array of results of the equation based on each repetition.
# @result
#        An integer that represents the success of this function.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_solve.restype = A2_IK
ANOPP2.a2py_math_solve.argtypes =                                            \
  [A2_IK, POINTER(500 * A2_CK), A2_IK, A2_IK, A2_IK, POINTER(100 * A2_CK), \
   POINTER(A2_RK), POINTER(1 * A2_RK)]



# -----------------------------------------------------------------------------------------
# This function parses an equation and identifies an array of variables used in the
# equation.
# -----------------------------------------------------------------------------------------
# @param strEquation
#        This is a string representing the equation to be solved.
# @param strEquationVariables
#        This is an 1D array of strings representing variables (e.q., MachNumber)
#        available in the equation.
# @param blnVariableIsString
#        This is a logical that tells whether the variable is a string (.true.) or a
#        number (.false.)
# @param strResult
#        This is the variable to which the results of the equations is attached.
# @result
#        An integer that represents the success of this function.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_get_variables.restype = A2_IK
ANOPP2.a2py_math_get_variables.argtypes =                                   \
  [A2_IK, POINTER(A2_CK), POINTER(A2_IK), A2_IK, POINTER(POINTER(A2_CK)), \
   POINTER(POINTER(A2_LK)), POINTER(A2_CK)]



#------------------------------------------------------------------------------------------
# This routine analyzes an array of known independent values provided to see where a 
# desired independent value falls. Basically, it will return an array of indices 
# corresponding to the lower corner of the bounding box along with an array of fractions 
# along the dimensions. All the values are assumed axi-aligned.
#------------------------------------------------------------------------------------------
# @param nDimensions
#        This is the number of dimensions.
# @param nKnownIndependentsSize
#        This is the maximum of the number o values provided for each dimension.
# @param nKnownIndependents
#        An array of integers representing the number of known values of the independent
#        variables along each of the dimensions.
# @param fltKnownIndependents
#        A two-dimensional array of known values of the independent variables to be used 
#        to determine the indices and fractions. Only the first nKnownIndependents 
#        along the dimension will be read and used to calculate the indices and the 
#        fractions.
#          1st dimension: the dimension of the independent variable; (1, 2, 3, ...).
#          2nd dimension: the known independent variable value corresponding to that 
#                         index.
# @param fltDesiredIndependents
#        An array of values that is to be used to determine the indices and fractions.
# @param intIndices
#        An array of indices corresponding to the lower corner of the bounding box of 
#        the given value, along the three dimensions, returned by this routine.
# @param fltFractions
#        An array of fractions along the three dimensions with respect to the box
#        dimensions, returned by this routine.
# @result
#        An integer representing the success (0) or failure (non-zero) of this routine.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_get_interpolation_factors.restype = int
ANOPP2.a2py_math_get_interpolation_factors.argtypes =                              \
  [A2_IK, A2_IK, POINTER(A2_IK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_IK), \
  POINTER(A2_RK)]



#------------------------------------------------------------------------------------------
# This routine analyzes an array of known independent values provided to see where a 
# desired independent value falls.  Basically, it will return an array of indices 
# corresponding to the lower corner of the bounding box along with an array of fractions 
# along the dimensions.  All the values are assumed axi-aligned.
#------------------------------------------------------------------------------------------
# @param nDimensions
#        This is the number of dimensions.
# @param nKnownIndependentsSize
#        This is the maximum of the number o values provided for each dimension.
# @param nKnownIndependents
#        An array of integers representing the number of known values of the independent
#        variables along each of the dimensions.
# @param fltKnownIndependents
#        A two-dimensional array of known values of the independent variables to be used 
#        to determine the indices and fractions.  Only the first nKnownIndependents 
#        along the dimension will be read and used to calculate the indices and the 
#        fractions.
#          1st dimension: the dimension of the independent variable; (1, 2, 3, ...).
#          2nd dimension: the known independent variable value corresponding to that 
#                         index.
# @param fltDesiredIndependents
#        An array of values that is to be used to determine the indices and fractions.
# @param intIndices
#        An array of indices corresponding to the lower corner of the bounding box of 
#        the given value, along the three dimensions, returned by this routine.
# @param fltFractions
#        An array of fractions along the three dimensions with respect to the box
#        dimensions, returned by this routine.
# ================== Fractions Sensitivity WRT Desired Independents =======================
# @param nDeltaFractionsDeltaDesiredIndependentsNonZero
#        The number of non-zero sensitivity array values.
# @param fltDeltaFractionsDeltaDesiredIndependentsNonZeroValues
#        Matrix of non-xero sensitivity values of the change in fractions wrt the
#        change in the desired independents.
# @param intDeltaFractionsDeltaDesiredIndependentsRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intDeltaFractionsDeltaDesiredIndependentsColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# ================== Fractions Sensitivity WRT Known Independents   =======================
# @param nDeltaFractionsDeltaKnownIndependentsNonZero
#        The number of non-zero sensitivity array values.
# @param fltDeltaFractionsDeltaKnownIndependentsNonZeroValues
#        Matrix of non-xero sensitivity values of the change in fractions wrt the
#        change in the known independents.
# @param intDeltaFractionsDeltaKnownIndependentsRowIndices
#        Matrix of row indices for non-zero sensitivity values.
# @param intDeltaFractionsDeltaKnownIndependentsColumnIndices
#        Matrix of column indices for non-zero sensitivity values.
# @result
#        An integer representing the success (0) or failure (non-zero) of this routine.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_get_interpolation_factors_sensitivity.restype = int
ANOPP2.a2py_math_get_interpolation_factors_sensitivity.argtypes =                  \
  [A2_IK, A2_IK, POINTER(A2_IK), POINTER(A2_RK), POINTER(A2_RK), POINTER(A2_IK), \
  POINTER(A2_RK),                                                                \
  POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),              \
  POINTER(POINTER(A2_IK)),                                                       \
  POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),              \
  POINTER(POINTER(A2_IK))]



#------------------------------------------------------------------------------------------
# This routine multiplies two 2-dimensional matrices provided in the COO format and
# returns the product as another COO matrix. [C] = [A] x [B]. [A], [B], AND [C] are 
# originally 2D matrices but because they contain lots of zeros, they are provided as 
# COO matrices. The dimensions of [A] and [B] should be (i,j) and (j,k) and the 
# resulting matrix will be with dimensions (i,k). However, when receiving this function 
# will receive matrices of whatever dimensions, but will check for this consistency.
#------------------------------------------------------------------------------------------
# @param dmy_nRows2DInA
#        This is the number of rows in the 2D original first matrix [A].
# @param dmy_nColumns2DInA
#        This is the number of columns in the 2D original first matrix [A].
# @param dmy_nNonZeroValuesInA
#        This is the number of non-zero values in matrix [A].
# @param dmy_fltNonZeroValuesInA
#        This is an array of non-zero values in matrix [A].
# @param dmy_intNonZeroRowIndicesInA
#        This is an array of row indices of matrix [A] at which the value is non-zero. 
# @param dmy_intNonZeroColumnIndicesInA
#        This is an array of column indices of matrix [A] at which the value is non-
#        zero. 
# @param dmy_nRows2DInB
#        This is the number of rows in the 2D original first matrix [B].
# @param dmy_nColumns2DInB
#        This is the number of columns in the 2D original first matrix [B].
# @param dmy_nNonZeroValuesInB
#        This is the number of non-zero values in matrix [B].
# @param dmy_fltNonZeroValuesInB
#        This is an array of non-zero values in matrix [B].
# @param dmy_intNonZeroRowIndicesInB
#        This is an array of row indices of matrix [B] at which the value is non-zero. 
# @param dmy_intNonZeroColumnIndicesInB
#        This is an array of column indices of matrix [B] at which the value is non-
#        zero. 
# @param dmy_nNonZeroValuesInC
#        This is the number of non-zero values in matrix [C].
# @param dmy_fltNonZeroValuesInC
#        This is an array of non-zero values in matrix [C].
# @param dmy_intNonZeroRowIndicesInC
#        This is an array of row indices of matrix [C] at which the value is non-zero. 
# @param dmy_intNonZeroColumnIndicesInC
#        This is an array of column indices of matrix [C] at which the value is non-
#        zero. 
# @result
#        This is an integer that denotes the success (0) or failure (non-zero) that will
#        be the result of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_coo_mult_coo.restype = int
ANOPP2.a2py_math_coo_mult_coo.argtypes =                                  \
  [A2_IK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), \
   A2_IK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), \
   POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),    \
   POINTER(POINTER(A2_IK))]



#-----------------------------------------------------------------------------------------
# This routine multiplies two 2-dimensional matrices provided in the COO format where the
# second COO matrix has been transposed and returns the product as another COO matrix.
# [C] = [A] x[B] ^ T.
# where[A], [B], AND[C] are originally 2D matrices but because they contain lots of
# zeros, they are provided as COO matrices.The dimensions of[A] and [B] should be(i, j)
# and (k, j) and the resulting matrix will be with dimensions(i, k).However, when
# receiving this function will receive matrices of whatever dimensions, but will check for
# this consistency.
#-----------------------------------------------------------------------------------------
# @param dmy_nRowsInA
#        This is the number of rows in the 2D original first matrix[A].
# @param dmy_nColumnsInA
#        This is the number of columns in the 2D original first matrix[A].
# @param dmy_nNonZeroValuesInA
#        This is the number of non - zero values in matrix[A].
# @param dmy_fltNonZeroValuesInA
#        This is an array of non - zero values in matrix[A].
# @param dmy_intNonZeroRowIndicesInA
#        This is an array of row indices of matrix[A] at which the value is non - zero.
# @param dmy_intNonZeroColumnIndicesInA
#        This is an array of column indices of matrix[A] at which the value is non - zero.
# @param dmy_nRowsInB
#        This is the number of rows in the 2D original first matrix[B].
# @param dmy_nColumnsInB
#        This is the number of columns in the 2D original first matrix[B].
# @param dmy_nNonZeroValuesInB
#        This is the number of non - zero values in matrix[B].
# @param dmy_fltNonZeroValuesInB
#        This is an array of non - zero values in matrix[B].
# @param dmy_intNonZeroRowIndicesInB
#        This is an array of row indices of matrix[B] at which the value is non - zero.
# @param dmy_intNonZeroColumnIndicesInB
#        This is an array of column indices of matrix[B] at which the value is non - zero.
# @param dmy_nNonZeroValuesInC
#        This is the number of non - zero values in matrix[C].
# @param dmy_fltNonZeroValuesInC
#        This is an array of non - zero values in matrix[C].
# @param dmy_intNonZeroRowIndicesInC
#        This is an array of row indices of matrix[C] at which the value is non - zero.
# @param dmy_intNonZeroColumnIndicesInC
#        This is an array of column indices of matrix[C] at which the value is non - zero.
# @result
#        This is an integer that denotes the success(0) or failure(non - zero) that will
#        be the result of this operation.
#-----------------------------------------------------------------------------------------
ANOPP2.a2py_math_coo_mult_transposed_coo.restype = int
ANOPP2.a2py_math_coo_mult_transposed_coo.argtypes =                       \
  [A2_IK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), \
   A2_IK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), \
   POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),    \
   POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This routine multiplies two CooMatrices given the handles for the two CooMatrices.  It
# returns the handle for the resulting CooMatrix.
# -----------------------------------------------------------------------------------------
# @param dmy_hdlMatrixA
#        The handle for the A CooMatrix.
# @param dmy_hdlMatrixB
#        The handle for the B CooMatrix.
# @param dmy_hdlMatrixC
#        The handle for the resulting C CooMatrix.
# @result 
#        An integer indicating the success of the operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_coo_mult_coo_by_handle.restype = A2_IK
ANOPP2.a2py_math_coo_mult_coo_by_handle.argtypes = \
  [POINTER(c_void_p), POINTER(c_void_p), POINTER(c_void_p)]



#------------------------------------------------------------------------------------------
# This routine adds two 2-dimensional matrices provided in the COO format and
# returns the sum as another COO matrix. [C] = [A] x [B]. [A], [B], AND [C] are 
# originally 2D matrices but because they contain lots of zeros, they are provided as 
# COO matrices. The dimensions of [A] and [B] should be (i,j) and (i,j) and the 
# resulting matrix will be with dimensions (i,j). However, when receiving this function 
# will receive matrices of whatever dimensions, but will check for this consistency.
#------------------------------------------------------------------------------------------
# @param dmy_nRows2DInA
#        This is the number of rows in the 2D original first matrix [A].
# @param dmy_nColumns2DInA
#        This is the number of columns in the 2D original first matrix [A].
# @param dmy_nNonZeroValuesInA
#        This is the number of non-zero values in matrix [A].
# @param dmy_fltNonZeroValuesInA
#        This is an array of non-zero values in matrix [A].
# @param dmy_intNonZeroRowIndicesInA
#        This is an array of row indices of matrix [A] at which the value is non-zero. 
# @param dmy_intNonZeroColumnIndicesInA
#        This is an array of column indices of matrix [A] at which the value is non-
#        zero. 
# @param dmy_nRows2DInB
#        This is the number of rows in the 2D original first matrix [B].
# @param dmy_nColumns2DInB
#        This is the number of columns in the 2D original first matrix [B].
# @param dmy_nNonZeroValuesInB
#        This is the number of non-zero values in matrix [B].
# @param dmy_fltNonZeroValuesInB
#        This is an array of non-zero values in matrix [B].
# @param dmy_intNonZeroRowIndicesInB
#        This is an array of row indices of matrix [B] at which the value is non-zero. 
# @param dmy_intNonZeroColumnIndicesInB
#        This is an array of column indices of matrix [B] at which the value is non-
#        zero. 
# @param dmy_nNonZeroValuesInC
#        This is the number of non-zero values in matrix [C].
# @param dmy_fltNonZeroValuesInC
#        This is an array of non-zero values in matrix [C].
# @param dmy_intNonZeroRowIndicesInC
#        This is an array of row indices of matrix [C] at which the value is non-zero. 
# @param dmy_intNonZeroColumnIndicesInC
#        This is an array of column indices of matrix [C] at which the value is non-
#        zero. 
# @result
#        This is an integer that denotes the success (0) or failure (non-zero) that will
#        be the result of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_coo_add_coo.restype = int
ANOPP2.a2py_math_coo_add_coo.argtypes =                                   \
  [A2_IK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), \
   A2_IK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), \
   POINTER(A2_IK), POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)),    \
   POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This routine adds two CooMatrices given the handles for the two CooMatrices.  It returns
# the handle for the resulting CooMatrix.
# -----------------------------------------------------------------------------------------
# @param dmy_hdlMatrixA
#        The handle for the A CooMatrix.
# @param dmy_hdlMatrixB
#        The handle for the B CooMatrix.
# @param dmy_hdlMatrixC
#        The handle for the resulting C CooMatrix.
# @result 
#        An integer indicating the success of the operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_coo_add_coo_by_handle.restype = A2_IK
ANOPP2.a2py_math_coo_add_coo_by_handle.argtypes = \
  [POINTER(c_void_p), POINTER(c_void_p), POINTER(c_void_p)]



#------------------------------------------------------------------------------------------
#  This routine augments the first CooMatrix, A, with the second CooMatrix, B, by
#  appending columns to the right of the CooMatrix, A. A minimum column width for A such
#  that, if A is less than the minimum column width, dmy_nMinimumColumnsBefore, A is first
#  augmented by a sufficient number of columns of zeroes. The result of Augmenting A with B
#  is returned in C. The CooMatrixes A and B are not changed.
#------------------------------------------------------------------------------------------
#  @param dmy_nRows2DInA
#         This is the number of rows in the 2D original first matrix [A].
#  @param dmy_nColumns2DInA
#         This is the number of columns in the 2D original first matrix [A].
#  @param dmy_nNonZeroValuesInA
#         This is the number of non-zero values in matrix [A].
#  @param dmy_fltNonZeroValuesInA
#         This is an array of non-zero values in matrix [A].
#  @param dmy_intNonZeroRowIndicesInA
#         This is an array of row indices of matrix [A] at which the value is non-zero. 
#  @param dmy_intNonZeroColumnIndicesInA
#         This is the minimum row height of A before appending B. 
#  @param dmy_nRows2DInB
#         This is the number of rows in the 2D original first matrix [B].
#  @param dmy_nColumns2DInB
#         This is the number of columns in the 2D original first matrix [B].
#  @param dmy_nNonZeroValuesInB
#         This is the number of non-zero values in matrix [B].
#  @param dmy_fltNonZeroValuesInB
#         This is an array of non-zero values in matrix [B].
#  @param dmy_intNonZeroRowIndicesInB
#         This is an array of row indices of matrix [B] at which the value is non-zero. 
#  @param dmy_intNonZeroColumnIndicesInB
#         This is an array of column indices of matrix [B] at which the value is non-zero. 
#  @param dmy_nMinimumColumnsBefore
#         This is the minimum width for matrix [A] prior to augmenting.
#  @param dmy_nRows2DInC
#         This is the number of rows in the 2D original first matrix [C].
#  @param dmy_nColumns2DInC
#         This is the number of columns in the 2D original first matrix [C].
#  @param dmy_nNonZeroValuesInC
#         This is the number of non-zero values in matrix [C].
#  @param dmy_fltNonZeroValuesInC
#         This is an array of non-zero values in matrix [C].
#  @param dmy_intNonZeroRowIndicesInc
#         This is an array of row indices of matrix [C] at which the value is non-zero. 
#  @param dmy_intNonZeroColumnIndicesInC
#         This is an array of column indices of matrix [C] at which the value is non-zero. 
#  @result
#         This is an integer that denotes the success (0) or failure (non-zero) that will
#         be the result of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_coo_augment_columns.restype = int
ANOPP2.a2py_math_coo_augment_columns.argtypes =  \
          [# The A CooMatrix (in)
           A2_IK, A2_IK, A2_IK,
           POINTER(A2_RK),
           POINTER(A2_IK),
           POINTER(A2_IK),
           # The B CooMatrix (in)
           A2_IK, A2_IK, A2_IK,
           POINTER(A2_RK),
           POINTER(A2_IK),
           POINTER(A2_IK),
           # Minimum prior column width for padding the A CooMatrix
           A2_IK,
           # The C CooMtrix (out)
           POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK),
           POINTER(POINTER(A2_RK)),
           POINTER(POINTER(A2_IK)),
           POINTER(POINTER(A2_IK))]



#------------------------------------------------------------------------------------------
#  This routine augments the first CooMatrix, A, with the second CooMatrix, B, by
#  appending rows to the bottom of the CooMatrix, A. A minimum row hight for A such that,
#  if A is less than the minimum row height, dmy_nMinimumRowsBefore, A is first augmented
#  by a sufficient number of rows of zeroes. The result of Augmenting A with B is returned
#  in C. The CooMatrixes A and B are not changed.
#------------------------------------------------------------------------------------------
#  @param dmy_nRows2DInA
#         This is the number of rows in the 2D original first matrix [A].
#  @param dmy_nColumns2DInA
#         This is the number of columns in the 2D original first matrix [A].
#  @param dmy_nNonZeroValuesInA
#         This is the number of non-zero values in matrix [A].
#  @param dmy_fltNonZeroValuesInA
#         This is an array of non-zero values in matrix [A].
#  @param dmy_intNonZeroRowIndicesInA
#         This is an array of row indices of matrix [A] at which the value is non-zero. 
#  @param dmy_intNonZeroColumnIndicesInA
#         This is the minimum row height of A before appending B. 
#  @param dmy_nRows2DInB
#         This is the number of rows in the 2D original first matrix [B].
#  @param dmy_nColumns2DInB
#         This is the number of columns in the 2D original first matrix [B].
#  @param dmy_nNonZeroValuesInB
#         This is the number of non-zero values in matrix [B].
#  @param dmy_fltNonZeroValuesInB
#         This is an array of non-zero values in matrix [B].
#  @param dmy_intNonZeroRowIndicesInB
#         This is an array of row indices of matrix [B] at which the value is non-zero. 
#  @param dmy_intNonZeroColumnIndicesInB
#         This is an array of column indices of matrix [B] at which the value is non-zero. 
#  @param dmy_nMinimumRowsBefore
#         This is the minimum height for matrix [A] prior to augmenting.
#  @param dmy_nRows2DInC
#         This is the number of rows in the 2D original first matrix [C].
#  @param dmy_nColumns2DInC
#         This is the number of columns in the 2D original first matrix [C].
#  @param dmy_nNonZeroValuesInC
#         This is the number of non-zero values in matrix [C].
#  @param dmy_fltNonZeroValuesInC
#         This is an array of non-zero values in matrix [C].
#  @param dmy_intNonZeroRowIndicesInc
#         This is an array of row indices of matrix [C] at which the value is non-zero. 
#  @param dmy_intNonZeroColumnIndicesInC
#         This is an array of column indices of matrix [C] at which the value is non-zero. 
#  @result
#         This is an integer that denotes the success (0) or failure (non-zero) that will
#         be the result of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_coo_augment_rows.restype = int
ANOPP2.a2py_math_coo_augment_rows.argtypes =  \
          [# The A CooMatrix (in)
           A2_IK, A2_IK, A2_IK,
           POINTER(A2_RK),
           POINTER(A2_IK),
           POINTER(A2_IK),
           # The B CooMatrix (in)
           A2_IK, A2_IK, A2_IK,
           POINTER(A2_RK),
           POINTER(A2_IK),
           POINTER(A2_IK),
           # Minimum prior row height for padding the A CooMatrix
           A2_IK,
           # The C CooMtrix (out)
           POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK),
           POINTER(POINTER(A2_RK)),
           POINTER(POINTER(A2_IK)),
           POINTER(POINTER(A2_IK))]



#------------------------------------------------------------------------------------------
#  This routine filters columns from a CooMatrix returning a CooMatrix of equal extent but
#  having values only in the filtered columns. The original Matrix A is the source matrix
#  and the returned Matrix B is the target matrix having only the filtered columns.
#------------------------------------------------------------------------------------------
#  @param dmy_nRows2DInA
#         This is the number of rows in the 2D original first matrix [A].
#  @param dmy_nColumns2DInA
#         This is the number of columns in the 2D original first matrix [A].
#  @param dmy_nNonZeroValuesInA
#         This is the number of non-zero values in matrix [A].
#  @param dmy_fltNonZeroValuesInA
#         This is an array of non-zero values in matrix [A].
#  @param dmy_intNonZeroRowIndicesInA
#         This is an array of row indices of matrix [A] at which the value is non-zero. 
#  @param dmy_intNonZeroColumnIndicesInA
#         This is an array of column indices of matrix [A] at which the value is non-zero. 
#  @param dmy_intFilteredColumns
#         This is the array of filtered column indices.
#  @param dmy_nRows2DInB
#         This is the number of rows in the 2D original first matrix [B].
#  @param dmy_nColumns2DInB
#         This is the number of columns in the 2D original first matrix [B].
#  @param dmy_nNonZeroValuesInB
#         This is the number of non-zero values in matrix [B].
#  @param dmy_fltNonZeroValuesInB
#         This is an array of non-zero values in matrix [B].
#  @param dmy_intNonZeroRowIndicesInB
#         This is an array of row indices of matrix [B] at which the value is non-zero. 
#  @param dmy_intNonZeroColumnIndicesInB
#         This is an array of column indices of matrix [B] at which the value is non-zero. 
#  @result
#         This is an integer that denotes the success (0) or failure (non-zero) that will
#         be the result of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_coo_filter_columns.restype = int
ANOPP2.a2py_math_coo_filter_columns.argtypes =  \
          [# The A CooMatrix (in)
           A2_IK, A2_IK, A2_IK,
           POINTER(A2_RK),
           POINTER(A2_IK),
           POINTER(A2_IK),
           # The list of filtered columns to keep and length of list (in)
           POINTER(A2_IK),
           A2_IK,
           # The B CooMtrix (out)
           POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK),
           POINTER(POINTER(A2_RK)),
           POINTER(POINTER(A2_IK)),
           POINTER(POINTER(A2_IK))]



#------------------------------------------------------------------------------------------
#  This routine filters rows from a CooMatrix returning a CooMatrix of equal extent but
#  having values only in the filtered rows. The original Matrix A is the source matrix
#  and the returned Matrix B is the target matrix having only the filtered rows.
#------------------------------------------------------------------------------------------
#  @param dmy_nRows2DInA
#         This is the number of rows in the 2D original first matrix [A].
#  @param dmy_nColumns2DInA
#         This is the number of columns in the 2D original first matrix [A].
#  @param dmy_nNonZeroValuesInA
#         This is the number of non-zero values in matrix [A].
#  @param dmy_fltNonZeroValuesInA
#         This is an array of non-zero values in matrix [A].
#  @param dmy_intNonZeroRowIndicesInA
#         This is an array of row indices of matrix [A] at which the value is non-zero. 
#  @param dmy_intNonZeroColumnIndicesInA
#         This is an array of column indices of matrix [A] at which the value is non-zero. 
#  @param dmy_intFilteredRows
#         This is the array of filtered row indices.
#  @param dmy_nRows2DInB
#         This is the number of rows in the 2D original first matrix [B].
#  @param dmy_nColumns2DInB
#         This is the number of columns in the 2D original first matrix [B].
#  @param dmy_nNonZeroValuesInB
#         This is the number of non-zero values in matrix [B].
#  @param dmy_fltNonZeroValuesInB
#         This is an array of non-zero values in matrix [B].
#  @param dmy_intNonZeroRowIndicesInB
#         This is an array of row indices of matrix [B] at which the value is non-zero. 
#  @param dmy_intNonZeroColumnIndicesInB
#         This is an array of column indices of matrix [B] at which the value is non-zero. 
#  @result
#         This is an integer that denotes the success (0) or failure (non-zero) that will
#         be the result of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_coo_filter_rows.restype = int
ANOPP2.a2py_math_coo_filter_rows.argtypes =  \
          [# The A CooMatrix (in)
           A2_IK, A2_IK, A2_IK,
           POINTER(A2_RK),
           POINTER(A2_IK),
           POINTER(A2_IK),
           # The list of filtered columns to keep and length of list (in)
           POINTER(A2_IK),
           A2_IK,
           # The B CooMtrix (out)
           POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK),
           POINTER(POINTER(A2_RK)),
           POINTER(POINTER(A2_IK)),
           POINTER(POINTER(A2_IK))]



#------------------------------------------------------------------------------------------
# Returns a handle to the CooMatrix having the size specified by the row and column
# arguments with the values specified in dmy_fltNonZeroValues at the coordinates
# specified by dmy_intNonZeroRowIndices and dmy_intNonZeroColumnIndices.
#------------------------------------------------------------------------------------------
# @param dmy_nRows
#        This is the number of rows in the matrix.
# @param dmy_nColumns
#        This is the number of columns in the matrix.
# @param dmy_nNonZeroValues
#        This is the number of non-zero values.
# @param dmy_fltNonZeroValues
#        This is an array of non-zero values.
# @param dmy_intNonZeroRowIndices
#        This is an array of row indices at which the value is non-zero. 
# @param dmy_intNonZeroColumnIndices
#        This is an array of column indices at which the value is non-zero. 
# @param dmy_hdlOfResult
#        The handle to the CooMatrix.
# @result
#        An integer that represents the success of this function.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_coo_get_handle.restype = int
ANOPP2.a2py_math_coo_get_handle.argtypes =                                \
  [A2_IK, A2_IK, A2_IK, POINTER(A2_RK), POINTER(A2_IK), POINTER(A2_IK), \
   POINTER(c_void_p)]



#------------------------------------------------------------------------------------------
#  This routine slices columns from a CooMatrix returning a CooMatrix of equal number of
#  rows but having only the filtered columns. The original Matrix A is the source matrix
#  and the returned Matrix B is the target matrix having only the filtered columns.
#------------------------------------------------------------------------------------------
#  @param dmy_nRows2DInA
#         This is the number of rows in the 2D original first matrix [A].
#  @param dmy_nColumns2DInA
#         This is the number of columns in the 2D original first matrix [A].
#  @param dmy_nNonZeroValuesInA
#         This is the number of non-zero values in matrix [A].
#  @param dmy_fltNonZeroValuesInA
#         This is an array of non-zero values in matrix [A].
#  @param dmy_intNonZeroRowIndicesInA
#         This is an array of row indices of matrix [A] at which the value is non-zero. 
#  @param dmy_intNonZeroColumnIndicesInA
#         This is an array of column indices of matrix [A] at which the value is non-zero. 
#  @param dmy_intFilteredColumns
#         This is the array of filtered column indices.
#  @param dmy_nRows2DInB
#         This is the number of rows in the 2D original first matrix [B].
#  @param dmy_nColumns2DInB
#         This is the number of columns in the 2D original first matrix [B].
#  @param dmy_nNonZeroValuesInB
#         This is the number of non-zero values in matrix [B].
#  @param dmy_fltNonZeroValuesInB
#         This is an array of non-zero values in matrix [B].
#  @param dmy_intNonZeroRowIndicesInB
#         This is an array of row indices of matrix [B] at which the value is non-zero. 
#  @param dmy_intNonZeroColumnIndicesInB
#         This is an array of column indices of matrix [B] at which the value is non-zero. 
#  @result
#         This is an integer that denotes the success (0) or failure (non-zero) that will
#         be the result of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_coo_slice_columns.restype = int
ANOPP2.a2py_math_coo_slice_columns.argtypes =  \
          [# The A CooMatrix (in)
           A2_IK, A2_IK, A2_IK,
           POINTER(A2_RK),
           POINTER(A2_IK),
           POINTER(A2_IK),
           # The list of filtered columns to keep and length of list (in)
           POINTER(A2_IK),
           A2_IK,
           # The B CooMtrix (out)
           POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK),
           POINTER(POINTER(A2_RK)),
           POINTER(POINTER(A2_IK)),
           POINTER(POINTER(A2_IK))]



#------------------------------------------------------------------------------------------
#  This routine slices rows from a CooMatrix returning a CooMatrix of equal number of
#  columns but having only the filtered rows. The original Matrix A is the source matrix
#  and the returned Matrix B is the target matrix having only the filtered rows.
#------------------------------------------------------------------------------------------
#  @param dmy_nRows2DInA
#         This is the number of rows in the 2D original first matrix [A].
#  @param dmy_nColumns2DInA
#         This is the number of columns in the 2D original first matrix [A].
#  @param dmy_nNonZeroValuesInA
#         This is the number of non-zero values in matrix [A].
#  @param dmy_fltNonZeroValuesInA
#         This is an array of non-zero values in matrix [A].
#  @param dmy_intNonZeroRowIndicesInA
#         This is an array of row indices of matrix [A] at which the value is non-zero. 
#  @param dmy_intNonZeroColumnIndicesInA
#         This is an array of column indices of matrix [A] at which the value is non-zero. 
#  @param dmy_intFilteredRows
#         This is the array of filtered row indices.
#  @param dmy_nRows2DInB
#         This is the number of rows in the 2D original first matrix [B].
#  @param dmy_nColumns2DInB
#         This is the number of columns in the 2D original first matrix [B].
#  @param dmy_nNonZeroValuesInB
#         This is the number of non-zero values in matrix [B].
#  @param dmy_fltNonZeroValuesInB
#         This is an array of non-zero values in matrix [B].
#  @param dmy_intNonZeroRowIndicesInB
#         This is an array of row indices of matrix [B] at which the value is non-zero. 
#  @param dmy_intNonZeroColumnIndicesInB
#         This is an array of column indices of matrix [B] at which the value is non-zero. 
#  @result
#         This is an integer that denotes the success (0) or failure (non-zero) that will
#         be the result of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_coo_slice_rows.restype = int
ANOPP2.a2py_math_coo_slice_rows.argtypes =  \
          [# The A CooMatrix (in)
           A2_IK, A2_IK, A2_IK,
           POINTER(A2_RK),
           POINTER(A2_IK),
           POINTER(A2_IK),
           # The list of filtered columns to keep and length of list (in)
           POINTER(A2_IK),
           A2_IK,
           # The B CooMtrix (out)
           POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK),
           POINTER(POINTER(A2_RK)),
           POINTER(POINTER(A2_IK)),
           POINTER(POINTER(A2_IK))]



#------------------------------------------------------------------------------------------
# This routine transposes CooMatrix parts such that
#      B = A^T
# which signifies that B is the transposed A matrix.
#------------------------------------------------------------------------------------------
# @param dmy_nRowsInA
#        This is the number of rows in the 2D original first matrix [A].
# @param dmy_nColumnsInA
#        This is the number of columns in the 2D original first matrix [A].
# @param dmy_nNonZeroValuesInA
#        This is the number of non-zero values in matrix [A].
# @param dmy_fltNonZeroValuesInA
#        This is an array of non-zero values in matrix [A].
# @param dmy_intNonZeroRowIndicesInA
#        This is an array of row indices of matrix [A] at which the value is non-zero. 
# @param dmy_intNonZeroColumnIndicesInA
#        This is an array of column indices of matrix [A] at which the value is non-zero. 
# @param dmy_fltNonZeroValuesInB
#        This is an array of non-zero values in matrix [B].
# @param dmy_intNonZeroRowIndicesInB
#        This is an array of row indices of matrix [B] at which the value is non-zero. 
# @param dmy_intNonZeroColumnIndicesInB
#        This is an array of column indices of matrix [B] at which the value is non-zero. 
# @result
#        This is an integer that denotes the success (0) or failure (non-zero) that will
#        be the result of this operation.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_coo_transpose.restype = int
ANOPP2.a2py_math_coo_transpose.argtypes =  \
          [# The A CooMatrix (in)
           A2_IK, A2_IK, A2_IK,
           POINTER(A2_RK),
           POINTER(A2_IK),
           POINTER(A2_IK),
           # The B CooMtrix (out - implied sizes)
           POINTER(POINTER(A2_RK)),
           POINTER(POINTER(A2_IK)),
           POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
# This routine creates a rotation matrix for a rotation for an angle around an axis.
# -----------------------------------------------------------------------------------------
# @param dmy_fltEulerAxis
#        This is the axis for the rotation matrix.
# @param dmy_fltEulerAngle
#        This is the angle for the rotation matrix.
# @param dmy_fltRotationMatrix
#        This is the resulting rotation matrix.
# @result
#        This is an integer representing the success of the operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_rotation_matrix.restype = A2_IK
ANOPP2.a2py_math_rotation_matrix.argtypes = [POINTER(A2_RK), A2_RK, POINTER(POINTER(A2_RK))]



# -----------------------------------------------------------------------------------------
# This routine returns a handle to the CooMatrix created from a full format two-
# dimensional array.
# -----------------------------------------------------------------------------------------
# @param dmy_nRows
#        This is the number of rows in the two-dimensional matrix.
# @param dmy_nColumns
#        This is the number of columns in the two-dimensional matrix.
# @param dmy_fltFullFormatMatrix
#        This is the two-dimensional array from which the COO matrix is formed.
# @param dmy_fltZeroTolerance
#        If a real value is less than this zero tolerance multiplied with the maximum in
#        a matrix, it will be treated as zero and therefore will not be stored in the
#        COO matrix.
# @param dmy_hdlOfResult
#        Handle to the created CooMatrix.
# @result 
#        An integer indicating the success status of the operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_coo_create_from_full_matrix.restype = A2_IK
ANOPP2.a2py_math_coo_create_from_full_matrix.argtypes = \
  [A2_IK, A2_IK, POINTER(A2_RK), A2_RK, POINTER(c_void_p)]



# -----------------------------------------------------------------------------------------
# This function generates a full format matrix from a CooMatrix.
# -----------------------------------------------------------------------------------------
# @param dmy_hdlCooMatrix
#        This is the handle for the CooMatrix from which the full matrix will be formed.
# @param dmy_nRows
#        The number of rows in the full matrix.
# @param dmy_nColumns
#        The number of columns in the full matrix.
# @param dmy_fltFullMatrix
#        This is the two-dimensional matrix generated from the specified CooMatrix.
# @result 
#        An integer indicating the success status of the operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_get_full_matrix_from_coo.restype = A2_IK
ANOPP2.a2py_math_get_full_matrix_from_coo.argtypes = \
  [POINTER(c_void_p), POINTER(A2_IK), POINTER(A2_IK), POINTER(POINTER(A2_RK))]



# -----------------------------------------------------------------------------------------
# Extracts information about a CooMatrix from the handle to the CooMatrix passed to the
# function.  Extracts the number of rows, columns, and non-zero values, and the non-zero
# values at the coordinates specified by the row and column indices.
# -----------------------------------------------------------------------------------------
# @param dmy_hdlOfMatrix
#        Handle (reference) to the source CooMatrix from which to extract.
# @param dmy_nRows
#        This is the number of rows in the two-dimensional matrix.
# @param dmy_nColumns
#        This is the number of columns in the two-dimensional matrix.
# @param dmy_nNonZeroValues
#        This is the number of non-zero values.
# @param dmy_fltNonZeroValues
#        This is an array of non-zero values.
# @param dmy_intNonZeroRowIndices
#        This is an array of row indices at which the value is non-zero. 
# @param dmy_intNonZeroColumnIndices
#        This is an array of column indices at which the value is non-zero. 
# @result 
#        An integer indicating the success status of the operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_get_coo_matrix_parts_from_handle.restype = A2_IK
ANOPP2.a2py_math_get_coo_matrix_parts_from_handle.argtypes =            \
  [POINTER(c_void_p), POINTER(A2_IK), POINTER(A2_IK), POINTER(A2_IK), \
   POINTER(POINTER(A2_RK)), POINTER(POINTER(A2_IK)), POINTER(POINTER(A2_IK))]



#------------------------------------------------------------------------------------------
# Define the internal atan2 interface to take in complex numbers.
#------------------------------------------------------------------------------------------
# @param dmy_fltInputY
#        The imaginary part of the input
# @param dmy_fltInputX
#        The real part of the input
# @result
#        The atan2 of the input.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_atan2.restype = A2_RK
ANOPP2.a2py_math_atan2.argtypes = [A2_RK, A2_RK]



#------------------------------------------------------------------------------------------
# Define the internal acos interface to take in complex numbers.
#------------------------------------------------------------------------------------------
# @param dmy_fltInput
#        The value whose acos is desired.
# @result
#        The acos of the input.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_acos.restype = A2_RK
ANOPP2.a2py_math_acos.argtypes = [A2_RK]



#------------------------------------------------------------------------------------------
# Define the internal asin interface to take in complex numbers.
#------------------------------------------------------------------------------------------
# @param dmy_fltInput
#        The value whose asin is desired.
# @result
#        The asin of the input.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_math_asin.restype = A2_RK
ANOPP2.a2py_math_asin.argtypes = [A2_RK]



# -----------------------------------------------------------------------------------------
# This routine makes a copy of an existing handle.
# -----------------------------------------------------------------------------------------
# @param hdlMatrixA
#        The handle for the A CooMatrix.
# @param dhdlMatrixB
#        The handle for the resulting B CooMatrix.
# @result 
#        An integer indicating the success of the operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_copy_handle.restype = A2_IK
ANOPP2.a2py_math_copy_handle.argtypes = [POINTER(c_void_p), POINTER(c_void_p)]



# -----------------------------------------------------------------------------------------
# This routine deletes an existing CooMatrix from its specified handle.
# -----------------------------------------------------------------------------------------
# @param dmy_hdlMatrix
#        The handle for the CooMatrix.
# @result 
#        An integer indicating the success of the operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_finalize_handle.restype = A2_IK
ANOPP2.a2py_math_finalize_handle.argtypes = [POINTER(c_void_p)]



# -----------------------------------------------------------------------------------------
# This routine determines the reference count number for a CooMatrix.
# -----------------------------------------------------------------------------------------
# @param dmy_hdlMatrix
#        The CooMatrix handle for which the reference count will be found.
# @param dmy_intReferenceCount
#        The reference count returned.
# @result 
#        An integer indicating the success of the operation.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_math_get_coo_matrix_reference_count.restype = A2_IK
ANOPP2.a2py_math_get_coo_matrix_reference_count.argtypes = \
  [POINTER(c_void_p), POINTER(A2_IK)]



#!/usr/bin/env python
# =========================================================================================
#  Second part of this section of the interface file contains hardcoded enumerators used by
#  the user's program to communicate parameters and settings to the API.
# =========================================================================================

# -----------------------------------------------------------------------------------------
#  This enum is a list of interpolation functions that can be used.  The second
#   is not allowed yet.
# -----------------------------------------------------------------------------------------

#  This enumerator is for scattered data interpolation using inverse distance weighting
a2_math_interp_inverse_distance = 1

#  This enumerator is for scattered data interpolation using the radial basis function
a2_math_interp_radial_basis = 2

# -----------------------------------------------------------------------------------------
#  This enum is a list of data alignment, only the first is currently allowed.
# -----------------------------------------------------------------------------------------

#  This enumerator is for interpolation on structured data that is axis aligned
a2__math_interp_axis_aligned = 1

#  This enumerator is for interpolation on structured data that is not axis aligned
a2_math_interp_axis_not_aligned = 2

# -----------------------------------------------------------------------------------------
# These enumerators are to communicate to the API which method should be used for
# boundary conditions of the cubic spline interpolation. The options for the end points
# are either zero slope or zero curvature
# -----------------------------------------------------------------------------------------

#  This enumerator is for the cubic spline boundary condition of zero slope.
a2_math_zero_slope = 1

#  This enumerator is for the cubic spline boundary condition of zero curvature.
a2_math_zero_curvature = 2

# =========================================================================================
#  The following part of the parameters definition relates to the parameters required in
#  the Native Equations Module.  These parameters govern the string lengths of the equation
#  strings and the variables.
# =========================================================================================



# -----------------------------------------------------------------------------------------
#  The following constants are for identifying the maximum lengths of the variables and
#  the equations used in the functions available in the Native Equations module. 
# -----------------------------------------------------------------------------------------

# This is an integer that defines the maximum length of an equation string allowed in the
# Native Equations Module.
a2_math_equation_length = 500

# This is an integer that defines the maximum length of a variable string allowed in the
# Native Equations Module.
a2_math_variable_length = 100


#!/usr/bin/env python
# -----------------------------------------------------------------------------------------
#  1. This SOFTWARE is owned by NASA, and there shall be no further distribution or
#  publication of this , neither the source code, nor the executable code, nor
#  associated run-time applications, whether standalone or embedded, for use by any
#  third party without the express prior written approval of NASA LaRC. \n
#  \n
#  2. This SOFTWARE, and/or any modified version thereof, shall not be published for
#  profit, given to another entity, or in any manner offered for sale to the U.S.
#  Government or any other entity. The Government shall not pay a second time for the
#  SOFTWARE or any modified/enhanced version thereof. \n
#  \n
#  3. Should the SOFTWARE be modified or enhanced pursuant to a U.S. Government
#  contract, grant, or other agreement, the Government will be provided the complete
#  source code of that modified/enhanced version and the intellectual property rights
#   will be defined by the regulations governing said contract. \n
#  \n
#  4. User agrees and has signed NASA Langley's Internal Software Usage Agreement
#  (provided with SOFTWARE). \n
# -----------------------------------------------------------------------------------------
# The tag api contains routines for keeping track of unique tags.
# -----------------------------------------------------------------------------------------
# @author The ANOPP2 Development Team
# @version 1.0.0
# @file ANOPP2.api.py
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
# Initialize the ANOPP2.API.
# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
# This routine executes the unit tests in the ANOPP2.module.  The unit tests execute all
# the tests implemented in the ANOPP2.API.
# -----------------------------------------------------------------------------------------
# @result
#        The total number of failed asserts found during unit testing.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_tag_unit_test.restype = A2_IK



# -----------------------------------------------------------------------------------------
# This routine adds a tag type to the tag registry internal to the library.
# -----------------------------------------------------------------------------------------
# @param  intANOPP2.ype
#         An integer representing the tag type to be registered with this class
# @param  nChars
#         This is the size of the description string.
# @param  strDescription
#         A character string description of the new type being added (does not need
#         to be unique).
# @result
#         An integer representing the success of this operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_tag_request_tag_type.restype = A2_IK
ANOPP2.a2py_tag_request_tag_type.argtypes = [POINTER(A2_IK), A2_IK, POINTER(A2_CK)]



# -----------------------------------------------------------------------------------------
# Gets the tag type from an integer
# -----------------------------------------------------------------------------------------
# @param  intTag
#         An integer that has been set by the ANOPP2.lass
# @param  intANOPP2.ype
#         An integer representing the type of tag
# @result
#         An integer representing success of this operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_tag_get_tag_type.restype = A2_IK
ANOPP2.a2py_tag_get_tag_type.argtypes = [A2_IK, POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# Get the type of the Tag
# -----------------------------------------------------------------------------------------
# @param  intANOPP2.ype
#         An integer indicating what type of tag is being requested
# @param  udtTag
#         A ANOPP2.Class that will be populated with the new tag value
# @result
#         An integer that indicates the success of the operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_tag_request_tag.restype = A2_IK
ANOPP2.a2py_tag_request_tag.argtypes = [A2_IK, POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# Registers a tag with the ANOPP2.egistry
# -----------------------------------------------------------------------------------------
# @param  udtTag
#         An integer indicating what type of tag is being requested
# @result
#         An integer that indicates the success of the operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_tag_register_tag.restype = A2_IK
ANOPP2.a2py_tag_register_tag.argtypes = [POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# Get the type of the Tag
# -----------------------------------------------------------------------------------------
# @param  intTag
#         An integer with a tag value that should be unregistered from the ANOPP2.egistry
# @result
#         An integer that indicates the success of the operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_tag_unregister_tag.restype = A2_IK
ANOPP2.a2py_tag_unregister_tag.argtypes = [POINTER(A2_IK)]



# -----------------------------------------------------------------------------------------
# Get the description of the tag type
# -----------------------------------------------------------------------------------------
# @param  intANOPP2.ype
#         An integer indicating what type of tag is being requested
# @param  nChars
#         This is the size of the description string.
# @param  strDescription
#         A character string that will be set with the description
# @result
#         An integer that indicates the success of the operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_tag_get_description.restype = A2_IK
ANOPP2.a2py_tag_get_description.argtypes = [A2_IK, A2_IK, POINTER(A2_BK)]



#!/usr/bin/env python
# -----------------------------------------------------------------------------------------
# The error API contains a list of functions for tracking errors and reporting them
# to a log file.  There is a registry in the ANOPP2.odule that contains all the errors
# in the current run.
# -----------------------------------------------------------------------------------------
# @author The ANOPP2 Development Team
# @version 1.0.0
# @file ANOPP2.api.py
# -----------------------------------------------------------------------------------------



# =========================================================================================
# First part of this section contains interfaces into the available routines included in
# this API.
# =========================================================================================



# -----------------------------------------------------------------------------------------
# This initializes the error API. This creates the internal registry and makes the
# error enumerators available.
# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
# This routine executes the unit tests in the ANOPP2.module.  The unit
# tests execute all the tests implemented in the ANOPP2.API.
# -----------------------------------------------------------------------------------------
# @result
#         The number of failed asserts found during unit testing.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_error_unit_test.restype = A2_IK



# -----------------------------------------------------------------------------------------
# This functions reports an error to the log file.  The file name is passed to this
# routine with the line numbers, error enumerator and a message.
# -----------------------------------------------------------------------------------------
# @param  strFileName
#         A character string containing the name of the file where the error occurred
# @param  intLineNumber
#         An integer containing the line number where the error has occured
# @param  enumError
#         An integer that indicates what kind of error has occured
# @param  strMessage
#         A character string containing a message that should be displayed with the
#         error
# @result
#         An integer that if positive represents an error tag and if negative indicates
#         that an error has occurred
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_error_report.restype = A2_IK
ANOPP2.a2py_error_report.argtypes = [POINTER(A2_CK), A2_IK, A2_EK, POINTER(A2_CK)]



# -----------------------------------------------------------------------------------------
# This functions reports a log to the log file.  
# -----------------------------------------------------------------------------------------
# @param  strMessage
#         A character string containing a message that should be displayed with the
#         log.
# @result
#         An integer representation of the log entry.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_error_report_log_entry.restype = int
ANOPP2.a2py_error_report_log_entry.argtypes = [c_char_p]



# -----------------------------------------------------------------------------------------
# This clears the error registry of any errors.
# -----------------------------------------------------------------------------------------
# @result
#         An integer that indicates the success of the operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_error_clear.restype = A2_IK



# -----------------------------------------------------------------------------------------
# This returns the number of errors in the registry.
# -----------------------------------------------------------------------------------------
# @result
#         An integer indicating the number of errors in the registry or if negative
#         indicates that the operation was not successful
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_error_count.restype = A2_IK



# -----------------------------------------------------------------------------------------
# This prints a list of errors to the screen instead of the log file.
# -----------------------------------------------------------------------------------------
# @result
#         An integer indicating the success of the operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_error_print.restype = A2_IK



# -----------------------------------------------------------------------------------------
# Sets where the errors should be reported to, logfile, screen, network
# -----------------------------------------------------------------------------------------
# @param blnLogFile
#        Logical value indicating if the errors should be reported to the log file
# @param blnStdOut
#        Logical value indicating if the errors should be reported to the standard out
# @param blnNetwork
#        Logical value indicating if the errors should be reported to a blnNetwork
#        connection
# @result
#        An integer less than 0 that communciates failure of the code.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_error_outputs.restype = A2_IK
ANOPP2.a2py_error_outputs.argtypes = [A2_LK, A2_LK, A2_LK]



# -----------------------------------------------------------------------------------------
# Sets the filename to write output to
# -----------------------------------------------------------------------------------------
# @result
#        An integer indicating the success of the operation
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_error_file.restype = A2_IK
ANOPP2.a2py_error_file.argtypes = [POINTER(A2_CK)]



# -----------------------------------------------------------------------------------------
# This routine wraps the file compare utitly.  It takes in 2 file names, a test name, 
# precision of the data, decimal to compare, and an option to skip lines.
# -----------------------------------------------------------------------------------------
# @param dmy_nCharsA
#        The length of the name of the first file to be compared.
# @param dmy_nCharsB
#        The length of the name of the second file to be compared.
# @param dmy_nCharsTest
#        The length of the test name.
# @param dmy_strFileA
#        The first file to be compared.
# @param dmy_strFileB
#        The second file to be compared.
# @param dmy_strTestName
#        Name of the comparison, this will print out a string if the compare fails.
# @param dmy_intPrecision
#        How close the binary representation of the numbers are.  50 is very close, 500
#        less so, 5000 less so, and so one.
# @param dmy_intLineSkip
#        This will skip the first N lines of the file.
# @param dmy_blnFilesAreEqual
#        A boolean that is true if the files are equal, false if not.
# @result
#        An integer representing success of this operation.
# -------------------------------------------------------------------------------------------
ANOPP2.a2py_error_compare_files.restype = A2_IK
ANOPP2.a2py_error_compare_files.argtype =                                   \
     [A2_IK, A2_IK, A2_IK, POINTER(A2_CK), POINTER(A2_CK), POINTER(A2_CK), \
      A2_IK, A2_IK, POINTER(A2_LK)]



#!/usr/bin/env python
# ---------------------------------------------------------------------------------------
#  These are the standard error values included by the error module.  The calling
#  function uses these as input to the ANOPP2.function which logs the output to a .log
#  file.
# ---------------------------------------------------------------------------------------
#  


#  This is a standard error for a generic failure and is the error constants equivalent \
#   to -1
a2_error_operation_failed = 1

#  This is a standard error level for out of bounds. This is passed to the error \
#   function and written out to the log file.
a2_error_out_of_bounds = 2

#  This enumerator is for divide by zero. A divide by zero error is a common error that \
#   can be thrown by code that uses this module.
a2_error_divide_by_zero = 3

#  This enumerator communicates that an array that should not have been allocated or \
#   associated is allocated or assiciated
a2_error_already_allocated = 4

#  This error occurs when a select case is used with an invalid option.
a2_error_invalid_option = 5

#  This error is for a feature that should be implemented but is as of yet not.
a2_error_not_implemented = 6

#  This error is when 2 arrays that should be the same size are not the same size.
a2_error_array_size_mismatch = 7

#  This error occurs when a string that should contain data is empty
a2_error_empty_string = 8

#  This error occurs if a function, \
#   class or other code block is not ready to perform its task
a2_error_not_ready = 9

#  This error occurs when a namelist fails to be read properly
a2_error_namelist_read_failure = 10

#  This error occurs when an object is trying to be retrieved from a data structure \
#   manager but was not found.
a2_error_object_not_found = 11

#  This error is for failing to open a file.
a2_error_io_failure = 12

#  This warning occurs when atmosphere data and NPSS input data do not reference the \
#   same properties for an altitude.
a2_error_condition_mismatch = 13

#  This is the enumeration for an allocation failure.
a2_error_failed_allocation = 14

#  This is the enumeration for an deallocation failure.
a2_error_failed_deallocation = 15

#  This is an enumerator for a parallel problem.
a2_error_parallel_failure = 16

#  This is a flag that is throw when the user is trying to run functionality not
#  included in the type of release.
a2_error_not_available = 17

# -----------------------------------------------------------------------------------------
# The input/output API contains routines, enumerators, and constants for input and output.
# -----------------------------------------------------------------------------------------
#  @file ANOPP2.api.f90
#  @author The ANOPP2 Development Team
#  @version 1.0.0:
# -----------------------------------------------------------------------------------------



#------------------------------------------------------------------------------------------
# Change to the User's home directory, if known or set the User's home directory to the
# current working directory.
#------------------------------------------------------------------------------------------
# @result
#         An integer which is zero if there was no error and non-zero otherwise.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_io_cd_home.restype = A2_IK
ANOPP2.a2py_io_cd_home.argtypes = []



#------------------------------------------------------------------------------------------
# Change to the ANOPP2 scratch directory, creating it off the ANOPP2 home directory, which
# by default is the current working directory, if the scratch directory does not already
# exist.
#------------------------------------------------------------------------------------------
# @param  dmy_strScratchDirectoryName
#         Name of the scratch directory to be created under the User's home directory.
# @result
#         An integer which is zero if there was no error and non-zero otherwise.
#------------------------------------------------------------------------------------------
ANOPP2.a2py_io_cd_scratch.restype = A2_IK
ANOPP2.a2py_io_cd_scratch.argtypes = [POINTER(A2_CK)]



# =========================================================================================
#  Next part of this file contains hardcoded enumerators used by the user's program.
# =========================================================================================



# -----------------------------------------------------------------------------------------
#  These enumeration are for the output file format options.  They are communicated to
#  the Export function.  The Export function then writes out the information in
#  the specified file format.
# -----------------------------------------------------------------------------------------

# This is the enumeration of formatted file format 
a2_formatted = 1

# This is the enumeration of binary file format 
a2_binary = 2

# This is the enumeration for unformatted file format 
a2_unformatted = 3



# -----------------------------------------------------------------------------------------
#  This enumeration is for the program that will read the output file generated by the
#  Export function.  Options may include Tecplot or NetCDF.
# -----------------------------------------------------------------------------------------

# This is the enumeration of Tecplot formatted file 
a2_tecplot = 1

# This is the enumeration of NetCDF formatted file format 
a2_netcdf = 2



# -----------------------------------------------------------------------------------------
# The input/output API contains routines, enumerators, and constants for input and output.
# -----------------------------------------------------------------------------------------
#  @file ANOPP2.api.py
#  @author The ANOPP2 Development Team
#  @version 1.0.0:
# -----------------------------------------------------------------------------------------



# =========================================================================================
# The next part of this interface file contains general constants for ANOPP2.
# =========================================================================================



# -----------------------------------------------------------------------------------------
# The input/output API contains routines, enumerators, and constants for input and output.
# -----------------------------------------------------------------------------------------
#  @file ANOPP2.api.py
#  @author The ANOPP2 Development Team
#  @version 1.0.0:
# -----------------------------------------------------------------------------------------



# =========================================================================================
# The next part of this interface file contains general constants for ANOPP2.
# =========================================================================================



# -----------------------------------------------------------------------------------------
#  This file is the interface file for the Fortran subroutines in the memory management
#  Application Programming Interface (API).  These routines are used to deallocate memory
#  that has been allocated by any of the APIs within ANOPP2.  These are required because
#  when an ANOPP2 API allocated memory and the user then deallocates that memory, an
#  inconsistency may arise that would cause memory leaks.  When called thousands of times,
#  even a small memory leak can become a large problem.  Use these routines to ensure that
#  memory is consistent between ANOPP2's APIs and the user's User Code.  See 'Memory
#  Management' section of the API manual.
# -----------------------------------------------------------------------------------------
#  @file ANOPP2.api.py
#  @author The ANOPP2 Development Team
#  @version 1.0.0:
# -----------------------------------------------------------------------------------------



# -----------------------------------------------------------------------------------------
#  The a2py_deallocate_int_ptr function deallocates an allocated integer array pointed to
#  by intArray.
# -----------------------------------------------------------------------------------------
#  @param intArray
#         One dimensional array of integer values.
#  @result
#         An integer returned with zero indicating success and non-zero indicating an
#         error condition.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_mm_deallocate_int_ptr.restype = A2_IK
ANOPP2.a2py_mm_deallocate_int_ptr.argtypes = [POINTER(POINTER(A2_IK))]



# -----------------------------------------------------------------------------------------
#  The a2py_deallocate_float_ptr function deallocates an allocated real array pointed
#  to by fltArray.
# -----------------------------------------------------------------------------------------
#  @param fltArray
#         One dimensional array of real values.
#  @result
#         An integer returned with zero indicating success and non-zero indicating an
#         error condition.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_mm_deallocate_float_ptr.restype = A2_IK
ANOPP2.a2py_mm_deallocate_float_ptr.argtypes = [POINTER(POINTER(A2_RK))]



# -----------------------------------------------------------------------------------------
#  The a2py_deallocate_char_ptr function deallocates an allocated character array pointed
#  to by strArray.
# -----------------------------------------------------------------------------------------
#  @param strArray
#         One dimensional array of characters.
#  @result
#         An integer returned with zero indicating success and non-zero indicating an
#         error condition.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_mm_deallocate_char_ptr.restype = A2_IK
ANOPP2.a2py_mm_deallocate_char_ptr.argtypes = [POINTER(POINTER(A2_CK))]



# -----------------------------------------------------------------------------------------
#  The a2py_deallocate_bool_ptr function deallocates an allocated boolean array pointed
#  to by blnArray.
# -----------------------------------------------------------------------------------------
#  @param blnArray
#         One dimensional array of boolean values.
#  @result
#         An integer returned with zero indicating success and non-zero indicating an
#         error condition.
# -----------------------------------------------------------------------------------------
ANOPP2.a2py_mm_deallocate_bool_ptr.restype = A2_IK
ANOPP2.a2py_mm_deallocate_bool_ptr.argtypes = [POINTER(POINTER(A2_LK))]



# =========================================================================================
# The next part of this interface file contains general constants for ANOPP2.
# =========================================================================================



# -----------------------------------------------------------------------------------------
# This is a general constant for string length.
# -----------------------------------------------------------------------------------------
a2_const_str_len = A2_IK (1024)

# -----------------------------------------------------------------------------------------
# This is a constant zero.
# -----------------------------------------------------------------------------------------
a2_const_zero = A2_RK (0)

# -----------------------------------------------------------------------------------------
# This is a constant one.
# -----------------------------------------------------------------------------------------
a2_const_one = A2_RK (1)

# -----------------------------------------------------------------------------------------
# This is a constant pi.
# -----------------------------------------------------------------------------------------
a2_const_pi = 3.141592653589793238462643383279

# -----------------------------------------------------------------------------------------
# The input/output API contains routines, enumerators, and constants for input and output.
# -----------------------------------------------------------------------------------------
#  @file ANOPP2.api.py
#  @author The ANOPP2 Development Team
#  @version 1.0.0:
# -----------------------------------------------------------------------------------------


# ANOPP2. Add a Variable.
ANOPP2.a2c_axnl_add_variable.restype = c_void_p
ANOPP2.a2c_axnl_add_variable.argtypes = [
  c_char_p, # namelist_name
  c_char_p, # variable_name
  c_char_p  # variable_type
]


# ANOPP2. Add a Vector.
ANOPP2.a2c_axnl_add_vector.restype = None
ANOPP2.a2c_axnl_add_vector.argtypes = [
  c_char_p, # namelist_name
  c_char_p, # vector_name
  c_char_p  # variable_type
]


# ANOPP2. Assert that Namelist exists.
ANOPP2.a2c_axnl_assert_namelist.restype = None
ANOPP2.a2c_axnl_assert_namelist.argtypes = [
  c_char_p # namelist_name
]


# ANOPP2. Invoke callback for real vector.
ANOPP2.a2c_axnl_callback_real_vector.restype = None
ANOPP2.a2c_axnl_callback_real_vector.argtypes = [
    c_char_p,                  # namelist_name,
    c_char_p,                  # vector_name,
    CFUNCTYPE(None, c_double)  # callback
]


# ANOPP2. Invoke callback for a string.
ANOPP2.a2c_axnl_callback_string.restype = None
ANOPP2.a2c_axnl_callback_string.argtypes = [
    c_char_p,                  # namelist_name,
    c_char_p,                  # vector_name,
    CFUNCTYPE(None, c_char_p)  # callback
]


# ANOPP2. Create new Namelist.
ANOPP2.a2c_axnl_create_namelist.restype = None
ANOPP2.a2c_axnl_create_namelist.argtypes = [
  c_char_p  # namelist_name
]


# ANOPP2. Delete a file input stream.
ANOPP2.a2c_axnl_delete_ifstream.restype = c_void_p
ANOPP2.a2c_axnl_delete_ifstream.argtypes = [
  c_void_p,   # input_stream
]


# ANOPP2. Get a string.
ANOPP2.a2c_axnl_get_string.restype = c_char_p
ANOPP2.a2c_axnl_delete_ifstream.argtypes = [
  c_void_p,   # Address of the std::string from a2c_axnl_add_variable.
]


# ANOPP2. Open a file input stream.
ANOPP2.a2c_axnl_open_ifstream.restype = c_void_p
ANOPP2.a2c_axnl_open_ifstream.argtypes = [
  c_char_p,   # namelist_name
]


# ANOPP2. Read a Namelist.
ANOPP2.a2c_axnl_read_namelist.restype = None
ANOPP2.a2c_axnl_read_namelist.argtypes = [
  c_char_p,   # namelist_name
  c_void_p    # input_stream
]


# ANOPP2. Write Namelist.
ANOPP2.a2c_axnl_write_namelist.restype = None
ANOPP2.a2c_axnl_write_namelist.argtypes = [
  c_char_p, # namelist_name
  c_char_p, # filename
]


# ANOPP2. Define a method for adding a variable to a namelist.
def _a2py_axnl_add_func (nl, T, name) :

  # Test if this is a string.
  if T == c_char :

    # Create property backing.
    setattr(
      nl,
      '_'+name,
      cast(
        ANOPP2.a2c_axnl_add_variable(nl.__name__, name, T.__name__),
        c_void_p
      )
    )
    # Create the property.
    setattr(
      nl,
      '_get'+name,
      lambda self : ANOPP2.a2c_axnl_get_string(nl.__dict__['_'+name])
    )
    setattr(nl, name, property(nl.__dict__['_get'+name]))

  # Otherwise, the T is a type that receives standard handling.
  else :

    # Create a variable, add name to nl and attach to value in ANOPP2 X Namelist.
    setattr(
      nl,
      name,
      cast(
        ANOPP2.a2c_axnl_add_variable(nl.__name__, name, T.__name__),
        POINTER(T)
      ).contents
    )


# ANOPP2. Attach the hidden add function to the imported ANOPP2.module.
ANOPP2.a2py_axnl_add = _a2py_axnl_add_func


# ANOPP2. Define a hidden create function for a namelist.
def _a2py_axnl_create_func (nl) :
  nl.namelist = nl.__name__
  ANOPP2.a2c_axnl_create_namelist(nl.namelist)


# ANOPP2. Attach the hidden create function to the imported ANOPP2.module.
ANOPP2.a2py_axnl_create = _a2py_axnl_create_func


# ANOPP2. Define a hidden read function for a namelist.
def _a2py_axnl_read_func (nl, input_stream) :
  ANOPP2.a2c_axnl_read_namelist(nl.namelist, input_stream)


# ANOPP2. Define a hidden read function for a namelist.
ANOPP2.a2py_axnl_read = _a2py_axnl_read_func
