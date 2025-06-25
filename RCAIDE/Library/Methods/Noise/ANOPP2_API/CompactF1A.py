## @ingroup DCode-C2_Aeroacoustics-ANOPP2_Python
# CompactF1A.py
#
# Created:  Mar 2023, R. Erhard
# Modified: 

# RCAIDE Imports
from RCAIDE.Library.Methods.Noise.ANOPP2_API.ANOPP2_api   import *
from RCAIDE.Framework.Core                                import Units
from .ANOPP2Noise                                         import ANOPP2Noise

# Python Imports
from math   import pi
from struct import pack
import numpy as np
import pandas as pd

class CompactF1A(ANOPP2Noise):
  """
  This is an interface between SUAVE and ANOPP2's Compact F1A formulation of the FW-H equation.
  
  Assumptions:
  None
  
  Source:
  None
  """

  def evaluate(self, rotor, conditions):
    """
    This converts the rotor aerodynamic outputs into the input config files necessary
    for running ANOPP2.
    
    """
    #==========================================================================================
    # Step 1: Initialize the aerodynamics, atmosphere, flight path, and observer modules
    #==========================================================================================
    self.initialize(rotor, conditions)
    
    #==========================================================================================
    # Step 2: Create Additional Data Structures
    # This step is to create all the data structures that are needed to do the F1A 
    # computation.  The data structures for the atmosphere, a flight path, and an observer
    # were previously created in initialization. Now the data structures for the rotor blade
    # geometries are created.
    #==========================================================================================
    # write lifting line data and geometry files
    self.write_lifting_line(rotor)
    self.write_lifting_line_geometry()
      
    #==========================================================================================
    # Step 3: Create Formulation 1A Functional Module
    # The F1A functional module needs an input file and several of the data structures
    # that we created previously.
    #==========================================================================================
    self.write_F1A_config_file()
    
    #==========================================================================================
    # Step 4: Execute F1A
    # This routine is to execute the formulation 1A functional module.
    #==========================================================================================
    self.execute_F1A()
    
    #==========================================================================================
    # Step 5: Export Noise Data
    # This step is to export all the noise data for analysis.
    #==========================================================================================
    self.export_monopole_dipole_terms()
    
    self.export_total_APTH()
    
    self.export_total_PSD()
    #self.export_OASPL()

    # ------------------------------------------------------------------------------------------------
    # Print success statement.
    if self.intSuccess == 0:
      print("\nDone!\n")
    else:
        raise("Module in Compact F1A analysis failed!")
    
    return
  
  def write_lifting_line(self, rotor):
    # ------------------------------------------------------------------------------------
    # Write the lifting line binary data file.
    # ------------------------------------------------------------------------------------
    output = open ("LiftingLine.a2", "wb")
    
    # Write the geometry header for the binary file.  This the following:
    # 42 (answer to everything), version 1.0, constant time type, node based, structured, 
    # double precision, calculate the normals, and an extension.
    output.write (pack ('i', 42))
    output.write (pack ('i', 1))
    output.write (pack ('i', 0))
    output.write (pack ('i', 1))
    output.write (pack ('i', 1))
    output.write (pack ('i', 2))
    output.write (pack ('i', 0))
    output.write (pack ('i', 0))
    
    # These are the dimensions of the surface.
    output.write (pack ('i', self.nRadialStations))
    
    # This is now the flow data header.  This is the following:
    # Version 1.0, periodic, 4 variable, double precision, block organized, and four
    # extension variables (all zero).
    output.write (pack ('i', 1))
    output.write (pack ('i', 0))
    output.write (pack ('i', 2))
    output.write (pack ('i', 4))
    output.write (pack ('i', 2))
    output.write (pack ('i', 1))
    output.write (pack ('i', 0))
    output.write (pack ('i', 0))
    output.write (pack ('i', 0))
    output.write (pack ('i', 0))
    
    # Write out the number of time steps in the flow data.  The time range varies from
    # 0 seconds to 1 period.
    output.write (pack ('i', self.nAzimuthStations))
    output.write (pack ('d', 0))
    output.write (pack ('d', self.fltPeriod))
    
    # And now write the x,y,z coordinates.
    for iRadialStation in range (self.nRadialStations):
      output.write (pack ('d', self.fltChordLineCoordinates[iRadialStation]))
    for iRadialStation in range (self.nRadialStations):
      output.write (pack ('d', self.fltSpanLineCoordinates[iRadialStation]))
    for iRadialStation in range (self.nRadialStations):
      output.write (pack ('d', self.fltThicknessLineCoordinates[iRadialStation]))
    
    # We can now loop over all the azimuth stations and write out the loads.  
    # Written out as time dependent (periodic).
    for iAzimuthStation in range (self.nAzimuthStations):
    
      # Write out the time iteration number.
      output.write (pack ('i', iAzimuthStation + 1))
    
      # We need the cross sectional area of the blade.  This is stored in the first 
      # flow data variable (even if it's a geometric one).
      for iRadialStation in range (self.nRadialStations):
        output.write (pack ('d', self.fltCrossSectionalAreas[iRadialStation]))
      
      # Get drag and lift forces along blade at this azimuth
      self.fltDynamicPressures = 0.5 * self.fltAmbientDensity.value * pow ((self.fltOmega * np.array(self.fltSpanLineCoordinates[0:self.nRadialStations])), 2)
      self.fltSpanForces = np.zeros_like(rotor.outputs.drag_coefficient)[0,:,iAzimuthStation]
      self.fltDragForces = self.fltDynamicPressures *  self.fltChordLengths * rotor.outputs.drag_coefficient[0,:,iAzimuthStation]
      self.fltLiftForces = self.fltDynamicPressures * self.fltChordLengths * rotor.outputs.lift_coefficient[0,:,iAzimuthStation]
      
      # Write out the drag, span, and lift forces. Negate forces because the force that is 
      # required is the force on the fluid, not the force on the structure.
      for iRadialStation in range (self.nRadialStations):
        output.write (pack ('d', -self.fltDragForces[iRadialStation]))
      for iRadialStation in range (self.nRadialStations):
        output.write (pack ('d', -self.fltSpanForces[iRadialStation]))
      for iRadialStation in range (self.nRadialStations):
        output.write (pack ('d', -self.fltLiftForces[iRadialStation]))
    
    # Close the file.
    output.close ()  
    return

  def write_lifting_line_geometry(self):
    fltZero = A2_RK(0)
    # ------------------------------------------------------------------------------------
    # Lifting line geometry
    # ------------------------------------------------------------------------------------    
    # Now let's open the binary file for the geometry. 
    output = open ("LiftingLine.config", "w")
    
    # Write out the acoustic data line namelist.
    output.write ("&AcousticDataLineNamelist\n")
    
    # This is the name of the lifting line binary data file.  This code will generate 
    # this file as guidance on how to create this data.
    output.write ("  strFileName = 'LiftingLine.a2'\n")
    
    # This tells the AGDS to read all the geometry data when the data structure is
    # created.
    output.write ("  blnReadOnTheFly = .FALSE.\n")
    
    # This key offset will set an offset to the key (or time).  It is useful when
    # creating periodic blades via namelists.  Another way of creating periodic copies
    # is through the [prefix]_geometry_copy routine which is used later in this code.
    output.write ("  fltGeometryKeyOffset = 0\n")
    
    # Similar to the geometry key offset, this allows a flow offset as well.
    output.write ("  fltFlowKeyOffset = 0\n")
    
    # The blade geometry and flow data is defined in the local frame.  This one frame
    # change will reorient this to the rotor frame.
    output.write ("  nFrameChanges = 1\n")
    
    # Close the namelist file.
    output.write ("/\n")
    
    # And now the kinematics.  The geometry binary file defines the location in the local 
    # frame. This may not include such things as offsets or rotor rotations.  This
    # kinematic namelist translates the blade frame to the rotor frame (rotation around the
    # z-axis).
    output.write ("  &PolynomialFrameChangeNamelist\n")
    
    # This is the axis of rotation.  This will rotate around the z-axis.
    output.write ("   fltRotationAxis = 0, 0, 1\n")
    
    # The rotation coefficient is 0 for time independent rotation and omega as a
    # velocity offset (X = fltOmega * R).
    output.write ("    fltRotationCoefficient = 0, " + '%.15f' % (self.fltOmega) + "\n")
    
    # Close out the kinematic namelist.
    output.write ("  /\n")
    
    # Close the file.
    output.close ()
    

    # Create the geometry from the lifting line config file which will read the lifting line
    # data file.  
    self.intSuccess =                       \
      ANOPP2.a2py_geo_create \
        (pointer (self.intBladeLineTags[0]), create_string_buffer (b"LiftingLine.config"))  
    
    # Now that we have a single lifting line, we can copy that to the other blades.  This 
    # will allow us to perform the calculation of a rotor with multiple blades by using 
    # only the values of 1 blade (including computations of temporal derivatives of all
    # flow and geometric properties).
    
    # There are a few things that we have to be aware of when making a copy.  The first is
    # that the geometry is time dependent, so we have to offset the grid and data.  Since 
    # the grid is time independent, there is no offset (0.0).  However, the data is offset
    # by ((iBlade - 1) / nBlades) * fltPeriod.  There is also a global position offset
    # around the rotor of 2 pi * (iBlade - 1) / nBlades.
    for iBlade in range(self.nBlades-1):
      self.intSuccess =                                           \
        ANOPP2.a2py_geo_copy_ads                   \
          (pointer (self.intBladeLineTags[1+iBlade]), self.intBladeLineTags[0], \
           (float (1+iBlade) / self.nBlades) * self.fltPeriod, fltZero, 2 * pi * (float (1+iBlade) / self.nBlades))
      
    return

  def write_F1A_config_file(self, config_name="Af1aifm"):
  
    # Next write out the config file for F1A
    output = open ("Af1aifm.config", "w")
    
    # Write out the header of the namelist, this tells ACE that we are creating an 
    # AF1AIFM.
    output.write ("&Af1aifmNamelist\n")
    
    # This is the number of emission times from the source.  Whatever emission time range
    # is determined from the reception time, the emission time is equally divided into
    # this number minus 1 segments.  A factor of 4 is here for a little overkill, it 
    # could be 1 or even 2.  This is approximately that every emission time step
    # in the data will be interpolated 4 times linearly between it's location in the 
    # data file.
    output.write ("  nEmissionTimes = " + str (4 * (self.nAzimuthStations - 1) + 1) + "\n")
    
    # This is the number of reception times.  We are using 8 times the number of azimuth
    # stations because the sources are moving toward and away from the observer.  A factor
    # of 2 ensures that we capture any fast source moving toward the observer.
    output.write ("  nReceptionTimes = " + str (8 * (self.nAzimuthStations - 1) + 1) + "\n")
    
    # Setting the emission time range to [0,0].  This tells AFFIFM to determine the 
    # emission time implicitly.  Every source structure (line in this case) will determine
    # a unique emission time range to completely fill the reception time defined next.
    # Since our source is periodic, this is easy.  If the source was aperiodic, we would
    # have to be careful that our source had enough emission time range.
    output.write ("  fltEmissionTimeRange = 0, 0\n")
    
    # This is the reception time.  Since the source is periodic, it is defined for all
    # time.  This tells the observer to listen for 1 period and the source to determine
    # a time range to fill this reception time range.
    output.write ("  fltReceptionTimeRange = 0, " + '%.15f' % (self.fltPeriod) + "\n")
    
    # This is the number of coefficients in the polynomial interpolation when doing the
    # observer time interpolation.
    output.write ("  nInterpolationCoefficients = 3\n")
    
    # This calculate on the fly setting to false means that the geometry data structure
    # (acoustic data line in this case) will calculate the velocity, acceleration, and
    # jerk ahead of time before the AFFIFM calculation (if time dependent).  The time 
    # derivatives of the cross sectional area and lift force (again, if time dependent)
    # are also calculated.
    output.write ("  blnCalculateOnTheFly = .FALSE.\n")
    
    # This is the results from AFFIFM desired by the user.  Source means that AFFIFM
    # will return 2 sources: one for monopole (thickness) and one for dipole (loading).
    # Other options include "TOTAL" (1 result) and "INTEGRAL" which will return a 
    # result for every integral in the formulation.  See the reference manual for 
    # definitions of the integral.
    output.write ("  strResultsDesired = 'SOURCE'\n")
    
    # Metadata export additional information about the formulation.  Options are: "NONE",
    # which does not print out a metadata file; "BASIC" which will export the source
    # (node) location, emission time, and reception time throughout the formulation (good
    # for debugging), "GEOMETRY" which exports everything the user provided to AFFIFM
    # including source properties (cross sectional area and force for an acoustic data
    # line) and segment length (velocity is also exported), "SOURCE" which provides all
    # the time derivatives of the source properties required plus the source terms like
    # Q and Fi, and "ALL" which exports the radiation vector, radiation length, radiation
    # coefficients (again see formulation documentation) and the acoustic pressure at the
    # observer from all sources.  Every setting includes all preceding settings.
    output.write ("  strMetadataSetting = 'None'\n")
    
    # The metadata will be put in a series of files (one file per geometry).  The file
    # is a TecPlot binary file.
    output.write ("  strMetadataIdentifier = 'Metadata'\n")
    
    # This is the retarded time tolerance.  When calculating the reception time from 
    # the emission time t = tau + r / c where r is a function of t and tau (t is 
    # reception time and tau is source time).  This is an iterative procedure.  The
    # solution is converged when |t_n-t_{n-1}| < tolerance * delta t where delta t is
    # the reception time range divided by the number of reception times minus 1.
    output.write ("  fltRetardedTimeTolerance = 0.000001\n")
    
    # It is possible to run AFFIFM at a series of waypoints.  By default, the noise from
    # all geometries is calculated at all waypoints.  This setting can be used to set
    # the number of geometries at every waypoint.  So, if we wanted to, for this demo
    # we could have 2 waypoints and nBlades * 2 number of geometries (lines).  By setting
    # the number of geometries per way point to [nBlades, nBlades], the first nBlades
    # would be used at the first way point, and the second at the second.  This allows
    # for maneuvering AFFIFM calculations.
    output.write ("  nGeometriesPerWayPoint = 0\n")
    
    # This is the coordinate system of the results cast onto the observer.  If the observer
    # is on the ground or moving independently of the source, then this is 'GROUND'.  If
    # the observer is following the source (such as a hemisphere around a source geometry)
    # then this is 'BODY'.
    output.write ("  strCoordinateSystem = 'GROUND'\n")
    
    # Close out the namelist.
    output.write ("/\n")
    
    # Close the file.
    output.close ()
    
    # This is the number of inputs that are required for the Functional Modules.  There
    # is nBlades lines and an atmosphere.
    nInputs = self.nBlades + 1
    
    # Allocate the array that will hold all the input tags.  The size of the array is 4
    # since there are 4: an atmosphere and 3 lines.
    intInputTags = (A2_IK * nInputs)()
    
    # Set the array to include the tags that were returned by the ANOPP2 API.  These are
    # all the inputs required by all the Functional Modules.
    for iBlade in range(len(self.intBladeLineTags)):
      intInputTags[iBlade] = self.intBladeLineTags[iBlade]
      
    intInputTags[self.nBlades] = self.intAtmosphereTag
    
    # Pass the config file to the create functional module routine.  Also pass the 
    # geometries, atmosphere, and observer data structures.
    self.intSuccess =                                                                \
      ANOPP2.a2py_exec_create_functional_module                       \
        (pointer (self.intFormulationTag), create_string_buffer ("{}.config".format(config_name).encode('ascii')), \
         nInputs, intInputTags, pointer (self.intObserverTag), pointer (self.nResultTags),      \
         pointer (self.intResultTags))
    
    return
  
  def execute_F1A(self):
    # Now we can execute the F1A functional module.
    self.intSuccess =                                           \
      ANOPP2.a2py_exec_execute_functional_module \
        (self.intFormulationTag, self.intAtmosphereTag, self.intFlightPathTag)
    return
  
  def export_monopole_dipole_terms(self):
    # Loop over each blade and export the monopole and dipole acoustic pressure terms.
    for iBlade in range(self.nBlades):
      blade_monopole_str = create_string_buffer("Blade.{}.monopole.out.dat".format(iBlade+1).encode('ascii'))
      blade_dipole_str = create_string_buffer("Blade.{}.dipole.out.dat".format(iBlade+1).encode('ascii'))
      
      # Set the filename to include the blade number for monopole noise.
      strExportFileName=blade_monopole_str
      # Now export the monopole pressure for this blade.
      self.intSuccess =                                                       \
        ANOPP2.a2py_obs_export                                 \
          (self.intObserverTag, self.intResultTags[iBlade * 2], (strExportFileName), \
           a2_aa_apth, a2_local, a2_formatted, a2_tecplot);
    
      # So the same for the dipole noise file name.
      strExportFileName=blade_dipole_str
    
      # And export the dipole pressure for this blade.
      self.intSuccess =                                                           \
        ANOPP2.a2py_obs_export                                     \
          (self.intObserverTag, self.intResultTags[iBlade * 2 + 1], strExportFileName, \
           a2_aa_apth, a2_local, a2_formatted, a2_tecplot);
    return
  
  def export_total_APTH(self):
    
    # Combine all the results into a total.
    self.intSuccess =                                                 \
      ANOPP2.a2py_obs_combine_results                  \
        (self.intObserverTag, self.nResultTags, self.intResultTags, a2_aa_apth, \
         create_string_buffer (b"Total"), pointer (self.intTotalResultTag))
    
    # Export the total noise.
    self.intSuccess =                                  \
      ANOPP2.a2py_obs_export            \
        (self.intObserverTag, self.intTotalResultTag,       \
         create_string_buffer (b"Total.out.dat"), \
         a2_aa_apth, a2_local, a2_formatted, a2_tecplot);
    return
  
  def export_total_PSD(self):
    # ------------------------------------------------------------------------------------------------
    # Calculate the power spectral density from the time history using the routine.
    # ------------------------------------------------------------------------------------------------
    totalPressureFile = "Total.out.dat"
    data = pd.read_table(totalPressureFile,skiprows=4, sep="\s+", names=['ObserverTime','AcousticPressure'])
  
    nTimes = len(data.ObserverTime)
    
    # Time array.
    fltTime = (A2_RK * (nTimes))()
    # Pressure array.
    fltPressure = (A2_RK * (nTimes))()
    
    fltTime[0:nTimes]     = data.ObserverTime
    fltPressure[0:nTimes] = data.AcousticPressure
    
    nFrequencies = A2_IK(0)
    # Returned PSD frequency array.
    fltPsdFrequency = pointer(A2_RK(0))
    # Returned PSD Msp array.
    fltPsdMsp = pointer(A2_RK(0))
    # Returned phase array.
    fltPhase = pointer(A2_RK(0))    
    
    # get psd from apth
    self.intSuccess =                                                                    \
      ANOPP2.a2py_aa_psd                                                  \
        (a2_aa_apth, a2_aa_pa, nTimes, fltTime, fltPressure, pointer(nFrequencies), \
         pointer(fltPsdFrequency), pointer(fltPsdMsp), pointer(fltPhase))
    
    
    # Write the output to a file
    output = open ("Psd.apth.pa.out.dat", "w")
    # Save the results for analysis.
    for i in range (0, nFrequencies.value) :
      output.write (str(fltPsdFrequency[i]) + "   ")
      output.write (str(fltPsdMsp[i]) + "\n")
    # Close out the file.
    output.close ()  
    return
  
  def others():
    ANOPP2.a2py_obs_insert_oaspl()
    return
