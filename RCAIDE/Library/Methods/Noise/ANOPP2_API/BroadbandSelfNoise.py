## @ingroup DCode-C2_Aeroacoustics-ANOPP2_Python
# CompactF1A.py
#
# Created:  Mar 2023, R. Erhard
# Modified: 

# RCAIDE Imports
from RCAIDE.Library.Methods.Noise.ANOPP2_API.ANOPP2_api   import *
from RCAIDE.Framework.Core                                import Units
from .ANOPP2Noise import ANOPP2Noise

# Python Imports
from math   import pi
from struct import pack
import numpy as np
import pandas as pd

class BroadbandSelfNoise(ANOPP2Noise):
    """
    This is an interface between SUAVE and ANOPP2's ASNIFM broadband self-noise calcualtions.

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
        self.write_trailing_edge_line(rotor)
        self.write_trailing_edge_line_binary()

        #==========================================================================================
        # Step 3: Create Formulation 1A Functional Module
        # The F1A functional module needs an input file and several of the data structures
        # that we created previously.
        #==========================================================================================
        self.write_ASNIFM_config_file()

        #==========================================================================================
        # Step 4: Execute F1A
        # This routine is to execute the formulation 1A functional module.
        #==========================================================================================
        self.execute_SelfNoise()

        #==========================================================================================
        # Step 5: Export Noise Data
        # This step is to export all the noise data for analysis.
        #==========================================================================================
        self.export_monopole_dipole_terms(self.nBlades, self.intObserverTag, 
                                      self.intResultTags, self.intSuccess)

        self.export_total_APTH(self.intObserverTag, self.nResultTags, self.intResultTags, 
                           self.intTotalResultTag, self.intSuccess)

        self.export_total_PSD(self.intSuccess)
        #self.export_OASPL()

        # ------------------------------------------------------------------------------------------------
        # Print success statement.
        if self.intSuccess == 0:
            print("\nDone!\n")
        else:
            raise("Module in Compact F1A analysis failed!")

        return

    def write_trailing_edge_line(self, rotor):
        #------------------------------------------------------------------------------------------
        # The Trailing Edge Line Configuration, 'TrailingEdgeLine.config'
        #------------------------------------------------------------------------------------------
        
        # Open new file for writing as the TrailingEdgeLine.config.
        with open("TrailingEdgeLine.config", "w") as f:
        
            # Write out the trailing edge data line namelist.
            print(" &TrailingEdgeDataLineNamelist", file=f)
        
            # This is the name of the trailing edge binary data file.This code will generate
            # this file as guidance on how to create this data.
            print("   strFileName = \"TrailingEdgeLine.a2\"", file=f)
          
            # This tells the AGDS to read all the geometry data when the data structure is
            # created.
            print("   blnReadOnTheFly = .FALSE.", file=f)
          
            # This key offset will set an offset to the key(or time).It is useful when
            # creating periodic blades via namelists.Another way of creating periodic copies
            # is through the[prefix]_geometry_copy routine which is used later in this code.
            print("   fltGeometryKeyOffset = 0", file=f)
          
            # Similar to the geometry key offset, this allows a flow offset as well.
            print("   fltFlowKeyOffset = 0", file=f)
          
            # The blade geometry and flow data is defined in the local frame.This one frame
            # change will reorient this to the rotor frame.
            print("   nFrameChanges = 1", file=f)
          
            # Close the namelist file.
            print(" /", file=f)
          
            # And now the kinematics.The geometry binary file defines the location in the local
            # frame.This may not include such things as offsets or rotor rotations.This
            # kinematic namelist translates the blade frame to the rotor frame(rotation around the
            # z - axis).
            print("   &PolynomialFrameChangeNamelist", file=f)
          
            # This is the axis of rotation.This will rotate around the z - axis.
            print("     fltRotationAxis = 0, 0, 1", file=f)
          
            # The rotation coefficient is 0 for time independent rotation and omega as a
            # velocity offset(X = fltOmega * R).
            print(
              "     fltRotationCoefficient = %22.15f, %22.15f"
                % (-a2_const_pi / 2, self.fltOmega),
              file=f
            )
          
            # Close out the kinematic namelist.
            print("   /", file=f)

        return

    def write_trailing_edge_line_binary(self):
        #------------------------------------------------------------------------------------------
        # The Trailing Edge Line Binary Data File, 'TrailingEdgeLine.a2'
        #------------------------------------------------------------------------------------------
        
        # Open new file for writing as the TrailingEdgeLine.config.
        with open("TrailingEdgeLine.a2", "wb") as f:
            # Initialize the binary header for the trailing edge binary file where the following
            # holds: 
            # 42 (answer to everything),
            # version 1.2,
            # constant time type,
            # node - based,
            # double precision,
            # 8 additional geometry properties,
            # and no extension variable.
            # A2_IK binary_header[] = { 42, 1, 2, 1, 1, 2, 8, 0 };
            for p in [42, 1, 2, 1, 1, 2, 8, 0]:
                f.write(pack('i', p))
        
            # Write the number of points on the line.
            # output_stream.write(
            #   reinterpret_cast<const char *>(&nRadialStations),
            #   sizeof(nRadialStations)
            # );
            f.write(pack('i', self.nRadialStations))
        
            # Initialize the binary flow data header where the following holds:
            # Version 1.0, periodic, 4 variables, double precision, block organized,
            # the tip is not rounded, and three unused (reserved) extension variables.
            # A2_IK binary_flow_data_header[] = { 1, 0, 2, 4, 2, 1, 0, 0, 0, 0 };
            for p in [1, 0, 2, 4, 2, 1, 0, 0, 0, 0]:
                f.write(pack('i', p))
          
            # Output the time range (next three writes).
            f.write(pack('i', self.nAzimuthStations)) # Write the number of time steps.
            f.write(pack('d', a2_const_zero.value))   # Write zero seconds.
            f.write(pack('d', self.fltPeriod))        # Write the Period time length.
          
            # Output x, y, z Coordinates. (Starting at 0x60)
            for p in self.fltChordLineCoordinates:
                f.write(pack('d', p))
            for p in self.fltSpanLineCoordinates:
                f.write(pack('d', p))
            for p in self.fltThicknessLineCoordinates:
                f.write(pack('d', p))
        
            # Write out the additional geometry properties including the coord length, zero lift
            # angles of attack, and the trailing edge thickness and angles.
          
            # Write ChordLengths.
            for p in self.fltChordLengths:
                f.write(pack('d', p))
          
            # Allocate a local instance of transformed ChordLengths.
            # std::vector<A2_RK> transformedChordLengths(fltChordLengths.size());
            transformed_coordinates=[]
          
            # Apply the transform -> fltChordLengths * COS (fltPitchAngles) * 3 / 4
            for i in range (self.nRadialStations):
                transformed_coordinates.append(
                  3 * self.fltChordLengths[i] * math.cos(self.fltPitchAngles[i]) / 4
                )
        
            # Write cos-tranformed ChordLengths.
            for p in transformed_coordinates:
                f.write(pack('d', p))
        
            # Reinitialize the values of transformed ChordLengths to zero.
            # transformedChordLengths.assign(transformedChordLengths.size(), a2_const_zero);
            transformed_coordinates=[]
        
            # Write zeroed tranformed ChordLengths.
            for p in self.fltChordLengths:
                f.write(pack('d', 0))
          
            # Apply the transform -> (-) fltChordLengths * SIN (fltPitchAngles) * 3 / 4
            for i in range (self.nRadialStations):
                transformed_coordinates.append(
                  -3 * self.fltChordLengths[i] * math.sin(self.fltPitchAngles[i]) / 4
                )
          
            # Write sin-tranformed ChordLengths.
            for p in transformed_coordinates:
                f.write(pack('d', p))
          
            # Write fltZeroLiftAnglesOfAttack.
            for p in self.fltZeroLiftAnglesOfAttack:
                f.write(pack('d', p))
          
            # Write fltTrailingEdgeThicknesses.
            for p in self.fltTrailingEdgeThicknesses:
                f.write(pack('d', p))
          
            # Write fltTrailingEdgeAngles.
            for p in self.fltTrailingEdgeAngles:
                f.write(pack('d', p))
          
            # Write fltTripSettings.
            for p in self.fltTripSettings:
                f.write(pack('d', p))
          
            # Loop over all the azimuth stations and write out the inflow.  Assume that
            # each time step is the same inflow (hovering rotor) which can be written out once
            # as a constant loading, but for demonstration, it will be written out as time
            # dependent (periodic).
            for iAzimuthStation in range (self.nAzimuthStations):
            
                # Write out the time station number.
                f.write(pack('i', iAzimuthStation + 1))
            
                # Write the effective angles of attack at cell centered locations to match BARC.
                # output_stream.write(
                #   reinterpret_cast<const char *>(fltEffectiveAnglesOfAttack.data()),
                #   sizeof(A2_RK) * fltEffectiveAnglesOfAttack.size()
                # );
                for p in self.fltEffectiveAnglesOfAttack:
                    f.write(pack('d', p))
            
                # Write the inflow velocity.  This is -lambda * omega R which has already beeen
                # calculated.  To match BARC, we are going to do a cell-centered approach.
            
                # Alloacte a column of induced velocity components.
                # std::vector<A2_RK> column(fltInducedVelocities.size());
                column = []
            
                # Copy the first column of fltInducedVelocities.
                # std::transform(
                #   fltInducedVelocities.begin(),
                #   fltInducedVelocities.end(),
                #   column.begin(),
                #   [](std::array<A2_RK, 3> p)->A2_RK { return p[0]; }
                # );
                for p in self.fltInducedVelocities:
                    column.append (
                      p[0]
                    )
            
                # Write the first column of fltInducedVelocities.
                for p in column:
                    f.write(pack('d', p))
            
                # Reset the contents of the column of fltInducedVelocities.
                column = []
            
                # Copy the second column of fltInducedVelocities.
                for p in self.fltInducedVelocities:
                    column.append (
                      p[1]
                    )
            
                # Write the second column of fltInducedVelocities.
                for p in column:
                    f.write(pack('d', p))
            
                # Reset the contents of the column of fltInducedVelocities.
                column = []
            
                # Copy the third column of fltInducedVelocities.
                for p in self.fltInducedVelocities:
                    column.append (
                      p[2]
                    )
            
                # Write the third column of fltInducedVelocities.
                for p in column:
                    f.write(pack('d', p))
            
                # End of the iAzimuthStation loop.
            
              # End of the TrailingEdgeLine.a2 output stream.
            
            # Allocate Tags for the Trailing Edge Geometries.
            intTrailingEdgeAgdsTags=(A2_IK*self.nBlades)()
            
            # Allocate a buffer for getting new AGDS tags.
            new_agds_tag = A2_IK(0)
            
            # Create the geometry from the trailing edge line config file which will read the
            # trailing edge line data file.
            # intSuccess = a2cpp_geo_create(&(intTrailingEdgeAgdsTags[0]), "TrailingEdgeLine.config");
            self.intSuccess = \
              ANOPP2.a2py_geo_create(
                 new_agds_tag,
                create_string_buffer(b"TrailingEdgeLine.config")
              )
            
            # Copy the new tag value into the array.
            intTrailingEdgeAgdsTags[0] =  new_agds_tag.value
            
            # Similar to the lifting line, copy the trailing edge line to the other blades.
            # for (A2_IK iBlade = 1; iBlade < nBlades; ++iBlade) {
            for iBlade in range(1, self.nBlades):
            
                # # Copy geometry from first blade.
                self.intSuccess = \
                  ANOPP2.a2py_geo_copy_ads(
                    new_agds_tag,
                    intTrailingEdgeAgdsTags[0],
                    A2_RK(iBlade * self.fltPeriod / self.nBlades),
                    a2_const_zero,
                    A2_RK((2 * a2_const_pi * iBlade) / self.nBlades)
                  )
    
                # Copy the new tag value into the array.
                intTrailingEdgeAgdsTags[iBlade] = new_agds_tag.value
            
                # End of the iBlade Loop for copying blade geometries.
        
        return

    def write_ASNIFM_config_file(self):
        
        # Open new file for writing as the ASNIFM.config.
        # std::ofstream output_stream("ASNIFM.config", std::ios::binary | std::ios::trunc);
        
        # output_stream
        with open("ASNIFM.config", "w") as f:
        
            # Write the header for the namelist, this tells ANOPP2's Command Executive to create
            # an ASNIFM.
            print(" &AsnifmNamelist", file=f)
          
            # This is the number of emission times in the RP1218 simulation. The emission time 
            # range will be determined implicitely to fulfill the desired reception time.  once
            # determined, they will be divided into this number of equally spaced emission times.
            print("   nEmissionTimes = %12i" % self.nAzimuthStations , file=f)
          
            # Since the emission time range is set to 0,0, it implies that the emission time
            # will be determined implicitely from the reception time.
            print("   fltEmissionTimeRange = 0,0", file=f)
          
            # This is the number of reception times in the resultant spectrogram.
            print("   nReceptionTimes = %12i" % self.nAzimuthStations, file=f)
          
            # This is the desired reception time range [0, T] where T is the period of the rotor.
            print(
              "   fltReceptionTimeRange = %23.15e, %23.15e" 
                % (a2_const_zero.value, self.fltPeriod.value),
              file=f
            )
          
            # This is the retarded time tolerance.  When determining emission time range and
            # when determining the reception time from those emission times, the reception
            # time will be this fraction times the delta reception time.
            print("   fltRetardedTimeTolerance = 0.000001", file=f)
          
            # This is the metadata setting.  We will export all the metadata from ASNIFM.
            #  Metadata for cell-centered lines not supported at this time.  To get metadata
            #       export the boundary layer condition lines as node based.
            print("   strMetadataSetting = \"VELOCITY\"", file=f)
          
            # This is the name of the trailing edge line metadata.
            print("   strMetadataIdentifier = \"TrailingEdgeLine\"", file=f)
          
            # Export all the sources from each blade.  We will get a result for each source
            # (such as tip, pressure side, suction side, etc.) for each data line.
            print("   strResultsDesired = \"SOURCE\"", file=f)
          
            # Since the observer is not moving, the coordinate system of the observer is in the
            # ground.
            print("   strCoordinateSystem = \"GROUND\"", file=f)
          
            # These are the desired band numbers that the spectrogram will be defined in.
            print("   intBandRange = 10, 49", file=f)
          
            # Tip noise is a factor of the (dL'/dy)/(dL'/dy)_ref.  See equation 66 of RP1218.
            print("   fltTipAoaFactor = 1.1", file=f)
          
            # This closes out the namelist.
            print(" /", file=f)
        
        # End of stream for the ASNIFM.config.
        
        # Allocate a buffer for the ASNIFM Functional Module Tag.
        # A2_IK intAsnifmTag;
        intAsnifmTag=A2_IK(0)
        
        # Allocate a pointer to an a sequence of observer Result Tags for ASNIFM.
        # A2_IK * intAsnifmResultTags;
        intAsnifmResultTags=POINTER(A2_IK)()
        
        # Allocate a field for the length of the sequence of observer Result Tags.
        # A2_IK nAsnifmResultTags;
        nAsnifmResultTags=A2_IK(0)
        
        # Initialize a sequence of input tags;
        # std::vector<A2_IK> intInputTags{ intTrailingEdgeAgdsTags };
        listInputTags=[x for x in self.intTrailingEdgeAgdsTags]
        
        # Append the Atmosphere Tag to the input tags.
        listInputTags.append(self.intAtmosphereTag)
        
        # Construct an ctypes array of the list of tags.
        intInputTags=(A2_IK*len(listInputTags))(*listInputTags)
        
        # Pass the config file to the create functional module routine.  Also pass the 
        # geometries, atmosphere, and observer data structures.
        self.intSuccess = \
          ANOPP2.a2py_exec_create_functional_module(
            pointer(intAsnifmTag),
            create_string_buffer(b"ASNIFM.config"),
            len(listInputTags),
            intInputTags,
            pointer(self.intObserverTag),
            pointer(nAsnifmResultTags),
            pointer(intAsnifmResultTags)
          )

        return


    def execute_SelfNoise(self):
        # Now we can execute the F1A functional module.
        self.intSuccess = \
            ANOPP2.a2py_exec_execute_functional_module(
              self.intAsnifmTag,
              self.intAtmosphereTag,
              self.intFlightPathTag
            )
        return
    
    def export_all_noise_metrics(self):
        
        # Calculate the total noise spectrum. Treat intTotalBroadbandResultTag as an array.
        self.intSuccess = \
            ANOPP2.a2py_obs_calc_metric(
              self.intObserverTag,
              self.nAsnifmResultTags,
              self.intAsnifmResultTags,
              a2_aa_pbs,
              a2_obs_segment
            )
        
        # Loop over each blade and export the monopole and dipole acoustic pressure terms and
        # the Self Noise results (all 6 of them for each blade).
        for iBlade in range(1, self.nBlades + 1):
        
            # Export the pressure side TBL-TE spectrogram.
            self.intSuccess = \
              ANOPP2.a2py_obs_export(
                self.intObserverTag,
                self.intAsnifmResultTags[(iBlade - 1) * 6],
                "Blade.{}.pressure.pbsg.out.dat".format(iBlade).encode('ascii'),
                a2_aa_pbsg,
                a2_local,
                a2_formatted,
                a2_tecplot
              )
        
            # Export the SPL PBS spectrum also.
            self.intSuccess = \
              ANOPP2.a2py_obs_export(
                self.intObserverTag,
                self.intAsnifmResultTags[(iBlade - 1) * 6],
                "Blade.{}.pressure.pbs.out.dat".format(iBlade).encode('ascii'),
                a2_aa_pbs,
                a2_local,
                a2_formatted,
                a2_tecplot
              )
        
            # Export the suction side TBL-TE spectrogram.
            self.intSuccess = \
              ANOPP2.a2py_obs_export(
                self.intObserverTag,
                self.intAsnifmResultTags[(iBlade - 1) * 6 + 1],
                "Blade.{}.suction.pbsg.out.dat".format(iBlade).encode('ascii'),
                a2_aa_pbsg,
                a2_local,
                a2_formatted,
                a2_tecplot
              )
        
            # Export the suction side TBL-TE spectrum.
            self.intSuccess = \
              ANOPP2.a2py_obs_export(
                self.intObserverTag,
                self.intAsnifmResultTags[(iBlade - 1) * 6 + 1],
                "Blade.{}.suction.pbs.out.dat".format(iBlade).encode('ascii'),
                a2_aa_pbs,
                a2_local,
                a2_formatted,
                a2_tecplot
              )
        
            # Export the TBL-TE noise due to separation spectrogram.
            self.intSuccess = \
              ANOPP2.a2py_obs_export(
                self.intObserverTag,
                self.intAsnifmResultTags[(iBlade - 1) * 6 + 2],
                "Blade.{}.separation.pbsg.out.dat".format(iBlade).encode('ascii'),
                a2_aa_pbsg,
                a2_local,
                a2_formatted,
                a2_tecplot
              )
        
            # Export the TBL-TE noise due to separation spectrum.
            self.intSuccess = \
              ANOPP2.a2py_obs_export(
                self.intObserverTag,
                self.intAsnifmResultTags[(iBlade - 1) * 6 + 2],
                "Blade.{}.separation.pbs.out.dat".format(iBlade).encode('ascii'),
                a2_aa_pbs,
                a2_local,
                a2_formatted,
                a2_tecplot
              )
        
            # Export the LBL-VS spectrogram.
            self.intSuccess = \
              ANOPP2.a2py_obs_export(
                self.intObserverTag,
                self.intAsnifmResultTags[(iBlade - 1) * 6 + 3],
                "Blade.{}.lbl-vs.pbsg.out.dat".format(iBlade).encode('ascii'),
                a2_aa_pbsg,
                a2_local,
                a2_formatted,
                a2_tecplot
              )
        
            # Export the LBL-VS spectrum.
            self.intSuccess = \
              ANOPP2.a2py_obs_export(
                self.intObserverTag,
                self.intAsnifmResultTags[(iBlade - 1) * 6 + 3],
                "Blade.{}.lbl-vs.pbs.out.dat".format(iBlade).encode('ascii'),
                a2_aa_pbs,
                a2_local,
                a2_formatted,
                a2_tecplot
              )
        
            # Export the bluntness spectrogram.
            self.intSuccess = \
              ANOPP2.a2py_obs_export(
                self.intObserverTag,
                self.intAsnifmResultTags[(iBlade - 1) * 6 + 4],
                "Blade.{}.bluntness.pbsg.out.dat".format(iBlade).encode('ascii'),
                a2_aa_pbsg,
                a2_local,
                a2_formatted,
                a2_tecplot
              )
        
            # Export the bluntness spectrum.
            self.intSuccess = \
              ANOPP2.a2py_obs_export(
                self.intObserverTag,
                self.intAsnifmResultTags[(iBlade - 1) * 6 + 4],
                "Blade.{}.bluntness.pbs.out.dat".format(iBlade).encode('ascii'),
                a2_aa_pbs,
                a2_local,
                a2_formatted,
                a2_tecplot
              )
        
            # Export the tip spectrogram.
            self.intSuccess = \
              ANOPP2.a2py_obs_export(
                self.intObserverTag,
                self.intAsnifmResultTags[(iBlade - 1) * 6 + 5],
                "Blade.{}.tip.pbsg.out.dat".format(iBlade).encode('ascii'),
                a2_aa_pbsg,
                a2_local,
                a2_formatted,
                a2_tecplot
              )
            
            # Export the tip spectrogram.
            self.intSuccess = \
              ANOPP2.a2py_obs_export(
                self.intObserverTag,
                self.intAsnifmResultTags[(iBlade - 1) * 6 + 5],
                "Blade.{}.tip.pbs.out.dat".format(iBlade).encode('ascii'),
                a2_aa_pbs,
                a2_local,
                a2_formatted,
                a2_tecplot
              )
        
            # Allocate a fields for a Total Blade Result Tag.
            intTotalBladeResultTag = A2_IK(0);
        
            # Combine all the broadband for the ith blade 1/3rd-Octave SPL spectra results into a 
            # total spectrum.
            tag_list = [
              self.intAsnifmResultTags[i] for i in range(
                (iBlade - 1) * 6,
                (iBlade - 1) * 6 + 6
              )
            ]
            self.intSuccess = \
              ANOPP2.a2py_obs_combine_results(
                self.intObserverTag,
                len(tag_list),
                (A2_IK*len(tag_list))(*tag_list),
                a2_aa_pbsg,
                create_string_buffer(b"Total Blade"),
                pointer(intTotalBladeResultTag)
              )
        
            # Export the total spectrogram.
            self.intSuccess = \
              ANOPP2.a2py_obs_export(
                self.intObserverTag,
                self.intTotalBladeResultTag,
                "Blade.{}.total.pbsg.out.dat".format(iBlade).encode('ascii'),
                a2_aa_pbsg,
                a2_local,
                a2_formatted,
                a2_tecplot
              )
        
            # Calculate the PBS for the total blade results.
            self.intSuccess = \
              ANOPP2.a2py_obs_calc_metric(
                self.intObserverTag,
                1,
                pointer(intTotalBladeResultTag),
                a2_aa_pbs,
                a2_obs_segment
              )
        
            # Export the total spectrogram.
            self.intSuccess = \
              ANOPP2.a2py_obs_export(
                self.intObserverTag,
                intTotalBladeResultTag,
                "Blade.{}.total.pbs.out.dat".format(iBlade).encode('ascii'),
                a2_aa_pbs,
                a2_local,
                a2_formatted,
                a2_tecplot
              )
        
            # End of looping over each blade and exporting results.
        
        # Allocate a buffer for the Total Broadband Result Tag.
        intTotalBroadbandResultTag = A2_IK(0)
        
        # Combine all the broadband 1/3rd-Octave SPL spectra results into a total spectrogram.
        self.intSuccess = \
            ANOPP2.a2py_obs_combine_results(
              self.intObserverTag,
              self.nAsnifmResultTags,
              self.intAsnifmResultTags,
              a2_aa_pbs,
              create_string_buffer(b"Total Broadband"),
              pointer(intTotalBroadbandResultTag)
            )
        
        # Export the total broadband PBS spectrogram.
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalBroadbandResultTag,
              create_string_buffer(b"Total.broadband.pbs.out.dat"),
              a2_aa_pbs,
              a2_local,
              a2_formatted,
              a2_tecplot
            )
        
        # Before moving on to the finish, last step here for regression testing is combining all
        # the SPL spectrograms from each blade into a total broadband result.  We will keep the
        # results separate for now and get total rotor pressure side, suction side, separation,
        # lbl-vs, bluntness, and tip noise spectrograms.
        
        # Allocate a sequnce of length 6 for intTotalAsnifmResultTags.
        # std::vector<A2_IK> intTotalAsnifmResultTags(6);
        intTotalAsnifmResultTags=(A2_IK*6)()
        
        # Allocate a buffer for a new_result_tag.
        new_result_tag=A2_IK(0)
        
        # Combine the pressure side from all the blades into a single result.  The indexing into
        # the result tags array can be a little confusing.  It is basically a starting index and
        # a number to skip, so we want to start at 1 for pressure side (for example), continue to
        # the end and skip every 6 (or :6).
        
        tag_list = [self.intAsnifmResultTags[i] for i in range(0, self.nAsnifmResultTags.value, 6)]
        intAsnifmPressureSideResultTags = (A2_IK*len(tag_list))(*tag_list)
        self.intSuccess = \
            ANOPP2.a2py_obs_combine_results(
              self.intObserverTag,
              len(tag_list),
              intAsnifmPressureSideResultTags,
              a2_aa_pbsg,
              create_string_buffer(b"Total Pressure Side TBL"),
              new_result_tag
            )
        
        # Store the new result tag.
        intTotalAsnifmResultTags[0]=new_result_tag.value
        
        # Do the same for suction side.  This one starts at the second index.
        tag_list = [self.intAsnifmResultTags[i] for i in range(1, self.nAsnifmResultTags.value, 6)]
        self.intSuccess = \
            ANOPP2.a2py_obs_combine_results(
              self.intObserverTag,
              len(tag_list),
              (A2_IK*len(tag_list))(*tag_list),
              a2_aa_pbsg,
              create_string_buffer(b"Total Suction Side TBL"),
              new_result_tag
            )
        
        # Store the new result tag.
        intTotalAsnifmResultTags[1]=new_result_tag.value
        
        # Do the same for separation.  This one starts at the third index.
        tag_list = [self.intAsnifmResultTags[i] for i in range(2, self.nAsnifmResultTags.value, 6)]
        self.intSuccess = \
            ANOPP2.a2py_obs_combine_results(
              self.intObserverTag,
              len(tag_list),
              (A2_IK*len(tag_list))(*tag_list),
              a2_aa_pbsg,
              create_string_buffer(b"Total Separation TBL"),
              new_result_tag
            )
        
        # Store the new result tag.
        intTotalAsnifmResultTags[2]=new_result_tag.value
        
        # Do the same for lbl-vs.  This one starts at the fourth index.
        tag_list = [self.intAsnifmResultTags[i] for i in range(3, self.nAsnifmResultTags.value, 6)]
        self.intSuccess = \
            ANOPP2.a2py_obs_combine_results(
              self.intObserverTag,
              len(tag_list),
              (A2_IK*len(tag_list))(*tag_list),
              a2_aa_pbsg,
              create_string_buffer(b"Total LBL-VS"),
              new_result_tag
            )
        
        # Store the new result tag.
        intTotalAsnifmResultTags[3]=new_result_tag.value
        
        # Do the same for bluntness.  This one starts at the fifth index.
        tag_list = [self.intAsnifmResultTags[i] for i in range(4, self.nAsnifmResultTags.value, 6)]
        self.intSuccess = \
            ANOPP2.a2py_obs_combine_results(
              self.intObserverTag,
              len(tag_list),
              (A2_IK*len(tag_list))(*tag_list),
              a2_aa_pbsg,
              create_string_buffer(b"Total Bluntness"),
              new_result_tag
            )
        
        # Store the new result tag.
        intTotalAsnifmResultTags[4]=new_result_tag.value
        
        # And lastly, do the same for tip.  This one starts at the sixth index.
        tag_list = [self.intAsnifmResultTags[i] for i in range(5, self.nAsnifmResultTags.value, 6)]
        self.intSuccess = \
            ANOPP2.a2py_obs_combine_results(
              self.intObserverTag,
              len(tag_list),
              (A2_IK*len(tag_list))(*tag_list),
              a2_aa_pbsg,
              create_string_buffer(b"Total Tip"),
              new_result_tag
            )
        
        # Store the new result tag.
        intTotalAsnifmResultTags[5]=new_result_tag.value
        
        # Now that we have spectrograms for each ASNIFM component, calculate the PBS for 
        # each and export.
        self.intSuccess = \
            ANOPP2.a2py_obs_calc_metric(
              self.intObserverTag,
              6,
              intTotalAsnifmResultTags,
              a2_aa_pbs,
              a2_obs_segment
              );
        
        # Export the pressure side TBL 1/3rd-octave SPL spectrogram and spectrum.
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalAsnifmResultTags[0],
              create_string_buffer(b"Total.pressure.pbsg.out.dat"),
              a2_aa_pbsg,
              a2_local,
              a2_formatted,
              a2_tecplot
            )
        
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalAsnifmResultTags[0],
              create_string_buffer(b"Total.pressure.pbs.out.dat"),
              a2_aa_pbs,
              a2_local,
              a2_formatted,
              a2_tecplot
              );
        
        # Do the same for the suction side 1/3rd-octave SPL spectrogram and spectrum.
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalAsnifmResultTags[1],
              create_string_buffer(b"Total.suction.pbsg.out.dat"),
              a2_aa_pbsg,
              a2_local,
              a2_formatted,
              a2_tecplot
            )
        
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalAsnifmResultTags[1],
              create_string_buffer(b"Total.suction.pbs.out.dat"),
              a2_aa_pbs,
              a2_local,
              a2_formatted,
              a2_tecplot
            )
        
        # Do the same for the separation 1/3rd-octave SPL spectrogram and spectrum.
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalAsnifmResultTags[2],
              create_string_buffer(b"Total.separation.pbsg.out.dat"),
              a2_aa_pbsg,
              a2_local,
              a2_formatted,
              a2_tecplot
            )
        
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalAsnifmResultTags[2],
              create_string_buffer(b"Total.separation.pbs.out.dat"),
              a2_aa_pbs,
              a2_local,
              a2_formatted,
              a2_tecplot
            )
        
        # Do the same for the separation 1/3rd-octave SPL spectrogram and spectrum.
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalAsnifmResultTags[3],
              create_string_buffer(b"Total.lbl-vs.pbsg.out.dat"),
              a2_aa_pbsg,
              a2_local,
              a2_formatted,
              a2_tecplot
            )
        
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalAsnifmResultTags[3],
              create_string_buffer(b"Total.lbl-vs.pbs.out.dat"),
              a2_aa_pbs,
              a2_local,
              a2_formatted,
              a2_tecplot
            )
        
        # Do the same for the separation 1/3rd-octave SPL spectrogram and spectrum.
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalAsnifmResultTags[4],
              create_string_buffer(b"Total.bluntness.pbsg.out.dat"),
              a2_aa_pbsg,
              a2_local,
              a2_formatted,
              a2_tecplot
            )
        
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalAsnifmResultTags[4],
              create_string_buffer(b"Total.bluntness.pbs.out.dat"),
              a2_aa_pbs,
              a2_local,
              a2_formatted,
              a2_tecplot
            )
        
        # Do the same for the separation 1/3rd-octave SPL spectrogram and spectrum.
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalAsnifmResultTags[5],
              create_string_buffer(b"Total.tip.pbsg.out.dat"),
              a2_aa_pbsg,
              a2_local,
              a2_formatted,
              a2_tecplot
              );
        
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalAsnifmResultTags[5],
              create_string_buffer(b"Total.tip.pbs.out.dat"),
              a2_aa_pbs,
              a2_local,
              a2_formatted,
              a2_tecplot
              );
        
        # Now combine all the broadband total (for all blades) component noise into a single
        # 1/3rd-octave SPL spectrogram.  This is the same as the total broadband found above by
        # first calculating 1/3rd-octave SPL spectra { summing.  We are going to overwrite the
        # result tag and use it again here.
        self.intSuccess = \
            ANOPP2.a2py_obs_combine_results(
              self.intObserverTag,
              self.nAsnifmResultTags,
              self.intAsnifmResultTags,
              a2_aa_pbsg,
              create_string_buffer(b"Total Broadband"),
              intTotalBroadbandResultTag
            )
        
        # Deleta the PBS that was calculate prior so we can calculate it again, this time from
        # the PBSG sum.
        self.intSuccess = \
            ANOPP2.a2py_obs_delete_metric(
              self.intObserverTag,
              1,
              intTotalBroadbandResultTag,
              a2_aa_pbs
            )
        
        # Calculate the PBS from the PBSG of the total broadband.
        self.intSuccess = \
            ANOPP2.a2py_obs_calc_metric(
              self.intObserverTag,
              1,
              intTotalBroadbandResultTag,
              a2_aa_pbs,
              a2_obs_segment
            )
        
        # Export the total broadband PBS spectrogram and spectrum.  The reason for the ".pbs2."
        # is the metric and what it represents are the same as before when we summed PBS (instead
        # of PBSG here).  This will allow for direct comparison and demonstration.
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalBroadbandResultTag,
              create_string_buffer(b"Total.broadband.pbsg.out.dat"),
              a2_aa_pbsg,
              a2_local,
              a2_formatted,
              a2_tecplot
            )
        
        self.intSuccess = \
            ANOPP2.a2py_obs_export(
              self.intObserverTag,
              intTotalBroadbandResultTag,
              create_string_buffer(b"Total.broadband.pbs2.out.dat"),
              a2_aa_pbs,
              a2_local,
              a2_formatted,
              a2_tecplot
            )
        
        return