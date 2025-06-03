# RCAIDE/Library/Methods/Aerodynamics/Athena_Vortex_Lattice/purge_files.py
#  
# Created: Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# package imports 
import os

# ----------------------------------------------------------------------------------------------------------------------
#  purge_files
# ---------------------------------------------------------------------------------------------------------------------- 
def purge_files(filenames_array,directory=''):
	""" Purges folder folder of conflicting files
	"""    	
	for f in filenames_array:
		try:
			os.remove(os.path.abspath(os.path.join(directory,f)))
		except OSError:
			pass
			#print 'File {} was not found. Skipping purge.'.format(f)