# RCAIDE/Library/Methods/Propulsor/Ducted_Fan_Propulsor/write_geometry.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from .purge_files import purge_files

# ---------------------------------------------------------------------------------------------------------------------- 
# Write Input Deck
# ----------------------------------------------------------------------------------------------------------------------  
def write_input_deck(dfdc_object):
    """
    Writes the execution commands input deck for DFDC analysis.
    
    Parameters
    ----------
    dfdc_object : DFDCAnalysis
        Analysis object containing the following attributes:
            - current_status : Data
                Current analysis state
                    - deck_file : str
                        Path to the input deck file to be created
            - settings : Data
                Configuration settings
                    - filenames : Data
                        File path information
                            - case : str
                                Case name for DFDC
                    - keep_files : bool
                        Flag to retain intermediate files
            - geometry : DuctedFan
                Ducted fan geometry with the following attributes:
                    - tag : str
                        Identifier for the ducted fan
                    - number_of_rotor_blades : int
                        Number of blades on the rotor
                    - number_of_radial_stations : int
                        Number of radial stations for blade definition
                    - cruise : Data
                        Cruise conditions
                            - design_thrust : float
                                Design thrust [N]
                            - design_altitude : float
                                Design altitude [m]
                            - design_angular_velocity : float
                                Design angular velocity [rad/s]
                            - design_freestream_velocity : float
                                Design freestream velocity [m/s]
                            - design_reference_velocity : float
                                Design reference velocity [m/s]
            - run_cases : list
                List of case objects with the following attributes:
                    - tag : str
                        Identifier for the case
                    - altitude : float
                        Altitude for the case [m]
                    - velocity : float
                        Freestream velocity for the case [m/s]
                    - RPM : float
                        Rotational speed for the case [RPM]
    
    Returns
    -------
    None
        
    Notes
    -----
    This function generates a DFDC input deck with the following sections:
        1. Header commands to load the case file
        2. Settings commands for the design case
        3. Case commands for each analysis case
        4. Quit command to terminate DFDC
    
    The input deck follows the command structure required by DFDC, including:
        - Atmospheric conditions (altitude)
        - Reference and freestream velocities
        - Blade count and radial station count
        - RPM setting
        - Thrust target
        - Design and execution commands
        - Output file specifications
        
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.purge_files
    RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.write_geometry
    RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.run_dfdc_analysis
    """
    # unpack 
    deck_filename = dfdc_object.current_status.deck_file 

    # purge old versions and write the new input deck
    purge_files([deck_filename]) 
    with open(deck_filename,'w') as input_deck:
        
        header_text     = make_header_text(dfdc_object)
        input_deck.write(header_text)
        
        settings_text     = make_settings_text(dfdc_object)
        input_deck.write(settings_text)
        
        for case in dfdc_object.run_cases:
            # write and store aerodynamic and static stability result files 
            case_command = make_case_command(dfdc_object,case)
            input_deck.write(case_command) 
        input_deck.write('\nQUIT\n')

    return

def make_header_text(dfdc_object): 
    base_header_text= \
'''LOAD
{0}
OPER
'''  
    header_text =  base_header_text.format(dfdc_object.settings.filenames.case)
    return header_text


def make_settings_text(dfdc_object):
    """ Makes commands for case execution in DFDC
    """  
    # This is a template (place holder) for the input deck. Think of it as the actually keys
    # you will type if you were to manually run an analysis
    base_settings_command = \
'''atmo
{0}
vref
{1}
vinf
{2}
nbld 
{3}
nrs 
{4}
rpm
{5}
thru
{6}
desi
shoo
{7}
'''
    ducted_fan        = dfdc_object.geometry 
    geometry_filename = ducted_fan.tag + '_geometry.txt'                    
    B                 = ducted_fan.number_of_rotor_blades         
    n                 = ducted_fan.number_of_radial_stations + 1           
    T                 = ducted_fan.cruise.design_thrust               
    alt               = ducted_fan.cruise.design_altitude /1000     
    RPM               = ducted_fan.cruise.design_angular_velocity /Units.rpm   
    V_inf             = ducted_fan.cruise.design_freestream_velocity  
    V_ref             = ducted_fan.cruise.design_reference_velocity  
    settings_command  = base_settings_command.format(alt,V_ref,V_inf,B,n,RPM,T,geometry_filename) 
 
    if not dfdc_object.settings.keep_files:
        purge_files([geometry_filename])         
    return settings_command

def make_case_command(dfdc_object,case):
    """ Makes commands for case execution in DFDC
    """  
    # This is a template (place holder) for the input deck. Think of it as the actually keys
    # you will type if you were to manually run an analysis
    base_case_command = \
'''atmo
{0}
rpm
{1} 
vinf
{2}
exec
writ
N
{3}
'''    
    alt               = case.altitude   
    V_inf             = case.velocity     
    RPM               = case.RPM        
    results_filename  = case.tag + '.txt'
    case_command      = base_case_command.format(alt,RPM,V_inf,results_filename)  
        
    return case_command 