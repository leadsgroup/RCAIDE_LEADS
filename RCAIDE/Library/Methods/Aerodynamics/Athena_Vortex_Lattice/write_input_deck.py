# RCAIDE/Library/Methods/Aerodynamics/Athena_Vortex_Lattice/write_input_deck.py
#
# Created: Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Library.Components.Wings.Control_Surfaces import Aileron , Elevator , Rudder  
from RCAIDE.Framework.Core import Units
from .purge_files import purge_files

# ----------------------------------------------------------------------------------------------------------------------
#  write_input_deck
# ---------------------------------------------------------------------------------------------------------------------- 
def write_input_deck(avl_object,trim_aircraft,control_surfaces):
    """
    Writes AVL execution command deck for automated analysis workflow and result extraction.

    Parameters
    ----------
    avl_object : Data
        AVL analysis object containing configuration and file management
            - current_status : Data
                Analysis execution status
                    - batch_file : str
                        Path to batch run case file
                    - deck_file : str
                        Path to command deck file for AVL execution
                    - cases : list
                        List of analysis cases to be executed
            - settings : Data
                Analysis configuration parameters
                    - filenames : Data
                        File naming configuration
                            - mass_file : str
                                Aircraft mass properties file path
    trim_aircraft : bool
        Flag to enable aircraft trimming for equilibrium flight conditions
    control_surfaces : bool
        Flag indicating presence of control surfaces requiring deflection commands

    Returns
    -------
    None
        Command deck file is written to disk for AVL execution

    Notes
    -----
    This function creates the AVL command deck that automates the complete
    analysis workflow including mass property loading, case execution, and
    result file generation. The deck serves as a script of keyboard commands
    that would otherwise be entered manually in interactive AVL operation.

    The command sequence includes:
        1. Mass property file loading for accurate CG and inertia
        2. Batch case file loading with flight conditions
        3. Operational mode selection for analysis type
        4. Individual case execution with result file generation
        5. Clean exit from AVL environment

    Each case generates multiple result files including stability derivatives,
    surface forces, strip forces, and body axis derivatives for comprehensive
    aerodynamic characterization.

    References
    ----------
    [1] Drela, M. and Youngren, H., "AVL", MIT, 2022, 

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.run_AVL_analysis : Function that executes the command deck
    """
    mass_file_input = \
'''MASS {0}
mset
0
PLOP
G

'''   
    open_runs = \
'''CASE {}
'''
    base_input = \
'''OPER
'''
    # unpack
    batch         = avl_object.current_status.batch_file
    deck_filename = avl_object.current_status.deck_file 
    mass_filename = avl_object.settings.filenames.mass_file

    # purge old versions and write the new input deck
    purge_files([deck_filename]) 
    with open(deck_filename,'w') as input_deck:
        input_deck.write(mass_file_input.format(mass_filename))
        input_deck.write(open_runs.format(batch))
        input_deck.write(base_input)
        for case in avl_object.current_status.cases:
            # write and store aerodynamic and static stability result files 
            case_command = make_case_command(avl_object,case,trim_aircraft,control_surfaces)
            input_deck.write(case_command)

        input_deck.write('\nQUIT\n')

    return


def make_case_command(avl_object,case,trim_aircraft,control_surfaces):
    """
    Creates execution commands for individual analysis case including trim and control surface configurations.

    Parameters
    ----------
    avl_object : Data
        AVL analysis object containing aircraft configuration
    case : Case
        Individual analysis case with flight conditions and result file specifications
    trim_aircraft : bool
        Flag to enable aircraft trimming commands
    control_surfaces : bool
        Flag to include control surface deflection commands

    Returns
    -------
    case_command : str
        Complete command sequence for case execution and result generation

    Notes
    -----
    This function generates the detailed command sequence for executing a
    single analysis case within the AVL environment. The commands include
    flight condition setup, trim configuration (if enabled), and systematic
    result file generation.

    Command organization follows AVL's operational flow:
        1. Case selection and flight condition setup
        2. Trim parameter configuration (angle of attack or lift coefficient)
        3. Rate parameter setup (roll/pitch rates, sideslip)
        4. Analysis execution and result file generation

    Multiple result files are generated to capture different aspects of the
    aerodynamic solution including force coefficients, stability derivatives,
    and spanwise load distributions.

    **Major Assumptions**
        * Case flight conditions are within AVL analysis capabilities
        * Trim convergence is achievable for specified conditions
        * All required result directories exist and are writable
    """
    # This is a template (place holder) for the input deck. Think of it as the actually keys
    # you will type if you were to manually run an analysis
    base_case_command = \
'''{0}{1}{2}{3}{4}
x 
{5}
{6}
{7}
{8}
{9}
{10}
{11}
{12}
'''  
    
    # if trim analysis is specified, this function writes the trim commands else it 
    # uses the defined deflection of the control surfaces of the aircraft
    if trim_aircraft:
        trim_command       = make_trim_text_command(case, avl_object)
        beta_command       = make_beta_text_command(case)
        roll_rate_command  = make_roll_rate_text_command(case)
        pitch_rate_command = make_pitch_rate_text_command(case)
    else:
        roll_rate_command  = ''
        pitch_rate_command = ''
        beta_command       = ''
        if control_surfaces:
            trim_command = control_surface_deflection_command(case,avl_object)
        else: 
            trim_command = ''
    
    index          = case.index
    case_tag       = case.tag
    
    # AVL executable commands which correlate to particular result types 
    aero_command_1 = 'st' # stability axis derivatives   
    aero_command_2 = 'fn' # surface forces 
    aero_command_3 = 'fs' # strip forces 
    aero_command_4 = 'sb' # body axis derivatives 
                   
    # create aliases for filenames for future handling
    aero_file_1    = case.aero_result_filename_1 
    aero_file_2    = case.aero_result_filename_2 
    aero_file_3    = case.aero_result_filename_3 
    aero_file_4    = case.aero_result_filename_4
    
    # purge files 
    if not avl_object.settings.keep_files:
        purge_files([aero_file_1])
        purge_files([aero_file_2])
        purge_files([aero_file_3])      
    
    # write input deck for avl executable 
    case_command = base_case_command.format(index,trim_command,roll_rate_command,pitch_rate_command ,beta_command,aero_command_1 , aero_file_1 ,aero_command_2  \
                                            , aero_file_2 , aero_command_3 , aero_file_3, aero_command_4 , aero_file_4) 
        
    return case_command

def make_trim_text_command(case,avl_object):
    """
    Generates trim commands for equilibrium flight analysis with automatic control surface assignment.

    Parameters
    ----------
    case : Case
        Analysis case containing flight condition specifications
            - conditions.aerodynamics.angles.alpha : float
                Target angle of attack in radians
            - conditions.aerodynamics.coefficients.lift.total : float or None
                Target lift coefficient for trimmed flight
    avl_object : Data
        AVL analysis object containing aircraft configuration
            - vehicle : Vehicle
                Aircraft with wing and control surface definitions

    Returns
    -------
    trim_command : str
        Complete trim command sequence for AVL execution

    Notes
    -----
    This function creates automatic trim commands that configure control
    surfaces for moment-balanced flight. Control surfaces are automatically
    assigned to appropriate control variables based on their type:
        - Ailerons: Roll moment control (RM)
        - Elevators: Pitch moment control (PM)  
        - Rudders: Yaw moment control (YM)

    Trim analysis can target either specified angle of attack (A command)
    or lift coefficient (C command) depending on case configuration.
    """
     
    cs_template = \
'''
D{0}
{1}
'''
    cs_idx = 1 
    cs_commands = ''
    for wing in avl_object.vehicle.wings:
        for ctrl_surf in wing.control_surfaces:
            if type(ctrl_surf) == Aileron:
                control = 'RM'
                cs_command = cs_template.format(cs_idx,control)
                cs_commands = cs_commands + cs_command
                cs_idx += 1   
            elif type(ctrl_surf) == Elevator:
                control = 'PM' 
                cs_command = cs_template.format(cs_idx,control)
                cs_commands = cs_commands + cs_command
                cs_idx += 1   
            elif type(ctrl_surf) == Rudder:
                control = 'YM' 
                cs_command = cs_template.format(cs_idx,control)
                cs_commands = cs_commands + cs_command
                cs_idx += 1   
            else:
                cs_idx += 1   
    base_trim_command = \
'''
c1
{0}
{1}
'''     
    # if Angle of Attack command is specified, write A 
    if case.conditions.aerodynamics.coefficients.lift.total == None:
        condition = 'A'
        val       = case.conditions.aerodynamics.angles.alpha
    elif case.conditions.aerodynamics.coefficients.lift.total > 0:  
        condition = 'C'
        val       = case.conditions.aerodynamics.coefficients.lift.total 
    else:   
        trim_command = ''
        return trim_command
        
    # write trim commands into template 
    trim_command = base_trim_command.format(condition,val)
    
    return cs_commands + trim_command

def make_roll_rate_text_command(case):
    """
    Generates roll rate commands for dynamic stability analysis.
    """
    base_roll_command = \
'''
R
R
{0}'''      
    roll_rate_coeff  = case.conditions.static_stability.coefficients.roll  
    if  roll_rate_coeff != 0.0:
        roll_command     = base_roll_command.format(roll_rate_coeff)
    else:
        roll_command = ''
    return roll_command

def make_pitch_rate_text_command(case):
    """
    Generates pitch rate commands for longitudinal dynamic stability analysis.
    """
    base_pitch_command = \
'''
Y
Y
{0}'''       
    pitch_rate_coeff   = case.conditions.static_stability.coefficients.pitch  
    if pitch_rate_coeff != 0.0:
        blade_pitch_command = base_pitch_command.format(pitch_rate_coeff)
    else:
        blade_pitch_command = ''
    return blade_pitch_command

def make_beta_text_command(case):
    """
    Generates sideslip angle commands for lateral-directional analysis.
    """
    base_roll_command = \
'''
B
B
{0}'''      
    beta = case.conditions.aerodynamics.angles.beta 
    if  beta != 0.0:
        beta_command     = base_roll_command.format(beta)
    else:
        beta_command = ''
    return beta_command

def control_surface_deflection_command(case,avl_object):
    """
    Generates control surface deflection commands for prescribed deflection analysis.
    """
    cs_template = \
'''
D{0}
D{1}
{2}'''
    cs_idx = 1 
    cs_commands = ''
    for wing in avl_object.vehicle.wings:
        for ctrl_surf in wing.control_surfaces:
            cs_command = cs_template.format(cs_idx,cs_idx,round(ctrl_surf.deflection/Units.degrees,4))
            cs_commands = cs_commands + cs_command
            cs_idx += 1
    return cs_commands 