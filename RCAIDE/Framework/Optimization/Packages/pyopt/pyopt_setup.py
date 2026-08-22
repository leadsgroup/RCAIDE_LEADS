# pyopt_setup.py
#
# Created:  Aug 2026, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# RCAIDE imports
import numpy as np
from RCAIDE.Framework.Optimization.Common import helper_functions as help_fun


# ----------------------------------------------------------------------
#  Pyoptsparse_Solve
# ----------------------------------------------------------------------
def Pyoptsparse_Solve(problem, solver='SLSQP', FD='single', sense_step=1.0E-6, nonderivative_line_search=False):
    """ This converts your RCAIDE Nexus problem into a pyoptsparse optimization problem and solves it.
        Supports SLSQP and IPOPT. No SNOPT: commercial license despite pyoptsparse
        itself being open source. CONMIN was tried and dropped -- see the ValueError
        message below for why.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        problem                   [nexus()]
        solver                    [str]
        FD (parallel or single)   [str]
        sense_step                [float]
        nonderivative_line_search [bool]

        Outputs:
        outputs                   [list]

        Properties Used:
        None
    """

    # Have the optimizer call the wrapper
    mywrap = lambda x: PyOpt_Problem(problem, x)

    inp = problem.optimization_problem.inputs
    obj = problem.optimization_problem.objective
    con = problem.optimization_problem.constraints

    if FD == 'parallel':
        from mpi4py import MPI
        comm   = MPI.COMM_WORLD
        myrank = comm.Get_rank()

    # Instantiate the problem and set objective
    try:
        import pyoptsparse as pyOpt
    except ImportError:
        # Not on PyPI under this name (confirmed) -- mdolab distributes it as a
        # source build only. pip can still build it directly from GitHub given a
        # working Fortran compiler:
        #   pip install git+https://github.com/mdolab/pyoptsparse.git
        # IPOPT additionally needs the native IPOPT library findable via
        # pkg-config (e.g. `brew install ipopt` on macOS, or
        # `conda install -c conda-forge ipopt`) plus `pip install cyipopt`.
        # SLSQP builds in without any external solver library.
        raise ImportError(
            'pyoptsparse not found. Install it with: '
            'pip install git+https://github.com/mdolab/pyoptsparse.git '
            '(requires a working Fortran compiler; IPOPT support additionally '
            'needs the native IPOPT library + `pip install cyipopt`).'
        )

    opt_prob = pyOpt.Optimization('RCAIDE', mywrap)
    for ii in range(len(obj)):
        opt_prob.addObj(obj[ii,0])

    # Set inputs
    nam  = inp[:,0] # Names
    ini  = inp[:,1] # Initials
    bndl = inp[:,2] # Bounds
    bndu = inp[:,3] # Bounds
    scl  = inp[:,4] # Scale
    typ  = inp[:,5] # Type

    # Pull out the constraints and scale them
    bnd_constraints     = help_fun.scale_const_bnds(con)
    scaled_constraints  = help_fun.scale_const_values(con, bnd_constraints)
    x                   = ini/scl

    for ii in range(0,len(inp)):
        lbd = (bndl[ii]/scl[ii])
        ubd = (bndu[ii]/scl[ii])
        #if typ[ii] == 'continuous':
        vartype = 'c'
        #if typ[ii] == 'integer':
            #vartype = 'i'
        opt_prob.addVar(nam[ii], vartype, lower=lbd, upper=ubd, value=x[ii])

    # Setup constraints
    for ii in range(0,len(con)):
        name = con[ii][0]
        edge = scaled_constraints[ii]

        if con[ii][1] == '<':
            opt_prob.addCon(name, upper=edge)
        elif con[ii][1] == '>':
            opt_prob.addCon(name, lower=edge)
        elif con[ii][1] == '=':
            opt_prob.addCon(name, lower=edge, upper=edge)

    # Finalize problem statement and run
    print(opt_prob)

    if solver == 'SLSQP':
        opt = pyOpt.SLSQP()
    elif solver == 'IPOPT':
        opt = pyOpt.IPOPT()
    else:
        raise ValueError(
            f"Unsupported mission_solver.method '{solver}' for the pyopt package. "
            f"Supported values are 'SLSQP', 'IPOPT'. (No SNOPT: commercial "
            f"license despite pyoptsparse itself being open source. CONMIN was "
            f"tried and dropped: pyoptsparse's CONMIN wrapper reports no "
            f"optInform at all and was observed reporting false convergence -- "
            f"declaring success while leaving every unknown at its unmoved "
            f"initial guess -- on RCAIDE's exactly-determined, equality-"
            f"constrained mission segments.)"
        )

    if nonderivative_line_search == True:
        opt.setOption('Nonderivative linesearch')
    if FD == 'parallel':
        outputs = opt(opt_prob, sens='FD', sensMode='pgc')
    else:
        # sens must be explicit for every backend, not just SLSQP -- pyoptsparse
        # 2.x raises "'None' value given for sens" if it's left unset, unlike
        # older versions that defaulted to FD automatically.
        outputs = opt(opt_prob, sens='FD', sensStep=sense_step)

    return outputs


# ----------------------------------------------------------------------
#  Problem Wrapper
# ----------------------------------------------------------------------
def PyOpt_Problem(problem, xdict):
    """ This wrapper runs the RCAIDE problem and is called by the pyoptsparse solver.
        Prints the inputs (x) as well as the objective values and constraints.
        If any values produce NaN then a fail flag is thrown.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        problem   [nexus()]
        xdict     [dict]

        Outputs:
        funcs     [dict]
        fail      [bool]

        Properties Used:
        None
    """

    x = []
    for key, val in xdict.items():
        x.append(float(val))

    obj   = problem.objective(x)
    const = problem.all_constraints(x).tolist()
    fail  = np.array(np.isnan(obj.tolist()) or np.isnan(np.array(const).any())).astype(int)

    funcs = {}
    for ii, obj_val in enumerate(obj):
        funcs[problem.optimization_problem.objective[ii,0]] = obj_val

    for ii, con_val in enumerate(const):
        funcs[problem.optimization_problem.constraints[ii,0]] = con_val

    print('Inputs')
    print(x)
    print('Obj')
    print(obj)
    print('Con')
    print(const)

    return funcs, fail
