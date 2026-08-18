# RCAIDE/Methods/Mission/Common/__init__.py
#

"""Mission Common Module — shared computational stages for mission simulation.

This module contains the stages that execute during a mission simulation. Each
stage has a distinct role in the pipeline:

Pre_Process
    Runs **once** before any simulation begins. Validates and prepares the
    vehicle configuration, powertrain topology, aerodynamic surrogates, and
    control variables. Sets up unknowns and residuals for the solver. This is
    where the energy network topology is analyzed — classifying components by
    domain, resolving hybridization factors (phi / psi), and registering any
    unresolved power splits as optimization variables.

Initialize
    Runs at the **start of each segment**. Allocates and populates the data
    structures (conditions arrays) that store results during iteration. For
    energy systems this includes initializing battery states, fuel masses,
    thermal conditions, and distributor power bookkeeping.

Update
    Runs **each solver iteration** within a segment. Computes the current
    state — atmosphere, aerodynamics, network performance, forces, moments,
    weights, and stability — from the current unknowns.

Unpack_Unknowns
    Runs **each solver iteration**. Maps the solver's unknown vector into
    physically meaningful segment variables (orientation, control surfaces).

Residuals
    Runs **each solver iteration**. Computes the mismatch between the current
    state and the required flight dynamics constraints, driving the solver
    toward convergence.
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .   import Initialize
from .   import Pre_Process
from .   import Residuals
from .   import Unpack_Unknowns
from .   import Update

from .Segments import *
 