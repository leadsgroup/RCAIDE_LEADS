<p align="center">
  <img src="https://github.com/leadsgroup/RCAIDE_Website/blob/main/assets/img/RCAIDE_Logo_No_Background.png" width=25% height=25%> 
</p> 

<div align="center">

[![CI](https://github.com/leadsgroup/RCAIDE_LEADS/actions/workflows/CI.yml/badge.svg?branch=master)](https://github.com/leadsgroup/RCAIDE_LEADS/actions/workflows/CI.yml)
[![Documentation](https://github.com/leadsgroup/RCAIDE_LEADS/actions/workflows/sphinx_docs.yml/badge.svg)](https://github.com/leadsgroup/RCAIDE_LEADS/actions/workflows/sphinx_docs.yml)
[![codecov](https://codecov.io/gh/leadsgroup/RCAIDE_LEADS/graph/badge.svg?token=WZOFW5EKWJ)](https://codecov.io/gh/leadsgroup/RCAIDE_LEADS)
[![PyPI Downloads](https://static.pepy.tech/badge/rcaide-leads)](https://pepy.tech/projects/rcaide-leads)

</div>

[RCAIDE: Research Community Aircraft Interdisciplinary Design Environment](https://www.rcaide.leadsresearchgroup.com/)
=======

The Research Community Aircraft Interdisciplinary Design Environment, or RCAIDE (pronounced “arcade”) is a powerful open-source Python platform for aircraft design and analysis. From commercial airliners to UAVs and next-generation hybrid-electric aircraft, RCAIDE provides comprehensive multi-disciplinary analysis tools backed by validated engineering methods. Its streamlined workflow and modular architecture help aerospace engineers and researchers accelerate development cycles and explore innovative designs with confidence. RCAIDE-LEADS is a GitHub fork of RCAIDE, developed and maintained by the [Lab for Electric Aircraft Design and Sustainability](https://www.leadsresearchgroup.com/).

## Table of Contents
- [Citing RCAIDE](#citing-rcaide)
- [Userbase](#userbase)
- [Code Architecture](#code-architecture)
- [Capabilities](#capabilities-of-rcaide)
- [External Interfaces](#external-interfaces)
- [Installing RCAIDE](#installing-rcaide)
- [Tutorials](#tutorials)
- [Contributing](#contributing-to-rcaide)
- [Get in Touch](#get-in-touch)

## Citing RCAIDE

If you use RCAIDE in your research, please cite:

> Clarke, Matthew A., et al. "RCAIDE: A Multidisciplinary Analysis Toolbox for Aircraft Design and Flight Simulation." *Aerospace Science and Technology* (2026): 112328.

```bibtex
@article{clarke2026rcaide,
  title   = {RCAIDE: A Multidisciplinary Analysis Toolbox for Aircraft Design and Flight Simulation},
  author  = {Clarke, Matthew A. and others},
  journal = {Aerospace Science and Technology},
  pages   = {112328},
  year    = {2026}
}
```

## Userbase
Shown below, the widespread adoption of RCAIDE demonstrates the reach of our codebase, providing validated and verified results to communities worldwide. Notable users include: 
* **Industry and Government:** NASA, Boeing, AFRL, Embraer, Joby, Vahana, Argonne National Labs, Bombardier, Raytheon, BAE, Google
* **Academia:** MIT, Purdue, Embry Riddle, Carnegie Mellon, Princeton, Virginia Tech, Georgia Tech, Michigan, Stanford University, Cranfield University, University of Sydney, TU Delft, IIT, University of Toronto, Concordia University, ISAE
<p align="center">
  <img src="https://github.com/leadsgroup/RCAIDE_Website/blob/main/assets/img/SUAVE_Usage.png" width=50% height=50%> 
</p> 

## Code Architecture 
The code is arranged into repositories that house native data structures, functions, components, and subroutines for discipline analyses and support number-crunching operations. This allows developers or avid users seeking to modify the source code to navigate intuitively. Solely written in Python, an RCAIDE installation
appears in one repository that is itself organized into two secondary-level repositories: 
* **RCAIDE** sub-directory, where its source code resides
* **Regressions** sub-directory, where unit tests for verification and validation are performed.

```mermaid
%%{init: {'flowchart': {'curve': 'linear', 'nodeSpacing': 50, 'rankSpacing': 50}}}%%
flowchart LR
    RCAIDE_LEADS[RCAIDE_LEADS]
    RCAIDE[RCAIDE]
    Regressions[Regressions]
    
    RCAIDE_LEADS ---> RCAIDE
    RCAIDE_LEADS ---> Regressions

    style RCAIDE_LEADS fill:#0d6dc5,color:#fff
    style RCAIDE fill:#09d0d9,color:#fff
    style Regressions fill:#09d0d9,color:#fff
```
The RCAIDE subdirectory is arranged into frameworks and methods modules. Its predecessor, SUAVE, was written primarily as a superseding framework. Think of framework modules as the glue or roadmap that connects all the functions housed in the Library folder. The framework folder mainly comprises core data structures, class instances of the various methods within the code, the mission and energy networks and the optimization framework. The Library module comprises five tertiary submodules: Attributes, Components, Methods, Mission and Plots.

```mermaid
%%{init: {'flowchart': {'curve': 'linear', 'nodeSpacing': 50, 'rankSpacing': 50}}}%%
flowchart TB
    RCAIDE[RCAIDE] --> Framework
    RCAIDE --> Libraries
    
    %% Framework components
    Framework --> Mission
    Framework --> Analyses
    Framework --> Optimization
    Framework --> Data
    
    %% Library components
    Libraries --> Aerodynamics
    Libraries --> Aeroacoustics
    Libraries --> Costs
    Libraries --> Stability
    Libraries --> Energy
    Libraries --> FlightPerf[Flight Performance]
    Libraries --> Weights
    
    %% Styling
    style RCAIDE fill:#09d0d9,color:#fff
    style Framework fill:#0fcf99,color:#fff
    style Libraries fill:#0fcf99,color:#fff
    
    %% Framework children styling - Burgundy
    style Mission fill:#ffaf33,color:#fff
    style Analyses fill:#ffaf33,color:#fff
    style Optimization fill:#ffaf33,color:#fff
    style Data fill:#ffaf33,color:#fff
    
    %% Libraries children styling - Purple
    style Aerodynamics fill:#ffaf33,color:#fff
    style Aeroacoustics fill:#ffaf33,color:#fff
    style Costs fill:#ffaf33,color:#fff
    style Stability fill:#ffaf33,color:#fff
    style Energy fill:#ffaf33,color:#fff
    style FlightPerf fill:#ffaf33,color:#fff
    style Weights fill:#ffaf33,color:#fff
```
## Capabilities of RCAIDE
RCAIDE currently possesses the ability to perform various analyses at multiple fidelity levels. Higher fidelity provides greater accuracy but requires more computational resources. The multi-fidelity capability enables:

### Aircraft Design & Analysis
* **Geometry**
  * Advanced parameterization
  * 3D visualization — click any model below to open an interactive viewer

  <table align="center">
    <tr>
      <td align="center" width="25%">
        <img src="Digital_Hangar/Boeing_737_Max_8/Boeing_737_Max_8.png" width="100%" alt="Boeing 737 MAX 8"/><br/>
        <b>Boeing 737 MAX 8</b>
      </td>
      <td align="center" width="25%">
        <img src="Digital_Hangar/Blended_Wing_Body_Hydrogen/BWB_LH2.png" width="100%" alt="Hydrogen BWB"/><br/>
        <b>Hydrogen BWB</b>
      </td>
      <td align="center" width="25%">
        <img src="Digital_Hangar/ATR_72_All_Electric/ATR_72_all_electric.png" width="100%" alt="ATR 72 All-Electric"/><br/>
        <b>ATR 72 All-Electric</b>
      </td>
      <td align="center" width="25%">
        <img src="Digital_Hangar/Concorde/Concorde.png" width="100%" alt="Concorde"/><br/>
        <b>Concorde</b>
      </td>
    </tr>
  </table>

* **Mission Analysis**
  * Complete flight vehicle simulation
  * Energy network analysis
  * Design space exploration

* **Performance Analysis**
  * Payload range studies
  * Aerodynamic characteristics
  * V-N diagrams
  * Propeller performance
  * Takeoff field length estimation

* **Weights & Balance**
  * Operating empty weight estimation
  * Component-level weight breakdown
  * Center of gravity analysis
  * Moment of inertia calculations

### Advanced Capabilities
* **Optimization**
  * Gradient-based methods
  * Non-gradient algorithms
  * Multi-fidelity approaches
* **Artificial Intelligence Integration**
* **Model-Based Systems Engineering**

## External Interfaces
RCAIDE currently supports two external packages, OpenVSP and AVL. Regarding the former, users can automatically generate OpenVSP geometry from RCAIDE and even read in geometry to perform mission simulations. RCAIDE’s AVL interface enables the automatic generation of AVL files in addition to running AVL directly through the built-in Python API. This allows the designers to focus on design and analysis. Currently, the development team is working on an API for SU2, a high-fidelity CFD solver. This capability was a feature of SUAVE, and we want to bring it back for new RCAIDE users.

<p align="center">
  <img src="https://github.com/leadsgroup/RCAIDE_Website/blob/main/assets/img/Extenal_Interfaces.png" width=50% height=50%> 
</p> 



## Installing RCAIDE 
RCAIDE is available on GNU/Linux, MacOS and Windows. We strongly recommend installing RCAIDE within a Python virtual environment to avoid altering any distribution of Python files. Please review the documentation for instructions on creating a virtual environment for RCAIDE.

* [See Installation Instructions](https://www.docs.rcaide.leadsresearchgroup.com/install.html)
* Using pip : `pip install RCAIDE-LEADS`
* Using conda (coming soon) 

## Tutorials
[See Tutorials here](https://docs.rcaide.leadsresearchgroup.com/tutorials.html)

## Contributing to RCAIDE

**Contributing Institutions**
* [University of Illinois — Lab for Electric Aircraft Design and Sustainability](https://www.leadsresearchgroup.com/)

**Getting Involved**

If you'd like to help develop RCAIDE by adding new methods, writing documentation, or fixing bugs, please read the [contributing guidelines](https://www.docs.rcaide.leadsresearchgroup.com/contributing.html) first.

Submit improvements or new features via a [pull request](https://github.com/leadsgroup/RCAIDE_LEADS/pulls).

## Get in touch

Share feedback, report issues, and request features via [GitHub Issues](https://github.com/leadsgroup/RCAIDE_LEADS/issues)

Engage with peers and maintainers in [GitHub Discussions](https://github.com/leadsgroup/RCAIDE_LEADS/discussions)


