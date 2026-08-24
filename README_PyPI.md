<p align="center">
  <img src="https://raw.githubusercontent.com/leadsgroup/RCAIDE_Website/main/assets/img/RCAIDE_Logo_No_Background.png" width=25% height=25%>
</p>

<p align="center">
  <a href="https://aerospace.illinois.edu">
    <img src="https://raw.githubusercontent.com/leadsgroup/RCAIDE_Website/main/assets/img/Illinois_logo_fullcolor_rgb.png" height="80">
  </a>
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
  <a href="https://www.leadsresearchgroup.com">
    <img src="https://raw.githubusercontent.com/leadsgroup/RCAIDE_Website/main/assets/img/LEADS_logo_1.png" height="80">
  </a>
</p>

<p align="center">
  Developed at the <strong>University of Illinois Urbana-Champaign</strong><br>
  <a href="https://www.leadsresearchgroup.com"><strong>Laboratory for Emerging Aircraft Design and Systems (LEADS)</strong></a>
</p>

<div align="center">

[![CI](https://github.com/leadsgroup/RCAIDE_LEADS/actions/workflows/CI.yml/badge.svg?branch=master)](https://github.com/leadsgroup/RCAIDE_LEADS/actions/workflows/CI.yml)
[![Documentation](https://github.com/leadsgroup/RCAIDE_LEADS/actions/workflows/sphinx_docs.yml/badge.svg)](https://github.com/leadsgroup/RCAIDE_LEADS/actions/workflows/sphinx_docs.yml)
[![codecov](https://codecov.io/gh/leadsgroup/RCAIDE_LEADS/graph/badge.svg?token=WZOFW5EKWJ)](https://codecov.io/gh/leadsgroup/RCAIDE_LEADS)
[![PyPI Downloads](https://static.pepy.tech/badge/rcaide-leads)](https://pepy.tech/projects/rcaide-leads)

</div>

---

# RCAIDE: Research Community Aerospace Interdisciplinary Design Environment

RCAIDE (pronounced "arcade") is an open-source Python platform for the modeling, analysis, and optimization of both conventional and unconventional aerospace systems. The successor to the globally recognized SUAVE code, RCAIDE builds on a robust foundation of validated engineering methods while extending its scope to address novel design paradigms including hybrid-electric propulsion, hydrogen-powered aircraft, urban air mobility vehicles, and supersonic transports.

RCAIDE's modular architecture supports multi-fidelity analysis, high-performance parallel computing, and a generalized energy-agnostic powertrain network that enforces thermodynamic closure across all powertrain domains. It is developed and maintained by the [Laboratory for Emerging Aircraft Design and Systems (LEADS)](https://www.leadsresearchgroup.com/) at the University of Illinois Urbana-Champaign.

## Key Features

### Multidisciplinary Analysis
| Discipline | Methods |
|---|---|
| Geometry | 2D & 3D parametric planform definition, OpenVSP & AVL interfaces |
| Atmosphere | US Standard 1976 atmospheric model |
| Aerodynamics | Vortex Lattice Method (VLM), viscous vortex panel method, SU2 CFD interface |
| Stability | VLM-based perturbation approach; longitudinal and lateral-directional derivatives |
| Mass Properties | FLOPS, Raymer, and physics-based weight methods; CG and MOI estimation |
| Powertrain | Polytropic turbomachinery, BEMT/actuator-disk rotors, electric motors, fuel cells, heat exchangers, cryogenic tanks |
| Aeroacoustics | Semi-empirical airframe and jet noise; physics-based rotor harmonic noise |
| Emissions | Empirical emission indices; chemical reactor network (CRN) via Cantera |
| Performance | Payload-range diagrams, V-n diagrams, takeoff/landing field length, stall speed |
| Optimization | Gradient-based (SLSQP) and non-gradient methods via pyOptSparse |

### Propulsion Architectures Supported
- Conventional turbofan, turbojet, turboprop
- Fully electric (battery-powered rotors and ducted fans)
- Series and parallel hybrid-electric
- Hydrogen (gaseous and liquid cryogenic)
- Internal combustion engine + propeller
- Fuel cell systems

### Mission Solver
RCAIDE employs a pseudospectral collocation method using Chebyshev polynomials for solving flight mechanics, departing from traditional time-marching techniques. A closed-loop powertrain network solver enforces energy balance across electrical, thermal, mechanical, pneumatic, and hydraulic domains at every mission point — eliminating the open-loop approximations that can mask design flaws in complex powertrains.

## Validation

RCAIDE's methods are validated against experimental data and high-fidelity simulations. Aerodynamic predictions using the VLM and semi-empirical drag buildup achieve R² = 0.96 (C_L) and R² = 0.94 (C_D) against National Transonic Facility wind-tunnel data for the NASA Common Research Model. Vehicle-level validation includes payload-range comparisons for the Boeing 737-800, ATR 72-600, Boeing 787-8, and Concorde.

Full validation tables are available in the [RCAIDE paper](https://doi.org/10.1016/j.ast.2026.112328) and the [Verification & Validation subdirectory](https://github.com/leadsgroup/RCAIDE_LEADS/tree/master/VnV).

## Installation

```bash
pip install RCAIDE-LEADS
```

We recommend installing within a virtual environment. See the full [installation guide](https://www.docs.rcaide.leadsresearchgroup.com/install.html) for environment setup, optional dependencies (OpenVSP, AVL, SU2), and platform-specific notes.

## Documentation & Tutorials

- [Documentation](https://www.docs.rcaide.leadsresearchgroup.com)
- [Tutorials](https://docs.rcaide.leadsresearchgroup.com/tutorials.html)
- [GitHub Repository](https://github.com/leadsgroup/RCAIDE_LEADS)

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

## Contributing

Contributions are welcome. Please read the [contributing guidelines](https://www.docs.rcaide.leadsresearchgroup.com/contributing.html) before submitting a pull request.

## Contact

- Bug reports and feature requests: [GitHub Issues](https://github.com/leadsgroup/RCAIDE_LEADS/issues)
- Community discussion: [GitHub Discussions](https://github.com/leadsgroup/RCAIDE_LEADS/discussions)
