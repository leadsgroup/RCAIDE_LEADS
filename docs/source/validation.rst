.. _validation:

##########
Validation
##########

RCAIDE's physics-based methods are systematically validated against experimental data and high-fidelity simulations to ensure engineering accuracy across a range of vehicle classes and flight conditions.

Aerodynamic Validation
======================

Aerodynamic predictions were evaluated against NASA wind-tunnel data for the **Common Research Model (CRM)**, a representative transonic transport configuration used as an industry benchmark.

.. image:: _static/NASA_CRM.jpg
   :alt: NASA Common Research Model
   :align: center
   :width: 600px

|

Results demonstrate strong agreement with experimental data:

- **Lift coefficient** (C\ :sub:`L`\ ): R² = 0.96
- **Drag coefficient** (C\ :sub:`D`\ ): R² = 0.94

.. image:: _static/Figure_4a.png
   :alt: Lift coefficient vs angle of attack
   :align: center
   :width: 500px

.. image:: _static/Figure_4b.png
   :alt: Drag coefficient vs angle of attack
   :align: center
   :width: 500px

.. image:: _static/Figure_4c.png
   :alt: Pitching moment vs angle of attack
   :align: center
   :width: 500px

|

Full V&V Suite
==============

The complete Verification and Validation suite is located in the ``VnV/`` directory of the RCAIDE repository:

- **VnV/Validation/** — regression tests against experimental and reference data
- **VnV/Verification/** — unit and integration tests for numerical correctness
- **VnV/Vehicles/** — reference vehicle configurations used in testing

For full methodology details and additional validation cases, see the methods paper:

   Clarke, M. A., et al. *RCAIDE: An Open-Source Framework for Multidisciplinary Aerospace Vehicle Design and Analysis.*
   Aerospace Science and Technology, 2026.
   `https://doi.org/10.1016/j.ast.2026.112328 <https://doi.org/10.1016/j.ast.2026.112328>`_
