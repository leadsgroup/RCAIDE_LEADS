.. _install:

##############
Installation
##############

This guide will help you install RCAIDE and its dependencies.

System Requirements
==================
Before installing RCAIDE, ensure your system meets these requirements:

* **Python**: Version 3.8 to 3.13
* **pip**: Version 20.0 or higher
* **Operating Systems**: Linux, macOS, Windows

Installation Methods
==================

Method 1: Using pip (Recommended)
--------------------------------

The simplest way to install RCAIDE is via pip:

.. code-block:: bash

    pip install RCAIDE-LEADS

Method 2: From Source
--------------------

For developers or users who need the latest features, install from source:

1. Clone the repository:

   .. code-block:: bash

      git clone https://github.com/leadsgroup/RCAIDE_LEADS.git
      cd RCAIDE_LEADS
      git checkout develop

2. Install in editable mode with dependencies:

   .. code-block:: bash

      pip install -e .

Verification
===========

Verify your installation:

.. code-block:: bash

    python3 -c "import RCAIDE; print(RCAIDE.__version__)"

The command should display the current version number without errors.

Troubleshooting
==============

If you encounter issues:

1. Ensure your Python version is compatible:

   .. code-block:: bash

      python3 --version

2. Update pip to the latest version:

   .. code-block:: bash

      python3 -m pip install --upgrade pip

3. If you see dependency conflicts, try installing in a fresh virtual environment:

   .. code-block:: bash

      python3 -m venv rcaide-env
      source rcaide-env/bin/activate  # On Windows: rcaide-env\Scripts\activate
      pip install RCAIDE-LEADS


