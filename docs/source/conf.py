# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

import os
import sys

from unittest.mock import Mock
os.environ['SPHINX_BUILD'] = 'sphinx'

# Add all potential module paths
sys.path.insert(0, os.path.abspath('../..'))  # Root directory
sys.path.insert(0, os.path.abspath('../../RCAIDE'))  # RCAIDE directory
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath('.'))))  # Parent of docs

project = 'RCAIDE'
copyright = '2026, Laboratory for Emerging Aircraft Design and Systems'
author = 'Laboratory for Emerging Aircraft Design and Systems'
release = '1.0.0'

# Add these lines for the logo
html_logo = 'source/_static/leads_logo.png'  # Add your logo file to the _static directory
html_title = "RCAIDE"


# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
    'sphinx.ext.autosummary',
    'sphinx.ext.napoleon'  ,
    'nbsphinx',
    'sphinx.ext.mathjax',
]

templates_path = ['_templates']
exclude_patterns = []
toctree_maxdepth = 40
# Autosummary settings
autosummary_generate = True  # Generate stub pages for autosummary directives
add_module_names = False     # Remove module names from generated documentation

# Autodoc settings
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'special-members': '__init__',
    'undoc-members': True,
    'exclude-members': '__weakref__',
    'inherited-members': False,
    'show-inheritance': False
}

# Napoleon settings
# napoleon_google_docstring = False #True
# napoleon_numpy_docstring = True
# napoleon_include_init_with_doc = False
# napoleon_include_private_with_doc = False
# napoleon_include_special_with_doc = False
# napoleon_use_admonition_for_examples = False
# napoleon_use_admonition_for_notes = False
# napoleon_use_admonition_for_references = False
# napoleon_use_ivar = False
# napoleon_use_param = True
# napoleon_use_rtype = True
# napoleon_type_aliases = None

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_static_path = ['_static']
html_theme = 'pydata_sphinx_theme'

html_theme_options = {
    "navbar_start": ["navbar-logo"],
    "navbar_center": ["navbar-nav"],
    "navbar_end": ["theme-switcher", "navbar-icon-links"],
    "navbar_persistent": ["search-button"],
    "primary_sidebar_end": ["sidebar-ethical-ads"],
    "navigation_with_keys": False,
    "navbar_align": "left",
    "show_toc_level": 2,
    "navigation_depth": 3,
    'logo_only': True,
    'display_version': False,
    "external_links": [
        {"name": "RCAIDE Home", "url": "https://www.rcaide.leadsresearchgroup.com"},
    ],
    "icon_links": [
        {
            "name": "GitHub",
            "url": "https://github.com/leadsgroup/RCAIDE_LEADS",
            "icon": "fab fa-github-square",
            "type": "fontawesome",
        }
    ],
    # Add these logo-related options
    "logo": {
        "image_dark": "_static/leads_logo.png",  
        "image_light": "_static/leads_logo.png",
        "image_dark_alt": "_static/leads_logo.png",
        "image_light_alt": "_static/leads_logo.png",
        "text": "RCAIDE",  # Optional: text to appear next to the logo
    },
    "default_mode": "dark",
}

html_baseurl = "https://docs.rcaide.leadsresearchgroup.com"

html_context = {
    "default_mode": "light",
    "header_links": [
        ("GitHub", "https://github.com/leadsgroup/RCAIDE_LEADS", True),
    ]
}

html_theme = 'pydata_sphinx_theme'

html_context = {
    "default_mode": "auto",  # Default to light/dark mode based on user preference
}

html_css_files = ['custom.css']  # Add custom styles (optional)


# Mock load_plugin to avoid runtime errors
sys.modules['RCAIDE.Framework.Plugins.load_plugin'] = Mock()
