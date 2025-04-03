# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'assignment_2_2024'
copyright = '2025, lisa mokrani'
author = 'lisa mokrani'
release = '1.0'

import os
import subprocess 
import sys
sys.path.insert(0, os.path.abspath('.')) 

show_authors=True

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [ 
'sphinx.ext.autodoc', 
'sphinx.ext.doctest', 
'sphinx.ext.intersphinx', 
'sphinx.ext.todo', 
'sphinx.ext.coverage', 
'sphinx.ext.mathjax', 
'sphinx.ext.ifconfig', 
'sphinx.ext.viewcode', 
'sphinx.ext.githubpages', 
"sphinx.ext.napoleon",
'sphinx.ext.inheritance_diagram',
'breathe'
]
extensions = ['myst_parser']

templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

# -- Ensure .nojekyll file is created after building the documentation --------
def create_nojekyll_file(app, exception):
    """Creates the .nojekyll file to prevent GitHub Pages from ignoring folders starting with '_'."""
    if exception is None:  # Ensure the build was successful
        nojekyll_path = os.path.join(app.outdir, ".nojekyll")
        with open(nojekyll_path, "w") as f:
            f.write("")

def setup(app):
    """Hook into Sphinx build process."""
    app.connect("build-finished", create_nojekyll_file)
    
