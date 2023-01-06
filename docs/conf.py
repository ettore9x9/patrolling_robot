# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys
import subprocess

sys.path.insert(0, os.path.abspath('..'))
subprocess.call('doxygen Doxyfile.in', shell=True)

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'patrolling_robot'
copyright = '2023, Ettore Sani'
author = 'Ettore Sani'
release = '0.0.1'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.viewcode',
    'sphinx.ext.napoleon',
    'sphinx.ext.doctest', 
    'sphinx.ext.intersphinx', 
    'sphinx.ext.todo', 
    'sphinx.ext.coverage', 
    'sphinx.ext.mathjax', 
    'sphinx.ext.ifconfig', 
    'sphinx.ext.githubpages', 
    'sphinx.ext.inheritance_diagram',
	'breathe'
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
highlight_language = 'c++' 
source_suffix = '.rst' 
master_doc = 'index'

# -- Options for intersphinx extension --------------------------------------

# Example configuration for intersphinx: refer to the Python standard library. 
intersphinx_mapping = {'https://docs.python.org/': None} 

# -- Options for todo extension ----------------------------------------------

# If true, `todo` and `todoList` produce output, else they produce nothing. 
todo_include_todos = True

# -- Options for breathe 

breathe_projects = { 
	"patrolling_robot": "_build/xml/"
}

breathe_default_project = "patrolling_robot"
breathe_default_members = ('members', 'undoc-members')