# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
# -- Path setup --------------------------------------------------------------
# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
import os
import sys
sys.path.insert(0, os.path.abspath('.'))
from sphinx import version_info as sphinx_version_info
# -- Project information -----------------------------------------------------
project = 'ROS2 Tutorials'
copyright = '2022, duyongquan'
author = 'duyongquan'
# -- General configuration ---------------------------------------------------
# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = []
source_suffix = ['.rst', '.md']
# The master toctree document.
master_doc = 'index'
# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']
# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', 'README.md', 'exercise']
extensions = ['sphinx.ext.githubpages']

if sphinx_version_info >= (1, 4):
  extensions += ['recommonmark']
else:
  from recommonmark.parser import CommonMarkParser

  source_parsers = {
      '.md': CommonMarkParser,
  }
# -- Options for HTML output -------------------------------------------------
# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
html_theme = 'sphinx_rtd_theme'
# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']
html_theme_options = {
    #'canonical_url': '',
    'analytics_id': '',
    'logo_only': False,
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,
    #'vcs_pageview_mode': '',
    # Toc options
    'collapse_navigation': True,
    'sticky_navigation': False,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False,
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']
