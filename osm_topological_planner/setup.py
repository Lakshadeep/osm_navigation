#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['osm_topological_planner'],
   package_dir={'osm_topological_planner': 'src/osm_topological_planner'}
)

setup(**d)
