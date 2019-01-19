#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['osm_trajectory_planner'],
   package_dir={'osm_trajectory_planner': 'src/osm_trajectory_planner'}
)

setup(**d)
