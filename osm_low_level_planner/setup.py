#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['osm_low_level_planner'],
   package_dir={'osm_low_level_planner': 'src/osm_low_level_planner'}
)

setup(**d)
