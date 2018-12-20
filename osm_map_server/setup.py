#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   packages=['osm_map_server'],
   package_dir={'osm_map_server': 'src/osm_map_server'}
)

setup(**d)
