#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
print("generating python modules")
d = generate_distutils_setup(
    packages=['pi4_peristaltic_pump_driver'],
    package_dir={'': 'src'}
)

setup(**d)
