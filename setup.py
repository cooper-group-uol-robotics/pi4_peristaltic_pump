#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
print("generating python modules")
d = generate_distutils_setup(
    packages=['peristaltic_dispenser_driver'],
    package_dir={'': 'src'}
)

setup(**d)
