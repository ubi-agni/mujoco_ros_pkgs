from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup, find_packages

packages = find_packages("python")
d = generate_distutils_setup(packages=packages, package_dir={"": "python"})

setup(**d)
