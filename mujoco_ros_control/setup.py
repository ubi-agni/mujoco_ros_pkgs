from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import find_packages
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=find_packages("python/src"),
    package_dir={"": "python/src"},
)

setup(**setup_args)
