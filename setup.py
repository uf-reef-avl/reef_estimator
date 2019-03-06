
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['reef_estimator'],
    scripts=['scripts/verify_estimates'],
    package_dir={'': 'scripts'}
)
