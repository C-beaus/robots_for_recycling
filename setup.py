from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['panda_hw','moveit_commander'],
    package_dir={'': 'franka'}
)


setup(**d)