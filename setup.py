from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["monkeywrench"],
    package_dir={"": "src"},
    scripts=["scripts/monkeylaunch"],
)

setup(**setup_args)
