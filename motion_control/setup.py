from distutils.core import setup

setup(
    version='0.0.0',
    scripts=['bin/src/motion_control.py'],
    packages=['motion_control'],
    package_dir={'': 'src'}
)
