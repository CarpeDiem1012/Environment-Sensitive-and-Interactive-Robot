from distutils.core import setup

setup(
    version='0.0.0',
    scripts=['bin/src/path_planning.py'],
    packages=['path_planning'],
    package_dir={'': 'src'}
)