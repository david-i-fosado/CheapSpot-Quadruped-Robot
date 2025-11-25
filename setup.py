from setuptools import setup, find_packages  # <--- agregar find_packages

setup(
    name='spot_micro_control',
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},
)
