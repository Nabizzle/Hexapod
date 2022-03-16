from setuptools import find_packages, setup
setup(
    name='hexapod',
    packages=find_packages(include=['hexapod']),
    version='0.1.0',
    description='Library of Hexapod Functions',
    author='Nabeel Chowdhury',
    license='GPL-2.0',
    install_requires=['numpy==1.22.3'],
    setup_requires=['pytest-runner==6.0.0'],
    tests_require=['pytest==7.1.0', 'numpy==1.22.3'],
    test_suite='tests',
)
