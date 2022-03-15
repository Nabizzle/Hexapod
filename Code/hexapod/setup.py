from setuptools import find_packages, setup
setup(
    name='hexapod',
    packages=find_packages(include=['hexapod']),
    version='0.1.0',
    description='My first Python library',
    author='Me',
    license='MIT',
    install_requires=[],
    setup_requires=['pytest-runner==6.0.0'],
    tests_require=['pytest==7.1.0'],
    test_suite='tests',
)
