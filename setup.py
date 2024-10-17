from setuptools import setup

setup(
    name='robotics_sim',
    version='',
    packages=['core', 'tests', 'tests.core', 'tests.ros2', 'tests.robots', 'tests.loggers', 'tests.simulators',
              'trials', 'modules'],
    url='https://github.com/VicenteMoraes/robotics_sim',
    license='',
    author='VicenteMoraes',
    author_email='',
    description='Library for implementing Distributed Robotics experiment trials with Docker.',
    install_requires=['docker>=5.0.3', 'numpy>=1.23']
)
