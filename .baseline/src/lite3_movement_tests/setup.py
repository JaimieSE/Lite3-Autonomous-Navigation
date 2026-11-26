from glob import glob
from setuptools import setup

setup(
    name='lite3_movement_tests',
    version='0.1.0',
    packages=['lite3_movement_tests'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/lite3_movement_tests']),
        ('share/lite3_movement_tests', ['package.xml', 'README.md']),
        ('share/lite3_movement_tests/config', glob('config/*.yaml') + glob('config/*.xml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Lite3 workspace developer',
    maintainer_email='user@example.invalid',
    description='Telemetry-verified, bounded Lite3 movement test sequences.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'movement_test = lite3_movement_tests.cli:main',
            'movement_test_sim = lite3_movement_tests.sim_node:main',
        ],
    },
)
