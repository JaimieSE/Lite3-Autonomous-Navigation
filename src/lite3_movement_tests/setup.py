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
    description='Lite3 autonomous roaming (M10) with its readiness checks and simulator.',
    license='Apache-2.0',
    entry_points={'console_scripts': []},   # roaming runs via python3 -m lite3_movement_tests.roam
)
