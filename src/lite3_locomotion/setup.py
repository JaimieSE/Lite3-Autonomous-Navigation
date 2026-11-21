from glob import glob
from setuptools import setup

setup(
    name='lite3_locomotion',
    version='0.1.0',
    packages=['lite3_locomotion'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/lite3_locomotion']),
        ('share/lite3_locomotion', ['package.xml', 'README.md']),
        ('share/lite3_locomotion/launch', glob('launch/*.launch.py')),
        ('share/lite3_locomotion/docs', glob('docs/*.md') + glob('docs/*.sha256')),
        ('share/lite3_locomotion/params', glob('params/*.yaml') + glob('params/*.xml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Lite3 workspace developer',
    maintainer_email='user@example.invalid',
    description='Lite3 navigation launch, terrain monitor and sensor relay used by roaming (M10).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'terrain_monitor = lite3_locomotion.terrain_monitor:main',
            'sensor_relay = lite3_locomotion.sensor_relay:main',
        ],
    },
)
