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
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Lite3 workspace developer',
    maintainer_email='user@example.invalid',
    description='Autonomous frontier missions using the existing university Nav2 stack; preview by default.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'locomotion_controller = lite3_locomotion.controller_node:main',
            'exploration_preview = lite3_locomotion.exploration_preview:main',
            'exploration_coordinator = lite3_locomotion.exploration_coordinator:main',
            'autonomy_control = lite3_locomotion.autonomy_control:main',
            'terrain_monitor = lite3_locomotion.terrain_monitor:main',
        ],
    },
)
