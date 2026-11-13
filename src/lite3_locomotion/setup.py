from glob import glob
from setuptools import find_packages, setup

setup(
    name='lite3_locomotion',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/lite3_locomotion']),
        ('share/lite3_locomotion', ['package.xml', 'README.md']),
        ('share/lite3_locomotion/config', glob('config/*')),
        ('share/lite3_locomotion/launch', glob('launch/*.launch.py')),
        ('share/lite3_locomotion/docs', [
            'docs/vendor_interface_audit.md', 'docs/movement_test_review.md',
            'docs/exploration_preview.md',
            'docs/autonomy_stack.md',
            'docs/robot_output.md',
            'docs/commissioning_pulse.md',
            'docs/daily_startup_checklist.md',
        ]),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Lite3 workspace developer',
    maintainer_email='user@example.invalid',
    description='Lite3 command-policy tests, bounded frontier missions, and a disarmed Nav2 adapter.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'locomotion_controller = lite3_locomotion.controller_node:main',
            'exploration_preview = lite3_locomotion.exploration_preview:main',
            'guarded_movement_adapter = lite3_locomotion.guarded_adapter:main',
            'exploration_coordinator = lite3_locomotion.exploration_coordinator:main',
            'autonomy_control = lite3_locomotion.autonomy_control:main',
            'motion_sender = lite3_locomotion.motion_sender:main',
            'commissioning_pulse = lite3_locomotion.commissioning_pulse:main',
        ],
    },
)
