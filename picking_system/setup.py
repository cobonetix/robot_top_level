from setuptools import setup

package_name = 'picking_system'

setup(
    name=package_name,
    version='0.0.0',
    package_dir={'': 'source'},
    py_modules=[
        'picking_main',
        'picking_utils',
        'pick_item',
        'arm_init_calibrate',
        'service_clients',
        'joint_state_listener',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bob',
    maintainer_email='bob@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'picking = picking_main:main',
        ],
    },
)
