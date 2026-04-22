from setuptools import setup, find_packages

package_name = 'yaw_and_distance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'label_detect',
        'shelf_distance',
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
    description='Shelf yaw detection and distance estimation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
