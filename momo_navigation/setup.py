import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'momo_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*')),
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml')),
        ),
        (
            os.path.join('share', package_name, 'config/rviz'),
            glob(os.path.join('config/rviz', '*.rviz')),
        ),
        (
            os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*')),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itlbot2',
    maintainer_email='todo@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'test_node = momo_navigation.test_node:main' 
        ],
    },
)
