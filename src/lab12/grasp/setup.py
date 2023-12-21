from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'grasp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tony',
    maintainer_email='ye.tian@iqr-robot.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'grasp = grasp.grasp:main',
            'grasp_causal = grasp.grasp_causal:main',
            'navigator = grasp.navigator:main',
            'my_navigator = grasp.my_navigator:main',
            'navigator_casual = grasp.navigator_casual:main'
        ],
    },
)
