from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'car_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), glob('urdf/*')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'models'), glob('car_lexus/car.urdf'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Description of the package',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = state_publisher:main'
        ],
    },
)

