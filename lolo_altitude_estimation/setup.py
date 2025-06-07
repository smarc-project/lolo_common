from setuptools import find_packages, setup
import os, glob 

package_name = 'lolo_altitude_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niklas',
    maintainer_email='nrol@kth.se',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'altitude_node = lolo_altitude_estimation.altitude:main',
            'fls_range = lolo_altitude_estimation.fls_range:main',
            'estimator_1 = lolo_altitude_estimation.fls_range_estimator_1:main',
            'estimator_2 = lolo_altitude_estimation.fls_range_estimator_2:main',
        ],
    },
)
