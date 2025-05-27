from setuptools import find_packages, setup

package_name = 'lolo_local_mqtt_translator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'lolo_local_translator_node = lolo_local_mqtt_translator.lolo_local_translator_node:main'
        ],
    },
)
