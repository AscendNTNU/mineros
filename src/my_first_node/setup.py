from setuptools import find_packages, setup
from glob import glob

package_name = 'my_first_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/json_files', glob('./json_files/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bård Wikmark',
    maintainer_email='bardwikmark2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm = my_first_node.fsm_node:main',
            'helper = my_first_node.helper_node:main',
        ]
    },
)
