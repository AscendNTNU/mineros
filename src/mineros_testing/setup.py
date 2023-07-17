from setuptools import find_packages, setup

package_name = 'mineros_testing'

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
    maintainer='Bård Wikmark',
    maintainer_email='bardwikmark2001@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movement_tester = mineros_testing.movement_test:main',
            'mining_tester = mineros_testing.mining_test:main',
        ]
    },
)
