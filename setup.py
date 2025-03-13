from setuptools import find_packages, setup

package_name = 'rotary_encoder_odometry'

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
    maintainer='pi',
    maintainer_email='andrewmjohnson549@gmail.com',
    description='A project used for taking raw rotary encoder values and mapping it to 1d odometry.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = rotary_encoder_odometry.publisher:main',
        ],
    },
)
