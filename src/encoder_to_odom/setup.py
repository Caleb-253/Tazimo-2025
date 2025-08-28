from setuptools import setup

package_name = 'encoder_to_odom'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/encoder_to_odom.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tazimo',
    maintainer_email='example@example.com',
    description='Reads encoder ticks from serial and publishes /odom and TF.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_to_odom = encoder_to_odom.encoder_to_odom_node:main',
        ],
    },
)

