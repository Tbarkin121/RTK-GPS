from setuptools import setup

package_name = 'rtk'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tyler Barkin',
    maintainer_email='tybarkin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtcm3_pub = rtk.publish_RTCM3:main',
            'rtcm3_sub = rtk.subscribe_RTCM3:main',
            'ubx_pub = rtk.publish_UBX:main',
            'ubx_sub = rtk.subscribe_UBX:main',
            'base_node = rtk.basestation_node:main',
            'rover_node = rtk.rover_node:main',
        ],
    },
)
