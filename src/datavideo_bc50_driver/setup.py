from setuptools import setup

package_name = 'datavideo_bc50_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}/launch', ['launch/bc50.launch.py']),
        (f'share/{package_name}/config', ['config/datavideo_bc50.yaml']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 driver for the DataVideo BC-50 zoom camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bc50_controller = datavideo_bc50_driver.controller:main',
        ],
    },
)
