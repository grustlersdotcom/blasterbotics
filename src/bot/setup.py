from setuptools import setup

package_name = 'bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','pyserial','sparkfun-ublox-gps','Jetson.GPIO'],
    zip_safe=True,
    maintainer='paddock-pal',
    maintainer_email='nhenderson1@mines.edu',
    description='Main Nodes from PaddockPal Senior Design Semester 2',
    license='TBD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_control = bot.motor_control:main'
            'gps_polling = bot.gps_polling:main'
        ],
    },
)
