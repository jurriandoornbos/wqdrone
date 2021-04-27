from setuptools import setup

package_name = 'teensy'

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
    maintainer='Jurrian Doornbos',
    maintainer_email='jurrian.doornbos@wur.nl',
    description='Acquiring Teensy data and publish on two nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "teensy_pub = teensy.teensy_pub:main",
        "gps = teensy.gps:main",
        "sensor = teensy.sensor:main",
        "serial_pub = teensy.serial_pub:main",],
    },
)
