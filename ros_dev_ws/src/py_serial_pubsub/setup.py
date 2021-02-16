from setuptools import setup

package_name = 'py_serial_pubsub'

setup(
    name=package_name,
    version='0.0.1',
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
    description='Serial Reading for Arduino @ 115200 baud',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		 'talker = py_serial_pubsub.publisher_member_function:main',
		'listener = py_serial_pubsub.subscriber_member_function:main',
        ],
    },
)
