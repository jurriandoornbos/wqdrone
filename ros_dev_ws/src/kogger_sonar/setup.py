from setuptools import setup

package_name = 'kogger_sonar'

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
    description='Kogger Sonar package for acquiring depth information via the USB connection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'sendto = kogger_sonar.sendtokogger:main',
        'listento = kogger_sonar.listentokogger:main',
        'distance_send = kogger_sonar.kogger_distance_send:main',
        'distance list = kogger_sonar.kogger_distance_list:main',
        ],
    },
)
