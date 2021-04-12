from setuptools import setup

package_name = 'powermon'

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
    description='Acquiring arduino data and publish on a set of topics',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "power_pub = powermon.powerpub:main",],
    },
)
