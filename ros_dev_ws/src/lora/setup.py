from setuptools import setup

package_name = 'lora'

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
    description='LoRa topic sender and receiver for inside the ROS system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "wq_send = lora.lora_wq_send:main",
        "wq_rec = lora.lora_wq_receive:main",
        "sonar_send = lora.lora_sonar_send:main",
        "sonar_rec = lora.lora_sonar_receive:main",
        "wq_gps_send = lora.lora_wq_gps_send:main",
        "wq_gps_rec = lora.lora_wq_gps_receive:main",
        "cmd_send = lora.lora_cmd_send:main",
        "cmd_receive = lora.lora_cmd_receive:main",
        ],
    },
)
