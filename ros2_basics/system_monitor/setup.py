from setuptools import setup

package_name = 'system_monitor'

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
    maintainer='quan',
    maintainer_email='quandy2020@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cpu_monitor = system_monitor.cpu_monitor:main',
            # 'hdd_monitor = system_monitor.hdd_monitor:main',
            # 'mem_monitor = system_monitor.mem_monitor:main',
            # 'net_monitor = system_monitor.net_monitor:main',
            # 'ntp_monitor = system_monitor.ntp_monitor:main',
        ],
    },
)
