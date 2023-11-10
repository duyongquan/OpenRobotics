from setuptools import find_packages, setup

package_name = 'casadi_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='quandy',
    maintainer_email='quandy2020@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_member_function = casadi_python.publisher_member_function:main',
            'tutorials01_test_casadi_env = casadi_python.tutorials01_test_casadi_env:main',
            'tutorials02_test_casadi_rosenbrock = casadi_python.tutorials02_test_casadi_rosenbrock:main',
            'tutorials03_test_casadi_himmelblau = casadi_python.tutorials03_test_casadi_himmelblau:main',
            'tutorials04_test_casadi_mpc_single_shooting = casadi_python.tutorials04_test_casadi_mpc_single_shooting:main',
        ],
    },
)
