from setuptools import find_packages, setup

package_name = 'arb_evaluation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/scenario.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Emin Ayhan',
    maintainer_email='aemin8343@gmail.com',
    description=(
        'Offline analysis toolkit for the Autonomous Robot Brain stack: '
        'trajectory metrics, covariance plots, planner benchmarks and '
        'fault-injection comparison reports.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fault_scheduler = arb_evaluation.fault_scheduler:main',
        ],
    },
)
