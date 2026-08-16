from setuptools import find_packages, setup

package_name = 'easyfleet_mission_manager_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, package_name + '.*'], exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco Martín Rico',
    maintainer_email='fmrico@gmail.com',
    description=(
        'Python mirror of easyfleet_mission_manager: FleetSession/RobotHandle/'
        'SimpleController for writing mission controllers in Python.'
    ),
    license='Apache-2.0',
)
