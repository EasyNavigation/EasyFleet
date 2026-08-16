from setuptools import find_packages, setup

package_name = 'easyfleet_easynav_collaboration_simple_api_deployment_py'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, package_name + '.*'], exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/easynav_collaboration_launch.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco Martín Rico',
    maintainer_email='fmrico@gmail.com',
    description=(
        'Python-mission-controller mirror of '
        'easyfleet_easynav_collaboration_simple_api_deployment.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'easynav_collaboration_mission_node = '
            'easyfleet_easynav_collaboration_simple_api_deployment_py.mission:main',
        ],
    },
)
