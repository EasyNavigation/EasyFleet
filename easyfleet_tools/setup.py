from setuptools import find_packages, setup

package_name = 'easyfleet_tools'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(
        include=[package_name, package_name + '.*'], exclude=['test', 'scripts']
    ),
    include_package_data=True,
    package_data={
        # include everything under easyfleet_tools/vendor in the installed package
        'easyfleet_tools': ['vendor/**'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools', 'rich>=13.3.0'],
    zip_safe=False,
    maintainer='Francisco Martín Rico',
    maintainer_email='fmrico@gmail.com',
    description='ROS 2 EasyFleet tools: TUI (Textual) + ros2cli commands for monitoring a fleet.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tui = easyfleet_tools.tui.app:run_app',
        ],
        # ros2 easyfleet <verb>
        'ros2cli.command': [
            'easyfleet = easyfleet_tools.cli.easyfleet:EasyfleetCommand',
        ],
        # IMPORTANT: extension point + verb group for ros2cli
        'ros2cli.extension_point': [
            'easyfleet.verb = ros2cli.verb:VerbExtension',
        ],
        'easyfleet.verb': [
            'fleet = easyfleet_tools.cli.fleet:FleetVerb',
            'describe = easyfleet_tools.cli.describe:DescribeVerb',
            'watch = easyfleet_tools.cli.watch:WatchVerb',
            'status = easyfleet_tools.cli.status:StatusVerb',
            'logs = easyfleet_tools.cli.logs:LogsVerb',
        ],
    },
)
