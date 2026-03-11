from setuptools import setup, find_packages
import sys

# colcon may pass args that older distutils parsers don't recognize.
# Strip unsupported args while preserving normal setup.py invocation.
def _sanitize_argv(argv):
    cleaned = [argv[0]]
    skip_next = False
    for arg in argv[1:]:
        if skip_next:
            skip_next = False
            continue
        if arg in ('--editable', '--uninstall'):
            continue
        if arg == '--build-directory':
            skip_next = True
            continue
        cleaned.append(arg)
    return cleaned


sys.argv = _sanitize_argv(sys.argv)

package_name = 'sjtu_drone_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='georg.novtony@aon.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'teleop = sjtu_drone_control.teleop:main',
            'teleop_joystick = sjtu_drone_control.teleop_joystick:main',
            'open_loop_control = sjtu_drone_control.open_loop_control:main',
            'drone_position_control = sjtu_drone_control.drone_position_control:main'
        ],
    },
)
