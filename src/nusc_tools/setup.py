from setuptools import find_packages, setup

package_name = 'nusc_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Register the package in the ament index so that `ros2 pkg list` finds it.
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    # Tells colcon to run the tests with pytest instead of unittest.
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Riccardo Valentini',
    maintainer_email='239980864+richival01-web@users.noreply.github.com',
    description='Python tools around nuScenes for the 3D multi-object tracker.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'print_scene = nusc_tools.print_scene:main',
        ],
    },
)
