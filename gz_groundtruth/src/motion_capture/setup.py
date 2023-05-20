from setuptools import setup

package_name = 'motion_capture'

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
    maintainer='judocero',
    maintainer_email='judocero@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocap_node_classic = motion_capture.mocap_node_classic:main',
            'mocap_node_garden = motion_capture.mocap_node_garden:main'
        ],
    },
)
