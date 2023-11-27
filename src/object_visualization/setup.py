from setuptools import find_packages, setup

package_name = 'object_visualization'

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
    maintainer='Ragib Arnab',
    maintainer_email='rae3840924@gmail.com',
    description='Visualize bounding boxes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object3d_visualizer_node = object_visualization.object3d_visualizer_node:main'
        ],
    },
)
