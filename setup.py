from setuptools import setup
from glob import glob

package_name = 'maze_solver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        ## NOTE: you must add this line to use launch files
        # Instruct colcon to copy launch files during package build 
        ('share/' + package_name + '/launch', glob('launch/*.launch.*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='usi',
    maintainer_email='usi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_proximity = maze_solver.controller_proximity:main',
            'controller_camera = maze_solver.controller_camera:main',
        ],
    },
)
