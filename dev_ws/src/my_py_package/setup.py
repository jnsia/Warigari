import os
from glob import glob
from setuptools import setup

package_name = 'my_py_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ssafy',
    maintainer_email='ssafy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_py_package.talker:main',
            'cam_to_num = my_py_package.cam_to_num:main',
            'speed = my_py_package.speed:main',
            'steering = my_py_package.steering:main',
            'distance = my_py_package.distance:main',
            'socket = my_py_package.socket:main',
            'custom_test = my_py_package.custom_test:main',
            'speed2 = my_py_package.speed2:main'
        ],
    },
    
)
