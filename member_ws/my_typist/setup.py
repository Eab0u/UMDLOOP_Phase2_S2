import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'my_typist'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='member',
    maintainer_email='member@todo.todo',
    description='Autonomous camera-guided keyboard typing node for UMD Loop S2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'typist = my_typist.typist:main',
        ],
    },
)
