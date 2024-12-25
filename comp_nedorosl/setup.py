import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'comp_nedorosl'

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),  
    (os.path.join('share', package_name, 'weights'), glob('weights/*')),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arina',
    maintainer_email='a.glushchenko2@g.nsu.ru',
    description='ros2 autorace competition',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid = comp_nedorosl.pid:main',
            'detect_line = comp_nedorosl.detect_line:main',
            'sign_recognition = comp_nedorosl.sign_recognition:main'
        ],
    },
)
