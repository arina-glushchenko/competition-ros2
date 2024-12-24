import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'comp_nedorosl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        #(os.path.join('share', package_name2, 'urdf'), glob(os.path.join('urdf', '*'))),
        #(os.path.join('share', package_name1, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        #(os.path.join('share', package_name1, 'worlds'), glob(os.path.join('worlds', 'materials', '*'))),
        #(os.path.join('share', package_name1, 'worlds'), glob(os.path.join('worlds', 'traffic_light', '*'))),
        #(os.path.join('share', package_name1, 'config'), glob(os.path.join('config', '*.yaml'))),
        #(os.path.join('share', package_name1, 'config'), glob(os.path.join('config', '*.rviz'))),
        #(os.path.join('share', package_name1, 'hooks'), glob(os.path.join('hooks', '*.in'))),
        #(os.path.join('share', package_name, 'signes'), glob(os.path.join('signes', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arina',
    maintainer_email='a.glushchenko2@g.nsu.ru',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'start = comp_nedorosl.start:main',
        ],
    },
)
