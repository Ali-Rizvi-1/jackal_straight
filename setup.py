from setuptools import find_packages, setup
import os
from glob import glob

setup(
    name='jackal_straight',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    # ament index
    ('share/ament_index/resource_index/packages',
        ['resource/jackal_straight']),
    # package manifest
    (os.path.join('share','jackal_straight'), ['package.xml']),
    # all launch files
    (os.path.join('share','jackal_straight','launch'),
        glob('launch/*.launch.py')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ali',
    maintainer_email='alirizvi277.ar@gmail.com',
    description='Jackal straight-stop demo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'straight_stop      = jackal_straight.straight_stop:main',
        'heading_maintainer   = jackal_straight.heading_maintainer:main',
        ],
    },
)
