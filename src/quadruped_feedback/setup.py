from setuptools import setup
import os
from glob import glob

package_name = 'quadruped_feedback'

setup(
    name=package_name,
    version='0.0.0',
    # Szukamy folderów z kodem (muszą mieć __init__.py)
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antoni',
    description='Debug trajektorii D dla robota psa',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'nazwa_komendy = nazwa_pakietu.nazwa_pliku:funkcja_main'
            'debug_trajectory = quadruped_feedback.debug_trajectory_node:main'
        ],
    },
)
