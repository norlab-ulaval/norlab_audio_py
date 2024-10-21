from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'norlab_audio_py'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        # Include all param files
        (os.path.join('share', package_name), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Matej Boxan',
    maintainer_email='matej.boxan@norlab.ulaval.ca',
    description='TODO: Package description',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'audio_driver = norlab_audio_py.audio_driver:main',
            'audio_recorder = norlab_audio_py.audio_recorder:main'
        ],
    },
)
