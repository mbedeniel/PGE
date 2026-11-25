from glob import glob

from setuptools import find_packages, setup

package_name = 'mfja_yaskawa_motoros2_sim'

setup(
    name=package_name,
    version='0.3.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Niel MBEDE',
    maintainer_email='ryan-niel.tayou-mbede@utoulouse.fr',
    description='Imitation of the MotoROS2 ROS 2 interface for the simulated MFJA Yaskawa arms.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motoros2_sim = mfja_yaskawa_motoros2_sim.motoros2_sim_node:main',
        ],
    },
)
