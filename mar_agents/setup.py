import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mar_agents'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config', '_base_'), glob('config/_base_/*.py')),
        (os.path.join('share', package_name, 'config', 'agent'), glob('config/agent/*.py')),
        (os.path.join('share', package_name, 'config', 'pipeline'), glob('config/pipeline/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='spencer',
    maintainer_email='spencer.hallyburton@duke.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "agent_passive = mar_agents.agent_passive:main"
        ],
    },
)
