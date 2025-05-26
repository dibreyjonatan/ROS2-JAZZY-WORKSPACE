from setuptools import find_packages, setup
import os
from glob import glob 

package_name = 'first_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jonatan',
    maintainer_email='kamdadibreyjonatan@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sender=first_project.publish_node:main',
            'reciever=first_project.subscriber_node:main',
        ],
    },
)
