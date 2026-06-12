from glob import glob
from setuptools import find_packages, setup

package_name = 'teleOp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='box',
    maintainer_email='jaydench@andrew.cmu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard = teleOp.keyboard:main',
            'wheels = teleOp.wheels:main',
            'legs = teleOp.legs:main',
            'web_teleop = teleOp.web_teleop:main',
            'teleop_receiver = teleOp.teleop_receiver:main',
            'teleop_sender = teleOp.pygame_sender:main',
        ],
    },
)
