from setuptools import setup

package_name = 'wheel_morph'

setup(
    name=package_name,
    version='0.0.0',
    packages=['wheel_morph'],
    package_dir={'': 'src'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='WheelMorph ROS2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_node = wheel_morph.wheel_node:main',
        ],
    },
)
