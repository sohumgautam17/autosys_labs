from setuptools import setup

package_name = 'safety_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sohum Gautam',
    maintainer_email='sgauta01@seas.upenn.edu',
    description='Automatic Emergency Braking at University of Pennsylvania',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = safety_node.safety_node:main',
        ],
    },
)
