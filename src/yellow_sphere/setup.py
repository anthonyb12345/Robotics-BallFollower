from setuptools import find_packages, setup

package_name = 'yellow_sphere'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', 
            ['launch/yellow_sphere.launch.py']),
    ],
    install_requires=['setuptools','opencv-python'],
    zip_safe=True,
    maintainer='anthony',
    maintainer_email='anthony.bassil3@net.usj.edu.lb',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_sphere = yellow_sphere.follow_sphere:main',
        ],
    },
)
