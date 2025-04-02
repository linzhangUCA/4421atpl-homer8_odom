from setuptools import find_packages, setup

package_name = 'turtlesim_play_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO: author name',
    maintainer_email='TODO: email address',
    description='TODO: Package description',
    license='GNU General Public License v3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'figure8_node = turtlesim_play_pkg.figure8_node:main'
        ],
    },
)
