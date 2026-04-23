from setuptools import setup

package_name = 'ld06_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    description='LD06 LiDAR driver',
    entry_points={
        'console_scripts': [
            'ld06_node = ld06_driver.ld06_node:main',
        ],
    },
)
