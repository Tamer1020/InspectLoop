from glob import glob

from setuptools import find_packages, setup

package_name = 'inspectloop_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tamer1020',
    maintainer_email='202830391+Tamer1020@users.noreply.github.com',
    description='Uncertainty-aware analog gauge reading (OpenCV) for InspectLoop.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gauge_reader_node = inspectloop_vision.gauge_reader_node:main',
        ],
    },
)
