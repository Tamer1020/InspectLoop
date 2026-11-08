from glob import glob

from setuptools import find_packages, setup

package_name = 'inspectloop_mission'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tamer1020',
    maintainer_email='202830391+Tamer1020@users.noreply.github.com',
    description='Uncertainty-aware gauge inspection mission (Nav2 viewpoint replanning), reports and evaluation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = inspectloop_mission.mission_manager:main',
            'generate_report = inspectloop_mission.report:main',
            'evaluate_run = inspectloop_mission.evaluate:main',
        ],
    },
)
