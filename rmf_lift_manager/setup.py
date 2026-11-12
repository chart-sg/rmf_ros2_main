from setuptools import find_packages
from setuptools import setup

package_name = 'rmf_lift_manager'

setup(
    name=package_name,
    version='2.14.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leong Teck Tey',
    maintainer_email='leong_teck_tey@cgh.com.sg',
    description='Lift manager: forwards fleet adapter lift requests to '
    'lifts. Replaces lift_supervisor.',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'lift_manager=rmf_lift_manager.main:main',
        ],
    },
)
